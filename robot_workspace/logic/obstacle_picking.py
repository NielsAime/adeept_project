"""
Logique de pick and place d'obstacle (bouteille d'eau).

Strategie :
  1. Surveillance ultrason dans un thread dedie (10 Hz)
  2. Si obstacle detecte sous le seuil → prise du verrou materiel
     (bloque les commandes TCP) et arret complet du robot
  3. Sequence PICK : bras tendu + pince ouverte → fermeture pince → lever
  4. Sequence PLACE : rotation base vers la gauche → descente → lacher
  5. Retour du bras en position initiale, liberation du verrou
  6. Le navigateur ArUco reprend le controle

Utilisation dans main_robot.py :
  picker = ObstaclePicker(motors, arm, sonar)
  picker.init_arm()
  picker.start_monitoring(hardware_lock)
"""

import time
import threading
from hardware.motor      import MotorController
from hardware.servo       import ArmController
from hardware.ultrasonic  import UltrasonicSensor


class ObstaclePicker:
    # --- Seuil de detection -----------------------------------------------
    THRESHOLD_CM    = 15.0    # distance (cm) en dessous de laquelle on saisit
    MONITOR_FREQ_HZ = 10     # frequence de lecture ultrason (Hz)

    # --- Positions du bras [base, epaule, coude, pince] --------------------
    #   base     : 90 = face avant,  0 = gauche, 180 = droite
    #   epaule   : 90 = repos,      60 = penche vers l'avant
    #   coude    : 90 = repos,     120 = bras etendu
    #   pince    : 30 = ouverte,   160 = fermee
    POS_HOME    = [90, 90,  90,  90]    # position de repos
    POS_OPEN    = [90, 60, 120,  30]    # bras tendu, pince ouverte
    POS_GRAB    = [90, 60, 120, 160]    # pince fermee sur l'objet
    POS_LIFT    = [90, 90,  90, 160]    # objet leve
    POS_SIDE    = [ 0, 90,  90, 160]    # rotation base vers la gauche
    POS_PLACE   = [ 0, 60, 120, 160]    # descente sur le cote
    POS_RELEASE = [ 0, 60, 120,  30]    # lacher l'objet

    # --- Pause entre mouvements (secondes) --------------------------------
    MOVE_PAUSE = 0.6

    def __init__(self, motors: MotorController, arm: ArmController,
                 sonar: UltrasonicSensor):
        self.motors = motors
        self.arm    = arm
        self.sonar  = sonar

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------
    def is_obstacle(self, distance_cm: float) -> bool:
        """Retourne True si un obstacle est a portee de saisie."""
        return 0.0 < distance_cm < self.THRESHOLD_CM

    # ------------------------------------------------------------------
    # Initialisation du bras
    # ------------------------------------------------------------------
    def init_arm(self) -> None:
        """Place le bras en position initiale (home)."""
        print("[Picker] Initialisation du bras en position home")
        self.arm.set_all(self.POS_HOME)

    # ------------------------------------------------------------------
    # Sequence pick and place
    # ------------------------------------------------------------------
    def pick_and_place(self) -> None:
        """
        Sequence complete de pick and place.
        Bloquant (~7 × MOVE_PAUSE secondes).

        Doit etre appele avec le verrou materiel pris pour bloquer
        les commandes TCP pendant toute la duree de la manoeuvre.
        """
        print("[Picker] Obstacle detecte — sequence pick and place")

        # 1. Arret complet du robot
        self.motors.stop()
        time.sleep(0.3)

        # --- PICK ---------------------------------------------------------
        print("[Picker] Ouverture pince et approche...")
        self.arm.set_all(self.POS_OPEN, pause=self.MOVE_PAUSE)

        print("[Picker] Fermeture pince sur l'objet...")
        self.arm.set_all(self.POS_GRAB, pause=self.MOVE_PAUSE)

        print("[Picker] Lever l'objet...")
        self.arm.set_all(self.POS_LIFT, pause=self.MOVE_PAUSE)

        # --- PLACE --------------------------------------------------------
        print("[Picker] Rotation du bras vers la gauche...")
        self.arm.set_all(self.POS_SIDE, pause=self.MOVE_PAUSE)

        print("[Picker] Descente pour deposer...")
        self.arm.set_all(self.POS_PLACE, pause=self.MOVE_PAUSE)

        print("[Picker] Lacher l'objet...")
        self.arm.set_all(self.POS_RELEASE, pause=self.MOVE_PAUSE)

        # --- RETOUR -------------------------------------------------------
        print("[Picker] Retour position initiale...")
        self.arm.set_all(self.POS_HOME, pause=self.MOVE_PAUSE)

        self.motors.stop()
        time.sleep(0.3)

        print("[Picker] Pick and place termine — reprise navigation ArUco")

    # ------------------------------------------------------------------
    # Thread de surveillance
    # ------------------------------------------------------------------
    def start_monitoring(self, hardware_lock: threading.Lock) -> threading.Thread:
        """
        Lance un thread daemon qui surveille le capteur ultrason.

        Quand un obstacle est detecte sous THRESHOLD_CM :
          - prend le verrou materiel (bloque les commandes TCP)
          - execute la sequence pick_and_place
          - libere le verrou

        Retourne le thread cree (deja demarre).
        """
        def _monitor_loop() -> None:
            delay = 1.0 / self.MONITOR_FREQ_HZ
            while True:
                dist = self.sonar.get_distance()
                if self.is_obstacle(dist):
                    with hardware_lock:
                        self.pick_and_place()
                time.sleep(delay)

        thread = threading.Thread(target=_monitor_loop, daemon=True)
        thread.start()
        print("[Picker] Surveillance ultrason pick and place active")
        return thread
