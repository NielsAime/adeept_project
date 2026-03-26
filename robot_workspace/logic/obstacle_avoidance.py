"""
Logique de contournement d'obstacle.

Stratégie : contournement en L (3 étapes, vitesse fixe)
  1. Arrêt
  2. Rotation droite ~90°  (dégager la trajectoire)
  3. Avance latérale       (passer à côté de l'obstacle)
  4. Rotation gauche ~90°  (reprendre le cap initial)
  5. Reprise du contrôle ArUco (le navigateur se réoriente seul)

Les durées TURN_90_S et BYPASS_FORWARD_S sont à calibrer selon
la vitesse réelle du robot à AVOID_SPEED % PWM :
  - Poser le robot, lancer test_motors.py à AVOID_SPEED
  - Mesurer le temps pour 90° de rotation → TURN_90_S
  - Mesurer le temps pour dépasser un obstacle de ~30 cm → BYPASS_FORWARD_S
"""

import time
from hardware.motor import MotorController


class ObstacleAvoider:
    # --- Seuil de détection -------------------------------------------
    THRESHOLD_CM = 25.0   # distance (cm) en dessous de laquelle on évite

    # --- Paramètres de la manœuvre ------------------------------------
    # À calibrer selon le robot (voir docstring ci-dessus)
    AVOID_SPEED      = 50.0   # % PWM pendant la manœuvre
    TURN_90_S        = 0.8    # secondes pour tourner ~90° à AVOID_SPEED
    BYPASS_FORWARD_S = 1.0    # secondes pour dépasser l'obstacle latéralement

    def __init__(self, motors: MotorController):
        self.motors = motors

    def is_obstacle(self, distance_cm: float) -> bool:
        """Retourne True si un obstacle est détecté à portée."""
        return 0.0 < distance_cm < self.THRESHOLD_CM

    def bypass(self) -> None:
        """
        Exécute la manœuvre complète de contournement.
        Bloquant (~2 × TURN_90_S + BYPASS_FORWARD_S secondes).
        Après cette méthode, le robot reprend son cap initial décalé
        latéralement ; le navigateur ArUco prend le relais pour
        se réorienter vers la cible.
        """
        print("[Obstacle] Obstacle détecté — contournement à droite")

        # 1. Arrêt
        self.motors.stop()
        time.sleep(0.3)

        # 2. Rotation droite ~90° (gauche avance, droite recule)
        self.motors.set_left( self.AVOID_SPEED)
        self.motors.set_right(-self.AVOID_SPEED)
        time.sleep(self.TURN_90_S)
        self.motors.stop()
        time.sleep(0.2)

        # 3. Avance latérale pour passer à côté de l'obstacle
        self.motors.set_left(self.AVOID_SPEED)
        self.motors.set_right(self.AVOID_SPEED)
        time.sleep(self.BYPASS_FORWARD_S)
        self.motors.stop()
        time.sleep(0.2)

        # 4. Rotation gauche ~90° pour reprendre le cap initial
        self.motors.set_left(-self.AVOID_SPEED)
        self.motors.set_right( self.AVOID_SPEED)
        time.sleep(self.TURN_90_S)
        self.motors.stop()
        time.sleep(0.3)

        print("[Obstacle] Manœuvre terminée — reprise navigation ArUco")
