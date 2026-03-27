"""
Test autonome du pick and place d'obstacle.

Comportement :
  1. Le bras est initialise en position home.
  2. Le robot avance tout droit a FORWARD_SPEED.
  3. Des qu'un obstacle est detecte (< THRESHOLD_CM), le robot s'arrete
     et la sequence pick and place s'execute.
  4. Apres le pick and place, le robot repart tout droit.
  5. Ctrl+C arrete proprement.

Parametres a calibrer (section PARAMETRES ci-dessous) :
  FORWARD_SPEED  — vitesse d'avance droite (% PWM, 0-100)
  THRESHOLD_CM   — distance declenchement saisie (cm)
  MOVE_PAUSE     — temps entre chaque mouvement du bras (secondes)

Procedure de calibration recommandee :
  1. Lancer test_ultrasonic.py pour confirmer la portee du capteur.
  2. Placer une bouteille d'eau a ~20 cm devant le robot.
  3. Lancer ce script et observer si le bras saisit correctement.
  4. Ajuster THRESHOLD_CM selon la position optimale de saisie.
  5. Ajuster les angles POS_* dans ObstaclePicker si le bras
     ne saisit pas bien la bouteille.

Lancer sur le Pi :
  cd ~/code_projet/robot_workspace
  python3 test/test_picking.py

Ctrl+C pour quitter.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from hardware.motor         import MotorController
from hardware.servo          import ArmController
from hardware.ultrasonic     import UltrasonicSensor
from logic.obstacle_picking  import ObstaclePicker

# =============================================================================
# PARAMETRES — modifier ici pour calibrer
# =============================================================================
FORWARD_SPEED  = 40.0    # % PWM avance droite
THRESHOLD_CM   = 15.0    # cm — seuil declenchement pick and place
MOVE_PAUSE     = 0.6     # secondes entre chaque mouvement du bras
SONAR_FREQ_HZ  = 10      # frequence de lecture du capteur
# =============================================================================


def main():
    print("[DEBUG] Demarrage main()")

    print("[DEBUG] Initialisation MotorController...")
    motors = MotorController()
    print("[DEBUG] MotorController OK")

    print("[DEBUG] Initialisation ArmController...")
    arm    = ArmController(motors._pwm)
    print("[DEBUG] ArmController OK")

    print("[DEBUG] Initialisation UltrasonicSensor...")
    sonar  = UltrasonicSensor()
    print("[DEBUG] UltrasonicSensor OK")

    picker = ObstaclePicker(motors, arm, sonar)
    picker.THRESHOLD_CM = THRESHOLD_CM
    picker.MOVE_PAUSE   = MOVE_PAUSE

    print("=== Test pick and place ===")
    print(f"  Avance        : {FORWARD_SPEED} % PWM")
    print(f"  Seuil         : {THRESHOLD_CM} cm")
    print(f"  Pause bras    : {MOVE_PAUSE} s")
    print("Ctrl+C pour arreter.\n")

    # Initialiser le bras en position home
    print("[DEBUG] init_arm()...")
    picker.init_arm()
    time.sleep(1.0)

    delay = 1.0 / SONAR_FREQ_HZ

    try:
        # Depart : avance tout droit
        print("[DEBUG] Envoi commande moteurs...")
        motors.set_left(FORWARD_SPEED)
        motors.set_right(FORWARD_SPEED)
        print("[Test] Avance...")

        while True:
            dist = sonar.get_distance()

            if dist < 0:
                # Capteur hors portee → pas d'obstacle
                pass
            elif picker.is_obstacle(dist):
                # Obstacle detecte : arret et pick and place
                motors.stop()
                picker.pick_and_place()
                # Reprendre l'avance apres la manoeuvre
                print("[Test] Reprise avance...")
                motors.set_left(FORWARD_SPEED)
                motors.set_right(FORWARD_SPEED)
            else:
                print(f"\r[Test] Distance : {dist:5.1f} cm          ",
                      end="", flush=True)

            time.sleep(delay)

    except KeyboardInterrupt:
        print("\n[Test] Arret demande.")
    finally:
        arm.cleanup()
        motors.cleanup()
        sonar.cleanup()
        print("[Test] Arret propre.")


if __name__ == "__main__":
    main()
