"""
Test autonome de l'évitement d'obstacle.

Comportement :
  1. Le robot avance tout droit à FORWARD_SPEED.
  2. Dès qu'un obstacle est détecté (< THRESHOLD_CM), la manœuvre
     de contournement s'exécute (ObstacleAvoider.bypass()).
  3. Après la manœuvre, il repart tout droit.
  4. Ctrl+C arrête proprement.

Paramètres à calibrer (section PARAMETRES ci-dessous) :
  FORWARD_SPEED    — vitesse d'avance droite (% PWM, 0-100)
  THRESHOLD_CM     — distance déclenchement évitement (cm)
  AVOID_SPEED      — vitesse pendant la manœuvre (% PWM)
  TURN_90_S        — durée d'un virage ~90° à AVOID_SPEED (secondes)
  BYPASS_FORWARD_S — durée d'avance latérale pour passer l'obstacle (secondes)

Procédure de calibration recommandée :
  1. Lancer test_ultrasonic.py pour confirmer la portée du capteur.
  2. Régler FORWARD_SPEED selon la vitesse souhaitée.
  3. Poser un obstacle à ~30 cm, lancer ce script.
  4. Si le robot tourne trop peu/trop : ajuster TURN_90_S.
  5. Si le robot ne passe pas complètement : ajuster BYPASS_FORWARD_S.
  6. Répéter jusqu'à obtenir un contournement propre.

Lancer sur le Pi :
  cd ~/code_projet/robot_workspace
  python3 test_avoidance.py

Ctrl+C pour quitter.
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))

from hardware.motor      import MotorController
from hardware.ultrasonic import UltrasonicSensor
from logic.obstacle_avoidance import ObstacleAvoider

# =============================================================================
# PARAMETRES — modifier ici pour calibrer
# =============================================================================
FORWARD_SPEED    = 50.0   # % PWM avance droite
THRESHOLD_CM     = 25.0   # cm — seuil déclenchement évitement
AVOID_SPEED      = 50.0   # % PWM pendant la manœuvre
TURN_90_S        = 0.8    # secondes pour ~90° à AVOID_SPEED
BYPASS_FORWARD_S = 1.0    # secondes d'avance latérale à AVOID_SPEED
SONAR_FREQ_HZ    = 10     # fréquence de lecture du capteur
# =============================================================================


def main():
    motors  = MotorController()
    sonar   = UltrasonicSensor()

    # Injecter les paramètres calibrables dans l'avoider
    avoider = ObstacleAvoider(motors)
    avoider.THRESHOLD_CM     = THRESHOLD_CM
    avoider.AVOID_SPEED      = AVOID_SPEED
    avoider.TURN_90_S        = TURN_90_S
    avoider.BYPASS_FORWARD_S = BYPASS_FORWARD_S

    print("=== Test évitement d'obstacle ===")
    print(f"  Avance        : {FORWARD_SPEED} % PWM")
    print(f"  Seuil         : {THRESHOLD_CM} cm")
    print(f"  Vitesse évi.  : {AVOID_SPEED} % PWM")
    print(f"  Virage 90°    : {TURN_90_S} s")
    print(f"  Avance lat.   : {BYPASS_FORWARD_S} s")
    print("Ctrl+C pour arrêter.\n")

    delay = 1.0 / SONAR_FREQ_HZ

    try:
        # Départ : avance tout droit
        motors.set_left(FORWARD_SPEED)
        motors.set_right(FORWARD_SPEED)
        print("[Test] Avance...")

        while True:
            dist = sonar.get_distance()

            if dist < 0:
                # Capteur hors portée → pas d'obstacle
                pass
            elif avoider.is_obstacle(dist):
                # Obstacle détecté : la manœuvre est bloquante
                motors.stop()
                avoider.bypass()
                # Reprendre l'avance après la manœuvre
                print("[Test] Reprise avance...")
                motors.set_left(FORWARD_SPEED)
                motors.set_right(FORWARD_SPEED)
            else:
                print(f"\r[Test] Distance : {dist:5.1f} cm          ", end="", flush=True)

            time.sleep(delay)

    except KeyboardInterrupt:
        print("\n[Test] Arrêt demandé.")
    finally:
        motors.cleanup()
        sonar.cleanup()
        print("[Test] Arrêt propre.")


if __name__ == "__main__":
    main()
