"""
Test du capteur ultrason HC-SR04.

Affiche la distance en temps réel avec un indicateur visuel.
Utile pour :
  - Vérifier que le câblage est correct
  - Calibrer THRESHOLD_CM dans obstacle_avoidance.py
  - Observer la stabilité des mesures

Lancer sur le Pi :
  cd ~/code_projet/robot_workspace
  python3 test_ultrasonic.py

Ctrl+C pour quitter.
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))

from hardware.ultrasonic import UltrasonicSensor

# Seuil visuel (même valeur que ObstacleAvoider.THRESHOLD_CM)
THRESHOLD_CM = 25.0
# Fréquence d'affichage
FREQ_HZ = 10


def bar(dist_cm: float, max_cm: float = 150.0, width: int = 40) -> str:
    """Barre de progression proportionnelle à la distance."""
    if dist_cm < 0:
        return "[" + "?" * width + "]"
    ratio = min(dist_cm / max_cm, 1.0)
    filled = int(ratio * width)
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def main():
    sonar = UltrasonicSensor()
    print("=== Test ultrason — Ctrl+C pour quitter ===")
    print(f"Seuil d'évitement : {THRESHOLD_CM} cm\n")

    try:
        while True:
            dist = sonar.get_distance()

            if dist < 0:
                label = "  hors portée"
            elif dist < THRESHOLD_CM:
                label = f"  *** OBSTACLE *** ({dist:5.1f} cm)"
            else:
                label = f"  {dist:5.1f} cm"

            print(f"\r{bar(dist)}{label}          ", end="", flush=True)
            time.sleep(1.0 / FREQ_HZ)

    except KeyboardInterrupt:
        print("\n[Ultrason] Arrêt.")
    finally:
        sonar.cleanup()


if __name__ == "__main__":
    main()
