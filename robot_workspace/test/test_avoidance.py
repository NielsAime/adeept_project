"""
Test autonome de l'évitement d'obstacle — modèle voiture (servo de direction).

Comportement :
  1. Le robot avance tout droit à --speed.
  2. Dès qu'un obstacle est détecté (< --threshold cm), la manœuvre en arc s'exécute.
  3. Après la manœuvre, il repart tout droit.
  4. Ctrl+C arrête proprement.

Usage :
  cd ~/adeept_project/robot_workspace
  python3 test/test_avoidance.py
  python3 test/test_avoidance.py --steer 25 --turn 1.2 --bypass 1.0

Arguments :
  --speed      Vitesse avance normale    (défaut 50   % PWM)
  --threshold  Seuil détection           (défaut 35   cm)
  --avoid-spd  Vitesse pendant manœuvre  (défaut 50   % PWM)
  --steer      Angle de braquage         (défaut 25   degrés, max 30)
  --turn       Durée de chaque arc       (défaut 1.2  s)
  --bypass     Durée avance droite       (défaut 1.0  s)

Symptômes et corrections :
  Fonce trop près avant de s'arrêter   → augmenter --threshold (ex: 40)
  Arc insuffisant (< 90° au total)     → augmenter --turn ou --steer
  Arc trop prononcé (> 90°)            → diminuer  --turn ou --steer
  Ne dépasse pas l'obstacle            → augmenter --bypass
  Manœuvre trop lente                  → augmenter --avoid-spd
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hardware.motor           import MotorController
from hardware.steering        import SteeringServo
from hardware.ultrasonic      import UltrasonicSensor
from logic.obstacle_avoidance import ObstacleAvoider

SONAR_FREQ_HZ = 10


def parse_args():
    p = argparse.ArgumentParser(description="Test évitement d'obstacle — modèle voiture")
    p.add_argument("--speed",      type=float, default=20.0, help="Vitesse avance %% PWM (défaut 50)")
    p.add_argument("--threshold",  type=float, default=35.0, help="Seuil détection cm (défaut 35)")
    p.add_argument("--avoid-spd",  type=float, default=20.0, help="Vitesse manœuvre %% PWM (défaut 50)")
    p.add_argument("--steer",      type=float, default=30.0, help="Angle braquage degrés (défaut 25, max 30)")
    p.add_argument("--turn",       type=float, default=2,  help="Durée arc en s (défaut 1.2)")
    p.add_argument("--bypass",     type=float, default=2.0,  help="Durée avance droite en s (défaut 1.0)")
    return p.parse_args()


def main():
    args = parse_args()

    motors   = MotorController()
    steering = SteeringServo(motors._pwm)   # partage le PCA9685
    sonar    = UltrasonicSensor()
    avoider  = ObstacleAvoider(motors, steering)

    avoider.THRESHOLD_CM     = args.threshold
    avoider.AVOID_SPEED      = args.avoid_spd
    avoider.STEER_ANGLE      = args.steer
    avoider.TURN_S           = args.turn
    avoider.BYPASS_FORWARD_S = args.bypass

    print("=== Test évitement d'obstacle (voiture) ===")
    print(f"  --speed      {args.speed} % PWM")
    print(f"  --threshold  {args.threshold} cm")
    print(f"  --avoid-spd  {args.avoid_spd} % PWM")
    print(f"  --steer      {args.steer}°")
    print(f"  --turn       {args.turn} s")
    print(f"  --bypass     {args.bypass} s")
    print("Ctrl+C pour arrêter.\n")

    delay = 1.0 / SONAR_FREQ_HZ

    try:
        motors.set_left(args.speed)
        motors.set_right(args.speed)
        steering.center()
        print("[Test] Avance...")

        while True:
            dist = sonar.get_distance()

            if dist > 0 and avoider.is_obstacle(dist):
                avoider.bypass()
                # Cooldown : avance librement le temps de s'éloigner de l'obstacle
                # avant de réactiver la détection (évite un re-déclenchement immédiat)
                print("[Test] Reprise avance (cooldown 2s)...")
                steering.center()
                motors.set_left(args.speed)
                motors.set_right(args.speed)
                time.sleep(2.0)
            elif dist > 0:
                print(f"\r[Test] Distance : {dist:5.1f} cm          ", end="", flush=True)

            time.sleep(delay)

    except KeyboardInterrupt:
        print("\n[Test] Arrêt demandé.")
    finally:
        motors.cleanup()
        steering.center()
        sonar.cleanup()
        print("[Test] Arrêt propre.")


if __name__ == "__main__":
    main()
