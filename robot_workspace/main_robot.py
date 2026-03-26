"""
Point d'entrée du robot (Raspberry Pi).

Rôle :
  - Lance le serveur TCP
  - Surveille le capteur ultrason en thread parallèle
  - Exécute les commandes moteur/bras reçues depuis le PC

Commandes JSON reconnues :
  {"type": "move",  "left": <-100..100>, "right": <-100..100>}
  {"type": "stop"}
  {"type": "grasp"}

Logique de déplacement :
  - Phase ROTATE (left + right ≈ 0) : rotation sur place, roues avant centrées
  - Phase MOVE   (left + right > 0) : vitesse = moyenne, direction = servo avant
    Le différentiel left/right est converti en angle de braquage.
"""

import sys
import os
import time
import threading

sys.path.insert(0, os.path.dirname(__file__))

from communication.tcp_server      import CommandServer
from hardware.motor                import MotorController
from hardware.steering             import SteeringServo
from hardware.ultrasonic           import UltrasonicSensor
from logic.obstacle_avoidance      import ObstacleAvoider

PORT = 5000

# Si abs(left + right) < seuil → rotation sur place (tank), sinon car-style
ROTATION_THRESHOLD = 15.0
# Vitesse max attendue depuis le navigateur (doit correspondre à Navigator.MAX_SPEED)
NAV_MAX_SPEED = 70.0

_hardware_lock = threading.Lock()


def differential_to_car(left: float, right: float, max_steer: float):
    """
    Convertit une commande différentielle (left/right) en (speed, steer_offset).

    En phase MOVE le navigateur envoie :
      left  = v - omega
      right = v + omega
    donc :
      speed        = (left + right) / 2  = v
      steer_offset = -(left - right) / NAV_MAX_SPEED * max_steer
                   = 2*omega / NAV_MAX_SPEED * max_steer
    Signe : omega > 0 → virer à gauche → steer_offset > 0 (gauche).
    """
    speed        = (left + right) / 2.0
    steer_offset = -(left - right) / NAV_MAX_SPEED * max_steer
    return speed, steer_offset


def main():
    motors   = MotorController()
    steering = SteeringServo(motors._pwm)
    sonar    = UltrasonicSensor()
    avoider  = ObstacleAvoider(motors, steering)
    server   = CommandServer(PORT)

    # ------------------------------------------------------------------
    # Callback commandes TCP
    # ------------------------------------------------------------------
    def on_command(cmd: dict) -> None:
        cmd_type = cmd.get("type")

        with _hardware_lock:
            if cmd_type == "stop":
                motors.stop()
                steering.center()
                print("[Robot] STOP")

            elif cmd_type == "move":
                left  = float(cmd.get("left",  0))
                right = float(cmd.get("right", 0))
                speed_sum = left + right

                if abs(speed_sum) < ROTATION_THRESHOLD:
                    # Rotation sur place : commande différentielle directe
                    motors.set_left(left)
                    motors.set_right(right)
                    steering.center()
                    print(f"[Robot] ROTATE left={left:.0f} right={right:.0f}")
                else:
                    # Avance : vitesse moyenne + servo de direction
                    speed, steer = differential_to_car(left, right, steering.MAX_STEER)
                    steering.steer(steer)
                    motors.set_left(speed)
                    motors.set_right(speed)
                    print(f"[Robot] MOVE speed={speed:.0f} steer={steer:.1f}°")

            elif cmd_type == "grasp":
                motors.stop()
                steering.center()
                print("[Robot] GRASP")

            else:
                print(f"[Robot] Commande inconnue : {cmd}")

    server.on_command(on_command)

    # ------------------------------------------------------------------
    # Thread surveillance ultrason (10 Hz)
    # Le verrou bloque les commandes TCP pendant la manœuvre.
    # Le cooldown (hors verrou) laisse le robot s'éloigner avant
    # de réactiver la détection.
    # ------------------------------------------------------------------
    def sonar_loop() -> None:
        while True:
            dist = sonar.get_distance()
            if avoider.is_obstacle(dist):
                with _hardware_lock:
                    avoider.bypass()
                # Cooldown hors verrou : ArUco reprend, on attend avant
                # de pouvoir déclencher un nouvel évitement
                time.sleep(2.0)
            time.sleep(0.1)

    sonar_thread = threading.Thread(target=sonar_loop, daemon=True)
    sonar_thread.start()
    print("[Robot] Surveillance ultrason active")

    # ------------------------------------------------------------------
    # Démarrage serveur (bloquant)
    # ------------------------------------------------------------------
    print("[Robot] Attente de connexion du PC...")
    try:
        server.start()
    except KeyboardInterrupt:
        print("[Robot] Arrêt demandé")
    finally:
        motors.cleanup()
        steering.center()
        sonar.cleanup()
        print("[Robot] Arrêt propre")


if __name__ == '__main__':
    main()
