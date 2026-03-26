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
"""

import sys
import os
import time
import threading

sys.path.insert(0, os.path.dirname(__file__))

from communication.tcp_server      import CommandServer
from hardware.motor                import MotorController
from hardware.ultrasonic           import UltrasonicSensor
from logic.obstacle_avoidance      import ObstacleAvoider

PORT = 5000

# Verrou partagé entre le thread sonar et le thread TCP :
# garantit qu'une seule source pilote les moteurs à la fois.
# Pendant la manœuvre d'évitement, les commandes TCP sont mises
# en attente (elles s'exécuteront dès que le verrou sera libéré).
_hardware_lock = threading.Lock()


def main():
    motors  = MotorController()
    sonar   = UltrasonicSensor()
    avoider = ObstacleAvoider(motors)
    server  = CommandServer(PORT)

    # ------------------------------------------------------------------
    # Callback commandes TCP
    # ------------------------------------------------------------------
    def on_command(cmd: dict) -> None:
        cmd_type = cmd.get("type")

        with _hardware_lock:
            if cmd_type == "stop":
                motors.stop()
                print("[Robot] STOP")

            elif cmd_type == "move":
                left  = cmd.get("left",  0)
                right = cmd.get("right", 0)
                print(f"[Robot] MOVE left={left} right={right}")
                motors.set_left(left)
                motors.set_right(right)

            elif cmd_type == "grasp":
                motors.stop()
                print("[Robot] GRASP (bras désactivé)")

            else:
                print(f"[Robot] Commande inconnue : {cmd}")

    server.on_command(on_command)

    # ------------------------------------------------------------------
    # Thread surveillance ultrason (10 Hz)
    # Priorité sur les commandes TCP : le verrou bloque on_command
    # pendant toute la durée de la manœuvre d'évitement.
    # ------------------------------------------------------------------
    def sonar_loop() -> None:
        while True:
            dist = sonar.get_distance()
            if avoider.is_obstacle(dist):
                with _hardware_lock:
                    avoider.bypass()
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
        sonar.cleanup()
        print("[Robot] Arrêt propre")


if __name__ == '__main__':
    main()
