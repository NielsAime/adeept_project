"""
Point d'entrée PC.

Boucle principale :
  1. Capture frame caméra
  2. Détection ArUco → SceneState
  3. Calcul commande moteur → NavigationCommand
  4. Envoi commande au robot via TCP

Commandes clavier :
  ESC → arrêt d'urgence et déconnexion
  r   → reset du navigateur (nouvelle mission)
"""

import sys
import os
import cv2
import time

# Permet d'importer les modules du projet sans installation
sys.path.insert(0, os.path.dirname(__file__))

from vision.aruco_detector       import ArucoScene
from navigation.path_planning    import Navigator, Phase
from communication.tcp_client    import RobotClient

# ---------------------------------------------------------------------------
# Configuration — à adapter
# ---------------------------------------------------------------------------
ROBOT_IP      = '192.168.1.100'   # IP du Raspberry Pi du robot
ROBOT_PORT    = 5000
CAMERA_INDEX  = 1                 # index de la webcam USB
CALIB_PATH    = os.path.join(os.path.dirname(__file__), '..', 'camera_calibration_data.npz')
LOOP_DELAY    = 0.05              # secondes entre deux itérations (~20 Hz)


def main():
    scene     = ArucoScene(calib_path=CALIB_PATH, camera_index=CAMERA_INDEX)
    navigator = Navigator()
    client    = RobotClient(ROBOT_IP, ROBOT_PORT)

    try:
        client.connect()
    except ConnectionRefusedError:
        print(f"[PC] Impossible de se connecter a {ROBOT_IP}:{ROBOT_PORT}")
        print("[PC] Verifiez que main_robot.py tourne sur le Raspberry Pi")
        scene.release()
        return

    print("[PC] Demarrage de la boucle de navigation. ESC pour quitter.")

    try:
        while True:
            frame = scene.get_frame()
            if frame is None:
                print("[PC] Impossible de lire la camera")
                break

            # --- Vision -------------------------------------------------------
            state, annotated = scene.update(frame)

            # --- Navigation ---------------------------------------------------
            cmd = navigator.compute(state)

            # --- Envoi commande -----------------------------------------------
            if cmd.phase == Phase.GRASP:
                client.send_command({"type": "grasp"})
                print("[PC] -> SAISIE declenchee")
                # Attendre la fin de la sequence de saisie avant de continuer
                time.sleep(3.0)
                navigator.reset()

            elif cmd.phase == Phase.DONE:
                client.send_command({"type": "stop"})
                print("[PC] -> Mission terminee")
                break

            else:
                client.send_command({
                    "type":  "move",
                    "left":  round(cmd.left_speed),
                    "right": round(cmd.right_speed),
                })

            # --- Affichage debug ---------------------------------------------
            phase_color = {
                Phase.ROTATE: (0, 200, 255),
                Phase.MOVE:   (0, 255, 0),
                Phase.GRASP:  (0, 0, 255),
                Phase.DONE:   (200, 200, 200),
            }.get(cmd.phase, (255, 255, 255))

            cv2.putText(annotated, f"Phase: {cmd.phase.name}",
                        (10, annotated.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, phase_color, 2)

            cv2.imshow('Robot Navigation', annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:            # ESC -> arret
                client.send_command({"type": "stop"})
                print("[PC] Arret manuel")
                break
            elif key == ord('r'):    # r -> reset mission
                navigator.reset()
                print("[PC] Navigateur reinitialise")

            time.sleep(LOOP_DELAY)

    finally:
        scene.release()
        client.disconnect()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
