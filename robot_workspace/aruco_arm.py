"""
Scan ArUco en W -> saisie -> depot (bras seul, sans deplacement du robot).
Lancer sur le Pi depuis robot_workspace/ :  python3 aruco_arm.py
Prerequis : pip install opencv-contrib-python
"""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import cv2
import numpy as np
import time

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from hardware.servo  import ArmController
from hardware.arm_ik import fk, ik, H_SHOULDER

# ---------- Parametres -------------------------------------------------------

MARKER_SIZE   = 0.05
CAMERA_INDEX  = 0
ARUCO_DICT_ID = cv2.aruco.DICT_5X5_50
TARGET_ID     = 3

GRIPPER_OPEN  = 10
GRIPPER_CLOSE = 160

DROP_POS = [0.12, -0.15, 0.05]

# Waypoints scan en W : (pan, shoulder, elbow, gripper)
# shoulder fixe 120 | elbow : 110=haut  70=bas | pan : 140=gauche  40=droite
W_WAYPOINTS = [
    (140, 120, 110, GRIPPER_OPEN),
    (110, 120,  70, GRIPPER_OPEN),
    ( 90, 120, 110, GRIPPER_OPEN),
    ( 70, 120,  70, GRIPPER_OPEN),
    ( 40, 120, 110, GRIPPER_OPEN),
]

SCAN_STEPS = 25
SCAN_DELAY = 0.03
IK_STEPS   = 40
IK_DELAY   = 0.03

HOME_Q = [90, 90, 90, GRIPPER_OPEN]

# ---------- Geometrie camera -> robot ----------------------------------------

DX, DY, DZ = 0.00, 0.10, 0.15

_R_CAM_BASE = np.array([[ 0,  0,  1],
                         [-1,  0,  0],
                         [ 0, -1,  0]], dtype=float)

W_IMG, H_IMG = 640, 480
CAMERA_MATRIX = np.array([[W_IMG,     0, W_IMG/2],
                           [    0, W_IMG, H_IMG/2],
                           [    0,     0,       1]], dtype=float)
DIST_COEFFS = np.zeros(5)


# ---------- Mouvement bras ---------------------------------------------------

def smooth_move(arm, q_from, q_to, steps, delay):
    q_f = np.array(q_from, dtype=float)
    q_t = np.array(q_to,   dtype=float)
    for i in range(1, steps + 1):
        t = i / steps
        arm.set_all((q_f + t * (q_t - q_f)).tolist(), pause=0)
        time.sleep(delay)


def smooth_ik_move(arm, target_xyz, q_current, gripper):
    """IK calculee une seule fois depuis q_current, puis interpolation d'angles."""
    q_sol, err, ok = ik(np.array(target_xyz), q0=np.array(q_current[:3]))
    if not ok:
        print(f"  [IK] err={err:.4f}m (approche)")
    q_target = list(q_sol) + [gripper]
    smooth_move(arm, q_current, q_target, IK_STEPS, IK_DELAY)
    return q_target


# ---------- ArUco ------------------------------------------------------------

def make_detector():
    if hasattr(cv2.aruco, 'ArucoDetector'):
        d   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        def detect(frame):
            c, i, _ = det.detectMarkers(frame)
            return c, i
    else:
        d = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
        p = cv2.aruco.DetectorParameters_create()
        def detect(frame):
            c, i, _ = cv2.aruco.detectMarkers(frame, d, parameters=p)
            return c, i
    return detect


def cam_to_robot(tvec, pan_deg):
    theta = np.radians(pan_deg - 90.0)
    c, s  = np.cos(theta), np.sin(theta)
    R_z   = np.array([[ c, -s, 0], [ s, c, 0], [0, 0, 1]], dtype=float)
    T     = np.array([DY*c - DX*s, DY*s + DX*c, H_SHOULDER + DZ])
    return R_z @ _R_CAM_BASE @ tvec + T


def check_marker(cap, detect_fn, pan_deg):
    ret, frame = cap.read()
    if not ret:
        return None
    corners, ids = detect_fn(frame)
    if ids is None:
        return None
    matches = np.where(ids.flatten() == TARGET_ID)[0]
    if len(matches) == 0:
        return None
    idx = matches[0]
    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[idx]], MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
    )
    tvec    = tvecs[0][0]
    p_robot = cam_to_robot(tvec, pan_deg)
    print(f"  [cam] ID={TARGET_ID}  dist={tvec[2]:.2f}m  "
          f"robot=({p_robot[0]:.3f}, {p_robot[1]:.3f}, {p_robot[2]:.3f})")
    return p_robot


# ---------- Pick and place ---------------------------------------------------

def pick_and_place(arm, x, y, z, q_start):
    q = list(q_start)

    print(f"[Pick] Approche ({x:.3f}, {y:.3f}, {z:.3f}) m...")
    q = smooth_ik_move(arm, [x, y, z], q, GRIPPER_OPEN)
    time.sleep(0.3)

    print("[Pick] Fermeture pince...")
    arm.set_angle(3, GRIPPER_CLOSE, pause=0.5)
    q[3] = GRIPPER_CLOSE

    print("[Pick] Lever...")
    q = smooth_ik_move(arm, [x, y, z + 0.07], q, GRIPPER_CLOSE)

    print("[Place] Depot...")
    q = smooth_ik_move(arm, DROP_POS, q, GRIPPER_CLOSE)

    print("[Place] Ouverture pince...")
    arm.set_angle(3, GRIPPER_OPEN, pause=0.5)
    q[3] = GRIPPER_OPEN

    print("[Place] Retour home...")
    smooth_move(arm, q, HOME_Q, IK_STEPS, IK_DELAY)
    return HOME_Q


# ---------- Main -------------------------------------------------------------

def main():
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=0x5f)
    pca.frequency = 50
    arm = ArmController(pca)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[Erreur] Camera introuvable (index", CAMERA_INDEX, ")")
        pca.deinit()
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W_IMG)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H_IMG)

    detect_fn = make_detector()

    print("[Init] Home, pince ouverte...")
    arm.set_all(HOME_Q, pause=1.0)
    current_q = list(HOME_Q)

    direction = 1   # 1 = gauche->droite,  -1 = droite->gauche

    try:
        while True:
            waypoints = W_WAYPOINTS if direction == 1 else list(reversed(W_WAYPOINTS))
            print(f"[Scan] Balayage en W ({'G->D' if direction == 1 else 'D->G'})...")
            found  = None
            q_prev = list(waypoints[0])
            smooth_move(arm, current_q, q_prev, SCAN_STEPS, SCAN_DELAY)

            for wp in waypoints:
                smooth_move(arm, q_prev, wp, SCAN_STEPS, SCAN_DELAY)
                q_prev = list(wp)
                p = check_marker(cap, detect_fn, wp[0])
                if p is not None:
                    print(f"[Scan] Trouve a pan={wp[0]}")
                    found = p
                    break

            direction *= -1   # inverser pour le prochain passage

            if found is None:
                print("[Scan] Rien -- passage suivant...")
                current_q = list(q_prev)
                continue

            current_q = pick_and_place(arm, *found, q_prev)

    except KeyboardInterrupt:
        print("\n[Arret] Ctrl+C.")
    finally:
        cap.release()
        smooth_move(arm, current_q, HOME_Q, SCAN_STEPS, SCAN_DELAY)
        pca.deinit()
        print("[Arret] Propre.")


if __name__ == "__main__":
    main()
