"""
Détection ArUco et calcul de l'état de la scène.

3 marqueurs :
  ID 0 → référentiel sol  (définit le repère monde)
  ID 1 → robot            (position + cap)
  ID 2 → cible            (position à atteindre)

Système de coordonnées :
  - Origine      : centre du marqueur de référence
  - Plan XY      : sol (plan horizontal)
  - Cap robot    : direction +X du marqueur robot dans le repère sol
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
# Identifiants des marqueurs ArUco
# ---------------------------------------------------------------------------
MARKER_REF    = 0   # référentiel sol
MARKER_ROBOT  = 1   # robot mobile
MARKER_TARGET = 2   # objet cible

MARKER_SIZE = 0.05  # taille physique du marqueur en mètres (5 cm)


# ---------------------------------------------------------------------------
# Structure de données : état complet de la scène
# ---------------------------------------------------------------------------
@dataclass
class SceneState:
    # Position du robot dans le référentiel sol (mètres)
    robot_x: float
    robot_y: float
    robot_heading: float   # cap en radians

    # Position de la cible dans le référentiel sol (mètres)
    target_x: float
    target_y: float

    # Vecteur robot → cible (calculé)
    distance: float        # distance euclidienne (m)
    angle_error: float     # erreur angulaire entre cap robot et direction cible (rad)


# ---------------------------------------------------------------------------
# Classe principale
# ---------------------------------------------------------------------------
class ArucoScene:
    def __init__(self, calib_path: str = 'camera_calibration_data.npz', camera_index: int = 1):
        # Chargement de la calibration caméra
        data = np.load(calib_path)
        self.camera_matrix = data['matrix']
        self.dist_coeffs   = data['distortion']

        # Détecteur ArUco
        aruco_dict     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params   = cv2.aruco.DetectorParameters()
        self.detector  = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # Capture vidéo
        self.cap = cv2.VideoCapture(camera_index)

    # ------------------------------------------------------------------
    # Utilitaires mathématiques
    # ------------------------------------------------------------------
    def _build_transform(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """Construit une matrice homogène 4×4 depuis un rvec/tvec (repère caméra)."""
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = tvec.flatten()
        return T

    def _relative_transform(self, T_ref: np.ndarray, T_marker: np.ndarray) -> np.ndarray:
        """Exprime T_marker dans le repère de T_ref : T_rel = T_ref⁻¹ · T_marker"""
        return np.linalg.inv(T_ref) @ T_marker

    def _extract_heading(self, R_rel: np.ndarray) -> float:
        """
        Extrait le cap (yaw) du robot depuis la matrice de rotation relative.
        Convention : le robot fait face à la direction +X de son marqueur ArUco.
        """
        forward = R_rel @ np.array([1.0, 0.0, 0.0])
        return np.arctan2(forward[1], forward[0])

    # ------------------------------------------------------------------
    # Acquisition
    # ------------------------------------------------------------------
    def get_frame(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        return frame if ret else None

    # ------------------------------------------------------------------
    # Traitement principal
    # ------------------------------------------------------------------
    def update(self, frame: np.ndarray) -> Tuple[Optional[SceneState], np.ndarray]:
        """
        Détecte les marqueurs sur la frame et calcule l'état de la scène.

        Returns
        -------
        state : SceneState ou None si les 3 marqueurs ne sont pas visibles
        annotated : frame avec les informations de debug dessinées
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        annotated = frame.copy()

        if ids is None:
            cv2.putText(annotated, "Marqueurs non détectés", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return None, annotated

        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        # Table id → index dans le tableau
        ids_flat     = ids.flatten()
        marker_index = {int(mid): i for i, mid in enumerate(ids_flat)}

        required = {MARKER_REF, MARKER_ROBOT, MARKER_TARGET}
        missing  = required - marker_index.keys()
        if missing:
            msg = f"Marqueurs manquants : {missing}"
            cv2.putText(annotated, msg, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)
            return None, annotated

        # Estimation de pose (rvec/tvec) pour tous les marqueurs
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, self.camera_matrix, self.dist_coeffs
        )

        # Matrices de transformation dans le repère caméra
        T_ref    = self._build_transform(rvecs[marker_index[MARKER_REF]],    tvecs[marker_index[MARKER_REF]])
        T_robot  = self._build_transform(rvecs[marker_index[MARKER_ROBOT]],  tvecs[marker_index[MARKER_ROBOT]])
        T_target = self._build_transform(rvecs[marker_index[MARKER_TARGET]], tvecs[marker_index[MARKER_TARGET]])

        # Transformation dans le repère sol (référentiel)
        T_robot_sol  = self._relative_transform(T_ref, T_robot)
        T_target_sol = self._relative_transform(T_ref, T_target)

        robot_x       = T_robot_sol[0, 3]
        robot_y       = T_robot_sol[1, 3]
        robot_heading = self._extract_heading(T_robot_sol[:3, :3])

        target_x = T_target_sol[0, 3]
        target_y = T_target_sol[1, 3]

        # Calcul erreur distance et angle
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = float(np.sqrt(dx**2 + dy**2))

        desired_angle = np.arctan2(dy, dx)
        angle_error   = float(np.arctan2(
            np.sin(desired_angle - robot_heading),
            np.cos(desired_angle - robot_heading)
        ))

        state = SceneState(
            robot_x=float(robot_x),
            robot_y=float(robot_y),
            robot_heading=float(robot_heading),
            target_x=float(target_x),
            target_y=float(target_y),
            distance=distance,
            angle_error=angle_error,
        )

        self._draw_overlay(annotated, state)
        return state, annotated

    def _draw_overlay(self, frame: np.ndarray, state: SceneState) -> None:
        lines = [
            f"Robot : ({state.robot_x:.2f}, {state.robot_y:.2f}) m  cap: {np.degrees(state.robot_heading):.1f} deg",
            f"Cible : ({state.target_x:.2f}, {state.target_y:.2f}) m",
            f"Distance: {state.distance:.3f} m   Err angle: {np.degrees(state.angle_error):.1f} deg",
        ]
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (10, 30 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

    def release(self) -> None:
        self.cap.release()
