"""
Calcul de la commande de navigation à partir de l'état ArUco.

Algorithme : contrôle proportionnel 2 phases
  1. ROTATE  : le robot tourne sur place pour s'orienter vers la cible
  2. MOVE    : le robot avance avec une correction angulaire fine
  3. GRASP   : distance suffisamment faible → déclenche la saisie
  4. DONE    : tâche terminée
"""

import numpy as np
from enum import Enum, auto
from dataclasses import dataclass


class Phase(Enum):
    ROTATE = auto()   # Rotation vers la cible
    MOVE   = auto()   # Avance vers la cible
    GRASP  = auto()   # Commande de saisie
    DONE   = auto()   # Tâche terminée


@dataclass
class NavigationCommand:
    left_speed:  float   # vitesse moteur gauche : -100 à +100
    right_speed: float   # vitesse moteur droit  : -100 à +100
    phase:       Phase


class Navigator:
    # -----------------------------------------------------------------------
    # Seuils
    # -----------------------------------------------------------------------
    DIST_THRESHOLD  = 0.07   # distance (m) en dessous de laquelle on déclenche GRASP
    ANGLE_THRESHOLD = 0.12   # erreur angulaire (rad, ~7°) en dessous de laquelle on avance

    # -----------------------------------------------------------------------
    # Gains proportionnels
    # -----------------------------------------------------------------------
    KP_ANGLE = 85.0    # gain rotation  (rad → %PWM)
    KP_SPEED = 55.0    # gain avance    (m   → %PWM)
    MAX_SPEED = 70.0   # vitesse max PWM

    def __init__(self):
        self.phase = Phase.ROTATE

    def compute(self, state) -> NavigationCommand:
        """
        Calcule la commande moteur depuis l'état de la scène.

        Parameters
        ----------
        state : SceneState ou None (si les marqueurs ne sont pas visibles)

        Returns
        -------
        NavigationCommand avec left_speed, right_speed et la phase courante
        """
        # Perte de vision → arrêt de sécurité
        if state is None:
            return NavigationCommand(0.0, 0.0, self.phase)

        if self.phase == Phase.DONE:
            return NavigationCommand(0.0, 0.0, Phase.DONE)

        # --- Arrivée : déclenchement de la saisie --------------------------
        if state.distance < self.DIST_THRESHOLD:
            self.phase = Phase.GRASP
            return NavigationCommand(0.0, 0.0, Phase.GRASP)

        # --- Phase ROTATE : rotation sur place si erreur angulaire trop grande
        if abs(state.angle_error) > self.ANGLE_THRESHOLD:
            self.phase = Phase.ROTATE
            # ω > 0 : tourne à gauche (right avance, left recule)
            omega = float(np.clip(
                self.KP_ANGLE * state.angle_error,
                -self.MAX_SPEED, self.MAX_SPEED
            ))
            return NavigationCommand(
                left_speed  = -omega,
                right_speed =  omega,
                phase       = Phase.ROTATE
            )

        # --- Phase MOVE : avance avec correction angulaire fine ------------
        self.phase = Phase.MOVE
        v     = float(np.clip(self.KP_SPEED * state.distance, 0.0, self.MAX_SPEED))
        omega = float(np.clip(
            self.KP_ANGLE * state.angle_error * 0.4,
            -self.MAX_SPEED * 0.4, self.MAX_SPEED * 0.4
        ))
        left  = float(np.clip(v - omega, -self.MAX_SPEED, self.MAX_SPEED))
        right = float(np.clip(v + omega, -self.MAX_SPEED, self.MAX_SPEED))
        return NavigationCommand(left_speed=left, right_speed=right, phase=Phase.MOVE)

    def reset(self) -> None:
        """Réinitialise le navigateur pour une nouvelle mission."""
        self.phase = Phase.ROTATE
