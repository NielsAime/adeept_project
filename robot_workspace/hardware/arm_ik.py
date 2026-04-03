"""
Cinématique inverse — méthode jacobienne numérique.
Bras 3 DOF : pan (lacet) + épaule (tangage) + coude (tangage).

Aucune dépendance hardware : ce module est testable sur PC.

Repère :
    origine = projection au sol du joint épaule
    x = devant le robot
    y = gauche
    z = haut

Usage minimal :
    from hardware.arm_ik import fk, ik

    q0  = np.array([90.0, 90.0, 90.0])        # position courante (servo °)
    pos = fk(q0)                               # → [x, y, z] mètres
    q, err, ok = ik(np.array([0.20, 0.0, 0.05]), q0)
"""

import numpy as np

# ─── Géométrie physique ────────────────────────────────────────────────────────
L1          = 0.107   # m  épaule → coude
L2          = 0.180   # m  coude  → bout pince
H_SHOULDER  = 0.10    # m  hauteur du joint épaule / sol

# ─── Calibration servo → angle géométrique  [À AJUSTER PAR MESURE PHYSIQUE] ──
#
#   PAN_CENTER_DEG    : servo 90° → regard droit devant  (en général correct)
#   SHOULDER_ZERO_DEG : servo ?°  → bras parfaitement horizontal
#                       Mesurer : mettre le bras horizontal, lire l'angle servo.
#   ELBOW_ZERO_DEG    : servo ?°  → avant-bras tendu dans le prolongement du bras
#                       Estimé à 180° (servo en butée haute = tendu).
#
PAN_CENTER_DEG    = 90.0    # servo 90° → regard droit devant (canal 1)
SHOULDER_ZERO_DEG = 180.0   # servo 180° → bras horizontal vers l'avant
#                             servo 90°  → vertical (orthogonal au sol)
#                             servo 0°   → horizontal vers l'arrière
ELBOW_ZERO_DEG    = 180.0   # servo 180° → coude tendu (avant-bras dans le prolongement)
#                             servo 90°  → angle droit, avant-bras pointe vers le bas
#                             servo 10°  → coude replié (minimum physique)

# ─── Limites servo (°) ────────────────────────────────────────────────────────
SERVO_MIN = np.array([0.0,   0.0,  10.0])   # coude : min 10° (sinon collision)
SERVO_MAX = np.array([180.0, 180.0, 180.0])

_DEG = np.pi / 180.0


# ─── Cinématique directe ──────────────────────────────────────────────────────

def fk(q: np.ndarray) -> np.ndarray:
    """
    Cinématique directe.

    Paramètres
    ----------
    q : array [pan_deg, shoulder_deg, elbow_deg]  — angles servo en degrés

    Retourne
    --------
    p : array [x, y, z]  — position bout-pince en mètres
    """
    q_pan   = (q[0] - PAN_CENTER_DEG)    * _DEG   # + = gauche
    theta_s = (SHOULDER_ZERO_DEG - q[1]) * _DEG   # + = monter (servo ↓ = bras monte)
    phi_e   = (q[2] - ELBOW_ZERO_DEG)    * _DEG   # - = plier vers le bas (0 = tendu)

    # angle absolu de l'avant-bras / horizontal
    theta_f = theta_s + phi_e

    # portée dans le plan sagittal
    r = L1 * np.cos(theta_s) + L2 * np.cos(theta_f)
    z = H_SHOULDER + L1 * np.sin(theta_s) + L2 * np.sin(theta_f)

    x = r * np.cos(q_pan)
    y = r * np.sin(q_pan)

    return np.array([x, y, z])


# ─── Jacobien numérique ────────────────────────────────────────────────────────

def jacobian(q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
    """
    Jacobien numérique 3×3.
    J[i, j] = ∂p_i / ∂q_j   (différences finies avant)
    """
    p0 = fk(q)
    J  = np.zeros((3, 3))
    for j in range(3):
        dq    = np.zeros(3)
        dq[j] = eps
        J[:, j] = (fk(q + dq) - p0) / eps
    return J


# ─── Solveur IK — moindres carrés amortis (damped least squares) ──────────────

def ik(
    target:   np.ndarray,
    q0:       np.ndarray = None,
    max_iter: int        = 500,
    tol:      float      = 1e-3,
    lam:      float      = 0.005,
    alpha:    float      = 0.8,
) -> tuple:
    """
    Cinématique inverse itérative (moindres carrés amortis).

    Paramètres
    ----------
    target   : [x, y, z] cible en mètres
    q0       : angles servo initiaux (°).
               Défaut ≈ position mi-tendu devant le robot.
    max_iter : nombre max d'itérations
    tol      : erreur de position acceptée (m)
    lam      : facteur d'amortissement (évite singularités — garder petit,
               ex. 0.001–0.01 ; trop grand → convergence très lente)
    alpha    : pas d'intégration (0 < alpha ≤ 1, réduire si oscillations)

    Retourne
    --------
    q        : angles servo solution (°)
    err      : norme de l'erreur résiduelle (m)
    ok       : True si convergé sous tol
    """
    if q0 is None:
        # Position de départ : bras légèrement en dessous de l'horizontal,
        # coude à mi-chemin entre tendu et plié — bon compromis pour atteindre
        # des cibles basses devant le robot.
        q0 = np.array([90.0, SHOULDER_ZERO_DEG - 20.0, ELBOW_ZERO_DEG - 50.0])

    q = np.clip(q0.copy().astype(float), SERVO_MIN, SERVO_MAX)

    for _ in range(max_iter):
        p   = fk(q)
        e   = target - p
        if np.linalg.norm(e) < tol:
            break

        J   = jacobian(q)
        JJT = J @ J.T + lam ** 2 * np.eye(3)
        dq  = alpha * (J.T @ np.linalg.solve(JJT, e))

        q = np.clip(q + dq, SERVO_MIN, SERVO_MAX)

    err = float(np.linalg.norm(target - fk(q)))
    return q, err, err < tol


# ─── Utilitaires ──────────────────────────────────────────────────────────────

def max_reach() -> float:
    """Portée maximale théorique (m) — bras horizontal et tendu."""
    return L1 + L2


def is_reachable(target: np.ndarray, margin: float = 0.01) -> bool:
    """
    Vérification rapide : la cible est-elle dans la sphère de travail ?
    Ne garantit pas qu'une solution existe (limites servo non vérifiées).
    """
    dx, dy, dz = target[0], target[1], target[2] - H_SHOULDER
    dist = np.sqrt(dx**2 + dy**2 + dz**2)
    return dist <= (max_reach() - margin)
