"""
Tests offline FK + IK — aucun hardware requis, tourne sur PC.

Lancer :
    cd robot_workspace
    python -m pytest test/test_ik.py -v
ou directement :
    python test/test_ik.py
"""

import sys
import os
import math

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from hardware.arm_ik import (
    fk, jacobian, ik,
    L1, L2, H_SHOULDER,
    SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG,
    SERVO_MIN, SERVO_MAX,
    max_reach, is_reachable,
)


# ─── FK : positions connues ────────────────────────────────────────────────────

class TestFK:

    def test_fk_returns_array_shape(self):
        q = np.array([90.0, 90.0, 90.0])
        p = fk(q)
        assert p.shape == (3,), "fk doit retourner un vecteur de taille 3"

    def test_fk_arm_fully_extended_horizontal(self):
        """
        Bras horizontal et tendu : shoulder = ZERO, elbow = ZERO.
        → portée = L1 + L2 devant le robot, hauteur = H_SHOULDER, y = 0.
        """
        q = np.array([90.0, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        p = fk(q)
        expected_x = L1 + L2
        assert abs(p[0] - expected_x) < 1e-6, f"x attendu {expected_x:.4f}, obtenu {p[0]:.4f}"
        assert abs(p[1]) < 1e-6,              f"y doit être 0, obtenu {p[1]:.6f}"
        assert abs(p[2] - H_SHOULDER) < 1e-6, f"z attendu {H_SHOULDER:.4f}, obtenu {p[2]:.4f}"

    def test_fk_pan_center_is_forward(self):
        """Pan à 90° → bras dans le plan xz, y ≈ 0."""
        q = np.array([90.0, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        p = fk(q)
        assert abs(p[1]) < 1e-9

    def test_fk_pan_symmetry(self):
        """Symétrie : pan = 90+d et pan = 90-d → |y| identique, x identique."""
        d = 30.0
        q_left  = np.array([90.0 + d, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        q_right = np.array([90.0 - d, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        p_l = fk(q_left)
        p_r = fk(q_right)
        assert abs(p_l[0] - p_r[0]) < 1e-9,   "x doit être identique des deux côtés"
        assert abs(p_l[1] + p_r[1]) < 1e-9,   "|y| doit être symétrique"

    def test_fk_shoulder_up_raises_z(self):
        """Monter l'épaule → z augmente (servo diminue car sens physique inversé)."""
        q_low  = np.array([90.0, SHOULDER_ZERO_DEG,        ELBOW_ZERO_DEG])
        q_high = np.array([90.0, SHOULDER_ZERO_DEG - 20.0, ELBOW_ZERO_DEG])
        assert fk(q_high)[2] > fk(q_low)[2], "z doit augmenter quand l'épaule monte"

    def test_fk_elbow_bend_reduces_reach(self):
        """Plier le coude (servo < ZERO) → portée x diminue."""
        q_ext  = np.array([90.0, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        q_bent = np.array([90.0, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG - 30.0])
        assert fk(q_ext)[0] > fk(q_bent)[0], "portée x doit diminuer quand le coude est plié"


# ─── Jacobien ──────────────────────────────────────────────────────────────────

class TestJacobian:

    def test_jacobian_shape(self):
        q = np.array([90.0, 90.0, 90.0])
        J = jacobian(q)
        assert J.shape == (3, 3)

    def test_jacobian_not_all_zeros(self):
        q = np.array([90.0, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        J = jacobian(q)
        assert np.any(np.abs(J) > 1e-6), "Jacobien ne doit pas être nul"

    def test_jacobian_finite_differences_consistent(self):
        """Vérifier que le Jacobien numérique est cohérent avec fk."""
        q   = np.array([90.0, SHOULDER_ZERO_DEG, ELBOW_ZERO_DEG])
        J   = jacobian(q, eps=1e-4)
        dq  = np.array([1.0, 0.5, -0.5])          # petite perturbation (°)
        dp_lin  = J @ dq                           # prédiction linéaire
        dp_real = fk(q + dq) - fk(q)              # vraie variation
        np.testing.assert_allclose(dp_lin, dp_real, atol=1e-3,
            err_msg="Prédiction linéaire J·dq doit approcher Δfk")


# ─── IK : convergence ─────────────────────────────────────────────────────────

class TestIK:

    def _assert_ik_converged(self, target, q0=None, tol=1e-3):
        q, err, ok = ik(target, q0=q0, tol=tol)
        assert ok,      f"IK non convergée — err={err:.4f} m pour cible {target}"
        assert err < tol, f"Erreur résiduelle trop grande : {err:.4f} m"
        # Vérifier que les angles sont dans les limites
        assert np.all(q >= SERVO_MIN - 1e-6), f"Angles sous limite : {q}"
        assert np.all(q <= SERVO_MAX + 1e-6), f"Angles au-dessus limite : {q}"
        return q

    def test_ik_fk_roundtrip(self):
        """IK → FK doit retrouver la cible."""
        q_ref   = np.array([85.0, SHOULDER_ZERO_DEG - 10, ELBOW_ZERO_DEG - 20])
        target  = fk(q_ref)
        q_sol, err, ok = ik(target, q0=np.array([90.0, 90.0, 90.0]))
        p_sol = fk(q_sol)
        np.testing.assert_allclose(p_sol, target, atol=1e-3,
            err_msg="FK(IK(target)) doit retrouver target")

    def test_ik_target_straight_ahead(self):
        """Cible directement devant, à mi-portée."""
        r = (L1 + L2) * 0.6
        target = np.array([r, 0.0, H_SHOULDER])
        self._assert_ik_converged(target)

    def test_ik_target_left(self):
        """Cible à gauche, en l'air."""
        target = np.array([0.15, 0.10, H_SHOULDER + 0.03])
        self._assert_ik_converged(target)

    def test_ik_target_low(self):
        """Cible basse, devant."""
        target = np.array([0.20, 0.0, 0.02])
        self._assert_ik_converged(target)

    def test_ik_warm_start(self):
        """Départ depuis une position proche doit converger rapidement."""
        q_ref  = np.array([92.0, SHOULDER_ZERO_DEG - 5, ELBOW_ZERO_DEG - 15])
        target = fk(q_ref)
        q_near = q_ref + np.array([2.0, -3.0, 4.0])
        q_sol, err, ok = ik(target, q0=q_near, max_iter=50)
        assert ok, f"Warm start non convergé — err={err:.4f}"

    def test_ik_unreachable_returns_best_effort(self):
        """
        Cible hors portée → IK retourne ok=False mais ne plante pas.
        """
        target = np.array([L1 + L2 + 0.5, 0.0, H_SHOULDER])   # bien trop loin
        q, err, ok = ik(target, max_iter=200)
        assert not ok, "IK doit signaler échec sur cible hors portée"
        assert np.isfinite(err), "Erreur doit être finie même en cas d'échec"

    def test_ik_output_clipped_within_servo_limits(self):
        """Les angles solution doivent toujours respecter SERVO_MIN / MAX."""
        # Cible délibérément difficile
        target = np.array([0.05, 0.25, 0.15])
        q, err, ok = ik(target, max_iter=300)
        assert np.all(q >= SERVO_MIN - 1e-9)
        assert np.all(q <= SERVO_MAX + 1e-9)


# ─── Utilitaires ──────────────────────────────────────────────────────────────

class TestUtils:

    def test_max_reach(self):
        assert abs(max_reach() - (L1 + L2)) < 1e-9

    def test_is_reachable_center(self):
        target = np.array([0.10, 0.0, H_SHOULDER])
        assert is_reachable(target)

    def test_is_reachable_too_far(self):
        target = np.array([L1 + L2 + 0.1, 0.0, H_SHOULDER])
        assert not is_reachable(target)


# ─── Lancement direct ─────────────────────────────────────────────────────────

if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
