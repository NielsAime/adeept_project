"""
Contrôle du bras robotique via PCA9685 (I2C, adresse 0x5f).
Adeept PiCar-Pro V2.0 — même chip que les moteurs et le servo de direction.

Canaux PCA9685 du bras :
  Canal 1 → base     (rotation gauche/droite)
  Canal 2 → épaule   (haut/bas)
  Canal 3 → coude    (haut/bas)
  Canal 4 → pince    (ouverture/fermeture)

L'instance PCA9685 est passée depuis MotorController pour éviter
une double initialisation du chip.
"""

import time
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as adafruit_servo
from hardware.arm_ik import fk, ik, is_reachable


class ArmController:
    # Canaux PCA9685 [base, épaule, coude, pince]
    CHANNELS = [1, 2, 3, 4]

    # Plage d'impulsions standard pour servos Adeept
    MIN_PULSE = 500
    MAX_PULSE = 2400

    # Positions prédéfinies [base, épaule, coude, pince]
    POS_HOME           = [90, 90,  90,  90]
    POS_GRASP_APPROACH = [90, 60, 120,  90]
    POS_GRASP_CLOSE    = [90, 60, 120, 160]
    POS_GRASP_LIFT     = [90, 90,  90, 160]

    def __init__(self, pwm: PCA9685):
        """
        pwm : instance PCA9685 déjà initialisée (partagée avec MotorController).
        """
        self._servos = [
            adafruit_servo.Servo(
                pwm.channels[ch],
                min_pulse=self.MIN_PULSE,
                max_pulse=self.MAX_PULSE,
                actuation_range=180,
            )
            for ch in self.CHANNELS
        ]

    def set_angle(self, servo_idx: int, angle: float, pause: float = 0.3) -> None:
        """Déplace un servo (0=base, 1=épaule, 2=coude, 3=pince) à l'angle donné."""
        self._servos[servo_idx].angle = max(0.0, min(180.0, angle))
        time.sleep(pause)

    def set_all(self, angles: list, pause: float = 0.5) -> None:
        """Déplace tous les servos simultanément puis attend."""
        for i, angle in enumerate(angles):
            self._servos[i].angle = max(0.0, min(180.0, angle))
        time.sleep(pause)

    def home(self) -> None:
        """Retourne en position de repos."""
        self.set_all(self.POS_HOME)

    def grasp(self) -> None:
        """Séquence complète de saisie : approche → fermeture → lever."""
        print("[Bras] Approche...")
        self.set_all(self.POS_GRASP_APPROACH)
        print("[Bras] Fermeture pince...")
        self.set_all(self.POS_GRASP_CLOSE)
        print("[Bras] Lever...")
        self.set_all(self.POS_GRASP_LIFT)
        print("[Bras] Saisie terminee")

    def release(self) -> None:
        """Ouvre la pince et retourne en position de repos."""
        self.set_angle(3, 90)
        time.sleep(0.3)
        self.home()

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        q0: list = None,
        tol: float = 1e-3,
        pause: float = 0.5,
    ) -> bool:
        """
        Déplace le bout de la pince vers la position (x, y, z) en mètres
        via cinématique inverse jacobienne.

        Paramètres
        ----------
        x, y, z : coordonnées cible (repère robot — x devant, y gauche, z haut)
        q0      : angles servo initiaux [pan°, shoulder°, elbow°].
                  Défaut : angles actuels si connus, sinon [90, 90, 90].
        tol     : précision acceptable (m)
        pause   : temps d'attente après déplacement (s)

        Retourne
        --------
        True si l'IK a convergé, False si la cible était hors portée ou
        si la précision n'a pas pu être atteinte (le bras s'est quand même
        déplacé vers la meilleure position trouvée).
        """
        target = np.array([x, y, z], dtype=float)

        q_init = np.array(q0, dtype=float) if q0 is not None else np.array([90.0, 90.0, 90.0])
        q_sol, err, ok = ik(target, q0=q_init, tol=tol)

        if not ok:
            print(f"[IK] Avertissement : convergence insuffisante (err={err:.4f} m) "
                  f"pour cible ({x:.3f}, {y:.3f}, {z:.3f})")

        # Déplacer les servos pan (0), shoulder (1), elbow (2)
        for idx in range(3):
            self._servos[idx].angle = max(0.0, min(180.0, float(q_sol[idx])))
        time.sleep(pause)

        return ok

    def cleanup(self) -> None:
        self.home()
