"""
Contrôle du bras robotique par servomoteurs (PWM 50 Hz).
Tourne sur Raspberry Pi avec RPi.GPIO.

Câblage par défaut — à adapter selon votre modèle Adeept :
  PIN_BASE     → GPIO 11   (rotation base)
  PIN_SHOULDER → GPIO 13   (épaule)
  PIN_ELBOW    → GPIO 15   (coude)
  PIN_GRIPPER  → GPIO 16   (pince)

Plage duty cycle : 2.5 % (0°) à 12.5 % (180°)
"""

import time
import RPi.GPIO as GPIO


class ArmController:
    # Pins GPIO (mode BCM)
    PIN_BASE     = 11
    PIN_SHOULDER = 13
    PIN_ELBOW    = 15
    PIN_GRIPPER  = 16

    PWM_FREQ = 50   # Hz (standard servos)

    # Séquences d'angles [base, épaule, coude, pince]
    POS_HOME           = [90, 90,  90,  90]   # position de repos
    POS_GRASP_APPROACH = [90, 60, 120,  90]   # bras tendu vers l'objet
    POS_GRASP_CLOSE    = [90, 60, 120, 160]   # pince fermée sur l'objet
    POS_GRASP_LIFT     = [90, 90,  90, 160]   # lever l'objet

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self._pins = [self.PIN_BASE, self.PIN_SHOULDER,
                      self.PIN_ELBOW, self.PIN_GRIPPER]
        self._pwms = []
        for pin in self._pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, self.PWM_FREQ)
            pwm.start(0)
            self._pwms.append(pwm)

    # ------------------------------------------------------------------
    def _angle_to_duty(self, angle: float) -> float:
        """Convertit un angle (0–180°) en duty cycle (2.5–12.5 %)."""
        return 2.5 + (max(0.0, min(180.0, angle)) / 180.0) * 10.0

    def set_angle(self, servo_idx: int, angle: float, pause: float = 0.3) -> None:
        """Déplace un servo à l'angle donné puis coupe le signal (évite vibrations)."""
        self._pwms[servo_idx].ChangeDutyCycle(self._angle_to_duty(angle))
        time.sleep(pause)
        self._pwms[servo_idx].ChangeDutyCycle(0)

    def set_all(self, angles: list, pause: float = 0.5) -> None:
        """Déplace tous les servos simultanément."""
        for i, angle in enumerate(angles):
            self._pwms[i].ChangeDutyCycle(self._angle_to_duty(angle))
        time.sleep(pause)
        for pwm in self._pwms:
            pwm.ChangeDutyCycle(0)

    # ------------------------------------------------------------------
    # Séquences de haut niveau
    # ------------------------------------------------------------------
    def home(self) -> None:
        """Retourne en position de repos."""
        self.set_all(self.POS_HOME)

    def grasp(self) -> None:
        """
        Séquence complète de saisie :
          1. Approche de l'objet
          2. Fermeture de la pince
          3. Lever l'objet
        """
        print("[Bras] Approche...")
        self.set_all(self.POS_GRASP_APPROACH)
        print("[Bras] Fermeture pince...")
        self.set_all(self.POS_GRASP_CLOSE)
        print("[Bras] Lever...")
        self.set_all(self.POS_GRASP_LIFT)
        print("[Bras] Saisie terminee")

    def release(self) -> None:
        """Ouvre la pince et retourne en position de repos."""
        self.set_angle(3, 90)   # ouverture pince (servo index 3)
        time.sleep(0.3)
        self.home()

    def cleanup(self) -> None:
        self.home()
        for pwm in self._pwms:
            pwm.stop()
