"""
Servo de direction des roues avant.
Adeept PiCar-Pro V2.0 — canal 0 du PCA9685 (adresse 0x5f).

Convention d'angle (offset depuis le centre) :
  +MAX_STEER  → tourner à gauche
  0           → tout droit
  -MAX_STEER  → tourner à droite

Le servo physique est centré à NEUTRAL_DEG (90°).
  angle_servo = NEUTRAL_DEG + offset

Le PCA9685 est partagé avec les moteurs : on lui passe l'instance
existante pour éviter une double initialisation du chip.
"""

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class SteeringServo:
    CHANNEL     = 0      # canal PCA9685
    NEUTRAL_DEG = 90.0   # angle servo quand les roues sont droites
    MAX_STEER   = 30.0   # amplitude max (degrés offset de chaque côté)

    def __init__(self, pwm: PCA9685):
        """
        pwm : instance PCA9685 déjà initialisée (partagée avec MotorController).
        """
        self._servo = servo.Servo(
            pwm.channels[self.CHANNEL],
            min_pulse=500,
            max_pulse=2400,
            actuation_range=180,
        )
        self.center()

    def steer(self, offset: float) -> None:
        """
        Applique un angle de braquage.
        offset : valeur entre -MAX_STEER (droite) et +MAX_STEER (gauche).
        """
        offset = max(-self.MAX_STEER, min(self.MAX_STEER, offset))
        self._servo.angle = self.NEUTRAL_DEG + offset

    def center(self) -> None:
        """Remet les roues droites."""
        self._servo.angle = self.NEUTRAL_DEG
