"""
Contrôle des moteurs via PCA9685 (I2C).
Spécifique au Adeept PiCar-Pro V2.0

Adresse I2C du PCA9685 moteurs : 0x5f
  M1 : channels 15 (IN1) / 14 (IN2) — gauche
  M2 : channels 12 (IN1) / 13 (IN2) — droite

Sens physique : les deux moteurs sont câblés "inversés" sur ce robot
(M1_Direction = M2_Direction = -1 dans le code officiel Adeept).
Convention : speed = +100 → avant, speed = -100 → arrière.
"""

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor as dc_motor


class MotorController:
    # Channels PCA9685
    M1_IN1 = 15  # gauche +
    M1_IN2 = 14  # gauche -
    M2_IN1 = 12  # droite +
    M2_IN2 = 13  # droite -

    FREQ = 50

    def __init__(self):
        i2c = busio.I2C(SCL, SDA)
        self._pwm = PCA9685(i2c, address=0x5f)
        self._pwm.frequency = self.FREQ

        self._motor_left = dc_motor.DCMotor(
            self._pwm.channels[self.M1_IN1],
            self._pwm.channels[self.M1_IN2]
        )
        self._motor_left.decay_mode = dc_motor.SLOW_DECAY

        self._motor_right = dc_motor.DCMotor(
            self._pwm.channels[self.M2_IN1],
            self._pwm.channels[self.M2_IN2]
        )
        self._motor_right.decay_mode = dc_motor.SLOW_DECAY

    def set_left(self, speed: float) -> None:
        """speed : -100 (arrière) à +100 (avant)"""
        speed = max(-100.0, min(100.0, speed))
        # Les moteurs sont montés en sens inverse (M1_Direction=-1 dans Adeept)
        self._motor_left.throttle = -(speed / 100.0)

    def set_right(self, speed: float) -> None:
        """speed : -100 (arrière) à +100 (avant)"""
        speed = max(-100.0, min(100.0, speed))
        # Les moteurs sont montés en sens inverse (M2_Direction=-1 dans Adeept)
        self._motor_right.throttle = -(speed / 100.0)

    def stop(self) -> None:
        """Coupe tous les moteurs."""
        self._motor_left.throttle = 0
        self._motor_right.throttle = 0

    def cleanup(self) -> None:
        self.stop()
        self._pwm.deinit()
