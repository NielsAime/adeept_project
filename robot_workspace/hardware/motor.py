"""
Contrôle des moteurs à courant continu via un driver L298N.
Tourne sur Raspberry Pi avec RPi.GPIO.

Câblage par défaut (Adeept AWR / similaire) — à adapter selon votre modèle :
  ENA → GPIO 17   (PWM moteurs gauche)
  IN1 → GPIO 27   (sens moteurs gauche : avant)
  IN2 → GPIO 22   (sens moteurs gauche : arrière)

  ENB → GPIO 13   (PWM moteurs droite)
  IN3 → GPIO 24   (sens moteurs droite : avant)
  IN4 → GPIO 25   (sens moteurs droite : arrière)
"""

import RPi.GPIO as GPIO


class MotorController:
    # Pins GPIO (mode BCM) — modifier si nécessaire
    ENA = 17;  IN1 = 27;  IN2 = 22
    ENB = 13;  IN3 = 24;  IN4 = 25

    PWM_FREQ = 1000   # Hz

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (self.ENA, self.IN1, self.IN2,
                    self.ENB, self.IN3, self.IN4):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self._pwm_left  = GPIO.PWM(self.ENA, self.PWM_FREQ)
        self._pwm_right = GPIO.PWM(self.ENB, self.PWM_FREQ)
        self._pwm_left.start(0)
        self._pwm_right.start(0)

    # ------------------------------------------------------------------
    def set_left(self, speed: float) -> None:
        """
        Commande le groupe de moteurs gauche.
        speed : -100 (arrière) à +100 (avant)
        """
        speed = max(-100.0, min(100.0, speed))
        if speed >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        self._pwm_left.ChangeDutyCycle(abs(speed))

    def set_right(self, speed: float) -> None:
        """
        Commande le groupe de moteurs droit.
        speed : -100 (arrière) à +100 (avant)
        """
        speed = max(-100.0, min(100.0, speed))
        if speed >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        self._pwm_right.ChangeDutyCycle(abs(speed))

    def stop(self) -> None:
        """Coupe tous les moteurs."""
        self._pwm_left.ChangeDutyCycle(0)
        self._pwm_right.ChangeDutyCycle(0)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    def cleanup(self) -> None:
        self.stop()
        self._pwm_left.stop()
        self._pwm_right.stop()
        GPIO.cleanup()
