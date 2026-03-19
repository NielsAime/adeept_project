"""
Capteur ultrason HC-SR04.
Tourne sur Raspberry Pi avec RPi.GPIO.

Câblage :
  TRIG → GPIO 23
  ECHO → GPIO 24

ATTENTION : le HC-SR04 sort 5 V sur ECHO.
  Utiliser un diviseur de tension (ex. 1 kΩ + 2 kΩ) pour ramener à 3.3 V
  avant de connecter au GPIO du Raspberry Pi.

Formule : distance (cm) = durée_écho (s) × 17 150
  (vitesse du son ≈ 343 m/s = 34 300 cm/s → aller + retour : / 2)
"""

import time
import RPi.GPIO as GPIO


class UltrasonicSensor:
    TRIG = 23
    ECHO = 24

    TIMEOUT_S = 0.04   # 40 ms → ~680 cm max

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.output(self.TRIG, GPIO.LOW)
        time.sleep(0.05)   # stabilisation

    def get_distance(self) -> float:
        """
        Mesure la distance à l'obstacle.

        Returns
        -------
        distance en centimètres, ou -1.0 en cas de timeout.
        """
        # Impulsion déclenchante : 10 µs sur TRIG
        GPIO.output(self.TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, GPIO.LOW)

        # Attente du front montant de l'écho
        t0 = time.time()
        while GPIO.input(self.ECHO) == GPIO.LOW:
            if time.time() - t0 > self.TIMEOUT_S:
                return -1.0
        pulse_start = time.time()

        # Attente du front descendant de l'écho
        while GPIO.input(self.ECHO) == GPIO.HIGH:
            if time.time() - pulse_start > self.TIMEOUT_S:
                return -1.0
        pulse_end = time.time()

        duration_s   = pulse_end - pulse_start
        distance_cm  = duration_s * 17150.0
        return round(distance_cm, 1)
