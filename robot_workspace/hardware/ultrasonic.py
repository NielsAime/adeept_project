"""
Capteur ultrason HC-SR04 via gpiozero (recommandé par Adeept).

Câblage :
  TRIG → GPIO 23
  ECHO → GPIO 24

gpiozero gère le diviseur de tension 5V→3.3V en interne via son backend.
Dépendance : sudo apt install python3-gpiozero  (ou pip install gpiozero)
"""

from gpiozero import DistanceSensor


class UltrasonicSensor:
    TRIG = 23
    ECHO = 24
    MAX_DISTANCE_M = 2.0   # portée max du capteur (2 m)

    def __init__(self):
        self._sensor = DistanceSensor(
            echo=self.ECHO,
            trigger=self.TRIG,
            max_distance=self.MAX_DISTANCE_M
        )

    def get_distance(self) -> float:
        """
        Retourne la distance mesurée en centimètres.
        Retourne -1.0 si rien n'est détecté dans MAX_DISTANCE_M.
        """
        d = self._sensor.distance   # valeur entre 0.0 et 1.0 (fraction de max_distance)
        if d is None:
            return -1.0
        return round(d * 100.0, 1)  # conversion en cm

    def cleanup(self) -> None:
        self._sensor.close()
