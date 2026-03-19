"""
Logique d'évitement / suppression d'obstacles.

Comportement :
  - Si le capteur ultrason détecte un obstacle à moins de THRESHOLD cm,
    le robot s'arrête et utilise son bras pour repousser l'obstacle
    latéralement, puis reprend sa mission.
"""

from hardware.motor  import MotorController
from hardware.servo  import ArmController


class ObstacleAvoider:
    THRESHOLD_CM = 20.0   # distance minimale avant obstacle (cm)

    def __init__(self, motors: MotorController, arm: ArmController):
        self.motors = motors
        self.arm    = arm

    def is_obstacle(self, distance_cm: float) -> bool:
        """Retourne True si un obstacle est détecté à portée."""
        return 0.0 < distance_cm < self.THRESHOLD_CM

    def push_obstacle(self) -> None:
        """
        Séquence pour repousser l'obstacle avec le bras :
          1. Arrêt des moteurs
          2. Extension du bras à l'horizontale
          3. Balayage latéral (gauche → droite)
          4. Retour position repos
        """
        print(f"[Obstacle] Obstacle detecte — repousse avec le bras")
        self.motors.stop()

        # Extension du bras horizontale (angle épaule ~45°, coude ~90°)
        self.arm.set_all([90, 45, 90, 90])

        # Balayage : rotation base gauche puis droite
        self.arm.set_angle(0, 130, pause=0.4)   # gauche
        self.arm.set_angle(0,  50, pause=0.4)   # droite
        self.arm.set_angle(0,  90, pause=0.3)   # centre

        # Retour position repos
        self.arm.home()
        print("[Obstacle] Obstacle ecarte — reprise de la mission")
