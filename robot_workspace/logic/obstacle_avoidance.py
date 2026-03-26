"""
Logique de contournement d'obstacle — modèle voiture (tricycle).

Stratégie simplifiée (ArUco reprend le relais) :
  1. Arrêt bref
  2. Braquer à droite + avancer  (dégager la trajectoire)
  3. Avancer tout droit           (s'éloigner de l'obstacle)
  4. Stop + roues droites
  5. Le navigateur ArUco se réoriente vers la cible automatiquement

Paramètres à calibrer (voir test/test_avoidance.py --help) :
  STEER_ANGLE      — braquage utilisé (degrés, max = SteeringServo.MAX_STEER = 30)
  AVOID_SPEED      — vitesse pendant la manœuvre (% PWM)
  TURN_S           — durée de l'arc initial (secondes)
  BYPASS_FORWARD_S — durée d'avance droite après l'arc (secondes)
"""

import time
from hardware.motor    import MotorController
from hardware.steering import SteeringServo


class ObstacleAvoider:
    # --- Seuil de détection -------------------------------------------
    THRESHOLD_CM = 35.0

    # --- Paramètres de la manœuvre ------------------------------------
    AVOID_SPEED      = 20.0   # % PWM pendant la manœuvre
    STEER_ANGLE      = 30.0   # degrés de braquage (0–30)
    TURN_S           = 2.0    # durée de l'arc
    BYPASS_FORWARD_S = 2.0    # durée d'avance droite

    def __init__(self, motors: MotorController, steering: SteeringServo):
        self.motors   = motors
        self.steering = steering

    def is_obstacle(self, distance_cm: float) -> bool:
        return 0.0 < distance_cm < self.THRESHOLD_CM

    def _drive(self, speed: float) -> None:
        self.motors.set_left(speed)
        self.motors.set_right(speed)

    def bypass(self) -> None:
        """
        Manœuvre de dégagement : arc droite puis avance droite.
        Bloquant (~TURN_S + BYPASS_FORWARD_S secondes).
        Après retour, le navigateur ArUco reprend et réoriente le robot.
        """
        print("[Obstacle] Obstacle détecté — dégagement à droite")

        # 1. Arrêt bref
        self._drive(0)
        self.steering.center()
        time.sleep(0.2)

        # 2. Arc droite : braquer puis avancer
        self.steering.steer(-self.STEER_ANGLE)
        time.sleep(0.15)   # servo atteint sa position
        self._drive(self.AVOID_SPEED)
        time.sleep(self.TURN_S)

        # 3. Avance droite pour s'éloigner
        self.steering.center()
        self._drive(self.AVOID_SPEED)
        time.sleep(self.BYPASS_FORWARD_S)

        # 4. Stop — ArUco reprend
        self._drive(0)
        self.steering.center()
        time.sleep(0.2)

        print("[Obstacle] Dégagement terminé — reprise navigation ArUco")
