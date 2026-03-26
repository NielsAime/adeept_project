# SPDX-License-Identifier: MIT

import time
from time import sleep
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from gpiozero import DistanceSensor

# ==============================
# Configuration Ultrasonic
# ==============================
Tr = 23
Ec = 24
sensor = DistanceSensor(echo=Ec, trigger=Tr, max_distance=2)  # max 2 m

def checkdist():
    return sensor.distance * 100  # distance en cm


# ==============================
# Configuration Servos PCA9685
# ==============================
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)   # adresse donnée dans ton code
pca.frequency = 50

def set_angle(ID, angle):
    servo_angle = servo.Servo(
        pca.channels[ID],
        min_pulse=500,
        max_pulse=2400,
        actuation_range=180
    )
    servo_angle.angle = angle


# ==============================
# Paramètres du bras
# ==============================
channel_2 = 2         # change ce canal si ton bras est branché ailleurs
REST_ANGLE = 0          # position repos
THRESHOLD = 25          # obstacle seuil en cm

def move_arm():
    """Fait bouger le bras."""
    print("Obstacle détecté : mouvement du bras")
    set_angle(2,180)
    set_angle(3,180)
    set_angle(1,45)
    time.sleep(1)
    set_angle(1,135)
    time.sleep(2)

    # rest angles
    set_angle(0, 90)
    set_angle(1, 90)
    set_angle(2, 0)
    set_angle(3,10)
    set_angle(4,90)
    set_angle(5,90)
    time.sleep(2)



def init_servos():
    print("All servos initialization : 90 deg")
    set_angle(0, 90)
    set_angle(1, 90)
    set_angle(2, 0)
    set_angle(3,10)
    set_angle(4,90)
    set_angle(5,90)
    time.sleep(1)


# ==============================
# Programme principal
# ==============================
if __name__ == "__main__":
    init_servos()

    arm_already_moved = False

    try:
        while True:
            distance = checkdist()
            print(f"Distance: {distance:.2f} cm")

            # Si obstacle à 10 cm ou moins
            if distance <= THRESHOLD:
                if not arm_already_moved:
                    move_arm()
                    arm_already_moved = True
            else:
                # Réinitialise l'autorisation de mouvement
                arm_already_moved = False
                init_servos()

            sleep(0.2)

    except KeyboardInterrupt:
        print("\nCtrl+C détecté. Remise en position neutre.")
        set_angle(channel_2, REST_ANGLE)
        set_angle(3,10)
        pca.deinit()