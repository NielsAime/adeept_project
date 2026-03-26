"""
Test interactif des moteurs — à lancer directement sur le Raspberry Pi.

Usage :
    python3 test_motors.py

Commandes clavier :
    z / s  → avance / recule  (les deux moteurs)
    q / d  → tourne gauche / droite (sur place)
    a      → moteur gauche seul +speed
    e      → moteur droit  seul +speed
    +/-    → augmente / diminue la vitesse de test (step 10)
    ESPACE → stop
    x      → quitter
"""

import sys
import os
import tty
import termios

sys.path.insert(0, os.path.dirname(__file__))

from hardware.motor import MotorController

SPEED_DEFAULT = 50
SPEED_STEP    = 10
SPEED_MIN     = 10
SPEED_MAX     = 100


def getch():
    """Lit un caractère clavier sans attendre Entrée."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_status(speed, left, right):
    print(f"\r  speed={speed:3d}  L={left:+4.0f}  R={right:+4.0f}    ", end='', flush=True)


def main():
    motors = MotorController()
    speed  = SPEED_DEFAULT
    left   = 0.0
    right  = 0.0

    print("=== Test moteurs ===")
    print("z/s=avance/recule  q/d=tourne G/D  a=gauche seul  e=droit seul")
    print("+/-=vitesse        ESPACE=stop     x=quitter")
    print_status(speed, left, right)

    try:
        while True:
            ch = getch()

            if ch == 'x':
                break

            elif ch == ' ':
                left, right = 0.0, 0.0

            elif ch == 'z':   # avance
                left, right = speed, speed

            elif ch == 's':   # recule
                left, right = -speed, -speed

            elif ch == 'q':   # tourne gauche (sur place)
                left, right = -speed, speed

            elif ch == 'd':   # tourne droite (sur place)
                left, right = speed, -speed

            elif ch == 'a':   # moteur gauche seul
                left, right = speed, 0.0

            elif ch == 'e':   # moteur droit seul
                left, right = 0.0, speed

            elif ch == '+':
                speed = min(SPEED_MAX, speed + SPEED_STEP)

            elif ch == '-':
                speed = max(SPEED_MIN, speed - SPEED_STEP)

            else:
                continue

            motors.set_left(left)
            motors.set_right(right)
            print_status(speed, left, right)

    finally:
        motors.stop()
        motors.cleanup()
        print("\n[TEST] Moteurs arrêtés.")


if __name__ == '__main__':
    main()
