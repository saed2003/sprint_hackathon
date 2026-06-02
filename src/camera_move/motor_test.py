"""
Power-rail check — do the MOTORS move? Servos and motors share the main battery
rail; the LED/logic runs off the Pi. If the LED works but nothing mechanical
moves, the battery rail is likely off.

SAFETY: lift the robot so all four wheels are OFF the ground before running.

  python3 camera_move/motor_test.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color


def main():
    print("LIFT THE ROBOT so the wheels are off the ground.")
    for n in (5, 4, 3, 2, 1):
        print(f"  spinning in {n}...")
        time.sleep(1.0)

    with RasBot() as bot:
        bot.set_all_leds_color(Color.YELLOW)
        print("rotate left 0.6s")
        bot.rotate_left(90)
        time.sleep(0.6)
        bot.stop()
        time.sleep(0.4)
        print("rotate right 0.6s")
        bot.rotate_right(90)
        time.sleep(0.6)
        bot.stop()
        print("\nDone. Did the wheels spin?")


if __name__ == "__main__":
    main()
