"""
Servo id probe — tries each servo id on the servo register with big swings, one
at a time, so you can see WHICH id (if any) moves the pan/tilt mount.

Only writes the servo register (0x02), so it cannot spin the wheels.
Watch the USB webcam mount (NOT the fixed D405 depth camera).

  python3 camera_move/servo_probe.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot
from setup_and_api.api.constants import Register

SERVO_REG = Register.SERVO          # 0x02


def main():
    print("Watch the USB-cam mount. Ctrl-C to stop.\n")
    with RasBot() as bot:
        for servo_id in (0, 1, 2, 3):
            print(f"=== trying servo id {servo_id} ===")
            for angle in (10, 170, 10, 170, 90):
                print(f"   id {servo_id} -> angle {angle}")
                bot._write_block(SERVO_REG, [servo_id, angle])
                time.sleep(0.7)
            print()
        print("Done. Did the mount move on ANY id? Which one?")


if __name__ == "__main__":
    main()
