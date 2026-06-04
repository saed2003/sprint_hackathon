#!/usr/bin/env python3
"""
simple_follow.py — discrete state line follower (copy of the old archive/simple_follow.py,
import path fixed for test_code/). Pivots hard on sharp corners (inner wheel reverses). No
junction logic. A second fallback if the proportional steer feels too soft on 90 corners.

    python3 simple_follow.py
Ctrl+C to stop.
"""
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

from setup_and_api.api import RasBot

BASE_SPEED     =  80
GENTLE_TURN    =  100
GENTLE_REVERSE =  20
SHARP_TURN     =  120
SHARP_REVERSE  = -60
LOOP_DELAY     = 0.02


def main():
    last_seen = "straight"

    with RasBot() as bot:
        print("Smooth line follower started. Ctrl+C to stop.")
        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()

                if L2 and R1:
                    bot._apply_motors(BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED)
                    last_seen = "straight"
                elif L1:
                    bot._apply_motors(SHARP_REVERSE, SHARP_REVERSE, SHARP_TURN, SHARP_TURN)
                    last_seen = "left"
                elif L2:
                    bot._apply_motors(GENTLE_REVERSE, GENTLE_REVERSE, GENTLE_TURN, GENTLE_TURN)
                    last_seen = "left"
                elif R2:
                    bot._apply_motors(SHARP_TURN, SHARP_TURN, SHARP_REVERSE, SHARP_REVERSE)
                    last_seen = "right"
                elif R1:
                    bot._apply_motors(GENTLE_TURN, GENTLE_TURN, GENTLE_REVERSE, GENTLE_REVERSE)
                    last_seen = "right"
                else:
                    if last_seen == "left":
                        bot._apply_motors(-60, -60, 80, 80)
                    elif last_seen == "right":
                        bot._apply_motors(80, 80, -60, -60)
                    else:
                        bot.stop()

                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            print("Motors off. Done.")


if __name__ == "__main__":
    main()
