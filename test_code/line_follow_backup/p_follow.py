#!/usr/bin/env python3
"""
p_follow.py — minimal proportional line follower (copy of the old archive/p_follow.py,
import path fixed for test_code/). The simplest thing that works: one error number ->
differential wheel speeds. No junction logic. Good baseline / fallback.

    python3 p_follow.py
Ctrl+C to stop.
"""
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

from setup_and_api.api import RasBot

BASE_SPEED = 100   # straight-line cruise speed
Kp         = 40    # proportional gain — raise if turns are sluggish
LOOP_DELAY = 0.01  # 100 Hz loop


def clamp(val, min_val=-255, max_val=255):
    return max(min_val, min(val, max_val))


def main():
    last_error = 0

    with RasBot() as bot:
        print("P-controller line follower started. Ctrl+C to stop.")
        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()
                # True = sensor sees BLACK tape
                # L1=outer-left  L2=inner-left  R1=inner-right  R2=outer-right

                if L2 and R1:
                    error = 0
                elif L2 and not R1:
                    error = -1
                elif L1 and L2:
                    error = -1.5
                elif L1 and not L2:
                    error = -2
                elif R1 and not L2:
                    error = 1
                elif R1 and R2:
                    error = 1.5
                elif R2 and not R1:
                    error = 2
                else:
                    error = -3 if last_error < 0 else (3 if last_error > 0 else 0)

                if 0 < abs(error) < 3:
                    last_error = error

                correction  = int(Kp * error)
                left_speed  = clamp(BASE_SPEED + correction)
                right_speed = clamp(BASE_SPEED - correction)

                bot._apply_motors(left_speed, left_speed, right_speed, right_speed)
                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            print("Motors off.")


if __name__ == "__main__":
    main()
