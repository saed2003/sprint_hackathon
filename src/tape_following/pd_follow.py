"""
pd_follow.py — PD (Proportional-Derivative) line follower
==========================================================
Derivative term dampens oscillation by braking corrections
that are changing too fast (rate of change of error).
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot

# ═══════════════════════════════════════════════════════
#  TUNING — only change these
# ═══════════════════════════════════════════════════════
BASE_SPEED = 90    # cruise speed
Kp         = 35    # proportional gain
Kd         = 15    # derivative gain — dampens wiggle/overshoot
LOOP_DELAY = 0.01  # 100 Hz — keep fast so derivative math is accurate
# ═══════════════════════════════════════════════════════


def clamp(val, min_val=-255, max_val=255):
    return max(min_val, min(val, max_val))


def _calc_error(L1, L2, R1, R2, last_error):
    if L2 and R1:
        return 0.0
    elif L2 and not R1 and not R2:
        return -1.0
    elif L1 and L2:
        return -1.5
    elif L1 and not L2:
        return -2.0
    elif R1 and not L1 and not L2:
        return 1.0
    elif R1 and R2:
        return 1.5
    elif R2 and not R1:
        return 2.0
    else:
        # lost line — sweep toward last known side
        return -3.0 if last_error < 0 else (3.0 if last_error > 0 else 0.0)


def main():
    last_error = 0.0

    with RasBot() as bot:
        print("PD line follower started. Ctrl+C to stop.")
        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()
                error = _calc_error(L1, L2, R1, R2, last_error)

                derivative = error - last_error
                correction = int((Kp * error) + (Kd * derivative))

                # only update last_error for real readings, not sweep ±3
                if abs(error) < 3:
                    last_error = error

                if abs(error) >= 3:
                    # full spin sweep
                    left_speed  = -80 if error < 0 else  80
                    right_speed =  80 if error < 0 else -80
                else:
                    left_speed  = clamp(BASE_SPEED + correction)
                    right_speed = clamp(BASE_SPEED - correction)

                bot._apply_motors(left_speed, left_speed, right_speed, right_speed)
                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            print("Motors off.")


def run(bot, stop_event=None, **kwargs):
    """Entry point called by drive.py F-key."""
    last_error = 0.0

    try:
        while stop_event is None or not stop_event.is_set():
            L1, L2, R1, R2 = bot.read_line_sensors()
            error = _calc_error(L1, L2, R1, R2, last_error)

            derivative = error - last_error
            correction = int((Kp * error) + (Kd * derivative))

            if abs(error) < 3:
                last_error = error

            if abs(error) >= 3:
                left_speed  = -80 if error < 0 else  80
                right_speed =  80 if error < 0 else -80
            else:
                left_speed  = clamp(BASE_SPEED + correction)
                right_speed = clamp(BASE_SPEED - correction)

            bot._apply_motors(left_speed, left_speed, right_speed, right_speed)
            time.sleep(LOOP_DELAY)
    finally:
        bot.stop()


if __name__ == "__main__":
    main()
