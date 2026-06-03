import time
import sys
import os

# go up one level: tape_following/ -> src/ so setup_and_api is importable
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from setup_and_api.api import RasBot

# ── P-CONTROLLER TUNING ──────────────────────────────────────────────
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

                # error < 0 → tape is LEFT  → steer left  (right wheel faster)
                # error > 0 → tape is RIGHT → steer right (left wheel faster)
                if L2 and R1:
                    error = 0        # perfectly centered
                elif L2 and not R1:
                    error = -1       # tape slightly left
                elif L1 and L2:
                    error = -1.5     # tape further left
                elif L1 and not L2:
                    error = -2       # tape far left (outer sensor only)
                elif R1 and not L2:
                    error = 1        # tape slightly right
                elif R1 and R2:
                    error = 1.5      # tape further right
                elif R2 and not R1:
                    error = 2        # tape far right (outer sensor only)
                else:
                    # lost line — sweep toward last known side
                    error = -3 if last_error < 0 else (3 if last_error > 0 else 0)

                # remember for recovery (ignore sweep values ±3)
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
