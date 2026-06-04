#!/usr/bin/env python3
"""
Simple line follower using 4 color sensors to navigate over black tape.

Robot has 4 sensors on a single bar, 70mm wide:
  L1 (outer-left), L2 (inner-left), R1 (inner-right), R2 (outer-right)

Each sensor: True = sees BLACK tape, False = sees no tape

USAGE:
  python3 src/line/follow.py              # run the follower
  python3 src/line/follow.py --calibrate  # test sensors and debug
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ═══════════════════════════════════════════════════════════════════
#  TUNING PARAMETERS
# ═══════════════════════════════════════════════════════════════════
SPEED          = 150      # base forward speed (0-255)
Kp             = 20       # proportional steering gain
Kd             = 12       # derivative gain (reduces wobble)
SMOOTH         = 0.3      # motor smoothing (0.1 smooth -> 0.5 snappy)

# Stopping conditions
LOST_DEBOUNCE  = 5        # number of "all-off" reads before declaring lost
END_LOST_SEC   = 2.0      # stop after line lost for this many seconds

LOOP_DELAY     = 0.008    # 125 Hz loop (8ms)
DEBUG          = True     # print live feedback
# ═══════════════════════════════════════════════════════════════════


class LineFollower:
    """
    PD controller that steers the robot to keep the black tape centered.
    Reads 4 sensors, computes error (which side the tape is), and adjusts
    left/right wheel speeds to center.
    """

    def __init__(self, bot, debug=True):
        self.bot = bot
        self.debug = debug
        self.prev_error = 0
        self.left_speed = SPEED
        self.right_speed = SPEED
        self.lost_count = 0
        self.start_time = time.time()

    def read_sensors(self):
        """Read all 4 sensors. Returns (L1, L2, R1, R2) as bools."""
        return self.bot.read_line_sensors()

    def compute_error(self, L1, L2, R1, R2):
        """
        Convert sensor pattern to signed error value.
        Negative = tape is LEFT, Positive = tape is RIGHT, None = all-off
        """
        # Both inner sensors see tape -> centered (error = 0)
        if L2 and R1:
            return 0.0

        # 3 sensors active -> tape still on center side
        if L1 and L2 and not R1:
            return -1.5  # tape mostly left
        if not L1 and L2 and R1:
            return -0.5  # tape slightly left
        if L2 and R1 and R2:
            return +1.5  # tape mostly right
        if L1 and not R1 and R1 and R2:
            return +0.5  # tape slightly right

        # Single sensor active
        if L2 and not R1:
            return -1.0  # left inner only
        if R1 and not L2:
            return +1.0  # right inner only
        if L1 and not L2:
            return -2.5  # left outer (sharp turn)
        if R2 and not R1:
            return +2.5  # right outer (sharp turn)

        # All off -> lost
        return None

    def step(self):
        """Run one control loop iteration. Returns True to continue, False to stop."""
        L1, L2, R1, R2 = self.read_sensors()
        error = self.compute_error(L1, L2, R1, R2)

        if error is None:
            # Line lost
            self.lost_count += 1
            if self.lost_count >= LOST_DEBOUNCE:
                if time.time() - self.start_time > END_LOST_SEC:
                    if self.debug:
                        print(f"[{self.elapsed():.1f}s] Line lost for {END_LOST_SEC:.1f}s → stopping")
                    self.bot.stop()
                    return False
            if self.debug and self.lost_count == LOST_DEBOUNCE:
                print(f"[{self.elapsed():.1f}s] Lost line (searching...)")
        else:
            # Line found
            self.lost_count = 0
            self.start_time = time.time()

        # PD steering control
        P = error if error is not None else 0
        D = error - self.prev_error if error is not None else 0
        steering = (Kp * P) + (Kd * D)
        self.prev_error = error if error is not None else 0

        # Apply steering to left/right wheel speeds
        # Positive steering = turn right (decrease left, increase right)
        left_adjust = steering
        right_adjust = -steering

        # Smooth the motor speeds
        target_left = SPEED - left_adjust
        target_right = SPEED - right_adjust

        self.left_speed = (SMOOTH * target_left) + ((1 - SMOOTH) * self.left_speed)
        self.right_speed = (SMOOTH * target_right) + ((1 - SMOOTH) * self.right_speed)

        # Clamp to valid range
        self.left_speed = max(-255, min(255, self.left_speed))
        self.right_speed = max(-255, min(255, self.right_speed))

        # Drive the robot
        self.bot.drift(self.left_speed, 90, 0)  # 90 = straight forward

        if self.debug and int(time.time() * 10) % 5 == 0:  # print every ~0.5s
            sensors_str = f"L1={L1} L2={L2} R1={R1} R2={R2}"
            error_str = f"error={error:+.1f}" if error is not None else "error=LOST"
            print(f"[{self.elapsed():.1f}s] {sensors_str} | {error_str} | speed_L={self.left_speed:.0f} R={self.right_speed:.0f}")

        time.sleep(LOOP_DELAY)
        return True

    def elapsed(self):
        """Return elapsed time since start."""
        return time.time() - self.start_time

    def run(self):
        """Main loop."""
        print("=== Line Follower Started ===")
        print("Sensors: L1(outer-L) L2(inner-L) R1(inner-R) R2(outer-R)")
        print("Press Ctrl+C to stop\n")

        try:
            while self.step():
                pass
        except KeyboardInterrupt:
            print("\n\nStopped by user")
        finally:
            self.bot.stop()
            self.bot.leds_off()


def calibrate():
    """Test mode: read sensors and print their status in real-time."""
    print("=== Sensor Calibration Mode ===")
    print("This will read the 4 sensors every 0.5s. Place tape under sensors and observe.")
    print("Press Ctrl+C to exit.\n")

    with RasBot() as bot:
        bot.set_all_leds_color(Color.YELLOW)
        bot.stop()

        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()
                print(f"L1={int(L1)} L2={int(L2)} R1={int(R1)} R2={int(R2)} | "
                      f"Binary: {int(L1)}{int(L2)}{int(R1)}{int(R2)}")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nCalibration complete.")
        finally:
            bot.stop()
            bot.leds_off()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Line follower using 4 color sensors')
    parser.add_argument('--calibrate', action='store_true',
                        help='Run sensor calibration mode instead of following')
    parser.add_argument('--no-debug', action='store_true',
                        help='Disable debug output')

    args = parser.parse_args()

    if args.calibrate:
        calibrate()
        return

    print("Connecting to robot...")
    with RasBot() as bot:
        bot.set_all_leds_color(Color.GREEN)
        bot.beep(0.1)

        follower = LineFollower(bot, debug=not args.no_debug)
        follower.run()


if __name__ == '__main__':
    main()
