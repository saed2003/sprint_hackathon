#!/usr/bin/env python3
"""
Simple line follower using 4 color sensors to navigate over black tape.

Robot has 4 sensors on a single bar, 70mm wide:
  L1 (outer-left), L2 (inner-left), R1 (inner-right), R2 (outer-right)

Each sensor: True = sees BLACK tape, False = sees no tape

Exits with:
  0 = success (reached end of line gracefully)
  1 = lost line / crashed
  2 = error (robot connection, etc)

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
SPEED          = 80       # base forward speed (0-255) — LOWER for better turn control
Kp             = 60       # proportional steering gain — MUCH HIGHER for aggressive steering
Kd             = 25       # derivative gain (reduces wobble)
SMOOTH         = 0.5      # motor smoothing (0.1 smooth -> 0.5 snappy)

# Turn detection & braking
BRAKE_AT_CORNER = False   # DISABLED — just use aggressive steering instead
CORNER_THRESHOLD = 3.0    # error magnitude that triggers corner detection
BRAKE_SPEED    = 20       # gentle reverse speed when braking
BRAKE_TICKS    = 1        # minimal braking (just 1 tick = 8ms)

# Line loss detection
LOST_DEBOUNCE  = 8        # consecutive "all-off" reads to declare lost (8 * 8ms = 64ms)
DRIFT_WARN     = 3        # consecutive off-line readings before warning
END_OF_LINE_TIME = 0.5    # time to wait at end before declaring success (graceful stop)

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
        self.lost_count = 0           # consecutive all-off reads
        self.lost_time = None         # when loss started
        self.drift_count = 0          # consecutive off-line readings
        self.start_time = time.time()
        self.brake_counter = 0        # counts down during braking
        self.exit_code = 0            # 0=success, 1=lost/crash, 2=error

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

        # ── Line loss detection ────────────────────────────────────────
        if error is None:
            # All sensors off — line is lost
            self.lost_count += 1
            if self.lost_time is None:
                self.lost_time = time.time()

            # Check if this is a graceful end-of-line (sustained loss)
            loss_duration = time.time() - self.lost_time
            if self.lost_count >= LOST_DEBOUNCE:
                self.bot.stop()
                if self.debug:
                    print(f"[{self.elapsed():.1f}s] ✓ END OF LINE REACHED (all sensors off for {loss_duration:.2f}s)")
                self.exit_code = 0  # graceful success
                return False
        else:
            # Line found — reset loss tracking
            if self.lost_count > 0 and self.debug:
                print(f"[{self.elapsed():.1f}s] ✓ Line regained (was lost for {self.lost_count * LOOP_DELAY:.3f}s)")
            self.lost_count = 0
            self.lost_time = None
            self.drift_count = 0  # reset drift warning

        # ── Drift detection (one-sided sensor reading) ──────────────────
        # If only one outer sensor sees tape while inner sensors are off,
        # the robot is drifting away from the line
        if error is not None:
            if (L1 and not L2 and not R1 and not R2) or (R2 and not R1 and not L2 and not L1):
                # Only outer sensor — drifting away
                self.drift_count += 1
                if self.drift_count >= DRIFT_WARN and self.debug:
                    side = "LEFT" if L1 else "RIGHT"
                    print(f"[{self.elapsed():.1f}s] ⚠ DRIFT WARNING: only {side} outer sensor, drifting away!")
            else:
                self.drift_count = 0

        # ── Corner detection ───────────────────────────────────────────
        if error is not None and abs(error) >= CORNER_THRESHOLD and BRAKE_AT_CORNER:
            if self.brake_counter == 0:
                if self.debug:
                    print(f"[{self.elapsed():.1f}s] ⚡ SHARP TURN DETECTED (error={error:+.1f}) — braking")
                self.brake_counter = BRAKE_TICKS

        # ── Motor control ──────────────────────────────────────────────
        if self.brake_counter > 0:
            # Braking phase
            self.left_speed = BRAKE_SPEED
            self.right_speed = BRAKE_SPEED
            self.brake_counter -= 1
            if self.debug and self.brake_counter == 0:
                print(f"[{self.elapsed():.1f}s] ✓ Brake complete, resuming steering")
        else:
            # PD steering control
            P = error if error is not None else 0
            D = error - self.prev_error if error is not None else 0
            steering = (Kp * P) + (Kd * D)
            self.prev_error = error if error is not None else 0

            # Apply steering
            left_adjust = steering
            right_adjust = -steering
            target_left = SPEED - left_adjust
            target_right = SPEED - right_adjust

            # Smooth
            self.left_speed = (SMOOTH * target_left) + ((1 - SMOOTH) * self.left_speed)
            self.right_speed = (SMOOTH * target_right) + ((1 - SMOOTH) * self.right_speed)

        # Clamp
        self.left_speed = max(-255, min(255, self.left_speed))
        self.right_speed = max(-255, min(255, self.right_speed))

        # Drive
        self.bot.drift(self.left_speed, 90, 0)

        # ── Debug output ───────────────────────────────────────────────
        if self.debug and int(time.time() * 10) % 5 == 0:
            sensors_str = f"L1={L1} L2={L2} R1={R1} R2={R2}"
            error_str = f"error={error:+.1f}" if error is not None else "error=LOST"
            lost_str = f" [LOST {self.lost_count}/{LOST_DEBOUNCE}]" if self.lost_count > 0 else ""
            brake_str = f" [BRAKING {self.brake_counter}]" if self.brake_counter > 0 else ""
            print(f"[{self.elapsed():.1f}s] {sensors_str} | {error_str} | speed_L={self.left_speed:.0f} R={self.right_speed:.0f}{lost_str}{brake_str}")

        time.sleep(LOOP_DELAY)
        return True

    def elapsed(self):
        """Return elapsed time since start."""
        return time.time() - self.start_time

    def run(self):
        """Main loop. Returns exit code."""
        print("=== Line Follower Started ===")
        print("Sensors: L1(outer-L) L2(inner-L) R1(inner-R) R2(outer-R)")
        print("Press Ctrl+C to stop\n")

        try:
            while self.step():
                pass
        except KeyboardInterrupt:
            print("\n\nStopped by user")
            self.exit_code = 1  # interrupted = failure
        except Exception as e:
            print(f"\n\nERROR: {e}")
            self.exit_code = 2  # error
        finally:
            self.bot.stop()
            self.bot.leds_off()

        return self.exit_code


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
    import sys

    parser = argparse.ArgumentParser(description='Line follower using 4 color sensors')
    parser.add_argument('--calibrate', action='store_true',
                        help='Run sensor calibration mode instead of following')
    parser.add_argument('--no-debug', action='store_true',
                        help='Disable debug output')

    args = parser.parse_args()

    if args.calibrate:
        calibrate()
        sys.exit(0)

    try:
        print("Connecting to robot...")
        with RasBot() as bot:
            bot.set_all_leds_color(Color.GREEN)
            bot.beep(0.1)

            follower = LineFollower(bot, debug=not args.no_debug)
            exit_code = follower.run()

            if exit_code == 0:
                print("\n" + "="*50)
                print("✓ SUCCESS: Completed line following (end reached)")
                print("="*50)
                bot.set_all_leds_color(Color.GREEN)
                bot.beep(0.2)
            else:
                print("\n" + "="*50)
                print("✗ FAILED: Lost line or interrupted")
                print("="*50)
                bot.set_all_leds_color(Color.RED)
                bot.beep(0.1)

            sys.exit(exit_code)
    except Exception as e:
        print(f"\n✗ ERROR: {e}", file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
