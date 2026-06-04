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
SPEED          = 120      # base forward speed (0-255)
Kp             = 24       # proportional steering gain
Kd             = 14       # derivative gain (reduces wobble)
SMOOTH         = 0.35     # motor smoothing (0.1 smooth -> 0.5 snappy)

# Sharp-corner pivot (when only the outer sensor sees tape, spin in place)
PIVOT_FWD      = 130      # outer wheel forward during a pivot
PIVOT_REV      = -85      # inner wheel reverse during a pivot

# Turn detection & braking
BRAKE_AT_CORNER = False   # DISABLED — just use aggressive steering instead
CORNER_THRESHOLD = 3.0    # error magnitude that triggers corner detection
BRAKE_SPEED    = 20       # gentle reverse speed when braking
BRAKE_TICKS    = 1        # minimal braking (just 1 tick = 8ms)

# Line loss & recovery
LOST_DEBOUNCE  = 6        # consecutive "all-off" reads to trigger search (6 * 8ms = 48ms)
SEARCH_MAX_TIME = 2.0     # max time to search for lost line before giving up (true end)
SEARCH_SPIN_SPEED = 80    # speed to spin while searching for line
RECOVERY_ATTEMPTS = 3     # number of times to try recovering before declaring end
DRIFT_WARN     = 3        # consecutive off-line readings before warning

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
        self.recovery_attempts = 0    # times we've lost and regained line
        self.search_active = False    # currently searching for line
        self.search_direction = 1     # +1 = spin right, -1 = spin left

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

        # ── Line loss detection & recovery ─────────────────────────────
        if error is None:
            # All sensors off — line is lost
            self.lost_count += 1
            if self.lost_time is None:
                self.lost_time = time.time()

            loss_duration = time.time() - self.lost_time

            # Trigger search when line lost for LOST_DEBOUNCE
            if self.lost_count >= LOST_DEBOUNCE and not self.search_active:
                self.search_active = True
                self.search_direction = 1  # try right first
                if self.debug:
                    print(f"[{self.elapsed():.1f}s] 🔍 Line lost! Starting search (recovery attempt #{self.recovery_attempts + 1})")

            # Give up if searching for too long
            if self.search_active and loss_duration >= SEARCH_MAX_TIME:
                self.bot.stop()
                if self.debug:
                    print(f"[{self.elapsed():.1f}s] ✓ SEARCH TIMEOUT — this is the END OF LINE")
                self.exit_code = 0  # graceful success
                return False
        else:
            # Line found!
            if self.search_active:
                # Successfully recovered during search
                self.recovery_attempts += 1
                if self.debug:
                    print(f"[{self.elapsed():.1f}s] ✓ Line recovered! (recovery #{self.recovery_attempts})")
                self.search_active = False
                self.lost_count = 0
                self.lost_time = None
            elif self.lost_count > 0 and self.debug:
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
        if self.search_active:
            # Searching: spin in place to find the line, watch for any sensor activation
            # When spinning, if ANY sensor sees tape, stop spinning and resume following
            if error is not None:
                # Line found during search! Stop spinning and resume normal steering
                if self.debug:
                    print(f"[{self.elapsed():.1f}s] ✓ Line found during search! Resuming (direction={'LEFT' if self.search_direction < 0 else 'RIGHT'})")
                self.search_active = False
                self.prev_error = error  # Reset derivative to avoid spike
                # Fall through to normal steering below
            else:
                # Still searching - spin to explore
                self.left_speed = SEARCH_SPIN_SPEED * self.search_direction
                self.right_speed = -SEARCH_SPIN_SPEED * self.search_direction
        elif self.brake_counter > 0:
            # Braking phase
            self.left_speed = BRAKE_SPEED
            self.right_speed = BRAKE_SPEED
            self.brake_counter -= 1
            if self.debug and self.brake_counter == 0:
                print(f"[{self.elapsed():.1f}s] ✓ Brake complete, resuming steering")
        elif not self.search_active and error is not None and abs(error) >= 2.5:
            # SHARP CORNER: only the outer sensor sees tape — pivot in place.
            # PD alone is too gentle for a 90° turn; spin so the inner sensors
            # swing back onto the tape.
            self.prev_error = error
            if error < 0:   # tape hard-left → pivot left
                self.left_speed = PIVOT_REV
                self.right_speed = PIVOT_FWD
            else:           # tape hard-right → pivot right
                self.left_speed = PIVOT_FWD
                self.right_speed = PIVOT_REV
        elif not self.search_active and error is not None:
            # Normal PD steering control (only if not searching and line found)
            P = error
            D = error - self.prev_error
            steering = (Kp * P) + (Kd * D)
            self.prev_error = error

            # Apply steering
            left_adjust = steering
            right_adjust = -steering
            target_left = SPEED - left_adjust
            target_right = SPEED - right_adjust

            # Smooth
            self.left_speed = (SMOOTH * target_left) + ((1 - SMOOTH) * self.left_speed)
            self.right_speed = (SMOOTH * target_right) + ((1 - SMOOTH) * self.right_speed)
        else:
            # Not searching, no line found — coast forward slowly to find it
            self.left_speed = SPEED * 0.5
            self.right_speed = SPEED * 0.5

        # Clamp
        self.left_speed = max(-255, min(255, self.left_speed))
        self.right_speed = max(-255, min(255, self.right_speed))

        # Drive — differential: left side wheels, right side wheels independently.
        # (drift() ignores right_speed entirely, so it could never steer — this is
        #  the same low-level call best_follow.py uses.)
        L = int(self.left_speed)
        R = int(self.right_speed)
        self.bot._apply_motors(L, L, R, R)

        # ── Debug output ───────────────────────────────────────────────
        if self.debug and int(time.time() * 10) % 5 == 0:
            sensors_str = f"L1={L1} L2={L2} R1={R1} R2={R2}"
            error_str = f"error={error:+.1f}" if error is not None else "error=LOST"
            lost_str = f" [LOST {self.lost_count}/{LOST_DEBOUNCE}]" if self.lost_count > 0 else ""
            search_str = f" [SEARCHING→{'R' if self.search_direction > 0 else 'L'}]" if self.search_active else ""
            brake_str = f" [BRAKING {self.brake_counter}]" if self.brake_counter > 0 else ""
            print(f"[{self.elapsed():.1f}s] {sensors_str} | {error_str} | speed_L={self.left_speed:.0f} R={self.right_speed:.0f}{lost_str}{search_str}{brake_str}")

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
