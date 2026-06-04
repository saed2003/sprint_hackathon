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
SPEED          = 60       # base forward speed (0-255)
Kp             = 24       # proportional steering gain
Kd             = 14       # derivative gain (reduces wobble)
SMOOTH         = 0.35     # motor smoothing (0.1 smooth -> 0.5 snappy)

# Corner turning — a corner shows up as 3-4 sensors lit (straights show 1-2).
# When detected we PIVOT in place and commit (latch) to the turn for a bit so
# the robot fully rounds the 90° bend instead of clipping it.
CORNER_COUNT   = 3        # >= this many sensors lit = a corner (bend in the line)
PIVOT_FWD      = 130      # outer wheel forward during a pivot
PIVOT_REV      = -85      # inner wheel reverse during a pivot
PIVOT_LATCH    = 45       # ticks committed to a corner turn (~0.36 s at 125 Hz)
REACQUIRE_ERR  = 1.0      # |error| this small (with <=2 sensors) ends the turn

# Lost-line recovery — REVERSE first (undo the overshoot), THEN spin to re-find.
LOST_DEBOUNCE  = 4        # consecutive "all-off" reads before declaring lost
REVERSE_TICKS  = 12       # back up this many ticks first (~0.10 s) to undo overshoot
REVERSE_SPEED  = -95      # both wheels reverse while backing up
SEARCH_SPIN_SPEED = 90    # speed to spin while searching for the line
SPIN_TOWARD_TIME = 0.45   # spin TOWARD the line's last heading for this long first
SPIN_TOTAL_TIME  = 1.0    # total spin budget (kept under a U-turn); then = end of line

# End-of-line (dead-end) detection. At a real end the robot drives off, the
# reverse backs it onto the tape, it drives off again — losing the line over and
# over in the same spot. That rapid repetition is the dead-end fingerprint.
DEADEND_WINDOW = 1.6      # losses closer together than this count as "the same spot"
DEADEND_LIMIT  = 3        # this many rapid losses in a row = end of line → halt + exit 0

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
        self.prev_error = 0.0
        self.left_speed = SPEED
        self.right_speed = SPEED
        self.start_time = time.time()
        self.exit_code = 0            # 0=success, 1=lost/crash, 2=error
        self.last_dir = 1             # which way the line last headed (+1 right, -1 left)
        self._tick = 0

        # corner-turn latch
        self.latch_ticks = 0          # >0 = committed to a corner turn
        self.latch_dir = 0            # +1 right, -1 left

        # lost-line recovery state machine
        self.lost_count = 0           # consecutive all-off reads
        self.lost_phase = None        # None | 'reverse' | 'spin'
        self.reverse_left = 0         # ticks of reverse remaining
        self.search_dir = 1           # spin direction while searching
        self.search_swept = False     # have we flipped to sweep the other way yet
        self.search_start = 0.0       # when the spin began
        self.recover_count = 0        # rapid recoveries in a row (dead-end fingerprint)
        self.last_recover_time = -999.0  # when we last entered recovery

    def read_sensors(self):
        """Read all 4 sensors. Returns (L1, L2, R1, R2) as bools."""
        return self.bot.read_line_sensors()

    def compute_error(self, L1, L2, R1, R2):
        """Signed error for the FOLLOW state (1-2 sensors lit).
        Negative = tape LEFT of centre, positive = tape RIGHT. None = all-off.
        Corners (3-4 sensors) are handled separately, not here."""
        if L2 and R1:                 return 0.0    # 0110 centred
        if L2 and not R1:             return -1.0   # 0100 inner-left
        if R1 and not L2:             return +1.0   # 0010 inner-right
        if L1 and not L2:             return -3.0   # 1000 outer-left (far)
        if R2 and not R1:             return +3.0   # 0001 outer-right (far)
        return None                                  # 0000 lost

    # ── low-level helpers ────────────────────────────────────────────
    def _drive(self):
        """Push current left/right speeds to the wheels (true differential)."""
        L = max(-255, min(255, int(self.left_speed)))
        R = max(-255, min(255, int(self.right_speed)))
        self.bot._apply_motors(L, L, R, R)

    def _pivot(self, direction):
        """Spin in place: direction +1 = turn right, -1 = turn left."""
        if direction > 0:             # right: left wheel fwd, right wheel back
            self.left_speed, self.right_speed = PIVOT_FWD, PIVOT_REV
        else:                         # left
            self.left_speed, self.right_speed = PIVOT_REV, PIVOT_FWD

    def _corner_dir(self, L1, L2, R1, R2):
        """At a 3-4 sensor corner, decide which way the line bends."""
        if L1 and not R2:   return -1   # left-biased (1110 / 1100)
        if R2 and not L1:   return +1   # right-biased (0111 / 0011)
        return self.last_dir            # 1111 etc → continue last heading

    def _reset_lost(self):
        self.lost_count = 0
        self.lost_phase = None

    def _dbg(self, bits, state, count):
        if self.debug and self._tick % 6 == 0:
            print(f"[{self.elapsed():5.1f}s] {bits} n={count} | {state:<22} "
                  f"L={self.left_speed:4.0f} R={self.right_speed:4.0f}", flush=True)

    # ── main step ─────────────────────────────────────────────────────
    def step(self):
        """One control iteration. Returns True to continue, False to stop."""
        self._tick += 1
        L1, L2, R1, R2 = self.read_sensors()
        count = int(L1) + int(L2) + int(R1) + int(R2)
        bits = f"{int(L1)}{int(L2)}{int(R1)}{int(R2)}"

        # ===== committed corner turn: finish the pivot before anything else =====
        if self.latch_ticks > 0:
            err = self.compute_error(L1, L2, R1, R2)
            reacquired = (count <= 2 and err is not None and abs(err) <= REACQUIRE_ERR)
            if reacquired:
                self.latch_ticks = 0
                self.prev_error = err
                self._reset_lost()
                # fall through to FOLLOW this same tick
            else:
                self.latch_ticks -= 1
                self._pivot(self.latch_dir)
                self._drive()
                self._dbg(bits, f"TURN({'R' if self.latch_dir>0 else 'L'}) latch={self.latch_ticks}", count)
                return True

        # ===== corner: 3-4 sensors lit = the line bends — pivot and commit =====
        if count >= CORNER_COUNT:
            d = self._corner_dir(L1, L2, R1, R2)
            self.latch_dir = d
            self.latch_ticks = PIVOT_LATCH
            self.last_dir = d
            self._reset_lost()
            self._pivot(d)
            self._drive()
            self._dbg(bits, f"CORNER->{'R' if d>0 else 'L'}", count)
            return True

        # ===== lost: 0 sensors = reverse first, then spin to re-find =====
        if count == 0:
            return self._handle_lost(bits)

        # ===== follow: 1-2 sensors = PD steering =====
        self._reset_lost()
        err = self.compute_error(L1, L2, R1, R2)
        if err != 0:
            self.last_dir = 1 if err > 0 else -1

        steering = (Kp * err) + (Kd * (err - self.prev_error))
        self.prev_error = err
        # error > 0 (tape on right) → steer RIGHT → left wheel faster, right slower
        target_left  = SPEED + steering
        target_right = SPEED - steering
        self.left_speed  = SMOOTH * target_left  + (1 - SMOOTH) * self.left_speed
        self.right_speed = SMOOTH * target_right + (1 - SMOOTH) * self.right_speed
        self._drive()
        self._dbg(bits, f"FOLLOW err={err:+.1f}", count)

        time.sleep(LOOP_DELAY)
        return True

    def _handle_lost(self, bits):
        """Lost the tape. Reverse to undo the overshoot, then a short spin to re-find.
        If the tape comes back, step() resumes following next tick (heading kept).
        Two ways this declares END OF LINE and halts with exit 0:
          1. We keep losing the tape in the same spot (dead-end fingerprint), or
          2. The short spin sweeps both ways and finds nothing."""
        # debounce brief flickers before committing to recovery
        if self.lost_phase is None:
            self.lost_count += 1
            if self.lost_count < LOST_DEBOUNCE:
                self._drive()  # keep coasting on last speeds briefly
                time.sleep(LOOP_DELAY)
                return True

            # Committing to recovery — first, is this a dead-end? If we've had to
            # recover several times in quick succession, the line keeps ending here.
            now = time.time()
            if now - self.last_recover_time < DEADEND_WINDOW:
                self.recover_count += 1
            else:
                self.recover_count = 1
            self.last_recover_time = now
            if self.recover_count >= DEADEND_LIMIT:
                self.bot.stop()
                if self.debug:
                    print(f"[{self.elapsed():5.1f}s] ✓ END OF LINE — tape keeps ending here "
                          f"({self.recover_count}x in a row). Halting.", flush=True)
                self.exit_code = 0
                return False

            # reverse phase first
            self.lost_phase = 'reverse'
            self.reverse_left = REVERSE_TICKS
            if self.debug:
                print(f"[{self.elapsed():5.1f}s] 🔄 Lost line — reversing (recover #{self.recover_count})", flush=True)

        if self.lost_phase == 'reverse':
            if self.reverse_left > 0:
                self.reverse_left -= 1
                self.left_speed = REVERSE_SPEED
                self.right_speed = REVERSE_SPEED
                self._drive()
                self._dbg(bits, f"REVERSE {self.reverse_left}", 0)
                time.sleep(LOOP_DELAY)
                return True
            # done backing up → start spinning toward where the line went
            self.lost_phase = 'spin'
            self.search_swept = False
            self.search_dir = self.last_dir
            self.search_start = time.time()
            if self.debug:
                side = 'RIGHT' if self.search_dir > 0 else 'LEFT'
                print(f"[{self.elapsed():5.1f}s] 🔍 Spinning {side} (toward last heading)", flush=True)

        # spin phase: short sweep one way, then the other (kept under a U-turn).
        # If nothing turns up, it's the end of line.
        swept = time.time() - self.search_start
        if not self.search_swept and swept >= SPIN_TOWARD_TIME:
            self.search_swept = True
            self.search_dir = -self.search_dir
            if self.debug:
                side = 'RIGHT' if self.search_dir > 0 else 'LEFT'
                print(f"[{self.elapsed():5.1f}s] 🔄 Not found — sweeping {side}", flush=True)
        elif self.search_swept and swept >= SPIN_TOTAL_TIME:
            self.bot.stop()
            if self.debug:
                print(f"[{self.elapsed():5.1f}s] ✓ END OF LINE — no tape either way. Halting.", flush=True)
            self.exit_code = 0
            return False

        self.left_speed = SEARCH_SPIN_SPEED * self.search_dir
        self.right_speed = -SEARCH_SPIN_SPEED * self.search_dir
        self._drive()
        self._dbg(bits, f"SPIN {'R' if self.search_dir>0 else 'L'}", 0)
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
