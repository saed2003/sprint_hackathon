"""
advanced_follow.py — 4-channel IR line follower (Raspbot V2 / Raspberry Pi 5).

Refactored from archive/p_follow.py (the original P-controller is preserved
there untouched). This version is a PD controller that solves four real
tracking problems:

  1. 90° TURN LOCKOUT      — a state lock stops the chassis "swinging past"
                             a hard turn and latching onto the opposite sensor.
  2. STRAIGHT-LINE TRIM     — MOTOR_TRIM cancels physical drift; an error
       + DEADBAND (anti-zigzag) deadband + higher BASE_SPEED let it glide instead of hunting.
  3. JUNCTIONS / CROSSES    — random branch at T-junctions; an all-four-black
                             cross is treated as a junction too (pick a side and
                             drive through). The robot NEVER terminates on a
                             marker — only Ctrl+C stops it.
  4. NEVER-STOP RECOVERY    — when the tape is lost, spin-search toward the
                             last-seen side and keep going until it re-acquires.
                             No terminal stop, no dead-end shutdown.

Every run is logged tick-by-tick to  tape_following/advanced_last_run.log.

Sensor mapping (read_line_sensors → True == black tape):
    L1, L2, R1, R2 = bot.read_line_sensors()
    L1 = left  OUTER   L2 = left  INNER
    R1 = right INNER   R2 = right OUTER
    Geometry:   [L1][L2] | [R1][R2]

Run:  python3 src/tape_following/advanced_follow.py
"""

import os
import sys
import time
import random

# tape_following/ -> src/  so `setup_and_api` is importable
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot


# ============================================================================
#   TUNABLE CONSTANTS — everything you adjust lives here
# ============================================================================

# ── drive / PD controller ────────────────────────────────────────────────────
BASE_SPEED = 120     # cruise speed. Raised vs. the old P-version so linear
                     # momentum carries the robot over small friction/drift
                     # differences WITHOUT needing constant electrical nudges.
Kp         = 40      # proportional gain — reaction to current error
Kd         = 25      # derivative gain — damps overshoot / smooths corrections
LOOP_DELAY = 0.01    # ~100 Hz control loop

# ── Problem 2: anti-zig-zag ──────────────────────────────────────────────────
DEADBAND   = 1.0     # |error| <= DEADBAND → apply NO correction (glide straight).
                     # This is what kills the violent zig-zag on straights.
MOTOR_TRIM = 1.00    # left-motor scale to cancel physical asymmetry.
                     #   < 1.0  → left wheels run weaker (robot drifts left? lower this)
                     #   > 1.0  → left wheels run stronger (robot drifts right? raise this)
                     # Tune on a straight line until it tracks dead center.

# ── Problem 1: hard-turn lockout ─────────────────────────────────────────────
TURN_SPEED = 130     # in-place pivot speed used for hard 90° turns

# ── Problem 3: junctions + lost-line recovery ────────────────────────────────
JUNCTION_CLEAR_TIME = 0.15   # s to force a turn so we fully clear an intersection
DEAD_END_CONFIRM    = 8      # consecutive all-white loops before we switch from
                             # "coast forward" to "spin and search" (debounce — a
                             # small tape gap should not trigger a search spin)

# PWM hard limits
PWM_MIN, PWM_MAX = -255, 255

# ── logging ──────────────────────────────────────────────────────────────────
LOG_TO_FILE = True    # write a full per-tick log to advanced_last_run.log
DEBUG_EVERY = 15      # also print every Nth tick to the console (file gets all)

_HERE    = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = os.path.join(_HERE, "advanced_last_run.log")


def clamp(val, lo=PWM_MIN, hi=PWM_MAX):
    return max(lo, min(val, hi))


class RunLog:
    """Writes every control tick to advanced_last_run.log for post-run review.

    Line format matches best_follow.py so the same eye can read both:
        [tick] [bits] err:±X L:nnn R:nnn  STATE
    """
    def __init__(self, enabled=LOG_TO_FILE):
        self.f = None
        self.tick = 0
        if enabled:
            try:
                self.f = open(LOG_FILE, "w")
                self.f.write(f"# advanced_follow run {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                self.f.write(f"# BASE={BASE_SPEED} Kp={Kp} Kd={Kd} DEADBAND={DEADBAND} "
                             f"TRIM={MOTOR_TRIM} TURN={TURN_SPEED}\n")
                self.f.write("# tick bits err L R state\n")
            except Exception as e:
                print(f"  log file open failed: {e}")

    def emit(self, bits, err, left, right, state):
        self.tick += 1
        line = (f"[{self.tick:06d}] [{bits}] err:{err:+.1f} "
                f"L:{int(left):4d} R:{int(right):4d}  {state}")
        if self.f:
            self.f.write(line + "\n")
        if self.tick % DEBUG_EVERY == 0:
            print(line, flush=True)

    def close(self):
        if self.f:
            self.f.flush()
            self.f.close()
            print(f"  log saved -> {LOG_FILE}")


# ============================================================================
#   LOW-LEVEL MOTOR HELPERS
# ============================================================================

def drive(bot, left, right):
    """Differential drive: both left wheels = `left`, both right = `right`.

    MOTOR_TRIM is applied to the left side here so the physical-drift
    compensation is in ONE place and affects every motion uniformly.
    """
    left = clamp(int(left * MOTOR_TRIM))
    right = clamp(int(right))
    bot._apply_motors(left, left, right, right)


def pivot_left(bot, speed=TURN_SPEED):
    """In-place spin LEFT (CCW): left wheels back, right wheels forward."""
    bot._apply_motors(-speed, -speed, speed, speed)


def pivot_right(bot, speed=TURN_SPEED):
    """In-place spin RIGHT (CW): left wheels forward, right wheels back."""
    bot._apply_motors(speed, speed, -speed, -speed)


# ============================================================================
#   ERROR MODEL
# ============================================================================

def sensor_error(L1, L2, R1, R2):
    """Weighted lateral error.  negative = tape LEFT, positive = tape RIGHT.

    Returns None when the line is completely lost (all white) so the caller
    can run dead-end logic instead of steering on stale data.
    """
    # NOTE: order matters — most-specific patterns first. The graduated
    # ±1.5 "both sensors on one side" cases must be tested before the
    # ±1.0 "inner only" cases, otherwise the inner-only test swallows them.
    if L2 and R1:            return 0.0    # both inner = perfectly centered
    if L1 and not L2:        return -2.0   # tape FAR left (outer only) → hard turn
    if L1 and L2:            return -1.5   # tape further left (both left sensors)
    if L2 and not R1:        return -1.0   # tape slightly left (inner only)
    if R2 and not R1:        return 2.0    # tape FAR right (outer only) → hard turn
    if R1 and R2:            return 1.5    # tape further right (both right sensors)
    if R1 and not L2:        return 1.0    # tape slightly right (inner only)
    return None                            # nothing seen → line lost


# ============================================================================
#   MAIN CONTROL LOOP
# ============================================================================

def main():
    # ── persistent state across loop iterations ──────────────────────────────
    last_error    = 0.0      # for the PD derivative term + recovery direction
    turning_state = "none"   # Problem 1 lockout: "none" | "left" | "right"
    lost_streak   = 0        # consecutive all-white loops (recovery debounce)
    log = RunLog(LOG_TO_FILE)

    with RasBot() as bot:
        print("=" * 60)
        print("ADVANCED LINE FOLLOWER — PD + lockout + junctions (never-stop)")
        print(f"BASE={BASE_SPEED} Kp={Kp} Kd={Kd} DEADBAND={DEADBAND} "
              f"TRIM={MOTOR_TRIM}")
        print(f"logging every tick -> {LOG_FILE}")
        print("Ctrl+C to stop.")
        print("=" * 60)

        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()
                bits = f"{int(L1)}{int(L2)}{int(R1)}{int(R2)}"

                # ── PROBLEM 1: TURN LOCKOUT (highest steering priority) ───────
                # While locked into a hard turn we IGNORE the opposite-side
                # sensors (which the fast-swinging chassis sweeps across the
                # tape) and keep pivoting until BOTH inner sensors re-center.
                if turning_state != "none":
                    if L2 and R1:
                        # Inner sensors re-acquired the line → unlock, resume PD.
                        turning_state = "none"
                        last_error = 0.0
                        # fall through to normal tracking this same loop
                    else:
                        if turning_state == "left":
                            pivot_left(bot);  l, r = -TURN_SPEED, TURN_SPEED
                        else:
                            pivot_right(bot); l, r = TURN_SPEED, -TURN_SPEED
                        log.emit(bits, last_error, l, r, f"LOCK({turning_state})")
                        time.sleep(LOOP_DELAY)
                        continue   # stay locked — ignore everything else

                # ── JUNCTIONS: T-junction OR full cross → pick a side and clear
                # L1+L2+R1 (left branch + ahead), L2+R1+R2 (right branch + ahead),
                # or all-four (a cross / former "stop square"). Pick a direction
                # at random and FORCE it long enough to clear the intersection.
                # NOTE: a cross is NO LONGER a terminal stop — we drive through it.
                t_left  = (L1 and L2 and R1 and not R2)
                t_right = (not L1 and L2 and R1 and R2)
                cross   = (L1 and L2 and R1 and R2)
                if t_left or t_right or cross:
                    choice = random.choice(["left", "right"])
                    kind = "CROSS" if cross else "T-JUNC"
                    print(f"  [{kind.lower()}] detected → forcing {choice} for "
                          f"{JUNCTION_CLEAR_TIME}s")
                    t_end = time.time() + JUNCTION_CLEAR_TIME
                    while time.time() < t_end:
                        if choice == "left":
                            pivot_left(bot);  l, r = -TURN_SPEED, TURN_SPEED
                        else:
                            pivot_right(bot); l, r = TURN_SPEED, -TURN_SPEED
                        log.emit(bits, 0.0, l, r, f"{kind}->{choice}")
                        time.sleep(LOOP_DELAY)
                    last_error = 0.0
                    lost_streak = 0
                    continue

                # ── compute steering error ───────────────────────────────────
                error = sensor_error(L1, L2, R1, R2)

                # ── LOST LINE → NEVER-STOP RECOVERY (all white) ──────────────
                if error is None:
                    lost_streak += 1
                    # Brief gap: a short break in the tape should NOT trigger a
                    # search. Creep forward in the last-known direction while we
                    # wait to see if the line comes back.
                    if lost_streak < DEAD_END_CONFIRM:
                        nudge = clamp(int(Kp * last_error))
                        l = BASE_SPEED * 0.6 + nudge
                        r = BASE_SPEED * 0.6 - nudge
                        drive(bot, l, r)
                        log.emit(bits, last_error, l, r, "COAST")
                        time.sleep(LOOP_DELAY)
                        continue

                    # Confirmed lost: spin in place toward the side the tape was
                    # last seen and KEEP SEARCHING — the robot never stops, it
                    # just hunts until it re-acquires the line.
                    side = "left" if last_error < 0 else "right"
                    if side == "left":
                        pivot_left(bot);  l, r = -TURN_SPEED, TURN_SPEED
                    else:
                        pivot_right(bot); l, r = TURN_SPEED, -TURN_SPEED
                    log.emit(bits, last_error, l, r, f"SEARCH({side})")
                    time.sleep(LOOP_DELAY)
                    continue

                # line is visible again → reset the lost counter
                lost_streak = 0

                # ── PROBLEM 1 (trigger): enter lockout on a hard outer-only turn
                # Only the far-left outer sensor (90° left) or far-right outer
                # sensor (90° right) latches the lockout state.
                if L1 and not L2 and not R1 and not R2:
                    turning_state = "left"
                    print("  [lockout] hard LEFT turn — opposite sensors ignored")
                    pivot_left(bot)
                    log.emit(bits, error, -TURN_SPEED, TURN_SPEED, "LOCK-ENTER(left)")
                    time.sleep(LOOP_DELAY)
                    continue
                if R2 and not R1 and not L2 and not L1:
                    turning_state = "right"
                    print("  [lockout] hard RIGHT turn — opposite sensors ignored")
                    pivot_right(bot)
                    log.emit(bits, error, TURN_SPEED, -TURN_SPEED, "LOCK-ENTER(right)")
                    time.sleep(LOOP_DELAY)
                    continue

                # ── PROBLEM 2: DEADBAND + PD CONTROL (anti-zig-zag) ───────────
                if abs(error) <= DEADBAND:
                    # Small error → glide. No electrical correction, just let
                    # momentum (and MOTOR_TRIM) hold the line. Kills the wobble.
                    correction = 0
                else:
                    derivative = error - last_error          # rate of change
                    correction = int(Kp * error + Kd * derivative)

                last_error = error

                left_speed  = clamp(BASE_SPEED + correction)
                right_speed = clamp(BASE_SPEED - correction)
                drive(bot, left_speed, right_speed)

                state = "STRAIGHT" if correction == 0 else "PD"
                log.emit(bits, error, left_speed, right_speed, state)
                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping (Ctrl+C)...")
        finally:
            bot.stop()
            log.close()
            print("Motors off.")


if __name__ == "__main__":
    main()
