"""
advanced_follow.py — 4-channel IR line follower (Raspbot V2 / Raspberry Pi 5).

Refactored from archive/p_follow.py (the original P-controller is preserved
there untouched). This version is a PD controller that solves four real
tracking problems:

  1. 90° TURN LOCKOUT      — a state lock stops the chassis "swinging past"
                             a hard turn and latching onto the opposite sensor.
  2. STRAIGHT-LINE TRIM     — MOTOR_TRIM cancels physical drift; an error
       + DEADBAND (anti-zigzag) deadband + higher BASE_SPEED let it glide instead of hunting.
  3. JUNCTIONS / DEAD-ENDS  — random branch at T-junctions; a dead-end counter
                             does a 180° turn the first time and stops the
                             second time (no infinite pacing).
  4. BLACK-SQUARE STOP      — all four sensors black = terminal marker → stop.

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

# ── Problem 3: junctions, dead-ends, recovery ────────────────────────────────
JUNCTION_CLEAR_TIME = 0.15   # s to force a turn so we fully clear an intersection
SPIN_180_TIME       = 0.90   # s of in-place spin that equals ~180° (TIMED — the
                             # robot has no encoder; calibrate for your floor/battery)
DEAD_END_CONFIRM    = 8      # consecutive all-white loops before we BELIEVE it is a
                             # real dead-end (debounce — stops a small tape gap from
                             # falsely triggering the 180° / shutdown logic)

# PWM hard limits
PWM_MIN, PWM_MAX = -255, 255


def clamp(val, lo=PWM_MIN, hi=PWM_MAX):
    return max(lo, min(val, hi))


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


def spin_180(bot, direction="right"):
    """Timed ~180° in-place turn (open-loop — tune SPIN_180_TIME)."""
    print(f"  [dead-end] spinning 180° ({direction}) to head back...")
    t_end = time.time() + SPIN_180_TIME
    while time.time() < t_end:
        if direction == "right":
            pivot_right(bot)
        else:
            pivot_left(bot)
        time.sleep(LOOP_DELAY)
    bot.stop()


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
    last_error     = 0.0      # for the PD derivative term + recovery direction
    turning_state  = "none"   # Problem 1 lockout: "none" | "left" | "right"
    dead_end_count = 0        # Problem 3: 0 → none yet, 1 → spun back, 2 → home → stop
    lost_streak    = 0        # consecutive all-white loops (dead-end debounce)

    with RasBot() as bot:
        print("=" * 60)
        print("ADVANCED LINE FOLLOWER — PD + lockout + junctions + dead-ends")
        print(f"BASE={BASE_SPEED} Kp={Kp} Kd={Kd} DEADBAND={DEADBAND} "
              f"TRIM={MOTOR_TRIM}")
        print("Ctrl+C to stop.")
        print("=" * 60)

        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()

                # ── PROBLEM 4: BLACK-SQUARE TERMINAL CHECK (first thing) ──────
                # All four sensors on black = the big terminal square. Done.
                if L1 and L2 and R1 and R2:
                    bot.stop()
                    print("\n*** BLACK SQUARE REACHED — checkpoint complete. ***")
                    return

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
                            pivot_left(bot)
                        else:
                            pivot_right(bot)
                        time.sleep(LOOP_DELAY)
                        continue   # stay locked — ignore everything else

                # ── PROBLEM 3a: T-JUNCTION (3 sensors in a T pattern) ─────────
                # L1+L2+R1 (left branch + ahead) or L2+R1+R2 (right branch + ahead).
                # Pick a direction at random and FORCE it long enough to clear
                # the intersection, so we don't immediately re-detect it.
                t_left  = (L1 and L2 and R1 and not R2)
                t_right = (not L1 and L2 and R1 and R2)
                if t_left or t_right:
                    choice = random.choice(["left", "right"])
                    print(f"  [junction] T detected → forcing {choice} for "
                          f"{JUNCTION_CLEAR_TIME}s")
                    t_end = time.time() + JUNCTION_CLEAR_TIME
                    while time.time() < t_end:
                        if choice == "left":
                            pivot_left(bot)
                        else:
                            pivot_right(bot)
                        time.sleep(LOOP_DELAY)
                    last_error = 0.0
                    lost_streak = 0
                    continue

                # ── compute steering error ───────────────────────────────────
                error = sensor_error(L1, L2, R1, R2)

                # ── PROBLEM 3b: DEAD-END / LOOP PREVENTION (all white) ────────
                if error is None:
                    lost_streak += 1
                    # Debounce: a brief gap in the tape should NOT count as a
                    # dead-end. Creep forward in the last-known direction while
                    # we wait to see if the line comes back.
                    if lost_streak < DEAD_END_CONFIRM:
                        nudge = clamp(int(Kp * (last_error if last_error else 0)))
                        drive(bot, BASE_SPEED * 0.6 + nudge,
                                   BASE_SPEED * 0.6 - nudge)
                        time.sleep(LOOP_DELAY)
                        continue

                    # Confirmed dead-end.
                    dead_end_count += 1
                    lost_streak = 0
                    bot.stop()
                    if dead_end_count == 1:
                        # First dead-end → turn around and head back to start.
                        print("\n[dead-end #1] reversing course toward start.")
                        spin_180(bot, direction="right")
                        last_error = 0.0
                        turning_state = "none"
                        continue
                    else:
                        # Second dead-end → we're back at the origin. Stop for good.
                        print("\n*** [dead-end #2] back at origin — STOP. ***")
                        return

                # line is visible again → reset the lost counter
                lost_streak = 0

                # ── PROBLEM 1 (trigger): enter lockout on a hard outer-only turn
                # Only the far-left outer sensor (90° left) or far-right outer
                # sensor (90° right) latches the lockout state.
                if L1 and not L2 and not R1 and not R2:
                    turning_state = "left"
                    print("  [lockout] hard LEFT turn — opposite sensors ignored")
                    pivot_left(bot)
                    time.sleep(LOOP_DELAY)
                    continue
                if R2 and not R1 and not L2 and not L1:
                    turning_state = "right"
                    print("  [lockout] hard RIGHT turn — opposite sensors ignored")
                    pivot_right(bot)
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

                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping (Ctrl+C)...")
        finally:
            bot.stop()
            print("Motors off.")


if __name__ == "__main__":
    main()
