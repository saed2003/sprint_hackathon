"""
best_follow.py — Combined best-of-3 line follower
==================================================
Takes the best from each version:
  - p_follow.py    : weighted error → smooth proportional turning
  - simple_follow.py: last_seen memory for lost-line recovery
  - line_follow.py  : motor smoothing (no jerk) + debounce for lost line
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot

# ═══════════════════════════════════════════════════════
#  TUNING — only change these
#  Sensor board: 70mm wide, inner pair ~12mm apart (P2/P3),
#  outer pair ~55mm apart (P1/P4).
# ═══════════════════════════════════════════════════════
BASE_SPEED    = 85     # cruise speed on a straight
Kp            = 28     # was 45 — lower stops straight-line oscillation
SMOOTH        = 0.22   # was 0.30 — slower settling reduces bounce
LOST_SPEED    = 80     # sweep speed when searching for lost line
DEBOUNCE      = 2      # consecutive all-off reads before declaring "lost"
LOOP_DELAY    = 0.01   # 100 Hz
# ═══════════════════════════════════════════════════════


def clamp(v, lo=-255, hi=255):
    return max(lo, min(hi, int(v)))


def main():
    # smoothed motor state (from line_follow.py)
    actual_L = 0.0
    actual_R = 0.0

    last_error   = 0.0   # memory for lost-line recovery (from simple_follow.py)
    lost_ticks   = 0     # debounce counter (from line_follow.py)

    with RasBot() as bot:
        print("best_follow started. Ctrl+C to stop.")
        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()
                # True = sensor sees BLACK tape
                # L1=outer-left  L2=inner-left  R1=inner-right  R2=outer-right

                # ── 1. WEIGHTED ERROR (from p_follow.py) ──────────────────
                # error < 0 → tape is LEFT  → steer left
                # error > 0 → tape is RIGHT → steer right
                if L2 and R1:
                    error = 0.0      # perfectly centered (both inner on tape)
                    lost_ticks = 0
                elif L2 and not R1 and not R2:
                    error = -1.0     # slight left — inner-left on, inner-right off
                    lost_ticks = 0
                elif L1 and L2:
                    error = -1.8     # medium left — both left sensors on
                    lost_ticks = 0
                elif L1 and not L2:
                    error = -2.5     # hard left — outer-left only (corner)
                    lost_ticks = 0
                elif R1 and not L1 and not L2:
                    error = 1.0      # slight right — inner-right on, inner-left off
                    lost_ticks = 0
                elif R1 and R2:
                    error = 1.8      # medium right — both right sensors on
                    lost_ticks = 0
                elif R2 and not R1:
                    error = 2.5      # hard right — outer-right only (corner)
                    lost_ticks = 0
                else:
                    # ── 2. DEBOUNCE lost line (from line_follow.py) ────────
                    lost_ticks += 1
                    if lost_ticks < DEBOUNCE:
                        error = last_error   # hold last error briefly
                    else:
                        # ── 3. MEMORY SWEEP (from simple_follow.py) ───────
                        error = -3.0 if last_error < 0 else (3.0 if last_error > 0 else 0.0)

                # save last real error for recovery
                if 0 < abs(error) < 3:
                    last_error = error

                # ── 4. P-CORRECTION ───────────────────────────────────────
                correction = Kp * error
                target_L   = BASE_SPEED + correction
                target_R   = BASE_SPEED - correction

                # override with sweep speed when fully lost
                if abs(error) >= 3:
                    target_L = -LOST_SPEED if error < 0 else LOST_SPEED
                    target_R =  LOST_SPEED if error < 0 else -LOST_SPEED

                # ── 5. MOTOR SMOOTHING (from line_follow.py) ──────────────
                actual_L += (target_L - actual_L) * SMOOTH
                actual_R += (target_R - actual_R) * SMOOTH

                bot._apply_motors(clamp(actual_L), clamp(actual_L),
                                  clamp(actual_R), clamp(actual_R))
                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            print("Motors off.")


def run(bot, stop_event=None, **kwargs):
    """Entry point called by drive.py F-key (mirrors line_follow.run API)."""
    actual_L = 0.0
    actual_R = 0.0
    last_error = 0.0
    lost_ticks = 0

    try:
        while stop_event is None or not stop_event.is_set():
            L1, L2, R1, R2 = bot.read_line_sensors()

            if L2 and R1:
                error = 0.0;      lost_ticks = 0
            elif L2 and not R1 and not R2:
                error = -1.0;     lost_ticks = 0
            elif L1 and L2:
                error = -1.8;     lost_ticks = 0
            elif L1 and not L2:
                error = -2.5;     lost_ticks = 0
            elif R1 and not L1 and not L2:
                error = 1.0;      lost_ticks = 0
            elif R1 and R2:
                error = 1.8;      lost_ticks = 0
            elif R2 and not R1:
                error = 2.5;      lost_ticks = 0
            else:
                lost_ticks += 1
                if lost_ticks < DEBOUNCE:
                    error = last_error
                else:
                    error = -3.0 if last_error < 0 else (3.0 if last_error > 0 else 0.0)

            if 0 < abs(error) < 3:
                last_error = error

            correction = Kp * error
            target_L   = BASE_SPEED + correction
            target_R   = BASE_SPEED - correction

            if abs(error) >= 3:
                target_L = -LOST_SPEED if error < 0 else LOST_SPEED
                target_R =  LOST_SPEED if error < 0 else -LOST_SPEED

            actual_L += (target_L - actual_L) * SMOOTH
            actual_R += (target_R - actual_R) * SMOOTH

            bot._apply_motors(clamp(actual_L), clamp(actual_L),
                              clamp(actual_R), clamp(actual_R))
            time.sleep(LOOP_DELAY)
    finally:
        bot.stop()


if __name__ == "__main__":
    main()
