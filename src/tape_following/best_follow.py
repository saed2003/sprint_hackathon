"""
best_follow.py — Adaptive PD line follower with dynamic speed + debug logging
==============================================================================
Design goals:
  • DYNAMIC speed — rolls fast on confirmed straights, decelerates smoothly
    into turns based on a rolling error history (no hard-coded speed tiers)
  • NEVER miss a turn — brake-pulse to kill momentum + commit latch through
    the all-off zone mid-rotation
  • NO wobble on straights — PD damping + EMA motor smoothing
  • LIVE DEBUG — set DEBUG=True to stream state every tick for remote analysis

Sensor board: Yahboom YB-MVX01, 70mm wide.
  L1=outer-left  L2=inner-left  R1=inner-right  R2=outer-right
  inner pair straddles the tape when centred (~12mm apart)
  outer pair fires only on sharp corners/big drift (~55mm apart)
  read_line_sensors() returns (L1, L2, R1, R2); True = sees BLACK tape

Run standalone:
  python3 src/tape_following/best_follow.py
From drive.py F-key:
  best_follow.run(bot, stop_event=evt)
"""

import time
import sys
import os
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot

# ═══════════════════════════════════════════════════════════════════
#  TUNING  — change values here only
# ═══════════════════════════════════════════════════════════════════
CRUISE_SPEED   = 150    # top speed on a confirmed straight
MIN_SPEED      = 60     # floor speed mid-pivot / hard brake
Kp             = 24     # proportional gain
Kd             = 14     # derivative gain — damps oscillation
BRAKE_K        = 0.55   # correction → extra speed penalty
SMOOTH         = 0.35   # EMA motor smoothing (0.15 silky → 0.5 snappy)

# Dynamic speed — rolling error window
DYN_WINDOW     = 25     # how many ticks to average (25 × 8ms = 0.2 s)
DYN_FAST_ERR   = 0.3    # mean |error| below this  → full CRUISE_SPEED
DYN_SLOW_ERR   = 2.5    # mean |error| above this  → MIN_SPEED
# speed interpolates linearly between (DYN_FAST_ERR, CRUISE) and (DYN_SLOW_ERR, MIN)
ACCEL_RATE     = 4.0    # max speed increase per tick (prevents speed jumps after a turn)

# Pivot / turn parameters
PIVOT_FWD      = 125    # outer wheel during pivot
PIVOT_REV      = -75    # inner wheel during pivot (negative = reverse)
PIVOT_LATCH    = 50     # ticks to COMMIT to a turn (50 × 8ms ≈ 0.4 s)
BRAKE_PULSE    = 3      # reverse ticks before pivot to kill momentum
BRAKE_SPEED    = -90    # both wheels this speed during brake pulse
JUNCTION_DIR   = +1     # default junction direction: +1 RIGHT, -1 LEFT

LOST_SPEED     = 95     # spin speed while hunting lost line
DEBOUNCE       = 2      # all-off reads before declaring lost
LOOP_DELAY     = 0.008  # 125 Hz loop

# Debug output
DEBUG          = True   # True = print live state every tick
DEBUG_EVERY    = 15     # print every N ticks (lower = more output)
# ═══════════════════════════════════════════════════════════════════

# Sensor error magnitudes
E_INNER = 1.0   # one inner sensor
E_BOTH  = 2.0   # inner + outer same side
E_OUTER = 4.0   # outer sensor only → sharp corner


def clamp(v, lo=-255, hi=255):
    return max(lo, min(hi, int(v)))


def _read_pattern(L1, L2, R1, R2):
    """0-2 active sensors → signed error, or None if all-off."""
    if L2 and R1:                               return 0.0    # 0110 centred
    if L1 and not L2 and not R1 and not R2:     return -E_OUTER  # 1000 hard-left corner
    if L1 and L2:                               return -E_BOTH   # 1100 leaning left
    if L2:                                      return -E_INNER  # 0100 slight left
    if R2 and not R1 and not L1 and not L2:     return  E_OUTER  # 0001 hard-right corner
    if R1 and R2:                               return  E_BOTH   # 0011 leaning right
    if R1:                                      return  E_INNER  # 0010 slight right
    return None                                                   # 0000 lost


def _junction_dir(L1, L2, R1, R2, count):
    """3-4 sensors on → decide direction."""
    if count == 4:       return JUNCTION_DIR
    if not R2 and L1:    return -1   # 1110 left-biased
    if not L1 and R2:    return +1   # 0111 right-biased
    return JUNCTION_DIR


class _Follower:
    def __init__(self):
        self.actual_L   = 0.0
        self.actual_R   = 0.0
        self.last_error = 0.0
        self.lost_ticks = 0
        self.latch_ticks = 0
        self.latch_dir   = 0
        self.brake_ticks = 0
        self._dyn_buf    = deque([0.0] * DYN_WINDOW, maxlen=DYN_WINDOW)
        self._dyn_speed  = float(MIN_SPEED)   # current target cruise speed
        self._tick       = 0

    # ── dynamic speed ─────────────────────────────────────────────
    def _update_dyn_speed(self, error):
        """Update the rolling error buffer and compute a dynamic cruise speed."""
        self._dyn_buf.append(abs(error))
        mean_err = sum(self._dyn_buf) / len(self._dyn_buf)
        # linear interpolation between (fast_err→CRUISE) and (slow_err→MIN)
        t = (mean_err - DYN_FAST_ERR) / max(DYN_SLOW_ERR - DYN_FAST_ERR, 0.001)
        t = max(0.0, min(1.0, t))
        target = CRUISE_SPEED * (1 - t) + MIN_SPEED * t
        # ramp up slowly, drop down instantly
        if target > self._dyn_speed:
            self._dyn_speed = min(target, self._dyn_speed + ACCEL_RATE)
        else:
            self._dyn_speed = target
        return self._dyn_speed

    # ── turn management ───────────────────────────────────────────
    def _arm_turn(self, direction):
        self.latch_dir   = direction
        self.latch_ticks = PIVOT_LATCH
        fwd = (self.actual_L + self.actual_R) / 2.0
        self.brake_ticks = BRAKE_PULSE if fwd > 80 else 0

    def _drive_turn(self, bot):
        if self.brake_ticks > 0:
            self.brake_ticks -= 1
            self._snap(bot, BRAKE_SPEED, BRAKE_SPEED)
        elif self.latch_dir < 0:
            self._snap(bot, PIVOT_REV, PIVOT_FWD)
        else:
            self._snap(bot, PIVOT_FWD, PIVOT_REV)

    # ── main step ─────────────────────────────────────────────────
    def step(self, bot):
        L1, L2, R1, R2 = bot.read_line_sensors()
        count  = int(L1) + int(L2) + int(R1) + int(R2)
        bits   = f"{int(L1)}{int(L2)}{int(R1)}{int(R2)}"
        self._tick += 1
        state  = "TRACK"

        # ── committed turn: keep going until inner sensors re-centre ──
        if self.latch_ticks > 0:
            raw = _read_pattern(L1, L2, R1, R2)
            if count < 3 and raw is not None and abs(raw) <= E_INNER:
                self.latch_ticks = 0         # line re-acquired → resume tracking
            else:
                self.latch_ticks -= 1
                self._drive_turn(bot)
                state = f"TURN({'L' if self.latch_dir < 0 else 'R'}) latch={self.latch_ticks}"
                self._debug(bits, 0, int(self._dyn_speed), self.actual_L, self.actual_R, state)
                return

        # ── junction: 3-4 sensors ─────────────────────────────────
        if count >= 3:
            d = _junction_dir(L1, L2, R1, R2, count)
            self._arm_turn(d)
            self.last_error = float(d) * E_OUTER
            self._drive_turn(bot)
            state = f"JUNC({'L' if d < 0 else 'R'})"
            self._debug(bits, 0, int(self._dyn_speed), self.actual_L, self.actual_R, state)
            return

        raw = _read_pattern(L1, L2, R1, R2)

        # ── lost line ─────────────────────────────────────────────
        if raw is None:
            self.lost_ticks += 1
            self._update_dyn_speed(E_OUTER)   # treat as max error for speed history
            if self.lost_ticks < DEBOUNCE:
                self._apply(bot, self.actual_L, self.actual_R)
                self._debug(bits, 0, int(self._dyn_speed), self.actual_L, self.actual_R, "COAST")
                return
            if self.last_error < 0:
                self._snap(bot, -LOST_SPEED, LOST_SPEED)
            elif self.last_error > 0:
                self._snap(bot, LOST_SPEED, -LOST_SPEED)
            else:
                self._snap(bot, -LOST_SPEED, LOST_SPEED)
            self._debug(bits, 0, int(self._dyn_speed), self.actual_L, self.actual_R, "LOST")
            return

        self.lost_ticks = 0
        error = raw

        # ── PD correction ─────────────────────────────────────────
        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = error

        # ── sharp corner → arm latch ──────────────────────────────
        if abs(error) >= E_OUTER:
            self._arm_turn(1 if error > 0 else -1)
            self._update_dyn_speed(E_OUTER)
            self._drive_turn(bot)
            state = f"CORNER({'R' if error > 0 else 'L'})"
            self._debug(bits, error, int(self._dyn_speed), self.actual_L, self.actual_R, state)
            return

        # ── normal tracking with dynamic speed ────────────────────
        cruise = self._update_dyn_speed(error)
        speed  = max(MIN_SPEED, cruise - abs(correction) * BRAKE_K)

        target_L = speed + correction
        target_R = speed - correction

        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH
        self._apply(bot, self.actual_L, self.actual_R)

        if error == 0:          state = "STRAIGHT"
        elif abs(error) <= E_INNER: state = "SLIGHT"
        else:                   state = "CURVE"
        self._debug(bits, error, int(cruise), self.actual_L, self.actual_R, state)

    def _debug(self, bits, error, speed, L, R, state):
        if not DEBUG: return
        if self._tick % DEBUG_EVERY != 0: return
        print(
            f"[{self._tick:06d}]"
            f" [{bits}]"
            f" err:{error:+.1f}"
            f" spd:{speed:3d}"
            f" L:{int(L):4d} R:{int(R):4d}"
            f"  {state}",
            flush=True
        )

    def _apply(self, bot, l, r):
        bot._apply_motors(clamp(l), clamp(l), clamp(r), clamp(r))

    def _snap(self, bot, l, r):
        self.actual_L, self.actual_R = float(l), float(r)
        self._apply(bot, l, r)


def main():
    follower = _Follower()
    with RasBot() as bot:
        print("best_follow started. Ctrl+C to stop.")
        if DEBUG:
            print(f"  CRUISE={CRUISE_SPEED} MIN={MIN_SPEED} Kp={Kp} Kd={Kd}")
            print(f"  DYN_WINDOW={DYN_WINDOW} ticks  ACCEL_RATE={ACCEL_RATE}/tick")
            print(f"  [tick] [bits] err speed  L   R   state")
        try:
            while True:
                follower.step(bot)
                time.sleep(LOOP_DELAY)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            print("Motors off.")


def run(bot, stop_event=None, **kwargs):
    """Entry point called by drive.py F-key."""
    follower = _Follower()
    try:
        while stop_event is None or not stop_event.is_set():
            follower.step(bot)
            time.sleep(LOOP_DELAY)
    finally:
        bot.stop()


if __name__ == "__main__":
    main()
