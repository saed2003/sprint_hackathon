"""
best_follow.py — Adaptive PD line follower (best combined version)
==================================================================
Combines the strongest idea from every follower in this folder:

  p_follow.py      → weighted proportional error
  pd_follow.py     → derivative term that kills oscillation
  line_follow.py   → predictive braking into corners + hard pivot on sharp turns
  simple_follow.py → memory-based lost-line recovery
  best_follow.py   → EMA motor smoothing for jerk-free drive

Design goals
  • FAST on straights (high cruise speed)
  • NEVER miss a turn (auto-brake into corners + hard pivot on outer-only)
  • NO wobble on straights (PD damping + motor smoothing)
  • SMOOTH drive (exponential motor interpolation)

Sensor board: Yahboom YB-MVX01, 70mm wide.
  L1 = outer-left   L2 = inner-left   R1 = inner-right   R2 = outer-right
  inner pair (L2/R1) ~12mm apart — straddle the tape when centred
  outer pair (L1/R2) ~55mm apart — only fire on a real corner / big drift
  read_line_sensors() returns (L1, L2, R1, R2); True = sees BLACK tape

Run standalone:
  python3 src/tape_following/best_follow.py
From drive.py F-key:
  best_follow.run(bot, stop_event=evt)
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot

# ═══════════════════════════════════════════════════════════════════
#  TUNING  — only change values in this block
# ═══════════════════════════════════════════════════════════════════
CRUISE_SPEED  = 150    # straight-line speed (fast). raise for more speed
CURVE_SPEED   = 95     # speed while leaning into a curve (|err|>=2) — slows BEFORE the corner
SLIGHT_SPEED  = 125    # speed on a slight drift (|err|=1) — ease off a touch
MIN_SPEED     = 65     # slowest allowed mid-turn (brake floor)
Kp            = 24     # proportional gain — sharper turns if raised
Kd            = 14     # derivative gain — damps wobble/overshoot
BRAKE_K       = 0.55   # auto-brake strength: speed drops as correction grows
SMOOTH        = 0.35   # motor EMA: 0.15=silky, 0.35=balanced, 0.5=snappy
PIVOT_FWD     = 125    # outer-wheel speed during a hard corner pivot
PIVOT_REV     = -75    # inner-wheel speed during a hard corner pivot (spins it round)
PIVOT_LATCH   = 50     # max ticks to COMMIT to a sharp turn (~0.4s) — fixes missed turns
BRAKE_PULSE   = 3      # ticks of hard brake when ENTERING a sharp turn — kills momentum
BRAKE_SPEED   = -90    # reverse speed during the brake pulse (both wheels)
JUNCTION_DIR  = +1     # which way to go at a true cross/T-junction: +1 = RIGHT, -1 = LEFT
LOST_SPEED    = 95     # in-place spin speed when hunting a lost line
DEBOUNCE      = 2      # consecutive all-off reads before declaring "lost"
LOOP_DELAY    = 0.008  # ~125 Hz — fast loop keeps the derivative accurate
# ═══════════════════════════════════════════════════════════════════

# error magnitudes (sign = side: negative = tape LEFT, positive = tape RIGHT)
E_INNER = 1.0    # one inner sensor only      — tiny drift
E_BOTH  = 2.0    # inner + outer, same side   — leaning into a curve
E_OUTER = 4.0    # outer sensor ONLY          — sharp corner, trigger pivot
E_LOST  = 6.0    # all sensors off            — memory sweep


def clamp(v, lo=-255, hi=255):
    return max(lo, min(hi, int(v)))


def _read_pattern(L1, L2, R1, R2):
    """Map a 0-2 sensor pattern to a signed error, or None if all-off.

    (3-4 sensor patterns are handled as junctions/corners in step(),
    so this only sees the normal tracking cases.)

    Negative error = tape is LEFT  → steer left.
    Positive error = tape is RIGHT → steer right.
    """
    if L2 and R1:                                   # 0110 centred
        return 0.0
    if L1 and not L2 and not R1 and not R2:         # 1000 sharp LEFT corner
        return -E_OUTER
    if L1 and L2:                                   # 1100 leaning left
        return -E_BOTH
    if L2:                                          # 0100 slight left
        return -E_INNER
    if R2 and not R1 and not L1 and not L2:         # 0001 sharp RIGHT corner
        return E_OUTER
    if R1 and R2:                                   # 0011 leaning right
        return E_BOTH
    if R1:                                          # 0010 slight right
        return E_INNER
    return None                                     # 0000 lost


def _junction_dir(L1, L2, R1, R2, count):
    """For a 3-4 sensor reading, decide which way to commit.

    count==4 (1111)  → true cross/T → use the configured JUNCTION_DIR
    count==3 1110    → tape spans left  → turn LEFT  (-1)
    count==3 0111    → tape spans right → turn RIGHT (+1)
    anything odd     → fall back to JUNCTION_DIR
    """
    if count == 4:
        return JUNCTION_DIR
    if not R2 and L1:        # 1110 — left-biased corner
        return -1
    if not L1 and R2:        # 0111 — right-biased corner
        return +1
    return JUNCTION_DIR


class _Follower:
    """Holds the PD + smoothing state so main() and run() share one code path."""

    def __init__(self):
        self.actual_L = 0.0      # smoothed motor outputs
        self.actual_R = 0.0
        self.last_error = 0.0    # for derivative + lost-line memory
        self.lost_ticks = 0      # debounce counter for all-off
        self.latch_ticks = 0     # >0 while COMMITTED to a sharp turn / junction
        self.latch_dir = 0       # which way we're committed: +1 right, -1 left
        self.brake_ticks = 0     # >0 while killing forward momentum into a turn

    def _arm_turn(self, direction):
        """Begin committing to a sharp turn / junction toward `direction`.

        If we're carrying real forward speed, queue a brief brake pulse first
        so the pivot rotates in place instead of arcing past the line.
        """
        self.latch_dir = direction
        self.latch_ticks = PIVOT_LATCH
        fwd = (self.actual_L + self.actual_R) / 2.0
        self.brake_ticks = BRAKE_PULSE if fwd > 100 else 0

    def _drive_turn(self, bot):
        """Execute the committed turn: brake-pulse to kill momentum, then pivot."""
        if self.brake_ticks > 0:
            self.brake_ticks -= 1
            self._snap(bot, BRAKE_SPEED, BRAKE_SPEED)     # both wheels reverse — kill speed
        elif self.latch_dir < 0:
            self._snap(bot, PIVOT_REV, PIVOT_FWD)         # pivot LEFT
        else:
            self._snap(bot, PIVOT_FWD, PIVOT_REV)         # pivot RIGHT

    def step(self, bot):
        L1, L2, R1, R2 = bot.read_line_sensors()
        count = int(L1) + int(L2) + int(R1) + int(R2)

        # ── ALREADY COMMITTED TO A TURN: finish it before anything else ─
        # Keep turning until the line is centred again under the inner
        # sensors. This is what stops "almost made it" misses.
        if self.latch_ticks > 0:
            raw = _read_pattern(L1, L2, R1, R2)
            reacquired = (count < 3 and raw is not None and abs(raw) <= E_INNER)
            if reacquired:
                self.latch_ticks = 0          # turn complete → fall through to tracking
            else:
                self.latch_ticks -= 1
                self._drive_turn(bot)
                return

        # ── JUNCTION / CROSS: 3-4 sensors on → commit to a branch fast ──
        if count >= 3:
            self._arm_turn(_junction_dir(L1, L2, R1, R2, count))
            self.last_error = float(self.latch_dir) * E_OUTER
            self._drive_turn(bot)
            return

        raw = _read_pattern(L1, L2, R1, R2)

        # ── lost-line handling with debounce ───────────────────────────
        if raw is None:
            self.lost_ticks += 1
            if self.lost_ticks < DEBOUNCE:
                # brief dropout — coast on the last motor command
                self._apply(bot, self.actual_L, self.actual_R)
                return
            # truly lost → spin toward the side we last saw tape
            if self.last_error < 0:
                self._snap(bot, -LOST_SPEED, LOST_SPEED)
            elif self.last_error > 0:
                self._snap(bot, LOST_SPEED, -LOST_SPEED)
            else:
                self._snap(bot, -LOST_SPEED, LOST_SPEED)   # default hunt left
            return

        self.lost_ticks = 0
        error = raw

        # ── PD correction ──────────────────────────────────────────────
        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = error                  # remember real readings only

        # ── SHARP CORNER (outer sensor only) → arm the commit latch ────
        if abs(error) >= E_OUTER:
            self._arm_turn(1 if error > 0 else -1)
            self._drive_turn(bot)
            return

        # ── normal tracking with tiered speed + predictive braking ─────
        # Brake the instant we leave dead-centre so we're ALREADY slow by
        # the time the sharp part arrives. Full speed returns on the straight.
        ae = abs(error)
        if ae == 0:
            base = CRUISE_SPEED          # dead straight → full speed
        elif ae <= E_INNER:
            base = SLIGHT_SPEED          # slight drift → ease off
        else:
            base = CURVE_SPEED           # leaning into a curve → slow for the corner

        speed = max(MIN_SPEED, base - abs(correction) * BRAKE_K)
        target_L = speed + correction
        target_R = speed - correction

        # EMA smoothing → no jerk
        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH
        self._apply(bot, self.actual_L, self.actual_R)

    def _apply(self, bot, l, r):
        bot._apply_motors(clamp(l), clamp(l), clamp(r), clamp(r))

    def _snap(self, bot, l, r):
        """Instant motor set (bypasses smoothing) for pivots / sweeps."""
        self.actual_L, self.actual_R = float(l), float(r)
        self._apply(bot, l, r)


def main():
    follower = _Follower()
    with RasBot() as bot:
        print("best_follow (adaptive PD) started. Ctrl+C to stop.")
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
    """Entry point called by drive.py F-key (mirrors line_follow.run API)."""
    follower = _Follower()
    try:
        while stop_event is None or not stop_event.is_set():
            follower.step(bot)
            time.sleep(LOOP_DELAY)
    finally:
        bot.stop()


if __name__ == "__main__":
    main()
