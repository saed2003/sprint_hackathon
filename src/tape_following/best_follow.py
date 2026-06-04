"""
best_follow.py — Fast constant-speed line follower (Yahboom RASPBOT V2)
======================================================================
Focused, simple, FAST. One cruise speed (no dynamic speed for now — the
dynamic/path-memory version is preserved in git history, commit 72de6dd).

WHAT IT DOES
  • Follows ONE continuous curvy line — there are NO junctions/forks/crosses,
    so there is deliberately no junction logic. (An earlier version treated any
    3-sensor reading as a "fork" and committed a pivot; on a single line that
    is just normal off-centre wobble, so it produced an endless false
    FORK(L)/FORK(R) limit cycle. See git history / last_run.log.)
  • Drives at ONE fast SPEED with PD steering for smooth tracking.
  • Sharp bends (only an OUTER sensor on the line) commit a short latched turn
    with a momentum-kill brake pulse; the turn ends the instant the inner
    sensors re-centre. All-black frames (e.g. a steep approach to a curve) read
    as "centred" → just keep going straight.
  • If the tape is fully lost it spin-searches toward the last-seen side, and
    ends the run after END_LOST_SEC (default 3 s) of nothing.
  • Writes a full per-tick log to  tape_following/last_run.log  every run.

SENSOR BOARD: Yahboom YB-MVX01, 70 mm wide.
  L1=outer-left  L2=inner-left  R1=inner-right  R2=outer-right
  read_line_sensors() -> (L1, L2, R1, R2); True = sees BLACK tape

RUN
  python3 src/tape_following/best_follow.py
From drive.py F-key:  best_follow.run(bot, stop_event=evt)
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot

# ═══════════════════════════════════════════════════════════════════
#  TUNING  — change values here only
# ═══════════════════════════════════════════════════════════════════
SPEED          = 185    # ONE constant cruise speed (fast). raise to go faster
Kp             = 15     # proportional steering gain. LOWERED (was 24) — the old
                        # value overcorrected on a single-inner-sensor (±1) read
                        # and threw the robot into the opposite ±1, i.e. zig-zag.
Kd             = 8      # derivative gain. LOWERED (was 14) — on digital sensors the
                        # error is quantized, so each ±1 flip spikes the derivative
                        # and FEEDS the wobble instead of damping it. Keep it small.
SMOOTH         = 0.22   # motor EMA smoothing (lower = silkier). LOWERED (was 0.35)
                        # so the wheels ease between speeds instead of snapping.

# Turns
PIVOT_FWD      = 130    # outer wheel during a pivot
PIVOT_REV      = -85    # inner wheel during a pivot (negative = reverse)
PIVOT_LATCH    = 22     # MAX ticks committed to a turn (22 x 8ms ~= 0.18 s). Short
                        # on purpose: the reacquire check ends the turn the instant
                        # the inner sensors re-center, so this is only a ceiling that
                        # bounds overshoot. (Was 70 = 0.56 s of blind spin → the
                        # robot sailed clean off the line; see last_run.log ticks
                        # 230-293 where one turn went [0111]→[1111]→[0000].)
BRAKE_PULSE    = 4      # reverse ticks on turn ENTRY to kill momentum (anti-miss)
BRAKE_SPEED    = -100   # both-wheel speed during the brake pulse
RECOVERY_LOCK  = 15     # ticks ignoring opposite outer sensor after a turn

# Lost line
LOST_SPEED     = 110    # in-place spin speed while searching for the line
DEBOUNCE       = 2      # all-off reads before declaring lost
END_LOST_SEC   = 3.0    # stop the run after the tape is gone this long

LOOP_DELAY     = 0.008  # 125 Hz loop

# Debug / logging
DEBUG          = True   # print live state to console
DEBUG_EVERY    = 15     # print every Nth tick (file gets EVERY tick)
LOG_TO_FILE    = True   # write full per-tick log to last_run.log

_HERE     = os.path.dirname(os.path.abspath(__file__))
LOG_FILE  = os.path.join(_HERE, "last_run.log")
# ═══════════════════════════════════════════════════════════════════

# Sensor error magnitudes (sign: - = tape LEFT, + = tape RIGHT)
E_INNER = 1.0   # one inner sensor
E_BOTH  = 2.0   # inner + outer same side
E_OUTER = 4.0   # outer sensor only -> sharp corner


def clamp(v, lo=-255, hi=255):
    return max(lo, min(hi, int(v)))


def _read_pattern(L1, L2, R1, R2):
    """0-2 active sensors -> signed error, or None if all-off."""
    if L2 and R1:                               return 0.0        # 0110 centred
    if L1 and not L2 and not R1 and not R2:     return -E_OUTER   # 1000 hard-left
    if L1 and L2:                               return -E_BOTH    # 1100 leaning left
    if L2:                                      return -E_INNER   # 0100 slight left
    if R2 and not R1 and not L1 and not L2:     return  E_OUTER   # 0001 hard-right
    if R1 and R2:                               return  E_BOTH    # 0011 leaning right
    if R1:                                      return  E_INNER   # 0010 slight right
    return None                                                    # 0000 lost


class _Log:
    """Writes every tick to last_run.log so we can review the run afterwards."""
    def __init__(self, enabled):
        self.f = None
        if enabled:
            try:
                self.f = open(LOG_FILE, "w")
                self.f.write(f"# best_follow run {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                self.f.write(f"# SPEED={SPEED} Kp={Kp} Kd={Kd} PIVOT_LATCH={PIVOT_LATCH} "
                             f"BRAKE_PULSE={BRAKE_PULSE} RECOVERY_LOCK={RECOVERY_LOCK}\n")
                self.f.write("# tick bits err L R state\n")
            except Exception as e:
                print(f"  log file open failed: {e}")

    def write(self, line):
        if self.f:
            self.f.write(line + "\n")

    def close(self):
        if self.f:
            self.f.flush()
            self.f.close()
            print(f"  log saved -> {LOG_FILE}")


class _Follower:
    def __init__(self, log=None):
        self.actual_L = 0.0
        self.actual_R = 0.0
        self.last_error = 0.0
        self.lost_ticks = 0
        self.latch_ticks    = 0
        self.latch_dir      = 0
        self.brake_ticks    = 0
        self._last_turn_dir = 0
        self._recov_lock    = 0
        self._tick = 0
        self._lost_since = None
        self.finished = False
        self.finish_reason = None
        self.log = log

    # ── turn management ───────────────────────────────────────────
    def _arm_turn(self, direction):
        self.latch_dir      = direction
        self.latch_ticks    = PIVOT_LATCH
        self._last_turn_dir = direction
        fwd = (self.actual_L + self.actual_R) / 2.0
        self.brake_ticks = BRAKE_PULSE if fwd > 80 else 0

    def _drive_turn(self, bot):
        if self.brake_ticks > 0:
            self.brake_ticks -= 1
            self._snap(bot, BRAKE_SPEED, BRAKE_SPEED)
        elif self.latch_dir < 0:
            self._snap(bot, PIVOT_REV, PIVOT_FWD)    # pivot LEFT
        else:
            self._snap(bot, PIVOT_FWD, PIVOT_REV)    # pivot RIGHT

    # ── main step ─────────────────────────────────────────────────
    def step(self, bot):
        L1, L2, R1, R2 = bot.read_line_sensors()
        bits  = f"{int(L1)}{int(L2)}{int(R1)}{int(R2)}"
        self._tick += 1
        now = time.time()

        # ── committed turn: finish it before anything else ────────
        # End the turn the instant we're back on the line — a centred reading
        # includes the all-black / 3-sensor frames ([1110],[0111],[1111]) that
        # happen on a steep approach to a curve, so those STOP the turn rather
        # than extend it (no more spinning in place on a black patch).
        if self.latch_ticks > 0:
            raw = _read_pattern(L1, L2, R1, R2)
            reacquired = (raw is not None and abs(raw) <= E_INNER)
            if reacquired:
                self.latch_ticks = 0
                self._recov_lock = RECOVERY_LOCK
            else:
                self.latch_ticks -= 1
                self._drive_turn(bot)
                self._lost_since = None
                self._emit(bits, float(self.latch_dir) * E_OUTER,
                           f"TURN({'L' if self.latch_dir < 0 else 'R'}) latch={self.latch_ticks}")
                return

        # NOTE: single continuous line — NO junction/fork handling on purpose.
        # A 3-sensor reading is just off-centre wobble; _read_pattern maps it to
        # "centred" so PD keeps the robot straight instead of pivoting.
        raw = _read_pattern(L1, L2, R1, R2)

        # ── lost line ─────────────────────────────────────────────
        if raw is None:
            self.lost_ticks += 1
            if self._lost_since is None:
                self._lost_since = now
            if now - self._lost_since >= END_LOST_SEC:
                self.finished = True
                self.finish_reason = f"tape lost {END_LOST_SEC:.0f}s"
                bot.stop()
                self._emit(bits, 0, "END(lost)")
                return
            if self.lost_ticks < DEBOUNCE:
                self._apply(bot, self.actual_L, self.actual_R)
                self._emit(bits, 0, "COAST")
                return
            # search toward the side the tape was last seen
            side = self._last_turn_dir if self._last_turn_dir != 0 else (
                -1 if self.last_error < 0 else 1)
            if side < 0:
                self._snap(bot, -LOST_SPEED, LOST_SPEED)
            else:
                self._snap(bot, LOST_SPEED, -LOST_SPEED)
            self._emit(bits, float(side) * E_OUTER, f"LOST({'L' if side < 0 else 'R'})")
            return

        # ── on the line ───────────────────────────────────────────
        self.lost_ticks = 0
        self._lost_since = None
        error = raw

        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = error

        # ── sharp corner (outer only) -> commit a turn ────────────
        if abs(error) >= E_OUTER:
            turn_dir = 1 if error > 0 else -1
            # recovery lock: just after a turn, an opposite outer hit is
            # usually the body still crossing the tape -> treat as a curve
            if self._recov_lock > 0 and turn_dir != self._last_turn_dir:
                self._recov_lock -= 1
                error = float(turn_dir) * E_BOTH
                correction = Kp * error + Kd * derivative
            else:
                if self._recov_lock > 0:
                    self._recov_lock -= 1
                self._arm_turn(turn_dir)
                self._drive_turn(bot)
                self._emit(bits, float(turn_dir) * E_OUTER,
                           f"CORNER({'R' if turn_dir > 0 else 'L'})")
                return

        # ── normal tracking at constant speed ─────────────────────
        if self._recov_lock > 0:
            self._recov_lock -= 1

        target_L = SPEED + correction
        target_R = SPEED - correction
        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH
        self._apply(bot, self.actual_L, self.actual_R)

        if error == 0:               st = "STRAIGHT"
        elif abs(error) <= E_INNER:  st = "SLIGHT"
        else:                        st = "CURVE"
        self._emit(bits, error, st)

    # ── output ────────────────────────────────────────────────────
    def _emit(self, bits, error, state):
        line = (f"[{self._tick:06d}] [{bits}] err:{error:+.1f} "
                f"L:{int(self.actual_L):4d} R:{int(self.actual_R):4d}  {state}")
        if self.log:
            self.log.write(line)                       # every tick to file
        if DEBUG and self._tick % DEBUG_EVERY == 0:
            print(line, flush=True)                    # sampled to console

    def _apply(self, bot, l, r):
        bot._apply_motors(clamp(l), clamp(l), clamp(r), clamp(r))

    def _snap(self, bot, l, r):
        self.actual_L, self.actual_R = float(l), float(r)
        self._apply(bot, l, r)


def run(bot, stop_event=None, **kwargs):
    """Called by drive.py F-key."""
    log = _Log(LOG_TO_FILE)
    f = _Follower(log=log)
    try:
        while stop_event is None or not stop_event.is_set():
            f.step(bot)
            if f.finished:
                print(f"\n*** RUN COMPLETE — {f.finish_reason} ***", flush=True)
                break
            time.sleep(LOOP_DELAY)
    finally:
        bot.stop()
        log.close()


def main():
    with RasBot() as bot:
        print("best_follow (fast constant speed) started. Ctrl+C to stop.")
        print(f"  SPEED={SPEED} Kp={Kp} Kd={Kd}  END after {END_LOST_SEC:.0f}s lost")
        print(f"  logging every tick to {LOG_FILE}")
        print("  [tick] [bits] err L R state")
        log = _Log(LOG_TO_FILE)
        f = _Follower(log=log)
        try:
            while True:
                f.step(bot)
                if f.finished:
                    print(f"\n*** RUN COMPLETE — {f.finish_reason} ***", flush=True)
                    break
                time.sleep(LOOP_DELAY)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            log.close()
            print("Motors off.")


if __name__ == "__main__":
    main()
