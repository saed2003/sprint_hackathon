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
  • Hard bends — 3-sensors-on-one-side or outer-only (1110/1000 -> LEFT,
    0111/0001 -> RIGHT) — STOP (brake pulse kills momentum so we don't sail
    past), pivot toward the line, then continue the instant the inner sensors
    re-centre. All-black [1111] reads as "centred" -> keep going straight.
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
SPEED          = 220    # ONE constant cruise speed (fast). raise to go faster
Kp             = 24     # proportional steering gain
Kd             = 14     # derivative gain (damps wobble)
SMOOTH         = 0.35   # motor EMA smoothing (0.15 silky -> 0.5 snappy)

# Turns — a sharp turn STOPS (brake), pivots toward the line, then continues
PIVOT_FWD      = 130    # outer wheel during a pivot
PIVOT_REV      = -85    # inner wheel during a pivot (negative = reverse)
PIVOT_LATCH    = 28     # MAX ticks committed to a turn (28 x 8ms ~= 0.22 s). The
                        # reacquire check ends the turn the instant the inner
                        # sensors re-center; this is just a ceiling on overshoot.
BRAKE_PULSE    = 6      # ticks of full brake on turn ENTRY — kills forward momentum
                        # so a fast robot STOPS instead of sailing past the turn.
BRAKE_SPEED    = -120   # both-wheel speed during the brake pulse (reverse = hard stop)
RECOVERY_LOCK  = 15     # ticks ignoring an opposite-side sharp turn just after a turn

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

# Discrete position error from the 4-bit pattern (sign: - = tape LEFT, + = RIGHT)
#   1000 -> -3   1100 -> -2   0100 -> -1   0110 -> 0   0010 -> +1   0011 -> +2   0001 -> +3
E_SLIGHT = 1.0   # one inner sensor only        (0100 / 0010)
E_LEAN   = 2.0   # inner + outer, same side     (1100 / 0011)
E_SHARP  = 3.0   # outer only / 3-on-one-side   (1000 / 0001 / 1110 / 0111) -> stop+pivot


def clamp(v, lo=-255, hi=255):
    return max(lo, min(hi, int(v)))


def _read_pattern(L1, L2, R1, R2):
    """4-bit pattern -> signed position error (-3..+3), or None if all-off.

    Both inner sensors on (x11x: 0110/1110/0111/1111) reads as CENTERED — the
    3-sensor and all-black frames are just a wide/steep crossing of the line.
    The hard turns (3-on-one-side / outer-only) are caught earlier in step()
    as a stop-and-pivot; here they map to ±E_SHARP as a PD fallback.
    """
    if L2 and R1:   return 0.0        # x11x centered (0110/1110/0111/1111)
    if L1 and L2:   return -E_LEAN    # 1100 leaning left
    if L2:          return -E_SLIGHT  # 0100 slight left
    if L1:          return -E_SHARP   # 1000 hard left (outer only)
    if R1 and R2:   return  E_LEAN    # 0011 leaning right
    if R1:          return  E_SLIGHT  # 0010 slight right
    if R2:          return  E_SHARP   # 0001 hard right (outer only)
    return None                        # 0000 lost


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

        # ── committed turn: pivot until back on the line ──────────
        # End the turn the instant we re-center (raw 0, incl. the 3-sensor /
        # all-black frames) or even just barely on (±E_SLIGHT), then resume PD.
        if self.latch_ticks > 0:
            raw = _read_pattern(L1, L2, R1, R2)
            reacquired = (raw is not None and abs(raw) <= E_SLIGHT)
            if reacquired:
                self.latch_ticks = 0
                self._recov_lock = RECOVERY_LOCK
            else:
                self.latch_ticks -= 1
                self._drive_turn(bot)
                self._lost_since = None
                self._emit(bits, float(self.latch_dir) * E_SHARP,
                           f"TURN({'L' if self.latch_dir < 0 else 'R'}) latch={self.latch_ticks}")
                return

        # ── SHARP TURN: STOP, pivot toward the line, then continue ────
        # 3-on-one-side or outer-only means a hard bend the PD loop would sail
        # past at speed:  1110 / 1000 -> LEFT,  0111 / 0001 -> RIGHT.
        # _arm_turn brakes to a stop first (BRAKE_PULSE) so momentum can't carry
        # us past, then we pivot until the inner sensors re-center. THIS is the
        # fix for missed turns.
        sharp_left  = bits in ("1110", "1000")
        sharp_right = bits in ("0111", "0001")
        if sharp_left or sharp_right:
            d = -1 if sharp_left else 1
            # just after a turn the body is still sweeping across the tape — don't
            # instantly fire the OPPOSITE turn; let PD ease through it instead.
            if not (self._recov_lock > 0 and d != self._last_turn_dir):
                self._arm_turn(d)
                self._drive_turn(bot)
                self.last_error = float(d) * E_SHARP
                self._lost_since = None
                self._emit(bits, float(d) * E_SHARP,
                           f"CORNER({'L' if d < 0 else 'R'})")
                return

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
            self._emit(bits, float(side) * E_SHARP, f"LOST({'L' if side < 0 else 'R'})")
            return

        # ── on the line: PD steering at constant speed ────────────
        # (Hard turns were already handled above as stop-and-pivot. The only way
        # we reach here with ±E_SHARP is when RECOVERY_LOCK suppressed an opposite
        # turn — then PD just steers hard through it, which is what we want.)
        self.lost_ticks = 0
        self._lost_since = None
        error = raw

        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = error

        if self._recov_lock > 0:
            self._recov_lock -= 1

        target_L = SPEED + correction
        target_R = SPEED - correction
        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH
        self._apply(bot, self.actual_L, self.actual_R)

        if error == 0:               st = "STRAIGHT"
        elif abs(error) <= E_SLIGHT:  st = "SLIGHT"
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
