"""
best_follow.py — Smart adaptive line follower for Yahboom RASPBOT V2
====================================================================
A single, self-contained follower that is fast, smooth, and learns the track.

FEATURES
  1. PD control + dynamic speed   — fast on straights, smooth deceleration into
                                     turns, no wobble (proven from live logs).
  2. Turn prediction              — watches the trend of the sensor error and
                                     starts slowing / leaning BEFORE the outer
                                     sensor fires, so turns are entered smoothly.
  3. Path memory                  — records each lap's segment sequence to a JSON
                                     file. On a lap it has seen before, it BOOSTS
                                     speed on known-long straights and PRE-BRAKES
                                     before known turns (predictive, not reactive).
  4. End conditions               — stops the run when:
                                       • the tape is lost for END_LOST_SEC (3 s), or
                                       • a finish marker (all-4 sensors) is held
                                         for END_FINISH_SEC  (set END_MODE).
  5. Auto-tune (`--tune`)         — drives a straight and sweeps Kp/Kd, measuring
                                     wobble vs. tracking, then recommends + saves
                                     the best controller settings for THIS robot.

SENSOR BOARD: Yahboom YB-MVX01, 70 mm wide.
  L1=outer-left  L2=inner-left  R1=inner-right  R2=outer-right
  read_line_sensors() -> (L1, L2, R1, R2); True = sees BLACK tape

RUN
  python3 src/tape_following/best_follow.py            # normal follow
  python3 src/tape_following/best_follow.py --tune     # auto-tune Kp/Kd
  python3 src/tape_following/best_follow.py --fresh     # ignore saved path memory
From drive.py F-key:  best_follow.run(bot, stop_event=evt)
"""

import time
import sys
import os
import json
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot

# ═══════════════════════════════════════════════════════════════════
#  TUNING  — change values here only (auto-tune can overwrite Kp/Kd)
# ═══════════════════════════════════════════════════════════════════
CRUISE_SPEED   = 150    # top speed on a confirmed straight
MEMORY_SPEED   = 210    # boosted speed on a straight the robot has driven before
MIN_SPEED      = 60     # floor speed mid-pivot / hard brake
Kp             = 24     # proportional gain
Kd             = 14     # derivative gain (damps oscillation)
BRAKE_K        = 0.55   # correction -> extra speed penalty
SMOOTH         = 0.35   # EMA motor smoothing (0.15 silky -> 0.5 snappy)

# Dynamic speed (rolling error window)
DYN_WINDOW     = 25     # ticks averaged for speed decision (25 x 8ms = 0.2 s)
DYN_FAST_ERR   = 0.3    # mean |error| below this -> full speed
DYN_SLOW_ERR   = 2.5    # mean |error| above this -> MIN_SPEED
ACCEL_RATE     = 4.0    # max speed increase per tick (smooth ramp-up)

# Turn prediction
PREDICT_WINDOW = 6      # ticks of error history used to detect a rising trend
PREDICT_SLOPE  = 0.8    # |error| rising faster than this -> a turn is coming
PREDICT_BRAKE  = 35     # extra speed cut when a turn is predicted

# Pivot / turn
PIVOT_FWD      = 125    # outer wheel during pivot
PIVOT_REV      = -75    # inner wheel during pivot (negative = reverse)
PIVOT_LATCH    = 70     # ticks committed to a turn (70 x 8ms ~= 0.56 s)
UTURN_EXTENSION = 40    # extra ticks if latch expires on all-off (completes U-turn)
RECOVERY_LOCK  = 25     # ticks ignoring opposite outer sensor after a turn
BRAKE_PULSE    = 3      # reverse ticks before pivot to kill momentum
BRAKE_SPEED    = -90    # both-wheel speed during the brake pulse
JUNCTION_DIR   = +1     # default cross/T direction: +1 RIGHT, -1 LEFT

# Lost-line recovery
LOST_SPEED     = 95     # in-place spin speed while hunting the line
DEBOUNCE       = 2      # all-off reads before declaring lost

# End-of-run conditions
END_MODE       = "lost"     # "lost" = stop after END_LOST_SEC lost
                            # "finish" = stop on a held finish marker (all-4)
                            # "none" = never auto-stop
END_LOST_SEC   = 3.0    # stop after the tape is gone this long
END_FINISH_SEC = 0.5    # all-4 sensors held this long counts as a finish marker

LOOP_DELAY     = 0.008  # 125 Hz loop

# Debug
DEBUG          = True   # stream live state
DEBUG_EVERY    = 15     # print every Nth tick

# Files
_HERE          = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE  = os.path.join(_HERE, "best_follow_settings.json")
MEMORY_FILE    = os.path.join(_HERE, "best_follow_path.json")
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


def _junction_dir(L1, L2, R1, R2, count):
    if count == 4:       return JUNCTION_DIR
    if not R2 and L1:    return -1   # 1110 left-biased
    if not L1 and R2:    return +1   # 0111 right-biased
    return JUNCTION_DIR


# ═══════════════════════════════════════════════════════════════════
#  SETTINGS PERSISTENCE  (auto-tuned Kp/Kd survive restarts)
# ═══════════════════════════════════════════════════════════════════

def load_settings():
    global Kp, Kd
    try:
        with open(SETTINGS_FILE) as f:
            s = json.load(f)
        Kp = float(s.get("Kp", Kp))
        Kd = float(s.get("Kd", Kd))
        print(f"  loaded tuned settings: Kp={Kp} Kd={Kd}")
    except Exception:
        pass


def save_settings(kp, kd):
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump({"Kp": kp, "Kd": kd}, f, indent=2)
        print(f"  saved settings -> {SETTINGS_FILE}")
    except Exception as e:
        print(f"  could not save settings: {e}")


# ═══════════════════════════════════════════════════════════════════
#  PATH MEMORY
#  Records the lap as a list of segments [(kind, length_ticks), ...]
#  kind: 'S' straight, 'L' left turn, 'R' right turn.
#  On a known lap, boosts speed on long straights and pre-brakes before
#  the turn that memory says is coming.
# ═══════════════════════════════════════════════════════════════════

class PathMemory:
    def __init__(self, enabled=True):
        self.enabled    = enabled
        self.known      = []      # recorded segments from a previous lap
        self.current    = []      # segments being recorded this lap
        self._cur_kind  = None
        self._cur_len   = 0
        self.idx        = 0       # index into self.known we believe we're at
        self.seg_tick   = 0       # ticks spent in the current live segment
        self.localized  = False   # True once live segments match memory
        self._match_run = 0       # consecutive matched segments
        if enabled:
            self._load()

    def _load(self):
        try:
            with open(MEMORY_FILE) as f:
                self.known = [tuple(x) for x in json.load(f).get("segments", [])]
            if self.known:
                print(f"  path memory: loaded {len(self.known)} segments from last run")
        except Exception:
            self.known = []

    def save(self):
        if not self.enabled:
            return
        self._flush()
        # only overwrite memory if this lap looks complete (enough segments)
        if len(self.current) >= 4:
            try:
                with open(MEMORY_FILE, "w") as f:
                    json.dump({"segments": self.current}, f, indent=2)
                print(f"  path memory: saved {len(self.current)} segments")
            except Exception as e:
                print(f"  path memory save failed: {e}")

    def _flush(self):
        if self._cur_kind is not None and self._cur_len > 0:
            self.current.append((self._cur_kind, self._cur_len))
            self._cur_kind = None
            self._cur_len = 0

    def record(self, kind):
        """Feed the current macro-event ('S'/'L'/'R') for this tick."""
        if not self.enabled:
            return
        if kind == self._cur_kind:
            self._cur_len += 1
        else:
            self._flush()
            self._cur_kind = kind
            self._cur_len = 1
            self._advance_localization(kind)
        self.seg_tick = self._cur_len

    def _advance_localization(self, kind):
        """Try to track where we are in the known lap by matching segment kinds."""
        if not self.known:
            return
        # Look for the next segment in memory matching this kind
        nxt = self.idx % len(self.known)
        if self.known[nxt][0] == kind:
            self.idx = nxt + 1
            self._match_run += 1
            if self._match_run >= 2:
                self.localized = True
        else:
            # mismatch -> we lost localization, try to re-find this kind ahead
            self._match_run = 0
            self.localized = False
            for off in range(len(self.known)):
                j = (self.idx + off) % len(self.known)
                if self.known[j][0] == kind:
                    self.idx = j + 1
                    break

    def predicted_straight_len(self):
        """If localized and currently on a known straight, return its recorded
        length (ticks), else None."""
        if not (self.enabled and self.localized and self.known):
            return None
        cur = (self.idx - 1) % len(self.known)
        kind, length = self.known[cur]
        if kind == 'S':
            return length
        return None


# ═══════════════════════════════════════════════════════════════════
#  CONTROLLER
# ═══════════════════════════════════════════════════════════════════

class _Follower:
    def __init__(self, memory=None, collect_metrics=False):
        self.actual_L = 0.0
        self.actual_R = 0.0
        self.last_error = 0.0
        self.lost_ticks = 0
        self.latch_ticks     = 0
        self.latch_dir       = 0
        self.brake_ticks     = 0
        self._latch_extended = False
        self._last_turn_dir  = 0
        self._recov_lock     = 0
        self._dyn_buf  = deque([0.0] * DYN_WINDOW, maxlen=DYN_WINDOW)
        self._pred_buf = deque([0.0] * PREDICT_WINDOW, maxlen=PREDICT_WINDOW)
        self._dyn_speed = float(MIN_SPEED)
        self._tick = 0
        self.mem = memory
        # end-of-run tracking
        self._lost_since   = None
        self._finish_since = None
        self.finished = False
        self.finish_reason = None
        # auto-tune metrics
        self.collect_metrics = collect_metrics
        self.m_abs_err = 0.0
        self.m_flips   = 0
        self.m_samples = 0
        self._prev_sign = 0

    # ── dynamic + predictive speed ────────────────────────────────
    def _update_dyn_speed(self, error, predicted_turn=False, mem_boost=False):
        self._dyn_buf.append(abs(error))
        mean_err = sum(self._dyn_buf) / len(self._dyn_buf)
        t = (mean_err - DYN_FAST_ERR) / max(DYN_SLOW_ERR - DYN_FAST_ERR, 0.001)
        t = max(0.0, min(1.0, t))
        top = MEMORY_SPEED if mem_boost else CRUISE_SPEED
        target = top * (1 - t) + MIN_SPEED * t
        if predicted_turn:
            target -= PREDICT_BRAKE          # pre-slow for an anticipated turn
        target = max(MIN_SPEED, target)
        if target > self._dyn_speed:
            self._dyn_speed = min(target, self._dyn_speed + ACCEL_RATE)
        else:
            self._dyn_speed = target
        return self._dyn_speed

    def _predict_turn(self, error):
        """True if the recent error trend is rising sharply toward a turn."""
        self._pred_buf.append(error)
        if len(self._pred_buf) < PREDICT_WINDOW:
            return False
        buf = list(self._pred_buf)
        slope = (abs(buf[-1]) - abs(buf[0])) / (PREDICT_WINDOW - 1)
        return slope >= PREDICT_SLOPE and abs(buf[-1]) >= E_INNER

    # ── turn management ───────────────────────────────────────────
    def _arm_turn(self, direction):
        self.latch_dir       = direction
        self.latch_ticks     = PIVOT_LATCH
        self._last_turn_dir  = direction
        self._latch_extended = False
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
        count = int(L1) + int(L2) + int(R1) + int(R2)
        bits  = f"{int(L1)}{int(L2)}{int(R1)}{int(R2)}"
        self._tick += 1
        now = time.time()

        # ── END: finish marker (all-4 held) ───────────────────────
        if END_MODE == "finish" and count == 4:
            if self._finish_since is None:
                self._finish_since = now
            elif now - self._finish_since >= END_FINISH_SEC:
                self.finished = True
                self.finish_reason = "finish marker"
                bot.stop()
                return
        else:
            self._finish_since = None

        # ── committed turn: finish before anything else ───────────
        if self.latch_ticks > 0:
            raw = _read_pattern(L1, L2, R1, R2)
            reacquired = (count < 3 and raw is not None and abs(raw) <= E_INNER)
            if reacquired:
                self.latch_ticks     = 0
                self._latch_extended = False
                self._recov_lock     = RECOVERY_LOCK
            else:
                self.latch_ticks -= 1
                if self.latch_ticks == 0 and count == 0 and not self._latch_extended:
                    self.latch_ticks     = UTURN_EXTENSION
                    self._latch_extended = True
                self._drive_turn(bot)
                if self.mem:
                    self.mem.record('L' if self.latch_dir < 0 else 'R')
                self._lost_since = None
                self._debug(bits, float(self.latch_dir) * E_OUTER, int(self._dyn_speed),
                            self.actual_L, self.actual_R,
                            f"TURN({'L' if self.latch_dir < 0 else 'R'}) latch={self.latch_ticks}")
                return

        # ── junction (only when not already turning) ──────────────
        if count >= 3 and self.latch_ticks == 0:
            d = _junction_dir(L1, L2, R1, R2, count)
            self._arm_turn(d)
            self.last_error = float(d) * E_OUTER
            self._drive_turn(bot)
            if self.mem:
                self.mem.record('L' if d < 0 else 'R')
            self._lost_since = None
            self._debug(bits, float(d) * E_OUTER, int(self._dyn_speed),
                        self.actual_L, self.actual_R, f"JUNC({'L' if d < 0 else 'R'})")
            return

        raw = _read_pattern(L1, L2, R1, R2)

        # ── lost line ─────────────────────────────────────────────
        if raw is None:
            self.lost_ticks += 1
            self._update_dyn_speed(E_OUTER)
            if self._lost_since is None:
                self._lost_since = now
            # END: lost too long
            if END_MODE == "lost" and now - self._lost_since >= END_LOST_SEC:
                self.finished = True
                self.finish_reason = f"tape lost {END_LOST_SEC:.0f}s"
                bot.stop()
                return
            if self.lost_ticks < DEBOUNCE:
                self._apply(bot, self.actual_L, self.actual_R)
                self._debug(bits, 0, int(self._dyn_speed), self.actual_L, self.actual_R, "COAST")
                return
            spin_dir = self._last_turn_dir if self._last_turn_dir != 0 else (
                -1 if self.last_error < 0 else 1)
            if spin_dir < 0:
                self._snap(bot, -LOST_SPEED, LOST_SPEED)
            else:
                self._snap(bot, LOST_SPEED, -LOST_SPEED)
            self._recov_lock = RECOVERY_LOCK
            self._debug(bits, float(spin_dir) * E_OUTER, int(self._dyn_speed),
                        self.actual_L, self.actual_R, f"LOST({'L' if spin_dir < 0 else 'R'})")
            return

        # on the line
        self.lost_ticks = 0
        self._lost_since = None
        error = raw

        # ── PD correction ─────────────────────────────────────────
        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = error

        # metrics for auto-tune
        if self.collect_metrics:
            self.m_abs_err += abs(error)
            self.m_samples += 1
            sign = (error > 0) - (error < 0)
            if sign != 0 and self._prev_sign != 0 and sign != self._prev_sign:
                self.m_flips += 1
            if sign != 0:
                self._prev_sign = sign

        # ── sharp corner -> arm latch (with recovery lock) ────────
        if abs(error) >= E_OUTER:
            turn_dir = 1 if error > 0 else -1
            if self._recov_lock > 0 and turn_dir != self._last_turn_dir:
                self._recov_lock -= 1
                error = float(turn_dir) * E_BOTH    # downgrade to curve
            else:
                if self._recov_lock > 0:
                    self._recov_lock -= 1
                self._arm_turn(turn_dir)
                self._update_dyn_speed(E_OUTER)
                self._drive_turn(bot)
                if self.mem:
                    self.mem.record('L' if turn_dir < 0 else 'R')
                self._debug(bits, float(turn_dir) * E_OUTER, int(self._dyn_speed),
                            self.actual_L, self.actual_R,
                            f"CORNER({'R' if turn_dir > 0 else 'L'})")
                return

        # ── normal tracking ───────────────────────────────────────
        if self._recov_lock > 0:
            self._recov_lock -= 1

        predicted = self._predict_turn(error)

        # path-memory: are we on a straight we've driven before?
        mem_boost = False
        if self.mem:
            self.mem.record('S')
            slen = self.mem.predicted_straight_len()
            if slen is not None:
                done = self.mem.seg_tick
                # boost early in a known straight, pre-brake near its known end
                if done < slen - PIVOT_LATCH:
                    mem_boost = True
                else:
                    predicted = True    # turn is coming per memory -> pre-brake

        cruise = self._update_dyn_speed(error, predicted_turn=predicted, mem_boost=mem_boost)
        speed  = max(MIN_SPEED, cruise - abs(correction) * BRAKE_K)

        target_L = speed + correction
        target_R = speed - correction
        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH
        self._apply(bot, self.actual_L, self.actual_R)

        if error == 0:               st = "BOOST" if mem_boost else "STRAIGHT"
        elif abs(error) <= E_INNER:  st = "SLIGHT"
        else:                        st = "CURVE"
        if predicted and not mem_boost:
            st += "*"     # * = turn predicted ahead
        self._debug(bits, error, int(cruise), self.actual_L, self.actual_R, st)

    # ── helpers ───────────────────────────────────────────────────
    def _debug(self, bits, error, speed, L, R, state):
        if not DEBUG or self._tick % DEBUG_EVERY != 0:
            return
        loc = ""
        if self.mem and self.mem.localized:
            loc = " [mem]"
        print(f"[{self._tick:06d}] [{bits}] err:{error:+.1f} spd:{speed:3d} "
              f"L:{int(L):4d} R:{int(R):4d}  {state}{loc}", flush=True)

    def _apply(self, bot, l, r):
        bot._apply_motors(clamp(l), clamp(l), clamp(r), clamp(r))

    def _snap(self, bot, l, r):
        self.actual_L, self.actual_R = float(l), float(r)
        self._apply(bot, l, r)


# ═══════════════════════════════════════════════════════════════════
#  AUTO-TUNE  —  sweep Kp/Kd on a real straight, pick the smoothest
# ═══════════════════════════════════════════════════════════════════

def auto_tune(bot):
    """Drive forward on the tape at several Kp values; pick the one with the
    least wobble while still tracking. Robot must start ON a long straight."""
    global Kp, Kd
    print("=== AUTO-TUNE === place robot on a LONG straight, it will drive ~3s per setting")
    print("    measuring wobble (sign flips) vs tracking error\n")
    test_secs = 3.0
    candidates_kp = [16, 20, 24, 28, 32]
    fixed_kd = 12
    results = []

    for test_kp in candidates_kp:
        Kp, Kd = test_kp, fixed_kd
        f = _Follower(memory=None, collect_metrics=True)
        print(f"  testing Kp={test_kp} Kd={fixed_kd} ...", flush=True)
        t0 = time.time()
        while time.time() - t0 < test_secs:
            f.step(bot)
            time.sleep(LOOP_DELAY)
        bot.stop(); time.sleep(0.4)
        avg_err = f.m_abs_err / max(f.m_samples, 1)
        flips_per_s = f.m_flips / test_secs
        # score: wobble is the enemy, but so is poor tracking
        score = flips_per_s * 1.5 + avg_err * 2.0
        results.append((score, test_kp, flips_per_s, avg_err))
        print(f"     flips/s={flips_per_s:.1f}  avg|err|={avg_err:.2f}  score={score:.2f}")

    results.sort()
    best_score, best_kp, _, _ = results[0]
    # Kd scales with Kp for consistent damping (~0.5 x Kp works well here)
    best_kd = round(best_kp * 0.55)
    print(f"\n  BEST: Kp={best_kp} Kd={best_kd}  (score {best_score:.2f})")
    save_settings(best_kp, best_kd)
    Kp, Kd = best_kp, best_kd
    return best_kp, best_kd


# ═══════════════════════════════════════════════════════════════════
#  ENTRY POINTS
# ═══════════════════════════════════════════════════════════════════

def run(bot, stop_event=None, use_memory=True, **kwargs):
    """Called by drive.py F-key (and standalone main)."""
    load_settings()
    mem = PathMemory(enabled=use_memory)
    f = _Follower(memory=mem)
    try:
        while stop_event is None or not stop_event.is_set():
            f.step(bot)
            if f.finished:
                print(f"\n*** RUN COMPLETE — {f.finish_reason} ***", flush=True)
                break
            time.sleep(LOOP_DELAY)
    finally:
        bot.stop()
        if mem:
            mem.save()


def main():
    args = sys.argv[1:]
    use_memory = "--fresh" not in args

    with RasBot() as bot:
        if "--tune" in args:
            try:
                auto_tune(bot)
            finally:
                bot.stop()
            print("Auto-tune done. Run normally to use the new settings.")
            return

        load_settings()
        print("best_follow (smart) started. Ctrl+C to stop.")
        if DEBUG:
            print(f"  CRUISE={CRUISE_SPEED} MEMORY={MEMORY_SPEED} MIN={MIN_SPEED} "
                  f"Kp={Kp} Kd={Kd}  END_MODE={END_MODE}")
            print("  [tick] [bits] err speed  L    R    state")

        mem = PathMemory(enabled=use_memory)
        f = _Follower(memory=mem)
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
            mem.save()
            print("Motors off.")


if __name__ == "__main__":
    main()
