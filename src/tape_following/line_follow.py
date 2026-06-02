"""
=============================================================================
  Yahboom RASPBOT V2 — Autonomous Line Follower
  Production-quality tape-following: full PID + prediction + watchdog.
=============================================================================

HOW TO RUN
──────────
  Standalone:   python3 src/tape_following/line_follow.py
  Calibrate:    python3 src/tape_following/line_follow.py --calibrate
  From drive.py: import line_follow; line_follow.run(bot, stop_event=evt)

PID TUNING GUIDE
────────────────
  Step 1 — Confirm sensors work: run --calibrate, slide robot
           over tape, verify True=tape / False=floor for all 4 sensors.

  Step 2 — Start conservative: KP_SMALL=5, KP_LARGE=30, KD=0, KI=0,
           BASE_SPEED=35. Robot should follow straight tape without
           oscillating.

  Step 3 — Raise KP_SMALL until tracking is tight.
           ↳ Too high: side-to-side wobble (oscillation) on straights.
           ↳ Too low:  lazy, drifts off gradually.

  Step 4 — Raise KD to damp overshoot on corrections.
           (Safe up to ~0.15 because we use a smoothed derivative.)
           ↳ Too high: jittery, jerky motors, chattering.
           ↳ Too low:  overshoots and slowly oscillates back.

  Step 5 — Leave KI = 0. Raise only if robot drifts persistently to
           one side on a straight for more than 1–2 seconds. Start at 0.01.

  Step 6 — Tune corners: adjust TIGHT_TURN_SPEED and SHARP_CORNER_TREND.
           ↳ TIGHT_TURN_SPEED too low:  misses sharp corners.
           ↳ TIGHT_TURN_SPEED too high: overshoots pivot, lands past tape.
           ↳ SHARP_CORNER_TREND too low:  pivot fires on smooth curves → spins off.
           ↳ SHARP_CORNER_TREND too high: pivot doesn't fire on 90° corners → flies off.

SYMPTOM → CAUSE → FIX
──────────────────────
  Wobbles on straight         → KP_SMALL too high     → lower KP_SMALL
  Drifts off gradually        → KP_SMALL too low      → raise KP_SMALL
  Overshoots curves           → KD too low            → raise KD
  Jittery / jerky             → KD too high           → lower KD
  Misses 90° corners          → SHARP_CORNER_TREND too high or
                                 TIGHT_TURN_SPEED too low
  Spins off smooth curves     → SHARP_CORNER_TREND too low
  Frequent false MISSes       → MISS_CREEP_TICKS too low or BASE_SPEED too high
  Robot stops, won't recover  → SEARCH_TIMEOUT_S too low
=============================================================================
"""

import os
import sys
import time
import threading
import random
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color


# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURABLE CONSTANTS — tune here, not inside the classes
# ═══════════════════════════════════════════════════════════════════════════════

# ── speeds (0-255) ────────────────────────────────────────────────────────────
BASE_SPEED          = 40    # cruise speed on centred / straight tape
MIN_SPEED           = 15    # floor — below this motors stall / skip
MAX_SPEED           = 255
TIGHT_TURN_SPEED    = 15    # in-place pivot speed for sharp corners

# Ramp rate: max speed-units change per loop tick.
# At 50 Hz, RAMP_RATE=4 → 0→40 in 10 ticks (200 ms).
RAMP_RATE           = 4

# ── PID gains ─────────────────────────────────────────────────────────────────
# KD is safe up to ~0.15 because we use a smoothed (multi-tick) derivative.
# With a 1-tick derivative, KD must stay ≤ 0.05 to avoid sensor-flip spikes.
KP_SMALL            = 7.0   # P-gain for |error| ≤ 2  (centre tracking)
KP_LARGE            = 40.0  # P-gain for |error| ≥ 3  (sharp-turn response)
KI                  = 0.0   # I-gain — leave 0 unless persistent side drift
KD                  = 0.10  # D-gain (applied to smoothed derivative)
MAX_INTEGRAL        = 20.0  # anti-windup clamp for integrator
MAX_CORRECTION      = 22    # PID output clamp — keeps wheel ratio ≤ ~3:1
CORRECTION_SCALE    = 0.30  # cruise-speed reduction per unit of |correction|

# ── prediction / history ──────────────────────────────────────────────────────
HISTORY_SIZE        = 10    # rolling buffer length  (= 200 ms at 50 Hz)
DERIV_WINDOW        = 4     # ticks for smoothed derivative  (= 80 ms)
TREND_THRESHOLD     = 1.0   # |trend| above this → curve-entry penalty fires
SPEED_PENALTY_MAX   = 8     # max extra speed reduction from prediction
SPEED_PENALTY_K     = 5.0   # penalty = min(MAX, (|trend| - threshold) × K)

# SHARP_CORNER_TREND: |trend| above this  AND  |error| ≥ 3  → use pivot.
# Lower value = more corners trigger pivot (safer for sharp tracks).
# Higher value = only very sudden corners trigger pivot (safer for smooth tracks).
SHARP_CORNER_TREND  = 1.5

# ── loop timing ───────────────────────────────────────────────────────────────
LOOP_HZ             = 50
LOOP_DELAY          = 1.0 / LOOP_HZ   # 20 ms target tick period

# ── debug output ──────────────────────────────────────────────────────────────
DEBUG               = True  # False = silent
DEBUG_EVERY         = 5     # print every N ticks to keep terminal readable

# ── MISS / recovery ───────────────────────────────────────────────────────────
MISS_CREEP_TICKS    = 15    # ticks of slow creep before spin-search (~300 ms)
MISS_CREEP_SPEED    = 18
SEARCH_SPEED        = 18    # spin speed during recovery
SEARCH_TIMEOUT_S    = 4.0   # total recovery time (split evenly per direction)
RECOVER_SETTLE_S    = 0.15  # pause after tape re-acquired before resuming PID

# ── junction / stop marker ────────────────────────────────────────────────────
JUNCTION_SPIN_SPEED = 20
JUNCTION_TIMEOUT_S  = 2.0
STOP_DEBOUNCE_S     = 2.0
CLEAR_MARKER_S      = 0.4
SKIP_SCAN           = True   # True = 1 s pause at stop marker; False = 360° scan

# ── camera (only when SKIP_SCAN=False) ───────────────────────────────────────
RED_PIXEL_THRESHOLD = 3000
RED_CHECK_INTERVAL  = 0.12

# ── watchdog ──────────────────────────────────────────────────────────────────
WATCHDOG_TIMEOUT_S  = 0.5   # stop motors if main loop freezes longer than this

# ═══════════════════════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════════════════════
#  DATA TYPE
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class SensorReading:
    """One snapshot of the 4-channel IR sensor."""
    lo: bool; li: bool; ri: bool; ro: bool
    all_on: bool
    all_off: bool
    error: Optional[float]   # None when all_off; range [-4, +4] otherwise

    @property
    def bits(self) -> str:
        return f"{int(self.lo)}{int(self.li)}{int(self.ri)}{int(self.ro)}"

    @property
    def state_label(self) -> str:
        """Human-readable name for the current sensor pattern."""
        if self.all_off: return "LOST"
        if self.all_on:  return "JUNCTION"
        return {
            "0001": "FAR-RIGHT(pivot←)", "0010": "R-INNER",   "0011": "BOTH-RIGHT",
            "0100": "L-INNER",           "0101": "LI+RO",     "0110": "CENTRED",
            "0111": "3-RIGHT-curve",     "1000": "FAR-LEFT(pivot→)",
            "1001": "LO+RO",             "1010": "LO+RI",     "1011": "LO+RI+RO",
            "1100": "BOTH-LEFT",         "1101": "LO+LI+RO",  "1110": "3-LEFT-curve",
        }.get(self.bits, self.bits)


# ═══════════════════════════════════════════════════════════════════════════════
#  1. SENSOR READER
# ═══════════════════════════════════════════════════════════════════════════════

class SensorReader:
    """Reads the 4-channel IR line sensor and computes a weighted position error.

    Sensor layout (looking down at the floor from above):
        [L_OUT][L_IN] | [R_IN][R_OUT]
        True  = sensor is over black tape
        False = sensor is over white floor

    Error weights: -3  -1  +1  +3
        Negative error → robot is to the right of tape centre → needs left correction
        Positive error → robot is to the left  of tape centre → needs right correction
        Zero           → centred (0110 pattern)
        None           → line lost (0000 pattern)
    """

    def read(self, bot) -> SensorReading:
        try:
            lo, li, ri, ro = bot.read_line_sensors()
        except Exception:
            lo, li, ri, ro = False, False, False, False

        all_on  = bool(lo and li and ri and ro)
        all_off = not (lo or li or ri or ro)
        error   = None if all_off else float((-3 * lo) + (-1 * li) + (1 * ri) + (3 * ro))
        return SensorReading(lo=bool(lo), li=bool(li), ri=bool(ri), ro=bool(ro),
                             all_on=all_on, all_off=all_off, error=error)


# ═══════════════════════════════════════════════════════════════════════════════
#  2. PID CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class PIDController:
    """Full P + I + D controller with dual proportional gain.

    Dual-KP:
        KP_SMALL for |error| < 3  → smooth centre tracking
        KP_LARGE for |error| ≥ 3  → fast response to sharp turns

    Smoothed derivative:
        Accepts a deriv_override from PredictionEngine.
        This is a multi-tick smoothed value that avoids binary-sensor spikes
        at 50 Hz (1-tick spike = 150 units/s; 4-tick smoothed = 37.5 units/s).
    """

    def __init__(self) -> None:
        self._integral: float = 0.0
        self._last_err: float = 0.0
        self._last_t: Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def compute(self, error: float,
                deriv_override: Optional[float] = None) -> float:
        """Return PID correction for the given error.

        Args:
            error:          weighted sensor position error in [-4, +4]
            deriv_override: smoothed derivative from PredictionEngine (preferred)
        """
        now = time.time()
        dt  = (now - self._last_t) if self._last_t is not None else LOOP_DELAY
        dt  = max(dt, 1e-3)
        self._last_t = now

        # Integrator with anti-windup
        self._integral = max(-MAX_INTEGRAL,
                             min(MAX_INTEGRAL, self._integral + error * dt))

        derivative = (deriv_override if deriv_override is not None
                      else (error - self._last_err) / dt)
        self._last_err = error

        kp = KP_LARGE if abs(error) >= 3 else KP_SMALL
        return kp * error + KI * self._integral + KD * derivative


# ═══════════════════════════════════════════════════════════════════════════════
#  3. PREDICTION ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

class PredictionEngine:
    """Rolling history buffer for curve prediction and smooth derivative.

    Signals provided each tick:
        smoothed_deriv()   — derivative over DERIV_WINDOW ticks (less spiky than 1-tick)
        trend()            — slope of error over full window (+ = heading right)
        weighted_trend()   — same but recent readings count more
        speed_penalty()    — speed reduction to pre-slow for an upcoming curve
        is_sharp_corner()  — True when a sudden 90° corner is detected
    """

    def __init__(self) -> None:
        self._buf: deque = deque([0.0] * HISTORY_SIZE, maxlen=HISTORY_SIZE)

    def push(self, error: float) -> None:
        self._buf.append(float(error))

    def smoothed_deriv(self) -> float:
        """(err[now] - err[now - DERIV_WINDOW]) / (DERIV_WINDOW × dt).

        Smoothing over 4 ticks reduces instantaneous spikes by 4×,
        letting KD be 0.10 instead of the usual 0.05 limit.
        """
        buf = list(self._buf)
        if len(buf) < DERIV_WINDOW + 1:
            return 0.0
        return (buf[-1] - buf[-(DERIV_WINDOW + 1)]) / (DERIV_WINDOW * LOOP_DELAY)

    def trend(self) -> float:
        """Compare mean of second half of window vs first half.

        Positive → error is rising  (robot heading toward right side of tape)
        Negative → error is falling (robot heading toward left  side of tape)
        """
        buf = list(self._buf)
        n   = len(buf)
        if n < 4:
            return 0.0
        mid    = n // 2
        first  = sum(buf[:mid]) / mid
        second = sum(buf[mid:]) / (n - mid)
        return second - first

    def weighted_trend(self) -> float:
        """Trend with linearly increasing weights (newest = highest weight).

        Reacts faster to recent direction changes than plain trend().
        Used for pre-steering: start correcting before the error grows.
        """
        buf     = list(self._buf)
        n       = len(buf)
        if n < 2:
            return 0.0
        weights = list(range(1, n + 1))   # [1, 2, ..., N]
        total_w = sum(weights)
        wmean   = sum(w * v for w, v in zip(weights, buf)) / total_w
        simple  = sum(buf) / n
        return wmean - simple   # bias of weighted vs unweighted mean

    def speed_penalty(self) -> float:
        """How much to subtract from BASE_SPEED when entering a curve.

        Fires when |trend| > TREND_THRESHOLD, scaling linearly above that.
        Example: trend=1.8, threshold=1.0, K=5 → penalty = min(8, 4.0) = 4.0
        The robot slows BEFORE reaching peak error — not after.
        """
        t = abs(self.trend())
        if t <= TREND_THRESHOLD:
            return 0.0
        return min(SPEED_PENALTY_MAX, (t - TREND_THRESHOLD) * SPEED_PENALTY_K)

    def is_sharp_corner(self, error: float) -> bool:
        """Detect a sudden 90° corner (as opposed to a gradual curve).

        A sharp corner shows up as a large, fast jump in trend combined
        with |error| ≥ 3. Gradual curves ramp up slowly so their trend
        stays below SHARP_CORNER_TREND even at the same error magnitude.
        """
        return abs(error) >= 3 and abs(self.trend()) > SHARP_CORNER_TREND

    def reset(self) -> None:
        self._buf = deque([0.0] * HISTORY_SIZE, maxlen=HISTORY_SIZE)


# ═══════════════════════════════════════════════════════════════════════════════
#  4. MOTOR CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class MotorController:
    """Applies motor commands with optional speed ramping.

    All wheel commands go through here so acceleration is limited —
    no sudden full-speed jumps that could cause wheel slip or overshoot.
    Pivot bypasses ramping so the robot responds immediately to corners.
    """

    def __init__(self) -> None:
        self._cur_L: float = 0.0
        self._cur_R: float = 0.0

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> int:
        return int(max(lo, min(hi, v)))

    def _ramp(self, current: float, target: float) -> float:
        """Limit speed change to RAMP_RATE per tick to smooth acceleration."""
        delta = target - current
        if abs(delta) <= RAMP_RATE:
            return float(target)
        return current + RAMP_RATE * (1.0 if delta > 0 else -1.0)

    def apply(self, bot, left: float, right: float,
              ramp: bool = True) -> Tuple[int, int]:
        """Send left/right wheel speeds to the robot."""
        if ramp:
            left  = self._ramp(self._cur_L, left)
            right = self._ramp(self._cur_R, right)
        L = self._clamp(left,  MIN_SPEED, MAX_SPEED)
        R = self._clamp(right, MIN_SPEED, MAX_SPEED)
        bot._apply_motors(L, L, R, R)
        self._cur_L, self._cur_R = float(L), float(R)
        return L, R

    def apply_correction(self, bot, correction: float,
                         speed_penalty: float = 0.0) -> Tuple[int, int]:
        """Differential drive from a PID correction value.

        correction > 0 → steer right  (left wheels faster)
        correction < 0 → steer left   (right wheels faster)

        speed_penalty reduces cruise speed to pre-slow before curves
        but does not change the correction magnitude (direction stays correct).
        """
        c      = self._clamp(correction, -MAX_CORRECTION, MAX_CORRECTION)
        cruise = self._clamp(
            BASE_SPEED - abs(c) * CORRECTION_SCALE - speed_penalty,
            MIN_SPEED + 5, MAX_SPEED,
        )
        return self.apply(bot, float(cruise + c), float(cruise - c))

    def pivot(self, bot, direction: int) -> Tuple[int, int]:
        """In-place rotation (bypasses ramp for instant response).

        direction > 0 = rotate right,  direction < 0 = rotate left.
        Used for: single-outer-sensor states AND sharp-corner prediction.
        """
        if direction > 0:
            bot.rotate_right(TIGHT_TURN_SPEED)
        else:
            bot.rotate_left(TIGHT_TURN_SPEED)
        spd = TIGHT_TURN_SPEED
        self._cur_L = float( spd if direction > 0 else -spd)
        self._cur_R = float(-spd if direction > 0 else  spd)
        return (spd if direction > 0 else -spd,
                -spd if direction > 0 else spd)

    def stop(self, bot) -> None:
        bot.stop()
        self._cur_L = 0.0
        self._cur_R = 0.0

    def reset(self) -> None:
        self._cur_L = 0.0
        self._cur_R = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  5. WATCHDOG
# ═══════════════════════════════════════════════════════════════════════════════

class Watchdog:
    """Background safety thread — stops motors if the main loop freezes.

    The main loop calls kick() every tick. If more than WATCHDOG_TIMEOUT_S
    passes without a kick, the watchdog calls bot.stop() automatically.
    This prevents the robot from driving uncontrolled if the Pi hangs.
    """

    def __init__(self) -> None:
        self._last_kick: float = time.time()
        self._bot = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self, bot) -> None:
        self._bot       = bot
        self._running   = True
        self._last_kick = time.time()
        self._thread    = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def kick(self) -> None:
        """Call once per main-loop tick to reset the watchdog timer."""
        self._last_kick = time.time()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        while self._running:
            time.sleep(0.05)   # check every 50 ms
            if time.time() - self._last_kick > WATCHDOG_TIMEOUT_S:
                if self._bot is not None:
                    try:
                        self._bot.stop()
                    except Exception:
                        pass


# ═══════════════════════════════════════════════════════════════════════════════
#  6. LINE FOLLOWER  (main state machine)
# ═══════════════════════════════════════════════════════════════════════════════

class LineFollower:
    """Autonomous line-following controller — ties all components together.

    States:
        STRAIGHT      → 0110, err≈0, full cruise speed
        TURNING       → non-zero error, PID differential drive
        PIVOT         → single outer sensor, in-place rotation
        CORNER-PIVOT  → |err|≥3 AND steep trend, in-place rotation
        LOST/creep    → all sensors off, creeping with last-known heading
        RECOVERY      → spin-searching for tape
        JUNCTION      → all sensors on, navigating T-junction
    """

    def __init__(self) -> None:
        self.sensors  = SensorReader()
        self.pid      = PIDController()
        self.pred     = PredictionEngine()
        self.motors   = MotorController()
        self.watchdog = Watchdog()

    def run(self, bot, cam=None,
            log=None,
            stop_event: Optional[threading.Event] = None) -> None:

        if log is None:
            log = lambda msg: print(f"\n{msg}", flush=True)

        log("LineFollower started  (P+I+D + prediction + watchdog)")
        log(f"BASE={BASE_SPEED}  KP_S={KP_SMALL}  KP_L={KP_LARGE}  KD={KD}  "
            f"SHARP_TREND={SHARP_CORNER_TREND}  RAMP={RAMP_RATE}")

        red_detector = None
        if not SKIP_SCAN:
            try:
                red_detector = RedTapeDetector()
                log("Red-tape camera detector started.")
            except Exception as e:
                log(f"Camera detector failed ({e}) — IR-only mode.")

        # ── wait for tape ─────────────────────────────────────────────────
        log("Waiting for tape... place robot on tape (LEDs = yellow)")
        bot.set_all_leds_color(Color.YELLOW)
        while True:
            if stop_event and stop_event.is_set():
                return
            r = self.sensors.read(bot)
            if not r.all_off:
                break
            time.sleep(0.05)
        log("Tape detected! Starting in 1 s...")
        time.sleep(1.0)
        bot.set_all_leds_color(Color.GREEN)

        self.watchdog.start(bot)

        # ── loop state ────────────────────────────────────────────────────
        last_error:     float = 0.0
        debounce_until: float = 0.0
        miss_count:     int   = 0
        in_recovery:    bool  = False
        just_scanned:   bool  = False
        tick:           int   = 0
        last_L:         int   = BASE_SPEED
        last_R:         int   = BASE_SPEED

        try:
            while True:
                t0 = time.time()

                if stop_event and stop_event.is_set():
                    break

                self.watchdog.kick()   # reset watchdog timer every tick

                r   = self.sensors.read(bot)
                now = time.time()

                # ── JUNCTION: all 4 sensors on ────────────────────────────
                if r.all_on:
                    if DEBUG and tick % DEBUG_EVERY == 0:
                        print(f"[{tick:06d}] {r.bits}  JUNCTION", flush=True)
                    if not self._navigate_junction(bot, log):
                        break
                    last_error = 0.0; miss_count = 0
                    self.pred.reset(); self.pid.reset()
                    tick += 1; continue

                # ── STOP MARKER: red tape seen by camera ──────────────────
                if (red_detector is not None
                        and red_detector.check()
                        and now >= debounce_until):
                    self.motors.stop(bot)
                    self.pid.reset(); self.pred.reset()
                    miss_count = 0; in_recovery = False; just_scanned = True
                    self._do_scan(bot, cam, log)
                    bot.forward(BASE_SPEED)
                    time.sleep(CLEAR_MARKER_S)
                    self.motors.stop(bot)
                    debounce_until = now + STOP_DEBOUNCE_S
                    last_error = 0.0
                    bot.set_all_leds_color(Color.GREEN)
                    tick += 1; continue

                # ── LOST: all sensors off ─────────────────────────────────
                if r.all_off:
                    if not in_recovery:
                        miss_count += 1
                        self.pred.push(last_error)  # keep history trending last direction

                    if miss_count <= MISS_CREEP_TICKS:
                        # Phase A — creep forward with last-known turn bias.
                        # Keeps momentum toward where the tape was.
                        corr  = max(-MAX_CORRECTION,
                                    min(MAX_CORRECTION,
                                        int(KP_SMALL * last_error * 0.4)))
                        last_L, last_R = self.motors.apply(
                            bot,
                            float(MISS_CREEP_SPEED + corr),
                            float(MISS_CREEP_SPEED - corr),
                            ramp=False,
                        )
                        if DEBUG and tick % DEBUG_EVERY == 0:
                            print(f"[{tick:06d}] {r.bits}"
                                  f"  MISS×{miss_count:02d}  creep"
                                  f"  L={last_L:3d} R={last_R:3d}", flush=True)
                        self._sleep_remainder(t0)
                        tick += 1; continue

                    # Phase B — confirmed lost: stop and spin-search.
                    if not in_recovery:
                        in_recovery = True
                        self.motors.stop(bot)
                        if just_scanned:
                            log("Tape ended after scan — end of path. Stopping.")
                            bot.beep(0.3)
                            bot.set_all_leds_color(Color.RED)
                            break
                        just_scanned = False
                        if not self._recover(bot, last_error, log):
                            break
                        miss_count  = 0
                        in_recovery = False
                        last_error  = 0.0
                    else:
                        time.sleep(LOOP_DELAY)
                    tick += 1; continue

                # ── NORMAL TRACKING ───────────────────────────────────────
                miss_count   = 0
                in_recovery  = False
                just_scanned = False
                last_error   = r.error  # not None here (all_off already handled)

                # Update prediction history
                self.pred.push(r.error)
                d_smooth  = self.pred.smoothed_deriv()   # smoothed D term
                penalty   = self.pred.speed_penalty()    # predictive slow-down
                trend_val = self.pred.trend()

                # Compute full PID with smoothed derivative
                correction   = self.pid.compute(r.error, deriv_override=d_smooth)
                corr_clamped = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

                # ── pivot decision ────────────────────────────────────────
                # Always pivot for single-outer states (1000 / 0001):
                #   robot is at the tape edge, forward motion overshoots.
                # Also pivot for any |err|≥3 state when trend is STEEP:
                #   steep trend = sudden 90° corner, not a gradual curve.
                #   Gradual curves (low trend) still use differential drive.
                _single_outer = ((r.lo and not r.li and not r.ri and not r.ro) or
                                 (r.ro and not r.lo and not r.li and not r.ri))
                _sharp_corner = self.pred.is_sharp_corner(r.error)
                use_pivot     = _single_outer or _sharp_corner

                direction = 1 if corr_clamped > 0 else -1

                if use_pivot:
                    last_L, last_R = self.motors.pivot(bot, direction)
                    if _sharp_corner and not _single_outer:
                        state = f"CORNER-PIVOT-{'R' if direction > 0 else 'L'}"
                    else:
                        state = f"PIVOT-{'R' if direction > 0 else 'L'}"
                else:
                    last_L, last_R = self.motors.apply_correction(
                        bot, correction, speed_penalty=penalty
                    )
                    state = "STRAIGHT" if abs(r.error) < 1 else "TURNING"

                # ── debug output ──────────────────────────────────────────
                if DEBUG and tick % DEBUG_EVERY == 0:
                    print(
                        f"[{tick:06d}]"
                        f" {r.bits}"
                        f"  err={r.error:+.0f}"
                        f"  corr={corr_clamped:+.0f}"
                        f"  L={last_L:4d} R={last_R:4d}"
                        f"  {state:<22}"
                        f"  trnd={trend_val:+.1f}"
                        f"  pen={penalty:.1f}"
                        f"  [{r.state_label}]",
                        flush=True,
                    )

                self._sleep_remainder(t0)
                tick += 1

        finally:
            self.watchdog.stop()
            if red_detector:
                red_detector.close()

    # ── internal helpers ──────────────────────────────────────────────────────

    def _sleep_remainder(self, t0: float) -> None:
        """Sleep the unused portion of this tick to maintain LOOP_HZ."""
        remaining = LOOP_DELAY - (time.time() - t0)
        if remaining > 0:
            time.sleep(remaining)

    def _recover(self, bot, last_error: float, log) -> bool:
        """Spin-search for tape. Returns True if tape is re-acquired.

        Tries the last-known direction first (bias from last_error sign),
        then the opposite direction if the first half times out.
        """
        log("Line lost — searching...")
        bot.set_all_leds_color(Color.YELLOW)

        start_left = (last_error <= 0)  # last drifted left → search left first
        half       = SEARCH_TIMEOUT_S / 2.0

        for spin_left in [start_left, not start_left]:
            label    = "left" if spin_left else "right"
            deadline = time.time() + half

            while time.time() < deadline:
                r = self.sensors.read(bot)
                if r.li or r.ri:
                    # Inner sensor found tape — creep forward to centre on it
                    self.motors.stop(bot);  time.sleep(0.1)
                    bot.forward(MISS_CREEP_SPEED); time.sleep(0.15)
                    self.motors.stop(bot);  time.sleep(RECOVER_SETTLE_S)
                    self.pid.reset(); self.pred.reset()
                    log(f"Line re-acquired (spun {label})")
                    bot.set_all_leds_color(Color.GREEN)
                    return True
                if spin_left:
                    bot.rotate_left(SEARCH_SPEED)
                else:
                    bot.rotate_right(SEARCH_SPEED)
                time.sleep(LOOP_DELAY)

            self.motors.stop(bot); time.sleep(0.1)

        log("Could not re-acquire line — stopping.")
        bot.set_all_leds_color(Color.RED)
        return False

    def _navigate_junction(self, bot, log) -> bool:
        """Handle a T-junction (all 4 sensors on tape simultaneously).

        Randomly picks a direction, spins off the junction bar, then
        continues spinning until an inner sensor finds the branch tape.
        Returns True if a branch was found.
        """
        self.motors.stop(bot); time.sleep(0.1)
        bot.set_all_leds_color(Color.YELLOW)

        directions = [(bot.rotate_left, "left"), (bot.rotate_right, "right")]
        random.shuffle(directions)

        for spin_fn, label in directions:
            log(f"Junction → trying {label}")
            deadline = time.time() + JUNCTION_TIMEOUT_S

            # Phase 1: spin until we exit the wide junction bar
            while time.time() < deadline:
                r = self.sensors.read(bot)
                if not r.all_on:
                    break
                spin_fn(JUNCTION_SPIN_SPEED)
                time.sleep(LOOP_DELAY)
            self.motors.stop(bot); time.sleep(0.05)

            # Phase 2: keep spinning until an inner sensor finds the branch
            while time.time() < deadline:
                r = self.sensors.read(bot)
                if r.li or r.ri:
                    self.motors.stop(bot); time.sleep(0.1)
                    bot.forward(MISS_CREEP_SPEED); time.sleep(0.2)
                    self.motors.stop(bot); time.sleep(0.1)
                    self.pid.reset(); self.pred.reset()
                    log(f"Junction → branch found ({label})")
                    bot.set_all_leds_color(Color.GREEN)
                    return True
                spin_fn(JUNCTION_SPIN_SPEED)
                time.sleep(LOOP_DELAY)

            self.motors.stop(bot); time.sleep(0.15)

        log("Junction → no branch found")
        bot.set_all_leds_color(Color.RED)
        return False

    def _do_scan(self, bot, cam, log) -> None:
        bot.set_all_leds_color(Color.BLUE)
        bot.beep(0.1)
        if SKIP_SCAN:
            log("Stop marker — 1 s pause (SKIP_SCAN=True)")
            time.sleep(1.0)
            return
        if cam is None:
            log("Scan skipped: no camera object")
            return
        import traceback
        from pointcloud import scan360
        try:
            session, _ = scan360.scan_and_build(bot, cam, log=log)
            log(f"Scan complete: {os.path.basename(session)}")
            bot.beep(0.15)
        except Exception:
            log(f"Scan error:\n{traceback.format_exc()}")


# ═══════════════════════════════════════════════════════════════════════════════
#  CAMERA STOP-MARKER DETECTOR  (only used when SKIP_SCAN=False)
# ═══════════════════════════════════════════════════════════════════════════════

class RedTapeDetector:
    """Background thread — detects red tape in camera frames."""

    def __init__(self) -> None:
        import numpy as np          # Pi-only deps — lazy import so laptop IDE stays clean
        import cv2                  # noqa: F401  (cv2 not resolvable on laptop, fine on Pi)
        self._np  = np
        self._cv2 = cv2
        self._L1  = np.array([0,   120,  70], dtype=np.uint8)
        self._U1  = np.array([10,  255, 255], dtype=np.uint8)
        self._L2  = np.array([165, 120,  70], dtype=np.uint8)
        self._U2  = np.array([180, 255, 255], dtype=np.uint8)
        self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self._hit     = False
        self._lock    = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def check(self) -> bool:
        with self._lock:
            v = self._hit; self._hit = False
        return v

    def close(self) -> None:
        self._running = False
        self._cap.release()

    def _loop(self) -> None:
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                h     = frame.shape[0]
                floor = self._cv2.cvtColor(frame[h // 2:], self._cv2.COLOR_BGR2HSV)
                mask  = self._cv2.bitwise_or(
                    self._cv2.inRange(floor, self._L1, self._U1),
                    self._cv2.inRange(floor, self._L2, self._U2),
                )
                if self._cv2.countNonZero(mask) >= RED_PIXEL_THRESHOLD:
                    with self._lock:
                        self._hit = True
            time.sleep(RED_CHECK_INTERVAL)


# ═══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION MODE
# ═══════════════════════════════════════════════════════════════════════════════

def calibrate() -> None:
    """Live sensor readout — verify True=tape / False=floor for all 4 sensors."""
    print("=== Calibration — Ctrl-C to exit ===")
    print("Slide robot over tape. All 4 sensors should read True over black tape.")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}  {'err':>5}  state")
    print("-" * 62)
    reader = SensorReader()
    with RasBot() as bot:
        try:
            while True:
                r     = reader.read(bot)
                err_s = f"{r.error:+.0f}" if r.error is not None else " —"
                print(
                    f"{str(r.lo):>8}  {str(r.li):>6}  {str(r.ri):>6}  {str(r.ro):>7}"
                    f"  {err_s:>5}  {r.state_label}          ",
                    end="\r", flush=True,
                )
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nCalibration done.")


# ═══════════════════════════════════════════════════════════════════════════════
#  COMPAT WRAPPER  — keeps drive.py working without changes
# ═══════════════════════════════════════════════════════════════════════════════

def run(bot, cam=None, log=None,
        stop_event: Optional[threading.Event] = None) -> None:
    """Drop-in compatible with the drive.py F-key line-follow toggle."""
    LineFollower().run(bot, cam=cam, log=log, stop_event=stop_event)


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "--calibrate":
        calibrate()
        return

    cam = None
    if not SKIP_SCAN:
        from camera.rs_capture import StereoCapture
        cam = StereoCapture()

    with RasBot() as bot:
        try:
            run(bot, cam)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            if cam:
                cam.close()
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            print("Motors stopped. Safe.")


if __name__ == "__main__":
    main()
