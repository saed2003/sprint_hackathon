"""
=============================================================================
  Yahboom RASPBOT V2 — Line Follower  (U-Turn Edition)
  State machine + per-state PID + predictive speed zones + smooth motors
=============================================================================

HOW TO RUN
──────────
  python3 src/tape_following/line_follow.py           ← standalone
  python3 src/tape_following/line_follow.py --calibrate
  From drive.py F-key: line_follow.run(bot, stop_event=evt)

═══════════════════════════════════════════════════════════════════
  TUNING GUIDE
═══════════════════════════════════════════════════════════════════

1. U-TURN PARAMETERS
   ─────────────────
   UTURN_SPEED (default 18):
     Too low  → robot barely moves, stalls in U-turn
     Too high → overshoots, misses tape on exit
     Test: watch the UTURN state in debug. Robot should complete
           a 180° turn in ~1-2 seconds.

   UTURN_TIMEOUT_S (default 3.0):
     Increase if your U-turns are very tight and slow.
     Decrease if you want faster failure detection.

   HISTORY_SIZE (default 15):
     More samples → smoother curve classification but slower reaction.
     Fewer samples → faster reaction but noisier state transitions.

2. SMOOTH_FACTOR
   ─────────────
   Controls motor interpolation: motor += (target - motor) × SMOOTH_FACTOR
     0.1 → very slow, silky smooth (good for carpet/rough surfaces)
     0.3 → balanced (default, good for smooth floors)
     0.5 → fast response, slightly jerky
   Increase on slippery floors. Decrease on rough surfaces.

3. PER-STATE PID TUNING
   ─────────────────────
   PID_STRAIGHT (KP_S, KI_S, KD_S):
     Tune first. Robot should hold centre on a straight.
     Wobbles → lower KP_S. Drifts → raise KP_S.

   PID_CURVE (KP_C, KI_C, KD_C):
     Higher KP needed to track curves. KD prevents overshoot.
     Oscillates through curve → lower KP_C or raise KD_C.

   PID_UTURN (KP_U, KI_U, KD_U):
     Used only during UTURN pivot. KP drives pivot intensity.
     Usually leave at defaults.

4. TESTING EACH STATE
   ───────────────────
   STRAIGHT: place robot on a long straight. Should hold 0110 pattern.
   CURVE:    place on a gentle curve. Should see smooth 0111/1110 tracking.
   SHARP:    place on a tight corner. Should see CORNER-PIVOT fire.
   UTURN:    hold robot at end of U-turn strip. Should pivot in correct dir.
   RECOVERY: lift robot off tape. Should probe then spin-search.

SYMPTOM → CAUSE → FIX
──────────────────────
  Misses U-turns          → UTURN_SPEED too high or CURVE_SPEED too high
  Spins wrong way at U    → last_turn_dir not updating (check sensor polarity)
  Wobbly on straight      → KP_S too high or SMOOTH_FACTOR too low
  Jerky at corners        → SMOOTH_FACTOR too low or SHARP_SPEED too high
  Slow to enter curves    → SHARP_CORNER_TREND too high
  Over-pivots on corners  → CORNER_LATCH_TICKS too high or UTURN_SPEED too high
=============================================================================
"""

import os
import sys
import time
import threading
import random
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple
from enum import Enum

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color


# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURABLE CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════

# ── speed zones (0-255) ───────────────────────────────────────────────────────
BASE_SPEED    = 35    # STRAIGHT state — full cruise speed
CURVE_SPEED   = 28    # CURVE state   — gentle curve (|err|=1-2)
SHARP_SPEED   = 22    # SHARP state   — tight curve  (|err|=3-4, differential)
UTURN_SPEED   = 18    # UTURN state   — pivot speed  (in-place rotation)
MIN_SPEED     = 15    # hard floor — motors stall below this
MAX_SPEED     = 255

# Motor interpolation: motor += (target - motor) × SMOOTH_FACTOR
# 0.1=silky, 0.3=balanced, 0.5=snappy
SMOOTH_FACTOR = 0.35

# Speed ramp rate (units/tick, only used for large jumps)
RAMP_RATE     = 5

# ── per-state PID gains ───────────────────────────────────────────────────────
# Derivative uses smoothed multi-tick window — safe up to ~0.15
KP_S, KI_S, KD_S = 7.0, 0.0, 0.10   # STRAIGHT — gentle centre tracking
KP_C, KI_C, KD_C = 9.0, 0.0, 0.10   # CURVE    — stronger correction
KP_U, KI_U, KD_U = 5.0, 0.0, 0.05   # UTURN    — pivot control (smooth)
MAX_INTEGRAL      = 20.0
MAX_CORRECTION    = 22

# ── prediction / history ──────────────────────────────────────────────────────
HISTORY_SIZE       = 15   # sensor-error history length (= 300 ms at 50 Hz)
DERIV_WINDOW       = 4    # ticks for smoothed derivative (80 ms)
TREND_THRESHOLD    = 1.0  # |trend| above this → speed penalty fires
SPEED_PENALTY_MAX  = 10   # max speed reduction from prediction
SPEED_PENALTY_K    = 5.0
SHARP_CORNER_TREND = 1.0  # |trend| + |err|≥3 → corner pivot + latch
CORNER_LATCH_TICKS = 4    # minimum ticks to hold pivot after corner detected

# ── state machine ─────────────────────────────────────────────────────────────
UTURN_TIMEOUT_S    = 3.0  # give up U-turn search after this long
RECOVERY_TIMEOUT_S = 4.0  # give up recovery (straight-loss) after this long
UTURN_PROBE_S      = 0.5  # how long to probe each direction at U-turn start
DEBOUNCE_TICKS     = 2    # consecutive all-off reads before confirming MISS

# ── loop ──────────────────────────────────────────────────────────────────────
LOOP_HZ    = 50
LOOP_DELAY = 1.0 / LOOP_HZ
DEBUG      = True
DEBUG_EVERY = 5

# ── junction / stop marker ────────────────────────────────────────────────────
JUNCTION_SPIN_SPEED  = 20
JUNCTION_TIMEOUT_S   = 2.0
STOP_DEBOUNCE_S      = 2.0
CLEAR_MARKER_S       = 0.4
SKIP_SCAN            = True
RED_PIXEL_THRESHOLD  = 3000
RED_CHECK_INTERVAL   = 0.12

# ── watchdog ──────────────────────────────────────────────────────────────────
WATCHDOG_TIMEOUT_S = 0.5

# ═══════════════════════════════════════════════════════════════════════════════


class RobotState(Enum):
    STRAIGHT = "STRAIGHT"
    CURVE    = "CURVE"
    SHARP    = "SHARP"
    UTURN    = "UTURN"
    RECOVERY = "RECOVERY"
    LOST     = "LOST"


# ═══════════════════════════════════════════════════════════════════════════════
#  SENSOR READER
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class SensorReading:
    lo: bool; li: bool; ri: bool; ro: bool
    all_on: bool; all_off: bool
    error: Optional[float]

    @property
    def bits(self) -> str:
        return f"{int(self.lo)}{int(self.li)}{int(self.ri)}{int(self.ro)}"

    @property
    def state_label(self) -> str:
        if self.all_off: return "LOST"
        if self.all_on:  return "JUNCTION"
        return {
            "0001":"FAR-R","0010":"R-IN","0011":"BOTH-R","0100":"L-IN",
            "0101":"LI+RO","0110":"CENTRED","0111":"3-RIGHT",
            "1000":"FAR-L","1001":"LO+RO","1010":"LO+RI","1011":"LO+RI+RO",
            "1100":"BOTH-L","1101":"LO+LI+RO","1110":"3-LEFT",
        }.get(self.bits, self.bits)

    @property
    def last_side(self) -> int:
        """Which side last saw tape: +1=right, -1=left, 0=centre/unknown."""
        if self.error is None or self.error == 0:
            return 0
        return 1 if self.error > 0 else -1


class SensorReader:
    """Reads the 4-channel IR sensor with optional debounce on all-off.

    Sensor layout (looking down):  [L_OUT][L_IN] | [R_IN][R_OUT]
    True = tape,  False = floor
    Error weights: -3, -1, +1, +3
      Negative = robot right of centre (correct left)
      Positive = robot left  of centre (correct right)
    """

    def __init__(self) -> None:
        self._prev_all_off = False
        self._debounce_count = 0

    def read(self, bot, debounce: bool = True) -> SensorReading:
        try:
            lo, li, ri, ro = bot.read_line_sensors()
        except Exception:
            lo, li, ri, ro = False, False, False, False

        all_on  = bool(lo and li and ri and ro)
        raw_off = not (lo or li or ri or ro)

        # Debounce: only confirm all-off after DEBOUNCE_TICKS consecutive reads
        if debounce:
            if raw_off:
                self._debounce_count += 1
            else:
                self._debounce_count = 0
            all_off = self._debounce_count >= DEBOUNCE_TICKS
        else:
            all_off = raw_off

        # If debounce is filtering the off, report last known sensors
        error = None if all_off else float((-3*lo) + (-1*li) + (1*ri) + (3*ro))
        return SensorReading(lo=bool(lo), li=bool(li), ri=bool(ri), ro=bool(ro),
                             all_on=all_on, all_off=all_off, error=error)


# ═══════════════════════════════════════════════════════════════════════════════
#  PID CONTROLLER  (per-state gains)
# ═══════════════════════════════════════════════════════════════════════════════

class PIDController:
    """PID with dual-KP (small errors vs large errors) and smoothed derivative.

    Call set_gains() when the state machine changes state to switch
    between per-state PID tunings.
    """

    def __init__(self, kp: float = KP_S, ki: float = KI_S, kd: float = KD_S):
        self.kp = kp; self.ki = ki; self.kd = kd
        self._integral: float = 0.0
        self._last_err: float = 0.0
        self._last_t: Optional[float] = None

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp; self.ki = ki; self.kd = kd

    def reset(self) -> None:
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def compute(self, error: float,
                deriv_override: Optional[float] = None) -> float:
        now = time.time()
        dt  = (now - self._last_t) if self._last_t is not None else LOOP_DELAY
        dt  = max(dt, 1e-3)
        self._last_t = now

        self._integral = max(-MAX_INTEGRAL,
                             min(MAX_INTEGRAL, self._integral + error * dt))

        derivative = (deriv_override if deriv_override is not None
                      else (error - self._last_err) / dt)
        self._last_err = error

        # Dual-KP: stronger gain for large errors (turns)
        kp = KP_C if abs(error) >= 3 else self.kp
        return kp * error + self.ki * self._integral + self.kd * derivative


# ═══════════════════════════════════════════════════════════════════════════════
#  PREDICTION ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

class PredictionEngine:
    """Rolling sensor-error history for curve classification and speed control.

    Classifies the track ahead into four zones based on recent error pattern:
      STRAIGHT  — errors near zero
      CURVE     — moderate rising/falling trend
      SHARP     — high trend + high error
      UTURN     — sudden all-off after non-zero errors
    """

    def __init__(self) -> None:
        self._buf: deque = deque([0.0] * HISTORY_SIZE, maxlen=HISTORY_SIZE)

    def push(self, error: float) -> None:
        self._buf.append(float(error))

    def smoothed_deriv(self) -> float:
        buf = list(self._buf)
        if len(buf) < DERIV_WINDOW + 1:
            return 0.0
        return (buf[-1] - buf[-(DERIV_WINDOW + 1)]) / (DERIV_WINDOW * LOOP_DELAY)

    def trend(self) -> float:
        """Mean of second half minus mean of first half."""
        buf = list(self._buf)
        n   = len(buf)
        if n < 4:
            return 0.0
        mid = n // 2
        return sum(buf[mid:]) / (n - mid) - sum(buf[:mid]) / mid

    def weighted_trend(self) -> float:
        """Trend with linearly increasing weights — reacts faster than plain trend."""
        buf     = list(self._buf)
        n       = len(buf)
        if n < 2:
            return 0.0
        weights = list(range(1, n + 1))
        wmean   = sum(w * v for w, v in zip(weights, buf)) / sum(weights)
        return wmean - sum(buf) / n

    def effective_trend(self) -> float:
        """max(|trend|, |weighted_trend|×0.8) — best of both signals."""
        return max(abs(self.trend()), abs(self.weighted_trend()) * 0.8)

    def speed_penalty(self) -> float:
        t = self.effective_trend()
        if t <= TREND_THRESHOLD:
            return 0.0
        return min(SPEED_PENALTY_MAX, (t - TREND_THRESHOLD) * SPEED_PENALTY_K)

    def is_sharp_corner(self, error: float) -> bool:
        return abs(error) >= 3 and self.effective_trend() > SHARP_CORNER_TREND

    def last_nonzero_side(self) -> int:
        """Return +1 if recent errors trended right, -1 left, 0 unknown.

        Used by U-turn handler to decide which way to pivot.
        """
        buf = [v for v in self._buf if abs(v) > 0.5]
        if not buf:
            return 0
        avg = sum(buf[-5:]) / min(5, len(buf[-5:]))
        return 1 if avg > 0 else -1

    def classify(self) -> str:
        """Quick zone classification for the state machine."""
        t = self.effective_trend()
        recent_err = max(abs(v) for v in list(self._buf)[-5:]) if self._buf else 0
        if recent_err < 1:
            return "STRAIGHT"
        if t < TREND_THRESHOLD:
            return "CURVE"
        if recent_err >= 3:
            return "SHARP"
        return "CURVE"

    def reset(self) -> None:
        self._buf = deque([0.0] * HISTORY_SIZE, maxlen=HISTORY_SIZE)


# ═══════════════════════════════════════════════════════════════════════════════
#  MOTOR CONTROLLER  (smooth interpolation)
# ═══════════════════════════════════════════════════════════════════════════════

class MotorController:
    """Motor commands with exponential smoothing and speed-zone support.

    All speeds go through the interpolation filter:
        actual += (target - actual) × SMOOTH_FACTOR
    This eliminates jerky jumps and makes transitions feel natural.
    """

    def __init__(self) -> None:
        self._actual_L: float = 0.0
        self._actual_R: float = 0.0
        self._target_L: float = 0.0
        self._target_R: float = 0.0

    @staticmethod
    def _clamp(v: float) -> int:
        return int(max(MIN_SPEED, min(MAX_SPEED, v)))

    def _smooth_step(self) -> Tuple[int, int]:
        """One step of exponential smoothing toward targets."""
        self._actual_L += (self._target_L - self._actual_L) * SMOOTH_FACTOR
        self._actual_R += (self._target_R - self._actual_R) * SMOOTH_FACTOR
        return self._clamp(self._actual_L), self._clamp(self._actual_R)

    def set_differential(self, bot, correction: float,
                         cruise: float, speed_penalty: float = 0.0) -> Tuple[int, int]:
        """Differential drive from PID correction + predictive penalty."""
        c = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
        spd = max(MIN_SPEED + 4,
                  int(cruise - abs(c) * 0.30 - speed_penalty))
        self._target_L = float(spd + c)
        self._target_R = float(spd - c)
        L, R = self._smooth_step()
        bot._apply_motors(L, L, R, R)
        return L, R

    def set_pivot(self, bot, direction: int,
                  speed: int = UTURN_SPEED) -> Tuple[int, int]:
        """In-place rotation — bypasses smoothing for instant response."""
        if direction > 0:
            bot.rotate_right(speed)
            self._actual_L, self._actual_R = float(speed), float(-speed)
        else:
            bot.rotate_left(speed)
            self._actual_L, self._actual_R = float(-speed), float(speed)
        self._target_L, self._target_R = self._actual_L, self._actual_R
        return (speed if direction > 0 else -speed,
                -speed if direction > 0 else speed)

    def stop(self, bot) -> None:
        bot.stop()
        self._actual_L = self._actual_R = 0.0
        self._target_L = self._target_R = 0.0

    def reset(self) -> None:
        self._actual_L = self._actual_R = 0.0
        self._target_L = self._target_R = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  WATCHDOG
# ═══════════════════════════════════════════════════════════════════════════════

class Watchdog:
    def __init__(self) -> None:
        self._last_kick = time.time()
        self._bot = None
        self._running = False

    def start(self, bot) -> None:
        self._bot = bot; self._running = True
        self._last_kick = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    def kick(self) -> None:
        self._last_kick = time.time()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        while self._running:
            time.sleep(0.05)
            if time.time() - self._last_kick > WATCHDOG_TIMEOUT_S:
                try:
                    self._bot.stop()
                except Exception:
                    pass


# ═══════════════════════════════════════════════════════════════════════════════
#  LINE FOLLOWER  —  full state machine
# ═══════════════════════════════════════════════════════════════════════════════

class LineFollower:
    """
    State machine:

    ┌──────────┐  error≈0          ┌──────────┐  |err|=1-2      ┌──────────┐
    │ STRAIGHT │ ◄──────────────── │  CURVE   │ ◄────────────── │  SHARP   │
    └────┬─────┘                   └────┬─────┘                  └────┬─────┘
         │ all-off                      │ all-off after curve          │ all-off/pivot
         ▼                              ▼                              ▼
    ┌──────────┐               ┌──────────────┐              ┌──────────────┐
    │ RECOVERY │               │    UTURN     │◄─────────────│    UTURN     │
    └────┬─────┘               └──────┬───────┘              └──────┬───────┘
         │ timeout                    │ tape found                   │
         ▼                            ▼                              │
    ┌──────────┐               back to STRAIGHT ◄───────────────────┘
    │   LOST   │
    └──────────┘
    """

    def __init__(self) -> None:
        self.sensor   = SensorReader()
        self.pid      = PIDController(KP_S, KI_S, KD_S)
        self.pred     = PredictionEngine()
        self.motors   = MotorController()
        self.watchdog = Watchdog()

    def run(self, bot, cam=None,
            log=None, stop_event: Optional[threading.Event] = None) -> None:

        if log is None:
            log = lambda m: print(f"\n{m}", flush=True)

        log(f"LineFollower — STATE MACHINE  BASE={BASE_SPEED} CURVE={CURVE_SPEED} "
            f"SHARP={SHARP_SPEED} UTURN={UTURN_SPEED} SMOOTH={SMOOTH_FACTOR}")

        red_detector = None
        if not SKIP_SCAN:
            try:
                red_detector = RedTapeDetector()
            except Exception as e:
                log(f"Camera detector failed: {e}")

        # ── wait for tape ─────────────────────────────────────────────────
        log("Waiting for tape... (LEDs = yellow)")
        bot.set_all_leds_color(Color.YELLOW)
        while True:
            if stop_event and stop_event.is_set():
                return
            r = self.sensor.read(bot, debounce=False)
            if not r.all_off:
                break
            time.sleep(0.05)
        log("Tape detected! Starting in 1 s...")
        time.sleep(1.0)
        bot.set_all_leds_color(Color.GREEN)

        self.watchdog.start(bot)

        # ── state ─────────────────────────────────────────────────────────
        state          = RobotState.STRAIGHT
        last_error     = 0.0
        last_turn_dir  = 0          # +1=right, -1=left, updated every tick
        debounce_until = 0.0
        tick           = 0
        last_L         = BASE_SPEED
        last_R         = BASE_SPEED
        _pivot_latch   = 0
        _latch_dir     = 0
        _miss_ticks    = 0          # consecutive all-off ticks

        # Per-state PID mapping
        _pid_gains = {
            RobotState.STRAIGHT: (KP_S, KI_S, KD_S),
            RobotState.CURVE:    (KP_C, KI_C, KD_C),
            RobotState.SHARP:    (KP_C, KI_C, KD_C),
            RobotState.UTURN:    (KP_U, KI_U, KD_U),
        }

        def transition(new_state: RobotState) -> None:
            nonlocal state
            if new_state != state:
                gains = _pid_gains.get(new_state)
                if gains:
                    self.pid.set_gains(*gains)
                if new_state in (RobotState.UTURN, RobotState.RECOVERY):
                    self.pid.reset()
                    self.pred.reset()
                state = new_state

        try:
            while True:
                t0 = time.time()
                if stop_event and stop_event.is_set():
                    break
                self.watchdog.kick()

                r   = self.sensor.read(bot)
                now = time.time()

                # ── JUNCTION ──────────────────────────────────────────────
                if r.all_on:
                    if DEBUG and tick % DEBUG_EVERY == 0:
                        print(f"[{tick:06d}] [{state.value:<10}] {r.bits}  JUNCTION",
                              flush=True)
                    if not self._navigate_junction(bot, log):
                        break
                    transition(RobotState.STRAIGHT)
                    _miss_ticks = 0; _pivot_latch = 0
                    tick += 1; continue

                # ── STOP MARKER ───────────────────────────────────────────
                if (red_detector and red_detector.check()
                        and now >= debounce_until):
                    self.motors.stop(bot)
                    self._do_scan(bot, cam, log)
                    bot.forward(BASE_SPEED); time.sleep(CLEAR_MARKER_S)
                    self.motors.stop(bot)
                    debounce_until = now + STOP_DEBOUNCE_S
                    transition(RobotState.STRAIGHT)
                    _miss_ticks = 0
                    tick += 1; continue

                # ── ALL OFF: classify as UTURN or RECOVERY ────────────────
                if r.all_off:
                    _miss_ticks += 1

                    # First 15 ticks: slow creep keeping last heading
                    if _miss_ticks <= 15:
                        bias = max(-MAX_CORRECTION,
                                   min(MAX_CORRECTION,
                                       int(KP_S * last_error * 0.35)))
                        last_L, last_R = self.motors.set_differential(
                            bot, float(bias), float(UTURN_SPEED), 0.0
                        )
                        if DEBUG and tick % DEBUG_EVERY == 0:
                            print(f"[{tick:06d}] [{state.value:<10}] {r.bits}"
                                  f"  CREEP×{_miss_ticks:02d}"
                                  f"  L={last_L:3d} R={last_R:3d}", flush=True)
                        self._sleep(t0); tick += 1; continue

                    # Tape truly gone — decide: U-turn or recovery?
                    was_on_curve = state in (RobotState.CURVE, RobotState.SHARP,
                                             RobotState.UTURN)
                    self.motors.stop(bot)

                    if was_on_curve or abs(last_error) >= 1:
                        # Last had non-zero error → likely a U-turn
                        transition(RobotState.UTURN)
                        found = self._handle_uturn(bot, last_turn_dir, log)
                    else:
                        # Came from straight → gap or track end
                        transition(RobotState.RECOVERY)
                        found = self._handle_recovery(bot, last_error, log)

                    if found:
                        transition(RobotState.STRAIGHT)
                        _miss_ticks = 0; _pivot_latch = 0
                    else:
                        transition(RobotState.LOST)
                        bot.set_all_leds_color(Color.RED)
                        break
                    tick += 1; continue

                # ── ON TAPE: normal tracking ──────────────────────────────
                _miss_ticks = 0
                last_error  = r.error

                # Update last known turn direction (never let it go stale)
                if r.last_side != 0:
                    last_turn_dir = r.last_side

                self.pred.push(r.error)
                d_smooth  = self.pred.smoothed_deriv()
                penalty   = self.pred.speed_penalty()
                trend_val = self.pred.trend()

                correction   = self.pid.compute(r.error, deriv_override=d_smooth)
                corr_clamped = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

                # ── state classification ──────────────────────────────────
                abs_err  = abs(r.error)
                if abs_err < 1:
                    transition(RobotState.STRAIGHT)
                    cruise = float(BASE_SPEED)
                elif abs_err <= 2:
                    transition(RobotState.CURVE)
                    cruise = float(CURVE_SPEED)
                else:
                    transition(RobotState.SHARP)
                    cruise = float(SHARP_SPEED)

                # ── pivot decision (corner latch) ─────────────────────────
                _single_outer = ((r.lo and not r.li and not r.ri and not r.ro) or
                                 (r.ro and not r.lo and not r.li and not r.ri))
                _sharp_corner = self.pred.is_sharp_corner(r.error)

                if abs_err < 2:
                    _pivot_latch = 0     # clear latch when near centre

                direction = 1 if corr_clamped > 0 else -1

                if _single_outer or _sharp_corner:
                    _pivot_latch = CORNER_LATCH_TICKS
                    _latch_dir   = direction
                    use_pivot    = True
                elif _pivot_latch > 0:
                    _pivot_latch -= 1
                    direction     = _latch_dir
                    use_pivot     = True
                else:
                    use_pivot = False

                if use_pivot:
                    last_L, last_R = self.motors.set_pivot(
                        bot, direction, UTURN_SPEED
                    )
                    pivot_label = ("CORNER-PIVOT" if _sharp_corner and not _single_outer
                                   else f"LATCH({_pivot_latch})" if _pivot_latch > 0 and not _single_outer and not _sharp_corner
                                   else "PIVOT")
                    mode = f"{pivot_label}-{'R' if direction > 0 else 'L'}"
                else:
                    saturated = abs(corr_clamped) >= MAX_CORRECTION
                    last_L, last_R = self.motors.set_differential(
                        bot, correction, cruise,
                        speed_penalty=0.0 if saturated else penalty
                    )
                    mode = state.value

                # ── debug ─────────────────────────────────────────────────
                if DEBUG and tick % DEBUG_EVERY == 0:
                    dir_lbl = "RIGHT" if last_turn_dir > 0 else ("LEFT" if last_turn_dir < 0 else "CTR")
                    print(
                        f"[{tick:06d}]"
                        f" [{state.value:<10}]"
                        f" [{r.bits}]"
                        f"  ERR:{r.error:+.0f}"
                        f"  PID:{corr_clamped:+.0f}"
                        f"  LEFT:{last_L:4d}"
                        f"  RIGHT:{last_R:4d}"
                        f"  DIR:{dir_lbl}"
                        f"  SPD:{int(cruise)}"
                        f"  trnd:{trend_val:+.1f}"
                        f"  pen:{penalty:.1f}"
                        f"  {mode}",
                        flush=True,
                    )

                self._sleep(t0)
                tick += 1

        finally:
            self.watchdog.stop()
            if red_detector:
                red_detector.close()

    # ── U-turn handler ────────────────────────────────────────────────────────

    def _handle_uturn(self, bot, last_turn_dir: int, log) -> bool:
        """Execute a U-turn using the last known turn direction.

        Phase 0: Short probe (0.5s each direction) — quick scan.
        Phase 1: Full pivot in the correct direction until tape found.
        Phase 2: Fallback pivot in opposite direction.

        Direction is decided from last_turn_dir (which side last had tape)
        — never guesses.
        """
        log(f"U-TURN — last direction: {'RIGHT' if last_turn_dir > 0 else 'LEFT' if last_turn_dir < 0 else 'UNKNOWN'}")
        bot.set_all_leds_color(Color.YELLOW)
        self.pred.reset(); self.pid.reset()

        # If we have a known direction, use it; otherwise try left first
        if last_turn_dir == 0:
            # No history — sample from prediction engine
            last_turn_dir = self.pred.last_nonzero_side()
            if last_turn_dir == 0:
                last_turn_dir = -1  # default left

        # ── Phase 0: quick probe (faster than full search) ────────────────
        for probe_dir in [last_turn_dir, -last_turn_dir]:
            label = "right" if probe_dir > 0 else "left"
            bot.rotate_right(UTURN_SPEED) if probe_dir > 0 else bot.rotate_left(UTURN_SPEED)
            time.sleep(UTURN_PROBE_S)
            bot.stop(); time.sleep(0.05)
            r = self.sensor.read(bot, debounce=False)
            if r.li or r.ri:
                bot.forward(UTURN_SPEED); time.sleep(0.2)
                bot.stop(); time.sleep(0.1)
                self.pid.reset(); self.pred.reset()
                log(f"U-turn found (probe {label})")
                bot.set_all_leds_color(Color.GREEN)
                return True

        # ── Phase 1: full pivot in correct direction ───────────────────────
        for pivot_dir in [last_turn_dir, -last_turn_dir]:
            label    = "right" if pivot_dir > 0 else "left"
            deadline = time.time() + UTURN_TIMEOUT_S / 2.0
            log(f"U-turn pivoting {label}...")

            while time.time() < deadline:
                r = self.sensor.read(bot, debounce=False)
                if r.li or r.ri:
                    bot.stop(); time.sleep(0.05)
                    bot.forward(UTURN_SPEED); time.sleep(0.25)
                    bot.stop(); time.sleep(0.1)
                    self.pid.reset(); self.pred.reset()
                    log(f"U-turn re-acquired ({label})")
                    bot.set_all_leds_color(Color.GREEN)
                    return True
                if pivot_dir > 0:
                    bot.rotate_right(UTURN_SPEED)
                else:
                    bot.rotate_left(UTURN_SPEED)
                time.sleep(LOOP_DELAY)
            bot.stop(); time.sleep(0.1)

        log("U-turn failed — no tape found")
        bot.set_all_leds_color(Color.RED)
        return False

    # ── straight-loss recovery ────────────────────────────────────────────────

    def _handle_recovery(self, bot, last_error: float, log) -> bool:
        """Recover from a tape gap or end-of-track (came from STRAIGHT state)."""
        log("Line lost from straight — searching...")
        bot.set_all_leds_color(Color.YELLOW)
        half = RECOVERY_TIMEOUT_S / 2.0

        for spin_left in [(last_error <= 0), (last_error > 0)]:
            label    = "left" if spin_left else "right"
            deadline = time.time() + half
            while time.time() < deadline:
                r = self.sensor.read(bot, debounce=False)
                if r.li or r.ri:
                    bot.stop(); time.sleep(0.1)
                    bot.forward(UTURN_SPEED); time.sleep(0.15)
                    bot.stop(); time.sleep(0.1)
                    self.pid.reset(); self.pred.reset()
                    log(f"Line re-acquired (spun {label})")
                    bot.set_all_leds_color(Color.GREEN)
                    return True
                if spin_left:
                    bot.rotate_left(UTURN_SPEED)
                else:
                    bot.rotate_right(UTURN_SPEED)
                time.sleep(LOOP_DELAY)
            bot.stop(); time.sleep(0.1)

        log("Could not re-acquire line — stopping.")
        bot.set_all_leds_color(Color.RED)
        return False

    # ── junction navigation ───────────────────────────────────────────────────

    def _navigate_junction(self, bot, log) -> bool:
        self.motors.stop(bot); time.sleep(0.1)
        bot.set_all_leds_color(Color.YELLOW)
        directions = [(bot.rotate_left, "left"), (bot.rotate_right, "right")]
        random.shuffle(directions)

        for spin_fn, label in directions:
            log(f"Junction → trying {label}")
            deadline = time.time() + JUNCTION_TIMEOUT_S

            while time.time() < deadline:
                r = self.sensor.read(bot, debounce=False)
                if not r.all_on:
                    break
                spin_fn(JUNCTION_SPIN_SPEED); time.sleep(LOOP_DELAY)
            self.motors.stop(bot); time.sleep(0.05)

            while time.time() < deadline:
                r = self.sensor.read(bot, debounce=False)
                if r.li or r.ri:
                    self.motors.stop(bot); time.sleep(0.1)
                    bot.forward(BASE_SPEED); time.sleep(0.5)
                    self.motors.stop(bot); time.sleep(0.15)
                    self.pid.reset(); self.pred.reset()
                    log(f"Junction → branch found ({label})")
                    bot.set_all_leds_color(Color.GREEN)
                    return True
                spin_fn(JUNCTION_SPIN_SPEED); time.sleep(LOOP_DELAY)
            self.motors.stop(bot); time.sleep(0.15)

        log("Junction → no branch found")
        bot.set_all_leds_color(Color.RED)
        return False

    # ── scan ──────────────────────────────────────────────────────────────────

    def _do_scan(self, bot, cam, log) -> None:
        bot.set_all_leds_color(Color.BLUE); bot.beep(0.1)
        if SKIP_SCAN:
            log("Stop marker — 1 s pause"); time.sleep(1.0)
            # ── GAME: checkpoint even when SKIP_SCAN ──────────────────────
            try:
                from game.game_engine import GameEngine
                GameEngine.instance().on_checkpoint("")
            except Exception:
                pass
            return
        if cam is None:
            log("Scan skipped: no camera"); return
        import traceback
        from pointcloud import scan360
        try:
            session, _ = scan360.scan_and_build(bot, cam, log=log)
            log(f"Scan: {os.path.basename(session)}"); bot.beep(0.15)
            # ── GAME: checkpoint + 360 scan ───────────────────────────────
            try:
                from game.game_engine import GameEngine
                ge = GameEngine.instance()
                ge.on_checkpoint(str(session))
                ge.on_scan_360(str(session))
            except Exception:
                pass
        except Exception:
            log(f"Scan error:\n{traceback.format_exc()}")

    def _sleep(self, t0: float) -> None:
        rem = LOOP_DELAY - (time.time() - t0)
        if rem > 0:
            time.sleep(rem)


# ═══════════════════════════════════════════════════════════════════════════════
#  CAMERA STOP-MARKER DETECTOR
# ═══════════════════════════════════════════════════════════════════════════════

class RedTapeDetector:
    def __init__(self) -> None:
        import numpy as np          # Pi-only — lazy import keeps laptop IDE clean
        import cv2                  # noqa: F401
        self._np = np; self._cv2 = cv2
        self._L1 = np.array([0,   120, 70],  dtype=np.uint8)
        self._U1 = np.array([10,  255, 255], dtype=np.uint8)
        self._L2 = np.array([165, 120, 70],  dtype=np.uint8)
        self._U2 = np.array([180, 255, 255], dtype=np.uint8)
        self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self._hit = False; self._lock = threading.Lock(); self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def check(self) -> bool:
        with self._lock:
            v = self._hit; self._hit = False
        return v

    def close(self) -> None:
        self._running = False; self._cap.release()

    def _loop(self) -> None:
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                h = frame.shape[0]
                floor = self._cv2.cvtColor(frame[h//2:], self._cv2.COLOR_BGR2HSV)
                mask  = self._cv2.bitwise_or(
                    self._cv2.inRange(floor, self._L1, self._U1),
                    self._cv2.inRange(floor, self._L2, self._U2))
                if self._cv2.countNonZero(mask) >= RED_PIXEL_THRESHOLD:
                    with self._lock: self._hit = True
            time.sleep(RED_CHECK_INTERVAL)


# ═══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION
# ═══════════════════════════════════════════════════════════════════════════════

def calibrate() -> None:
    print("=== Calibration — Ctrl-C to exit ===")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}  {'err':>5}  state")
    print("-" * 62)
    reader = SensorReader()
    with RasBot() as bot:
        try:
            while True:
                r     = reader.read(bot, debounce=False)
                err_s = f"{r.error:+.0f}" if r.error is not None else " —"
                print(f"{str(r.lo):>8}  {str(r.li):>6}  {str(r.ri):>6}  {str(r.ro):>7}"
                      f"  {err_s:>5}  {r.state_label}          ",
                      end="\r", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nDone.")


# ═══════════════════════════════════════════════════════════════════════════════
#  COMPAT WRAPPER  (drive.py F-key interface)
# ═══════════════════════════════════════════════════════════════════════════════

def run(bot, cam=None, log=None,
        stop_event: Optional[threading.Event] = None) -> None:
    LineFollower().run(bot, cam=cam, log=log, stop_event=stop_event)


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "--calibrate":
        calibrate(); return

    cam = None
    if not SKIP_SCAN:
        from camera.rs_capture import StereoCapture
        cam = StereoCapture()

    with RasBot() as bot:
        try:
            run(bot, cam)
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            if cam: cam.close()
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            print("Motors stopped. Safe.")


if __name__ == "__main__":
    main()
