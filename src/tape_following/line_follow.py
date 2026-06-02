"""
Line-following — Yahboom RASPBOT V2, Raspberry Pi 5.
4-channel IR sensor + full PID (P+I+D) + predictive curve detection.

Sensor layout (looking straight down at the floor):
    [L_OUT][L_IN] | [R_IN][R_OUT]
    1 = black tape,  0 = white floor
    read_line_sensors() returns (lo, li, ri, ro)

All 16 IR sensor states and their meaning:
    0000  → LINE LOST              all off → creep then spin-search
    0001  → far-right only         robot far LEFT  of tape → PIVOT left  (err=+3)
    0010  → right-inner only       slight right  → PID  (err=+1)
    0011  → both right sensors     strong right  → PID  (err=+4)
    0100  → left-inner only        slight left   → PID  (err=-1)
    0101  → li + ro  (bridge)      gentle right  → PID  (err=+2)
    0110  → both inner             CENTRED       → PID  (err= 0)  ← ideal
    0111  → three right sensors    curve right   → PID  (err=+3)  ← NOT pivot
    1000  → far-left only          robot far RIGHT of tape → PIVOT right (err=-3)
    1001  → lo + ro  (bridge)      centered      → PID  (err= 0)
    1010  → lo + ri  (skip)        gentle left   → PID  (err=-2)
    1011  → lo + ri + ro           gentle right  → PID  (err=+1)
    1100  → both left sensors      strong left   → PID  (err=-4)
    1101  → lo + li + ro           gentle left   → PID  (err=-1)
    1110  → three left sensors     curve left    → PID  (err=-3)  ← NOT pivot
    1111  → all on                 JUNCTION / STOP MARKER

CRITICAL TUNING NOTES:
  • KD uses a SMOOTHED derivative (DERIV_WINDOW ticks, not 1 tick).
    Instantaneous d/dt at 50 Hz spikes to 150 on a single sensor edge.
    Smoothed over 4 ticks → max 37.5 → KD can be 0.10 safely.
  • MAX_CORRECTION = 25 keeps L:R ratio ≤ 3.5:1 (curving, not spinning).
  • Pivot fires ONLY for 1000 / 0001 (single outer sensor).
    Patterns 1110 / 0111 have err=±3 too but mean "gentle curve" — pivot
    here spins the robot OFF the tape.
  • Prediction: HISTORY_LEN errors are stored each tick. If the trend
    (second-half mean minus first-half mean) exceeds TREND_THRESHOLD the
    robot slows down BEFORE the error reaches MAX — entering curves early.

Run standalone:   python3 src/tape_following/line_follow.py
Calibrate:        python3 src/tape_following/line_follow.py --calibrate
"""

import os, sys, time, threading, random
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ── speed / PID tunables ───────────────────────────────────────────────────────
BASE_SPEED       = 45     # cruise speed when centred (0-255)
MAX_SPEED        = 255
MIN_SPEED        = 15     # minimum wheel speed — keeps motors spinning

KP_SMALL         = 10.0   # proportional gain for |err| ≤ 2 (centre tracking)
KP_LARGE         = 45.0   # proportional gain for |err| ≥ 3 (sharp turns)
KD               = 0.10   # derivative gain — safe because we use a smoothed deriv
KI               = 0.0    # integral gain — leave 0 unless drift is persistent
MAX_INTEGRAL     = 20.0

MAX_CORRECTION   = 25     # PID output clamp (keeps wheel ratio ≤ 3.5:1)
CORRECTION_SCALE = 0.35   # how much to reduce cruise speed per unit correction

# Pivot (in-place rotation) for single-outer-sensor states (1000 / 0001).
TIGHT_TURN_SPEED = 20

# ── prediction / history tunables ─────────────────────────────────────────────
HISTORY_LEN       = 10    # rolling error buffer length (10 ticks = 200 ms at 50 Hz)
DERIV_WINDOW      = 4     # smooth derivative over this many ticks (4 ticks = 80 ms)
                          # instantaneous spike: 3/0.02=150; 4-tick: 3/0.08=37.5
TREND_THRESHOLD   = 0.8   # abs(trend) above this → robot is entering a curve
SPEED_PENALTY_MAX = 12    # maximum speed reduction from curve-entry prediction
SPEED_PENALTY_K   = 6.0   # speed_penalty = min(MAX, (trend - threshold) * K)

# ── loop timing ───────────────────────────────────────────────────────────────
LOOP_HZ          = 50
LOOP_PAUSE       = 1.0 / LOOP_HZ

# ── MISS / recovery ───────────────────────────────────────────────────────────
MISS_CREEP_TICKS = 12     # ticks at creep speed before entering full search (~240 ms)
MISS_CREEP_SPEED = 18
SEARCH_SPEED     = 18
SEARCH_TIMEOUT_S = 4.0
RECOVER_SETTLE_S = 0.15

# ── junction / stop marker ────────────────────────────────────────────────────
JUNCTION_SPIN_SPEED = 20
JUNCTION_TIMEOUT_S  = 2.0
STOP_DEBOUNCE_S     = 2.0
CLEAR_MARKER_S      = 0.4
SKIP_SCAN           = True

# ── camera stop-marker detection (SKIP_SCAN=False only) ──────────────────────
RED_PIXEL_THRESHOLD = 3000
RED_CHECK_INTERVAL  = 0.12

# ── debug output ──────────────────────────────────────────────────────────────
DEBUG_EVERY = 5   # print every N ticks (0 = silent)
# ─────────────────────────────────────────────────────────────────────────────


# ── sensor helpers ────────────────────────────────────────────────────────────

def read_sensors(bot):
    """Return (lo, li, ri, ro, all_on, all_off) as bools."""
    lo, li, ri, ro = bot.read_line_sensors()
    return lo, li, ri, ro, (lo and li and ri and ro), not (lo or li or ri or ro)


def sensor_error(lo, li, ri, ro):
    """Weighted position error in [-4, +4].

    Negative = robot right of tape centre (needs left correction).
    Positive = robot left  of tape centre (needs right correction).
    None     = line lost (all sensors off).
    """
    if not (lo or li or ri or ro):
        return None
    return float((-3 * lo) + (-1 * li) + (1 * ri) + (3 * ro))


def state_name(lo, li, ri, ro, all_on, all_off):
    """Human-readable label for the current sensor pattern (used in debug log)."""
    if all_off: return "LOST"
    if all_on:  return "JUNCTION"
    bits = f"{int(lo)}{int(li)}{int(ri)}{int(ro)}"
    return {
        "0001": "FAR-RIGHT(pivot←)", "0010": "R-INNER",    "0011": "BOTH-RIGHT",
        "0100": "L-INNER",           "0101": "LI+RO-bridge","0110": "CENTRED",
        "0111": "3-RIGHT-curve",     "1000": "FAR-LEFT(pivot→)","1001": "LO+RO-bridge",
        "1010": "LO+RI-skip",        "1011": "LO+RI+RO",   "1100": "BOTH-LEFT",
        "1101": "LO+LI+RO",          "1110": "3-LEFT-curve",
    }.get(bits, bits)


# ── error history & prediction ────────────────────────────────────────────────

class ErrorHistory:
    """Rolling buffer of the last HISTORY_LEN sensor errors.

    Provides three signals for the prediction system:
      smoothed_deriv() — derivative averaged over DERIV_WINDOW ticks.
                         Far less spiky than a 1-tick diff at 50 Hz.
      trend()          — slope of error over the full window.
                         Positive = trending right, negative = left.
      speed_penalty()  — how much to subtract from BASE_SPEED when a
                         curve entry is predicted (trend > TREND_THRESHOLD).
    """

    def __init__(self):
        self._buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)

    def push(self, error):
        self._buf.append(float(error))

    def smoothed_deriv(self):
        """(err[now] - err[now - DERIV_WINDOW]) / (DERIV_WINDOW * dt).

        Using a wider window damps single-sensor flip spikes so KD can be
        larger without causing oscillation.
        """
        buf = list(self._buf)
        if len(buf) < DERIV_WINDOW + 1:
            return 0.0
        return (buf[-1] - buf[-(DERIV_WINDOW + 1)]) / (DERIV_WINDOW * LOOP_PAUSE)

    def trend(self):
        """Compare mean of first half vs mean of second half of the window.

        Returns the difference (second_half_mean - first_half_mean).
        A rising trend means the robot is heading to the right — it saw low
        errors recently but errors are climbing, so a right curve is coming.
        """
        buf = list(self._buf)
        n   = len(buf)
        if n < 4:
            return 0.0
        mid    = n // 2
        first  = sum(buf[:mid]) / mid
        second = sum(buf[mid:]) / (n - mid)
        return second - first

    def speed_penalty(self):
        """Predictive speed reduction: fires when |trend| > TREND_THRESHOLD.

        Returns a value in [0, SPEED_PENALTY_MAX] that is subtracted from
        BASE_SPEED — slowing the robot down BEFORE it reaches peak error.

        Example: trend=1.5, threshold=0.8, K=6 → penalty = min(12, 4.2) = 4.2
        """
        t = abs(self.trend())
        if t <= TREND_THRESHOLD:
            return 0.0
        return min(SPEED_PENALTY_MAX, (t - TREND_THRESHOLD) * SPEED_PENALTY_K)

    def reset(self):
        self._buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)


# ── PID controller (full P + I + D) ──────────────────────────────────────────

class PID:
    """Full PID controller.

    KP_SMALL is used for gentle tracking (|err| < 3).
    KP_LARGE kicks in for sharp turns (|err| >= 3) to react quickly.
    The derivative term accepts an optional smoothed value from ErrorHistory
    so it predicts direction rather than just reacting to 1-tick spikes.
    """

    def __init__(self, kp=KP_SMALL, ki=KI, kd=KD):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def reset(self):
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def compute(self, error, deriv_override=None):
        """Compute PID correction.

        deriv_override: smoothed derivative from ErrorHistory.smoothed_deriv().
        If None, falls back to the standard 1-tick finite difference.
        """
        now = time.time()
        dt  = (now - self._last_t) if self._last_t else LOOP_PAUSE
        dt  = max(dt, 0.001)
        self._last_t = now

        self._integral = max(-MAX_INTEGRAL,
                             min(MAX_INTEGRAL, self._integral + error * dt))

        if deriv_override is not None:
            derivative = deriv_override
        else:
            derivative = (error - self._last_err) / dt

        self._last_err = error

        kp = KP_LARGE if abs(error) >= 3 else self.kp
        return kp * error + self.ki * self._integral + self.kd * derivative


# ── motor application ─────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_correction(bot, correction, pivot=False, speed_penalty=0.0):
    """Drive both sides from a PID correction + optional predictive speed penalty.

    pivot=True       → pure in-place rotation (only for 1000 / 0001 states).
    speed_penalty    → extra speed reduction from curve-entry prediction;
                       subtracted from BASE_SPEED before computing wheel speeds.

    correction > 0 → steer right  (left wheels faster)
    correction < 0 → steer left   (right wheels faster)
    """
    c = _clamp(correction, -MAX_CORRECTION, MAX_CORRECTION)

    if pivot:
        if c > 0:
            bot.rotate_right(TIGHT_TURN_SPEED)
        else:
            bot.rotate_left(TIGHT_TURN_SPEED)
        return None, None

    # Adaptive cruise: reduce for correction magnitude AND prediction penalty.
    cruise = _clamp(
        int(BASE_SPEED - abs(c) * CORRECTION_SCALE - speed_penalty),
        MIN_SPEED + 5, MAX_SPEED,
    )
    left  = _clamp(int(cruise + c), MIN_SPEED, MAX_SPEED)
    right = _clamp(int(cruise - c), MIN_SPEED, MAX_SPEED)
    bot._apply_motors(left, left, right, right)
    return left, right


# ── camera stop-marker detector ───────────────────────────────────────────────

class RedTapeDetector:
    """Background thread — detects red tape via camera (SKIP_SCAN=False mode)."""

    def __init__(self):
        import numpy as np
        import cv2
        self._np  = np
        self._cv2 = cv2
        self._L1  = np.array([0,   120,  70], dtype=np.uint8)
        self._U1  = np.array([10,  255, 255], dtype=np.uint8)
        self._L2  = np.array([165, 120,  70], dtype=np.uint8)
        self._U2  = np.array([180, 255, 255], dtype=np.uint8)
        self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self._hit     = False
        self._lock    = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def check(self):
        with self._lock:
            v = self._hit; self._hit = False
        return v

    def close(self):
        self._running = False
        self._cap.release()

    def _loop(self):
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


# ── recovery ──────────────────────────────────────────────────────────────────

def _log(msg):
    print(f"\n{msg}", flush=True)


def recover_line(bot, pid, history, last_error, log):
    """Spin search: try last-known direction first, then the opposite.

    Returns True if an inner sensor re-acquired the tape.
    """
    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)

    start_left = (last_error is None or last_error <= 0)
    half       = SEARCH_TIMEOUT_S / 2.0

    for spin_left in [start_left, not start_left]:
        label    = "left" if spin_left else "right"
        deadline = time.time() + half

        while time.time() < deadline:
            _, li, ri, _, _, _ = read_sensors(bot)
            if li or ri:
                bot.stop(); time.sleep(0.1)
                bot.forward(MISS_CREEP_SPEED); time.sleep(0.15)
                bot.stop(); time.sleep(RECOVER_SETTLE_S)
                pid.reset(); history.reset()
                log(f"line re-acquired (spun {label})")
                bot.set_all_leds_color(Color.GREEN)
                return True
            if spin_left:
                bot.rotate_left(SEARCH_SPEED)
            else:
                bot.rotate_right(SEARCH_SPEED)
            time.sleep(LOOP_PAUSE)

        bot.stop(); time.sleep(0.1)

    log("could not re-acquire line — stopping.")
    bot.set_all_leds_color(Color.RED)
    return False


# ── junction navigation ───────────────────────────────────────────────────────

def navigate_junction(bot, pid, history, log):
    """All-sensors-on = T-junction or crossing.

    Randomly pick a direction, spin off the junction bar, then spin until
    an inner sensor finds the branch. Returns True if branch found.
    """
    bot.stop(); time.sleep(0.1)
    bot.set_all_leds_color(Color.YELLOW)

    directions = [(bot.rotate_left, "left"), (bot.rotate_right, "right")]
    random.shuffle(directions)

    for spin_fn, label in directions:
        log(f"junction → trying {label}")
        deadline = time.time() + JUNCTION_TIMEOUT_S

        while time.time() < deadline:
            _, _, _, _, all_on, _ = read_sensors(bot)
            if not all_on:
                break
            spin_fn(JUNCTION_SPIN_SPEED); time.sleep(LOOP_PAUSE)
        bot.stop(); time.sleep(0.05)

        while time.time() < deadline:
            _, li, ri, _, _, _ = read_sensors(bot)
            if li or ri:
                bot.stop(); time.sleep(0.1)
                bot.forward(MISS_CREEP_SPEED); time.sleep(0.2)
                bot.stop(); time.sleep(0.1)
                pid.reset(); history.reset()
                log(f"junction → branch found ({label})")
                bot.set_all_leds_color(Color.GREEN)
                return True
            spin_fn(JUNCTION_SPIN_SPEED); time.sleep(LOOP_PAUSE)
        bot.stop(); time.sleep(0.15)

    log("junction → no branch found")
    bot.set_all_leds_color(Color.RED)
    return False


# ── stop-marker scan ──────────────────────────────────────────────────────────

def do_scan(bot, cam, log):
    bot.set_all_leds_color(Color.BLUE)
    bot.beep(0.1)
    if SKIP_SCAN:
        log("stop marker — 1 s pause (SKIP_SCAN=True)")
        time.sleep(1.0)
        return
    if cam is None:
        log("scan skipped: no camera object")
        return
    log("stop marker → 360° scan")
    import traceback
    from pointcloud import scan360
    try:
        session, ply = scan360.scan_and_build(bot, cam, log=log)
        log(f"scan complete: {os.path.basename(session)}  cloud={ply}")
        bot.beep(0.15)
    except Exception:
        log(f"scan error (continuing):\n{traceback.format_exc()}")


# ── main control loop ─────────────────────────────────────────────────────────

def run(bot, cam=None, log=_log, stop_event=None):
    """Start autonomous line-following.  Call from drive.py or standalone."""
    log("Line-following started  (P+I+D + predictive speed)")
    log(f"BASE={BASE_SPEED}  KP_S={KP_SMALL}  KP_L={KP_LARGE}  KD={KD}  "
        f"HISTORY={HISTORY_LEN}  DERIV_WIN={DERIV_WINDOW}  "
        f"TREND_THRESH={TREND_THRESHOLD}  SPD_PENALTY_MAX={SPEED_PENALTY_MAX}")

    red_detector = None
    if not SKIP_SCAN:
        try:
            red_detector = RedTapeDetector()
            log("Red-tape camera detector started.")
        except Exception as e:
            log(f"Camera detector failed ({e}) — IR-only mode.")

    # ── wait for tape ─────────────────────────────────────────────────────────
    log("Waiting for tape... (LEDs = yellow)")
    bot.set_all_leds_color(Color.YELLOW)
    while True:
        if stop_event is not None and stop_event.is_set():
            return
        _, _, _, _, _, all_off = read_sensors(bot)
        if not all_off:
            break
        time.sleep(0.05)
    log("Tape detected! Starting in 1 s...")
    time.sleep(1.0)
    bot.set_all_leds_color(Color.GREEN)

    # ── state ─────────────────────────────────────────────────────────────────
    pid            = PID(KP_SMALL, KI, KD)
    history        = ErrorHistory()
    last_error     = 0.0
    debounce_until = 0.0
    miss_count     = 0
    in_recovery    = False
    just_scanned   = False
    tick           = 0
    last_L = BASE_SPEED
    last_R = BASE_SPEED

    try:
        while True:
            if stop_event is not None and stop_event.is_set():
                break

            lo, li, ri, ro, all_on, all_off = read_sensors(bot)
            now   = time.time()
            error = sensor_error(lo, li, ri, ro)

            # ── 1. JUNCTION ───────────────────────────────────────────────────
            if all_on:
                if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
                    print(f"[{tick:06d}] sensors=1111  JUNCTION", flush=True)
                if not navigate_junction(bot, pid, history, log):
                    break
                last_error = 0.0; miss_count = 0
                tick += 1; continue

            # ── 2. STOP MARKER ────────────────────────────────────────────────
            if (red_detector is not None
                    and red_detector.check()
                    and now >= debounce_until):
                bot.stop(); pid.reset(); history.reset()
                miss_count = 0; in_recovery = False; just_scanned = True
                do_scan(bot, cam, log)
                bot.forward(BASE_SPEED); time.sleep(CLEAR_MARKER_S); bot.stop()
                debounce_until = now + STOP_DEBOUNCE_S
                last_error = 0.0
                bot.set_all_leds_color(Color.GREEN)
                tick += 1; continue

            # ── 3. LINE LOST ──────────────────────────────────────────────────
            if all_off:
                if not in_recovery:
                    miss_count += 1
                    history.push(last_error)  # keep history trending toward last known

                if miss_count <= MISS_CREEP_TICKS:
                    corr = _clamp(int(KP_SMALL * last_error * 0.4),
                                  -MAX_CORRECTION, MAX_CORRECTION)
                    lspd = _clamp(int(MISS_CREEP_SPEED + corr), MIN_SPEED, MAX_SPEED)
                    rspd = _clamp(int(MISS_CREEP_SPEED - corr), MIN_SPEED, MAX_SPEED)
                    bot._apply_motors(lspd, lspd, rspd, rspd)
                    last_L, last_R = lspd, rspd
                    if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
                        print(f"[{tick:06d}] sensors=0000"
                              f"  MISS×{miss_count:02d}  creep"
                              f"  L={lspd:3d} R={rspd:3d}", flush=True)
                    time.sleep(LOOP_PAUSE); tick += 1; continue

                if not in_recovery:
                    in_recovery = True
                    bot.stop()
                    if just_scanned:
                        log("tape ended after scan — end of path. Stopping.")
                        bot.beep(0.3); bot.set_all_leds_color(Color.RED)
                        break
                    just_scanned = False
                    if not recover_line(bot, pid, history, last_error, log):
                        break
                    miss_count = 0; in_recovery = False; last_error = 0.0
                else:
                    time.sleep(LOOP_PAUSE)
                tick += 1; continue

            # ── 4. NORMAL TRACKING ────────────────────────────────────────────
            miss_count   = 0
            in_recovery  = False
            just_scanned = False
            last_error   = error

            # Prediction signals from the rolling history.
            history.push(error)
            d_smooth = history.smoothed_deriv()   # smoothed D term
            penalty  = history.speed_penalty()    # predictive speed reduction
            trend    = history.trend()            # raw trend (for debug)

            # Full PID with smoothed derivative.
            correction   = pid.compute(error, deriv_override=d_smooth)
            corr_clamped = _clamp(correction, -MAX_CORRECTION, MAX_CORRECTION)

            # Pivot ONLY for single-outer-sensor states (1000 or 0001).
            # 1110 / 0111 also have |err|=3 but are gentle curves — pivot
            # here would spin the robot off the tape.
            _single_outer = (lo and not li and not ri and not ro) or \
                            (ro and not lo and not li and not ri)

            if _single_outer:
                apply_correction(bot, correction, pivot=True)
                last_L = TIGHT_TURN_SPEED * (1 if corr_clamped > 0 else -1)
                last_R = -last_L
                _mode  = f"PIVOT-{'R' if corr_clamped > 0 else 'L'}"
            else:
                result = apply_correction(bot, correction,
                                          pivot=False, speed_penalty=penalty)
                if result[0] is not None:
                    last_L, last_R = result
                _mode  = "PID"

            if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
                sname = state_name(lo, li, ri, ro, all_on, all_off)
                print(
                    f"[{tick:06d}]"
                    f" sens={int(lo)}{int(li)}{int(ri)}{int(ro)}"
                    f"  err={error:+.0f}"
                    f"  corr={corr_clamped:+.0f}"
                    f"  L={last_L:4d} R={last_R:4d}"
                    f"  {_mode}"
                    f"  trnd={trend:+.1f}"
                    f"  pen={penalty:.1f}"
                    f"  [{sname}]",
                    flush=True,
                )

            time.sleep(LOOP_PAUSE)
            tick += 1

    finally:
        if red_detector:
            red_detector.close()


# ── calibration mode ──────────────────────────────────────────────────────────

def calibrate():
    """Print live sensor readings — confirm True=tape, False=floor."""
    print("=== Calibration — Ctrl-C to exit ===")
    print("Move robot over tape: True=tape, False=floor")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}  {'err':>5}  state")
    print("-" * 58)
    with RasBot() as bot:
        try:
            while True:
                lo, li, ri, ro, all_on, all_off = read_sensors(bot)
                err   = sensor_error(lo, li, ri, ro)
                sname = state_name(lo, li, ri, ro, all_on, all_off)
                err_s = f"{err:+.0f}" if err is not None else " —"
                print(f"{str(lo):>8}  {str(li):>6}  {str(ri):>6}  {str(ro):>7}"
                      f"  {err_s:>5}  {sname}          ", end="\r", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nDone.")


# ── entry point ───────────────────────────────────────────────────────────────

def main():
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
