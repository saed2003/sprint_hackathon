"""
Autonomous line-following — Raspbot V2, 4 IR sensors, PID control.

PID control gives smooth proportional steering instead of on/off switching:
  • Position error  = weighted sensor reading (how far off-centre in cm)
  • PID correction  = KP*error + KD*d(error)/dt
  • Left  motors    = BASE_SPEED + correction  (faster = steer right)
  • Right motors    = BASE_SPEED - correction  (faster = steer left)

Sensor layout (looking down at floor beneath robot):
    [L_OUT]  [L_IN]  |  [R_IN]  [R_OUT]
    True = sensor over dark tape.
    Weights: -3, -1, +1, +3  (negative = left, positive = right)

Run:       python3 src/line_following/line_follow.py
Calibrate: python3 src/line_following/line_follow.py --calibrate

── Tuning checklist ──────────────────────────────────────────────────────────
 1. Run --calibrate, place robot on tape, confirm True=black / False=floor.
 2. Set SKIP_SCAN=True.  Place robot ON tape.  Run.
 3. Watch "err" and "corr" in the debug line.
    • Oscillating left-right → lower KP or raise KD.
    • Corrects too slowly    → raise KP.
    • Overshoots on turns    → raise KD.
 4. When steering is clean, set SKIP_SCAN=False for the real run.
─────────────────────────────────────────────────────────────────────────────
"""

import os, sys, time, threading
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ── tunables ──────────────────────────────────────────────────────────────────
BASE_SPEED   = 40    # straight-ahead speed — keep low so PID has time to correct
MAX_SPEED    = 255   # motor speed ceiling
MIN_SPEED    = 0     # motor speed floor (0 = allow one side to stop, no reverse)

# PID gains — tuned for 50 Hz binary sensors on Raspbot V2
KP           = 30.0  # proportional: raised so small errors get caught fast
KD           = 0.5   # derivative:   keep small for binary sensors (big jumps)
KI           = 0.0   # integral:     leave 0 unless robot consistently drifts
MAX_INTEGRAL = 20.0  # anti-windup clamp for the I term

LOOP_HZ      = 50
LOOP_PAUSE   = 1.0 / LOOP_HZ

# Recovery — what happens when sensors all go dark
MISS_CREEP_TICKS = 10    # ticks at creep speed (~200 ms) before full stop
MISS_CREEP_SPEED = 18    # very slow creep during brief gap
SEARCH_SPEED     = 18    # slow spin during recovery
SEARCH_TIMEOUT_S = 2.5   # give up and stop after this long
RECOVER_SETTLE_S = 0.2   # pause after re-acquiring before moving

# Stop-marker behaviour
STOP_DEBOUNCE_S  = 2.0   # ignore re-triggers after a scan for this long
CLEAR_MARKER_S   = 0.4   # drive forward after scan to clear the cross-mark

# Red tape camera detection (SKIP_SCAN=False mode)
RED_PIXEL_THRESHOLD = 3000
RED_CHECK_INTERVAL  = 0.12

# Skip 360° scan while tuning (True = stop 1 s at marker, no camera needed)
SKIP_SCAN        = True

# Live debug output every N ticks in the terminal (0 = silent)
DEBUG_EVERY      = 5     # every 100 ms at 50 Hz
# ─────────────────────────────────────────────────────────────────────────────


# ── sensor helpers ────────────────────────────────────────────────────────────

def read_sensors(bot):
    """Return (lo, li, ri, ro, all_on, all_off)."""
    lo, li, ri, ro = bot.read_line_sensors()
    all_on  = lo and li and ri and ro
    all_off = not (lo or li or ri or ro)
    return lo, li, ri, ro, all_on, all_off


def sensor_error(lo, li, ri, ro):
    """Weighted position error:  -4 (far left) … 0 (centred) … +4 (far right).

    Weights -3/-1/+1/+3 give finer resolution than ±1/±2.
    Returns None when no sensor is active (line lost).
    """
    if not (lo or li or ri or ro):
        return None
    return float((-3 * lo) + (-1 * li) + (1 * ri) + (3 * ro))


def error_to_cm(error):
    """Rough estimate of lateral distance from tape centre (cm).

    Based on ~1.5 cm spacing between sensors on the Raspbot V2.
    """
    if error is None:
        return None
    return error * 1.5          # ±1 sensor unit ≈ 1.5 cm


# ── PID controller ────────────────────────────────────────────────────────────

class PID:
    """Simple PD/PID controller for line-following."""

    def __init__(self, kp, ki, kd, max_integral=MAX_INTEGRAL):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._max_i    = max_integral
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def reset(self):
        self._integral = 0.0
        self._last_err = 0.0
        self._last_t   = None

    def compute(self, error):
        """Return the correction value for the given error."""
        now = time.time()
        dt  = (now - self._last_t) if self._last_t else LOOP_PAUSE
        dt  = max(dt, 0.001)
        self._last_t = now

        self._integral = max(-self._max_i,
                             min(self._max_i, self._integral + error * dt))
        derivative = (error - self._last_err) / dt
        self._last_err = error

        return self.kp * error + self.ki * self._integral + self.kd * derivative


# ── motor application ─────────────────────────────────────────────────────────

def apply_correction(bot, correction):
    """Drive both sides from a PID correction value.

    correction > 0  → tape is RIGHT of robot → steer right
                      (left side faster, right side slower)
    correction < 0  → tape is LEFT  of robot → steer left
                      (right side faster, left side slower)

    _apply_motors(lf, lr, rf, rr)
    """
    left  = int(BASE_SPEED + correction)
    right = int(BASE_SPEED - correction)
    left  = max(MIN_SPEED, min(MAX_SPEED, left))
    right = max(MIN_SPEED, min(MAX_SPEED, right))
    bot._apply_motors(left, left, right, right)


# ── red tape detector (real-run stop marker) ──────────────────────────────────

class RedTapeDetector:
    _L1 = np.array([0,   120, 70], dtype=np.uint8)
    _U1 = np.array([10,  255, 255], dtype=np.uint8)
    _L2 = np.array([165, 120, 70], dtype=np.uint8)
    _U2 = np.array([180, 255, 255], dtype=np.uint8)

    def __init__(self):
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
                h = frame.shape[0]
                floor = cv2.cvtColor(frame[h//2:], cv2.COLOR_BGR2HSV)
                mask  = cv2.bitwise_or(cv2.inRange(floor, self._L1, self._U1),
                                       cv2.inRange(floor, self._L2, self._U2))
                if cv2.countNonZero(mask) >= RED_PIXEL_THRESHOLD:
                    with self._lock:
                        self._hit = True
            time.sleep(RED_CHECK_INTERVAL)


# ── recovery ──────────────────────────────────────────────────────────────────

def _log(msg):
    print(f"\n{msg}")


def recover_line(bot, pid, last_error, log):
    """Spin toward last known tape direction until an inner sensor fires.

    Returns True if re-acquired, False if timed out.
    """
    spin_left = (last_error is None or last_error <= 0)
    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)
    deadline = time.time() + SEARCH_TIMEOUT_S

    while time.time() < deadline:
        _, li, ri, _, _, _ = read_sensors(bot)
        if li or ri:                         # inner sensor = properly on tape
            bot.stop()
            time.sleep(0.1)
            bot.forward(MISS_CREEP_SPEED)    # creep forward to centre
            time.sleep(0.15)
            bot.stop()
            time.sleep(RECOVER_SETTLE_S)
            pid.reset()
            log("line re-acquired")
            bot.set_all_leds_color(Color.GREEN)
            return True
        if spin_left:
            bot.rotate_left(SEARCH_SPEED)
        else:
            bot.rotate_right(SEARCH_SPEED)
        time.sleep(LOOP_PAUSE)

    bot.stop()
    log("could not re-acquire line — stopping.")
    bot.set_all_leds_color(Color.RED)
    return False


# ── stop-marker scan ──────────────────────────────────────────────────────────

def do_scan(bot, cam, log):
    bot.set_all_leds_color(Color.BLUE)
    bot.beep(0.1)
    if SKIP_SCAN:
        log("stop marker — waiting 1 s (SKIP_SCAN=True)")
        time.sleep(1.0)
        return
    log("stop marker → 360° scan")
    try:
        from pointcloud import scan360
        _, ply = scan360.scan_and_build(bot, cam, log=log)
        log(f"scan complete → {ply}")
        bot.beep(0.15)
    except Exception as e:
        log(f"scan error: {e}")


# ── main loop ─────────────────────────────────────────────────────────────────

def run(bot, cam=None, log=_log):
    log("Line-following started (PID). Ctrl-C to stop.")
    log(f"BASE={BASE_SPEED}  KP={KP}  KD={KD}  KI={KI}  SKIP_SCAN={SKIP_SCAN}")

    red_detector = None
    if not SKIP_SCAN:
        try:
            red_detector = RedTapeDetector()
            log("Red-tape camera detector started.")
        except Exception as e:
            log(f"Camera detector failed ({e}) — using IR fallback.")

    bot.set_all_leds_color(Color.GREEN)
    pid              = PID(KP, KI, KD)
    last_error       = 0.0
    debounce_until   = 0.0
    miss_count       = 0
    in_recovery      = False
    tick             = 0
    just_scanned     = False   # True right after a stop-marker scan

    try:
        while True:
            lo, li, ri, ro, all_on, all_off = read_sensors(bot)
            now   = time.time()
            error = sensor_error(lo, li, ri, ro)
            dist  = error_to_cm(error)

            # ── debug (P-only estimate — does NOT advance PID state) ───────────
            if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
                if error is not None:
                    p_est = KP * error          # show P term only for display
                    state = (f"err={error:+.0f}  "
                             f"dist={dist:+.1f}cm  "
                             f"P≈{p_est:+.0f}")
                else:
                    state = f"MISS×{miss_count}" if all_off else "STOP"
                print(f"lo={int(lo)} li={int(li)} ri={int(ri)} ro={int(ro)}"
                      f"  {state}          ", end="\r")
            tick += 1

            # ── stop marker ────────────────────────────────────────────────────
            use_cam = red_detector is not None
            stop_triggered = (
                (use_cam  and red_detector.check()) or
                (not use_cam and all_on)
            ) and now >= debounce_until

            if stop_triggered:
                bot.stop()
                pid.reset()
                miss_count   = 0
                in_recovery  = False
                just_scanned = True
                do_scan(bot, cam, log)
                bot.forward(BASE_SPEED)
                time.sleep(CLEAR_MARKER_S)
                bot.stop()
                debounce_until = now + STOP_DEBOUNCE_S
                last_error = 0.0
                bot.set_all_leds_color(Color.GREEN)
                continue

            # ── line lost ──────────────────────────────────────────────────────
            if all_off:
                if not in_recovery:
                    miss_count += 1

                if miss_count <= MISS_CREEP_TICKS:
                    # Phase 1: creep with last PID correction — keeps turning
                    last_corr = KP * last_error   # P-only for creep
                    apply_correction(bot, last_corr * (MISS_CREEP_SPEED / BASE_SPEED))
                    time.sleep(LOOP_PAUSE)
                    continue

                # Phase 2: confirmed lost — stop and search
                if not in_recovery:
                    in_recovery = True
                    bot.stop()
                    # If tape ends right after a scan = end of path → stop cleanly
                    if just_scanned:
                        log("tape ended after scan — end of path. Stopping.")
                        bot.beep(0.3)
                        bot.set_all_leds_color(Color.RED)
                        break
                    just_scanned = False
                    if not recover_line(bot, pid, last_error, log):
                        break
                    miss_count  = 0
                    in_recovery = False
                    last_error  = 0.0
                else:
                    time.sleep(LOOP_PAUSE)
                continue

            # ── normal tracking ────────────────────────────────────────────────
            miss_count   = 0
            in_recovery  = False
            just_scanned = False
            last_error   = error
            correction  = pid.compute(error)
            apply_correction(bot, correction)
            time.sleep(LOOP_PAUSE)

    finally:
        if red_detector:
            red_detector.close()


# ── calibration ───────────────────────────────────────────────────────────────

def calibrate():
    print("=== Calibration — Ctrl-C to exit ===")
    print("Move robot on/off tape: True=black  False=floor")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}"
          f"  {'error':>6}  {'dist_cm':>8}  state")
    print("-" * 62)
    with RasBot() as bot:
        try:
            while True:
                lo, li, ri, ro, all_on, all_off = read_sensors(bot)
                err  = sensor_error(lo, li, ri, ro)
                dist = error_to_cm(err)
                state = "STOP MARKER" if all_on else ("LINE LOST" if all_off else "tracking")
                err_s  = f"{err:+.0f}" if err is not None else " —"
                dist_s = f"{dist:+.1f}" if dist is not None else "  —"
                print(f"{str(lo):>8}  {str(li):>6}  {str(ri):>6}  {str(ro):>7}"
                      f"  {err_s:>6}  {dist_s:>7}cm  {state}          ",
                      end="\r")
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
            print("\nStopped.")
        finally:
            if cam:
                cam.close()
            bot.stop()
            bot.set_all_leds_color(Color.RED)


if __name__ == "__main__":
    main()
