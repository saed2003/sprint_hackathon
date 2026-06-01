"""
Autonomous line-following — Mode 2 (black tape, 4 IR sensors).

Line following  : black tape detected by 4 IR sensors.
Stop markers    : RED tape cross detected by USB camera (HSV color filter).
                  Falls back to all-4-IR-sensors when SKIP_SCAN=True.

Sensor layout (looking down at the floor beneath the robot):
    [L_OUT]  [L_IN]  |  [R_IN]  [R_OUT]
    True = sensor over dark tape.

Run on the Pi from the project root:
    python3 src/line_following/line_follow.py

Calibrate (verify sensor polarity):
    python3 src/line_following/line_follow.py --calibrate
"""

import os, sys, time, threading
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ── tunables ──────────────────────────────────────────────────────────────────
BASE_SPEED        = 55   # forward speed when centered (0–255)
MIN_SPEED         = 25   # minimum speed allowed during sharp corrections
TURN_SPEED        = 15   # gentle correction reduction (error ±1)
HARD_TURN_SPEED   = 30   # reversed-side speed for strong pivot (error ±2)
SEARCH_SPEED      = 20   # recovery spin speed — slow to avoid overshooting
LOOP_HZ           = 50
LOOP_PAUSE        = 1.0 / LOOP_HZ

MISS_CREEP_SPEED  = 20   # creep speed during phase-1 brief gap
MISS_CREEP_TICKS  = 8    # ~160 ms of creep before stopping (~8 ticks)

SEARCH_TIMEOUT_S  = 4.0  # max spin time in recovery before giving up
RECOVER_SETTLE_S  = 0.15 # settle after recovery before resuming
STOP_DEBOUNCE_S   = 2.0  # ignore stop-marker re-triggers for this long
CLEAR_MARKER_S    = 0.4  # drive forward after scan to clear the cross

# Red tape stop-marker detection (camera)
RED_PIXEL_THRESHOLD = 3000   # min red pixels in lower frame half to trigger
RED_CHECK_INTERVAL  = 0.12   # seconds between camera grabs (~8 fps)

# Set True while tuning steering (uses IR all-on as stop marker, no camera)
SKIP_SCAN         = True

# Live debug print every N ticks
DEBUG_EVERY       = 10
# ─────────────────────────────────────────────────────────────────────────────


# ── sensor reading ────────────────────────────────────────────────────────────

def read_sensors(bot):
    """Return (lo, li, ri, ro, error, all_on, all_off).

    Physical error map (corrected for severity):
        Pattern         error   Meaning
        lo only          -2     robot at LEFT edge  → STRONGEST right turn
        lo + li          -3     robot moderately left → medium right arc
        li only          -1     robot slightly left  → gentle right curve
        li + ri           0     centered             → straight
        ri only          +1     robot slightly right → gentle left curve
        ri + ro          +3     robot moderately right→ medium left arc
        ro only          +2     robot at RIGHT edge  → STRONGEST left turn
        all on            0*    STOP MARKER (handled separately)
    """
    lo, li, ri, ro = bot.read_line_sensors()
    error  = (-2 * lo) + (-1 * li) + (1 * ri) + (2 * ro)
    all_on  = lo and li and ri and ro
    all_off = not (lo or li or ri or ro)
    return lo, li, ri, ro, error, all_on, all_off


# ── adaptive steering ─────────────────────────────────────────────────────────

def _adaptive_speed(error):
    """Reduce speed when at the tape edge — more time to correct."""
    if abs(error) == 2:          # single outer sensor = at the edge (worst)
        return max(MIN_SPEED, BASE_SPEED - 25)
    elif abs(error) == 3:        # both sensors on one side = moderate drift
        return max(MIN_SPEED, BASE_SPEED - 10)
    return BASE_SPEED            # centered or slight drift = full speed


def steer(bot, error, spd=None):
    """Physically correct graduated steering.

    _apply_motors(lf, lr, rf, rr)

    error  0 : straight
    error ±1 : gentle curve    — one side slowed
    error ±3 : medium arc      — one side stopped  (two sensors on tape)
    error ±2 : strong pivot    — one side REVERSED (single outer sensor only)
               ↑ treated MORE aggressively than ±3 because the robot is
               further off the tape (only the outermost sensor touching).
    """
    if spd is None:
        spd = _adaptive_speed(error)

    if error == 0:
        bot.forward(spd)

    elif error == -1:
        # Slightly left drift → gentle RIGHT curve (slow right side)
        slow = max(0, spd - TURN_SPEED)
        bot._apply_motors(spd, spd, slow, slow)

    elif error == 1:
        # Slightly right drift → gentle LEFT curve (slow left side)
        slow = max(0, spd - TURN_SPEED)
        bot._apply_motors(slow, slow, spd, spd)

    elif error == -3:
        # Moderate left drift (lo+li on tape) → medium RIGHT arc
        bot._apply_motors(spd, spd, 0, 0)     # right stopped

    elif error == 3:
        # Moderate right drift (ri+ro on tape) → medium LEFT arc
        bot._apply_motors(0, 0, spd, spd)     # left stopped

    elif error == -2:
        # AT LEFT EDGE (lo only) → STRONG pivot RIGHT
        bot._apply_motors(spd, spd, -HARD_TURN_SPEED, -HARD_TURN_SPEED)

    else:  # error == 2 or any other positive
        # AT RIGHT EDGE (ro only) → STRONG pivot LEFT
        bot._apply_motors(-HARD_TURN_SPEED, -HARD_TURN_SPEED, spd, spd)


# ── red tape detector (background thread) ─────────────────────────────────────

class RedTapeDetector:
    """Grab USB camera frames in a daemon thread and flag red tape."""

    _RED_L1 = np.array([0,   120, 70], dtype=np.uint8)
    _RED_U1 = np.array([10,  255, 255], dtype=np.uint8)
    _RED_L2 = np.array([165, 120, 70], dtype=np.uint8)
    _RED_U2 = np.array([180, 255, 255], dtype=np.uint8)

    def __init__(self):
        self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self._detected = False
        self._lock     = threading.Lock()
        self._running  = True
        self._thread   = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def check(self):
        """Return True (and reset) if red tape was seen since last call."""
        with self._lock:
            val = self._detected
            self._detected = False
        return val

    def close(self):
        self._running = False
        self._cap.release()

    def _loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                h = frame.shape[0]
                floor_hsv = cv2.cvtColor(frame[h // 2:], cv2.COLOR_BGR2HSV)
                mask = cv2.bitwise_or(
                    cv2.inRange(floor_hsv, self._RED_L1, self._RED_U1),
                    cv2.inRange(floor_hsv, self._RED_L2, self._RED_U2),
                )
                if cv2.countNonZero(mask) >= RED_PIXEL_THRESHOLD:
                    with self._lock:
                        self._detected = True
            time.sleep(RED_CHECK_INTERVAL)


# ── line-lost recovery ────────────────────────────────────────────────────────

def _log(msg):
    """Print clearing the debug \r line first."""
    print(f"\n{msg}")


def recover_line(bot, last_error, log):
    """Slow spin toward last known direction until an inner sensor fires.

    Returns True if re-acquired, False if timed out.
    """
    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)

    spin_left = (last_error <= 0)
    deadline  = time.time() + SEARCH_TIMEOUT_S

    while time.time() < deadline:
        _, li, ri, _, _, _, _ = read_sensors(bot)
        if li or ri:                           # inner sensor = properly on tape
            bot.stop()
            time.sleep(0.1)
            bot.forward(MISS_CREEP_SPEED)      # creep forward to centre
            time.sleep(0.15)
            bot.stop()
            time.sleep(RECOVER_SETTLE_S)
            log("line re-acquired")
            bot.set_all_leds_color(Color.GREEN)
            return True
        if spin_left:
            bot.rotate_left(SEARCH_SPEED)
        else:
            bot.rotate_right(SEARCH_SPEED)
        time.sleep(LOOP_PAUSE)

    bot.stop()
    log("could not re-acquire — stopping.")
    bot.set_all_leds_color(Color.RED)
    return False


# ── stop-marker scan ──────────────────────────────────────────────────────────

def do_scan(bot, cam, log):
    """Stop, beep, scan (or wait if SKIP_SCAN), then clear the cross-mark."""
    bot.set_all_leds_color(Color.BLUE)
    bot.beep(0.1)
    if SKIP_SCAN:
        log("stop marker — SKIP_SCAN=True, waiting 1 s")
        time.sleep(1.0)
    else:
        log("stop marker → 360° scan (do not touch the robot)")
        try:
            from pointcloud import scan360
            _, ply = scan360.scan_and_build(bot, cam, log=log)
            log(f"scan complete → {ply}")
            bot.beep(0.15)
        except Exception as exc:
            log(f"scan error (continuing): {exc}")


# ── main control loop ─────────────────────────────────────────────────────────

def run(bot, cam=None, log=_log):
    """Follow the black tape. Stop on red tape cross (or IR all-on in test mode)."""
    log("Line-following started. Ctrl-C to stop.")
    log(f"BASE={BASE_SPEED} MIN={MIN_SPEED} TURN={TURN_SPEED} "
        f"HARD={HARD_TURN_SPEED} SKIP_SCAN={SKIP_SCAN}")

    # Start red-tape detector only for real runs
    red_detector = None
    if not SKIP_SCAN:
        try:
            red_detector = RedTapeDetector()
            log("Red-tape camera detector started.")
        except Exception as e:
            log(f"Camera detector failed ({e}), falling back to IR stop-marker.")

    bot.set_all_leds_color(Color.GREEN)

    last_error     = 0
    debounce_until = 0.0
    miss_count     = 0
    in_recovery    = False
    tick           = 0

    try:
        while True:
            lo, li, ri, ro, error, all_on, all_off = read_sensors(bot)
            now = time.time()

            # ── debug print ───────────────────────────────────────────────────
            if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
                spd = _adaptive_speed(error)
                state = "STOP" if all_on else (f"MISS×{miss_count}" if all_off else f"spd={spd}")
                print(f"lo={int(lo)} li={int(li)} ri={int(ri)} ro={int(ro)}"
                      f"  err={error:+d}  {state}          ", end="\r")
            tick += 1

            # ── stop marker detection ─────────────────────────────────────────
            use_red = red_detector is not None
            stop_triggered = (
                (use_red  and red_detector.check()) or
                (not use_red and all_on)
            ) and now >= debounce_until

            if stop_triggered:
                miss_count  = 0
                in_recovery = False
                bot.stop()
                do_scan(bot, cam, log)
                bot.forward(BASE_SPEED)
                time.sleep(CLEAR_MARKER_S)
                bot.stop()
                debounce_until = time.time() + STOP_DEBOUNCE_S
                last_error = 0
                bot.set_all_leds_color(Color.GREEN)
                continue

            # ── line lost ─────────────────────────────────────────────────────
            if all_off:
                if not in_recovery:
                    miss_count += 1

                if miss_count <= MISS_CREEP_TICKS:
                    # Phase 1: keep steering at creep speed (bridges small gaps)
                    steer(bot, last_error, MISS_CREEP_SPEED)
                    time.sleep(LOOP_PAUSE)
                    continue

                # Phase 2: confirmed lost
                if not in_recovery:
                    in_recovery = True
                    bot.stop()
                    if not recover_line(bot, last_error, log):
                        break
                    miss_count  = 0
                    in_recovery = False
                    last_error  = 0
                else:
                    time.sleep(LOOP_PAUSE)
                continue

            # ── normal tracking ───────────────────────────────────────────────
            miss_count  = 0
            in_recovery = False
            last_error  = error
            steer(bot, error)
            time.sleep(LOOP_PAUSE)

    finally:
        if red_detector:
            red_detector.close()


# ── calibration mode ──────────────────────────────────────────────────────────

def calibrate():
    """Live sensor readout — verify polarity before a real run."""
    print("=== Calibration — Ctrl-C to exit ===")
    print("Move robot on/off tape: True=black tape  False=floor")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}  {'error':>6}  state")
    print("-" * 55)
    with RasBot() as bot:
        try:
            while True:
                lo, li, ri, ro, error, all_on, all_off = read_sensors(bot)
                state = "STOP MARKER" if all_on else ("LINE LOST" if all_off else "tracking")
                print(f"{str(lo):>8}  {str(li):>6}  {str(ri):>6}  {str(ro):>7}"
                      f"  {error:>+6}  {state}          ", end="\r")
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


if __name__ == "__main__":
    main()
