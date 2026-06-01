"""
Autonomous line-following — Mode 2 (black tape, 4 IR sensors).

Line following  : black tape, 4 IR sensors, pattern-based steering.
Stop markers    : RED tape cross detected by USB camera (HSV color filter).
                  Falls back to all-4-IR-sensors when SKIP_SCAN=True.

Sensor layout (looking down at the floor beneath the robot):
    [L_OUT]  [L_IN]  |  [R_IN]  [R_OUT]
    True = sensor over dark tape.

Run:       python3 src/line_following/line_follow.py
Calibrate: python3 src/line_following/line_follow.py --calibrate
"""

import os, sys, time, threading
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ── tunables ──────────────────────────────────────────────────────────────────
BASE_SPEED        = 55   # straight-ahead speed (0–255)
MIN_SPEED         = 25   # minimum allowed speed (sharp turns / edge)
TURN_SPEED        = 15   # slow-side reduction for gentle curves
SEARCH_SPEED      = 20   # recovery spin speed (slow = less overshoot)
LOOP_HZ           = 50
LOOP_PAUSE        = 1.0 / LOOP_HZ

MISS_CREEP_SPEED  = 20   # speed during phase-1 brief gap
MISS_CREEP_TICKS  = 8    # ticks of creep before full stop (~160 ms)

SEARCH_TIMEOUT_S  = 4.0
RECOVER_SETTLE_S  = 0.15
STOP_DEBOUNCE_S   = 2.0
CLEAR_MARKER_S    = 0.4

RED_PIXEL_THRESHOLD = 3000
RED_CHECK_INTERVAL  = 0.12

SKIP_SCAN         = True   # True = IR stop-marker + no scan (tuning mode)
DEBUG_EVERY       = 10
# ─────────────────────────────────────────────────────────────────────────────


# ── sensor reading ────────────────────────────────────────────────────────────

def read_sensors(bot):
    """Return (lo, li, ri, ro, all_on, all_off)."""
    lo, li, ri, ro = bot.read_line_sensors()
    all_on  = lo and li and ri and ro
    all_off = not (lo or li or ri or ro)
    return lo, li, ri, ro, all_on, all_off


# ── pattern-based steering ────────────────────────────────────────────────────
#
# Why pattern-based instead of weighted-error:
#   The weighted sum collapses distinct situations into the same number.
#   e.g. (li+ri+ro) and (ro only) both give error=+2, but need very different
#   responses — gentle curve vs strong arc. Checking each sensor directly fixes
#   this without ambiguity.
#
# Sensor pattern → physical position → required action:
#
#   lo  li  ri  ro   position               turn needed
#   0   1   1   0    centred                straight
#   0   1   1   1    3-right (slight right) gentle LEFT  (slow left side)
#   1   1   1   0    3-left  (slight left)  gentle RIGHT (slow right side)
#   0   0   1   0    ri only (slight right) gentle LEFT
#   0   1   0   0    li only (slight left)  gentle RIGHT
#   0   0   1   1    ri+ro   (medium right) LEFT arc     (left side stopped)
#   1   1   0   0    lo+li   (medium left)  RIGHT arc    (right side stopped)
#   0   0   0   1    ro only (far right)    LEFT arc     (left side stopped)
#   1   0   0   0    lo only (far left)     RIGHT arc    (right side stopped)
#   all four          stop marker           handled elsewhere
#   none              lost                  handled elsewhere
#
# NO wheel reversals during normal tracking — prevents overshoot.

def _speed(lo, li, ri, ro):
    """Adaptive speed: slow down near the edge."""
    n = int(lo) + int(li) + int(ri) + int(ro)
    if n == 1 and (lo or ro):        # single outer sensor = at edge
        return max(MIN_SPEED, BASE_SPEED - 20)
    if n == 2 and (lo and ro) or (lo and ri) or (li and ro):
        return max(MIN_SPEED, BASE_SPEED - 15)   # skipped sensors, off-centre
    return BASE_SPEED


def steer(bot, lo, li, ri, ro, spd=None):
    """Pattern-based differential steering. No wheel reversals."""
    if spd is None:
        spd = _speed(lo, li, ri, ro)

    slow = max(0, spd - TURN_SPEED)

    if li and ri:
        if not lo and not ro:
            # ── centred ──────────────────────────────────────────────────────
            bot.forward(spd)
        elif ro and not lo:
            # ── li+ri+ro : slight right → gentle LEFT ────────────────────────
            bot._apply_motors(slow, slow, spd, spd)
        elif lo and not ro:
            # ── lo+li+ri : slight left → gentle RIGHT ────────────────────────
            bot._apply_motors(spd, spd, slow, slow)
        else:
            bot.forward(spd)                    # all four = stop marker case

    elif ri and not li:
        if ro:
            # ── ri+ro : medium right → LEFT arc ──────────────────────────────
            bot._apply_motors(0, 0, spd, spd)
        else:
            # ── ri only : slight right → gentle LEFT ─────────────────────────
            bot._apply_motors(slow, slow, spd, spd)

    elif li and not ri:
        if lo:
            # ── lo+li : medium left → RIGHT arc ──────────────────────────────
            bot._apply_motors(spd, spd, 0, 0)
        else:
            # ── li only : slight left → gentle RIGHT ─────────────────────────
            bot._apply_motors(spd, spd, slow, slow)

    elif ro:
        # ── ro only : far right edge → LEFT arc (strong) ─────────────────────
        bot._apply_motors(0, 0, spd, spd)

    elif lo:
        # ── lo only : far left edge → RIGHT arc (strong) ─────────────────────
        bot._apply_motors(spd, spd, 0, 0)

    else:
        bot.forward(spd)                        # fallback


# ── red tape detector (background thread) ─────────────────────────────────────

class RedTapeDetector:
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
                floor = cv2.cvtColor(frame[h // 2:], cv2.COLOR_BGR2HSV)
                mask = cv2.bitwise_or(
                    cv2.inRange(floor, self._RED_L1, self._RED_U1),
                    cv2.inRange(floor, self._RED_L2, self._RED_U2),
                )
                if cv2.countNonZero(mask) >= RED_PIXEL_THRESHOLD:
                    with self._lock:
                        self._detected = True
            time.sleep(RED_CHECK_INTERVAL)


# ── recovery ──────────────────────────────────────────────────────────────────

def _log(msg):
    print(f"\n{msg}")


def recover_line(bot, last_sensors, log):
    """Spin toward last known tape direction until an inner sensor fires."""
    lo, li, ri, ro = last_sensors
    # spin toward whichever side last saw the tape
    spin_left = int(ri) + int(ro) > int(lo) + int(li)

    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)
    deadline = time.time() + SEARCH_TIMEOUT_S

    while time.time() < deadline:
        _, li2, ri2, _, _, _ = read_sensors(bot)
        if li2 or ri2:                       # inner sensor = properly on tape
            bot.stop()
            time.sleep(0.1)
            bot.forward(MISS_CREEP_SPEED)
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
    except Exception as exc:
        log(f"scan error: {exc}")


# ── main loop ─────────────────────────────────────────────────────────────────

def run(bot, cam=None, log=_log):
    log("Line-following started. Ctrl-C to stop.")
    log(f"BASE={BASE_SPEED} MIN={MIN_SPEED} TURN={TURN_SPEED} SKIP_SCAN={SKIP_SCAN}")

    red_detector = None
    if not SKIP_SCAN:
        try:
            red_detector = RedTapeDetector()
            log("Red-tape detector started.")
        except Exception as e:
            log(f"Camera detector failed ({e}) — using IR fallback.")

    bot.set_all_leds_color(Color.GREEN)

    last_sensors   = (False, False, False, False)
    debounce_until = 0.0
    miss_count     = 0
    in_recovery    = False
    tick           = 0

    try:
        while True:
            lo, li, ri, ro, all_on, all_off = read_sensors(bot)
            now = time.time()

            # ── debug ─────────────────────────────────────────────────────────
            if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
                spd = _speed(lo, li, ri, ro)
                n   = int(lo)+int(li)+int(ri)+int(ro)
                state = "STOP" if all_on else (f"MISS×{miss_count}" if all_off else f"n={n} spd={spd}")
                print(f"lo={int(lo)} li={int(li)} ri={int(ri)} ro={int(ro)}"
                      f"  {state}          ", end="\r")
            tick += 1

            # ── stop marker ───────────────────────────────────────────────────
            use_cam = red_detector is not None
            stop_triggered = (
                (use_cam  and red_detector.check()) or
                (not use_cam and all_on)
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
                last_sensors = (False, False, False, False)
                bot.set_all_leds_color(Color.GREEN)
                continue

            # ── line lost ─────────────────────────────────────────────────────
            if all_off:
                if not in_recovery:
                    miss_count += 1

                if miss_count <= MISS_CREEP_TICKS:
                    # Phase 1: keep last steering at creep speed
                    steer(bot, *last_sensors, MISS_CREEP_SPEED)
                    time.sleep(LOOP_PAUSE)
                    continue

                # Phase 2: confirmed lost
                if not in_recovery:
                    in_recovery = True
                    bot.stop()
                    if not recover_line(bot, last_sensors, log):
                        break
                    miss_count  = 0
                    in_recovery = False
                    last_sensors = (False, False, False, False)
                else:
                    time.sleep(LOOP_PAUSE)
                continue

            # ── normal tracking ───────────────────────────────────────────────
            miss_count   = 0
            in_recovery  = False
            last_sensors = (lo, li, ri, ro)
            steer(bot, lo, li, ri, ro)
            time.sleep(LOOP_PAUSE)

    finally:
        if red_detector:
            red_detector.close()


# ── calibrate ─────────────────────────────────────────────────────────────────

def calibrate():
    print("=== Calibration — Ctrl-C to exit ===")
    print("Move robot on/off tape: True=black  False=floor")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}  state")
    print("-" * 50)
    with RasBot() as bot:
        try:
            while True:
                lo, li, ri, ro, all_on, all_off = read_sensors(bot)
                state = "STOP MARKER" if all_on else ("LINE LOST" if all_off else "tracking")
                print(f"{str(lo):>8}  {str(li):>6}  {str(ri):>6}  {str(ro):>7}  {state}          ",
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
