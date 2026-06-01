"""
Autonomous line-following — Mode 2 (black tape, 4 IR sensors).

The robot follows a dark tape path on the floor. A stop marker (a
perpendicular cross-mark that trips ALL four sensors at once) triggers
a 360° point-cloud scan, then the robot resumes to the next marker.

Sensor layout (looking down at the floor beneath the robot):
    [L_OUT]  [L_IN]  |  [R_IN]  [R_OUT]
    True  = sensor is over the dark tape.
    False = sensor is over the light floor.

Run on the Pi from the project root:
    python3 src/line_following/line_follow.py

Calibrate (verify sensor polarity before a real run):
    python3 src/line_following/line_follow.py --calibrate

── Tuning order ─────────────────────────────────────────────────────────────
 1. Run --calibrate on the tape and confirm True = black, False = floor.
 2. Set SKIP_SCAN = True. Place robot on tape and run.
    Tune BASE_SPEED / TURN_SPEED until it tracks cleanly without oscillating.
 3. Confirm stop-marker cross triggers the beep and 1-second pause.
 4. Set SKIP_SCAN = False for the real run.
─────────────────────────────────────────────────────────────────────────────
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from setup_and_api.api import RasBot, Color
from camera.rs_capture import StereoCapture
from pointcloud import scan360

# ── tunables ──────────────────────────────────────────────────────────────────
BASE_SPEED       = 55    # straight-ahead speed (0–255)
TURN_SPEED       = 25    # how much to reduce the slow side for gentle correction
HARD_TURN_SPEED  = 45    # how much to reduce the slow side for sharp correction
SEARCH_SPEED     = 35    # spin speed during line-lost recovery
LOOP_HZ          = 50
LOOP_PAUSE       = 1.0 / LOOP_HZ

# "Line lost" debounce: only enter recovery after this many consecutive
# all_off reads. Prevents a brief sensor gap from triggering a full spin.
MISS_THRESHOLD   = 5     # ~100 ms at 50 Hz

# After a scan, ignore stop-marker triggers for this many seconds.
STOP_DEBOUNCE_S  = 1.5

# How long to drive forward after a scan to clear the cross-mark.
CLEAR_MARKER_S   = 0.35

# How long to spin searching for a lost line before giving up.
SEARCH_TIMEOUT_S = 3.0

# Set True to skip the 360° scan — use this while tuning steering.
SKIP_SCAN        = True
# ─────────────────────────────────────────────────────────────────────────────


def read_sensors(bot):
    """Return (lo, li, ri, ro, error, all_on, all_off).

    Weighted error: lo=-2  li=-1  ri=+1  ro=+2
    Negative = line to the left.  Positive = line to the right.
    all_on  = stop marker.   all_off = line completely lost.
    """
    lo, li, ri, ro = bot.read_line_sensors()
    error   = (-2 * lo) + (-1 * li) + (1 * ri) + (2 * ro)
    all_on  = lo and li and ri and ro
    all_off = not (lo or li or ri or ro)
    return lo, li, ri, ro, error, all_on, all_off


def steer(bot, error):
    """Differential steering — NO in-place rotation during normal tracking.

    Motor layout: _apply_motors(lf, lr, rf, rr)
    Curve LEFT  → slow LEFT wheels   (right side faster)
    Curve RIGHT → slow RIGHT wheels  (left side faster)
    """
    if error == 0:
        bot.forward(BASE_SPEED)

    elif error == -1:
        # Line slightly left → gentle curve left
        bot._apply_motors(
            BASE_SPEED - TURN_SPEED,   # LF slow
            BASE_SPEED - TURN_SPEED,   # LR slow
            BASE_SPEED,                # RF full
            BASE_SPEED,                # RR full
        )

    elif error == 1:
        # Line slightly right → gentle curve right
        bot._apply_motors(
            BASE_SPEED,                # LF full
            BASE_SPEED,                # LR full
            BASE_SPEED - TURN_SPEED,   # RF slow
            BASE_SPEED - TURN_SPEED,   # RR slow
        )

    elif error <= -2:
        # Line hard left → sharp differential left (no spinning)
        bot._apply_motors(
            BASE_SPEED - HARD_TURN_SPEED,   # LF much slower
            BASE_SPEED - HARD_TURN_SPEED,   # LR much slower
            BASE_SPEED,                     # RF full
            BASE_SPEED,                     # RR full
        )

    else:  # error >= 2
        # Line hard right → sharp differential right (no spinning)
        bot._apply_motors(
            BASE_SPEED,                     # LF full
            BASE_SPEED,                     # LR full
            BASE_SPEED - HARD_TURN_SPEED,   # RF much slower
            BASE_SPEED - HARD_TURN_SPEED,   # RR much slower
        )


def recover_line(bot, last_error, log):
    """Slow spin toward last known line direction until a sensor fires.

    Returns True if re-acquired, False if timed out.
    """
    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)

    spin_left = (last_error <= 0)
    deadline  = time.time() + SEARCH_TIMEOUT_S

    while time.time() < deadline:
        _, _, _, _, _, all_on, all_off = read_sensors(bot)
        if not all_off:
            log("line re-acquired")
            bot.set_all_leds_color(Color.GREEN)
            return True
        if spin_left:
            bot.rotate_left(SEARCH_SPEED)
        else:
            bot.rotate_right(SEARCH_SPEED)
        time.sleep(LOOP_PAUSE)

    bot.stop()
    log("line lost — could not re-acquire. Stopping.")
    bot.set_all_leds_color(Color.RED)
    return False


def do_scan(bot, cam, log):
    """Run the 360° scan at a stop marker."""
    bot.set_all_leds_color(Color.BLUE)
    if SKIP_SCAN:
        log("stop marker — SKIP_SCAN=True, waiting 1 s")
        bot.beep(0.1)
        time.sleep(1.0)
        return

    log("stop marker → running 360° scan (do not touch the robot)")
    bot.beep(0.1)
    try:
        session, ply = scan360.scan_and_build(bot, cam, log=log)
        log(f"scan complete → {ply}")
        bot.beep(0.15)
    except Exception as exc:
        log(f"scan error (continuing): {exc}")


def run(bot, cam, log=print):
    """Follow the tape. At each stop marker: scan, clear, resume."""
    log("Line-following started. Ctrl-C to stop.")
    log(f"  BASE={BASE_SPEED}  TURN={TURN_SPEED}  HARD={HARD_TURN_SPEED}  SKIP_SCAN={SKIP_SCAN}")
    bot.set_all_leds_color(Color.GREEN)

    last_error     = 0
    debounce_until = 0.0
    miss_count     = 0      # consecutive all_off reads

    while True:
        lo, li, ri, ro, error, all_on, all_off = read_sensors(bot)
        now = time.time()

        # ── stop marker ───────────────────────────────────────────────────────
        if all_on and now >= debounce_until:
            miss_count = 0
            bot.stop()
            do_scan(bot, cam, log)
            bot.forward(BASE_SPEED)
            time.sleep(CLEAR_MARKER_S)
            bot.stop()
            debounce_until = time.time() + STOP_DEBOUNCE_S
            last_error = 0
            bot.set_all_leds_color(Color.GREEN)
            continue

        # ── line lost (debounced) ─────────────────────────────────────────────
        if all_off:
            miss_count += 1
            if miss_count < MISS_THRESHOLD:
                # brief gap — keep moving, don't panic yet
                time.sleep(LOOP_PAUSE)
                continue
            # confirmed lost
            bot.stop()
            if not recover_line(bot, last_error, log):
                break
            miss_count = 0
            last_error = 0
            continue

        # ── normal tracking ───────────────────────────────────────────────────
        miss_count = 0
        last_error = error
        steer(bot, error)
        time.sleep(LOOP_PAUSE)


def calibrate():
    """Live sensor readout — verify polarity before a real run."""
    print("=== Calibration mode — Ctrl-C to exit ===")
    print("Move the robot on/off the tape and confirm True=black, False=floor.")
    print(f"{'L_OUT':>8}  {'L_IN':>6}  {'R_IN':>6}  {'R_OUT':>7}  {'error':>6}  state")
    print("-" * 55)
    with RasBot() as bot:
        try:
            while True:
                lo, li, ri, ro, error, all_on, all_off = read_sensors(bot)
                state = "STOP MARKER" if all_on else ("LINE LOST" if all_off else "tracking")
                print(
                    f"{str(lo):>8}  {str(li):>6}  {str(ri):>6}  {str(ro):>7}"
                    f"  {error:>+6}  {state}",
                    end="\r",
                )
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nCalibration done.")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--calibrate":
        calibrate()
        return

    with RasBot() as bot:
        cam = StereoCapture()
        try:
            run(bot, cam)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            cam.close()
            bot.stop()
            bot.set_all_leds_color(Color.RED)


if __name__ == "__main__":
    main()
