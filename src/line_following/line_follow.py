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
 1. Run --calibrate: put the robot on the tape and confirm each sensor reads
    True when over the black line and False when off it.
 2. Set SKIP_SCAN = True. Run on the floor and tune BASE_SPEED / TURN_SPEED
    until the robot tracks the line cleanly without oscillating.
 3. Tape a stop-marker cross on the floor and confirm the robot stops on it.
 4. Set SKIP_SCAN = False for the real run.
─────────────────────────────────────────────────────────────────────────────
"""

import os
import sys
import time

# Add src/ to sys.path so imports work from anywhere.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from setup_and_api.api import RasBot, Color
from camera.rs_capture import StereoCapture
from pointcloud import scan360

# ── tunables ──────────────────────────────────────────────────────────────────
BASE_SPEED       = 80    # straight-ahead speed (0–255); start low, increase slowly
TURN_SPEED       = 50    # gentle correction: how much to reduce the slower side
SEARCH_SPEED     = 45    # spin speed during line-lost recovery
LOOP_HZ          = 50    # sensor reads per second
LOOP_PAUSE       = 1.0 / LOOP_HZ

# After a scan, ignore stop-marker triggers for this many seconds so the robot
# does not re-trigger while still sitting on the cross-mark.
STOP_DEBOUNCE_S  = 1.5

# How long to drive forward after a scan to physically clear the cross-mark.
CLEAR_MARKER_S   = 0.35

# How long to spin searching for a lost line before giving up.
SEARCH_TIMEOUT_S = 3.0

# Set True to skip the 360° scan — use this while tuning the steering.
SKIP_SCAN        = True
# ─────────────────────────────────────────────────────────────────────────────


# ── sensor helpers ────────────────────────────────────────────────────────────

def read_sensors(bot):
    """Read the 4 IR sensors and compute a weighted steering error.

    Returns (lo, li, ri, ro, error, all_on, all_off).

    Weighted error (negative = line to the left, positive = line to the right):
        lo:  -2   li: -1   ri: +1   ro: +2
    Centered on tape (li + ri both True) → error = 0.
    all_on  → stop marker (all four sensors triggered at once).
    all_off → line completely lost (no sensor triggered).
    """
    lo, li, ri, ro = bot.read_line_sensors()
    error   = (-2 * lo) + (-1 * li) + (1 * ri) + (2 * ro)
    all_on  = lo and li and ri and ro
    all_off = not (lo or li or ri or ro)
    return lo, li, ri, ro, error, all_on, all_off


# ── steering ──────────────────────────────────────────────────────────────────

def steer(bot, error):
    """One steering command derived from the weighted sensor error.

    Motor layout for _apply_motors(lf, lr, rf, rr):
        lf = left-front   lr = left-rear
        rf = right-front  rr = right-rear

    Differential rule:
        Curve LEFT  → slow LEFT wheels, keep RIGHT full
        Curve RIGHT → slow RIGHT wheels, keep LEFT full
        Hard LEFT   → rotate_left()  (in-place CCW)
        Hard RIGHT  → rotate_right() (in-place CW)
    """
    if error == 0:
        # Perfectly centred — go straight.
        bot.forward(BASE_SPEED)

    elif error == -1:
        # Line slightly to the LEFT → curve LEFT (slow left side).
        bot._apply_motors(
            BASE_SPEED - TURN_SPEED // 2,   # LF slower
            BASE_SPEED - TURN_SPEED // 2,   # LR slower
            BASE_SPEED,                     # RF full
            BASE_SPEED,                     # RR full
        )

    elif error == 1:
        # Line slightly to the RIGHT → curve RIGHT (slow right side).
        bot._apply_motors(
            BASE_SPEED,                     # LF full
            BASE_SPEED,                     # LR full
            BASE_SPEED - TURN_SPEED // 2,   # RF slower
            BASE_SPEED - TURN_SPEED // 2,   # RR slower
        )

    elif error <= -2:
        # Line hard to the LEFT → rotate left (CCW) in place.
        bot.rotate_left(TURN_SPEED)

    else:  # error >= 2
        # Line hard to the RIGHT → rotate right (CW) in place.
        bot.rotate_right(TURN_SPEED)


# ── line-lost recovery ────────────────────────────────────────────────────────

def recover_line(bot, last_error, log):
    """Spin toward the last known line direction until a sensor fires.

    Returns True if the line was re-acquired, False if timed out.
    """
    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)

    # Spin toward whichever side the line was last seen on.
    spin_left = (last_error <= 0)
    deadline  = time.time() + SEARCH_TIMEOUT_S

    while time.time() < deadline:
        _, _, _, _, _, all_on, all_off = read_sensors(bot)
        if not all_off:                  # at least one sensor sees the line
            log("line re-acquired")
            bot.set_all_leds_color(Color.GREEN)
            return True
        if spin_left:
            bot.rotate_left(SEARCH_SPEED)
        else:
            bot.rotate_right(SEARCH_SPEED)
        time.sleep(LOOP_PAUSE)

    bot.stop()
    log("line lost and could not be re-acquired — stopping. Check the tape path.")
    bot.set_all_leds_color(Color.RED)
    return False


# ── stop-marker scan ──────────────────────────────────────────────────────────

def do_scan(bot, cam, log):
    """Run the 360° point-cloud scan at a stop marker."""
    bot.set_all_leds_color(Color.BLUE)
    if SKIP_SCAN:
        log("stop marker detected — SKIP_SCAN=True, waiting 1 s")
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
        log(f"scan error (skipping and continuing): {exc}")


# ── main control loop ─────────────────────────────────────────────────────────

def run(bot, cam, log=print):
    """Follow the tape. At each stop marker: scan, clear, resume."""
    log("Line-following started. Press Ctrl-C to stop.")
    log(f"  BASE_SPEED={BASE_SPEED}  TURN_SPEED={TURN_SPEED}  SKIP_SCAN={SKIP_SCAN}")
    bot.set_all_leds_color(Color.GREEN)

    last_error     = 0
    debounce_until = 0.0

    while True:
        lo, li, ri, ro, error, all_on, all_off = read_sensors(bot)
        now = time.time()

        # ── stop marker ───────────────────────────────────────────────────────
        if all_on and now >= debounce_until:
            bot.stop()
            do_scan(bot, cam, log)

            # Drive forward to physically clear the cross-mark.
            bot.forward(BASE_SPEED)
            time.sleep(CLEAR_MARKER_S)
            bot.stop()

            debounce_until = time.time() + STOP_DEBOUNCE_S
            last_error = 0
            bot.set_all_leds_color(Color.GREEN)
            continue

        # ── line completely lost ──────────────────────────────────────────────
        if all_off:
            bot.stop()
            if not recover_line(bot, last_error, log):
                break       # give up — operator must intervene
            last_error = 0
            continue

        # ── normal steering ───────────────────────────────────────────────────
        last_error = error
        steer(bot, error)
        time.sleep(LOOP_PAUSE)


# ── calibration helper ────────────────────────────────────────────────────────

def calibrate():
    """Print live sensor readings so you can verify polarity before a real run.

    Move the robot over and off the tape and confirm:
        True  when the sensor is over the BLACK line
        False when the sensor is over the LIGHT floor
    """
    print("=== Calibration mode — Ctrl-C to exit ===")
    print("Place robot on/off the tape and verify sensor polarity.")
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


# ── entry point ───────────────────────────────────────────────────────────────

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
