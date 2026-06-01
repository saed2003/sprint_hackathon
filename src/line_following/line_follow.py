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
 1. Run --calibrate on the tape: confirm True=black tape, False=floor.
 2. Set SKIP_SCAN = True. Place robot ON tape and run.
    Tune BASE_SPEED / TURN_SPEED until it tracks cleanly.
 3. Test stop-marker cross: robot should stop, beep, wait 1 s, resume.
 4. Set SKIP_SCAN = False for the real run.
─────────────────────────────────────────────────────────────────────────────
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from setup_and_api.api import RasBot, Color

# ── tunables ──────────────────────────────────────────────────────────────────
BASE_SPEED        = 60   # normal forward speed (0–255)
TURN_SPEED        = 20   # speed reduction on slow side — gentle curve
HARD_TURN_SPEED   = 35   # speed reduction on slow side — sharp curve
SEARCH_SPEED      = 25   # spin speed during recovery (slow = don't overshoot)
LOOP_HZ           = 50
LOOP_PAUSE        = 1.0 / LOOP_HZ

# When sensors all go dark:
#   Phase 1 (MISS_CREEP_TICKS): creep slowly — bridges small tape gaps.
#   Phase 2: stop fully, then spin to search for the line.
MISS_CREEP_SPEED  = 30   # very slow creep during phase 1
MISS_CREEP_TICKS  = 8    # ~160 ms at 50 Hz before stopping

# How long to spin searching before giving up and staying stopped.
SEARCH_TIMEOUT_S  = 3.0

# After finding the line in recovery: stop and settle before resuming.
RECOVER_SETTLE_S  = 0.25

# After a stop-marker scan: ignore new triggers for this long.
STOP_DEBOUNCE_S   = 1.5

# How long to drive forward after a scan to clear the cross-mark.
CLEAR_MARKER_S    = 0.35

# Live sensor printout every N ticks while running (0 = silent).
DEBUG_EVERY       = 10   # ~200 ms between prints

# Skip the 360° scan — keep True while tuning steering.
SKIP_SCAN         = True
# ─────────────────────────────────────────────────────────────────────────────


def read_sensors(bot):
    """Return (lo, li, ri, ro, error, all_on, all_off).

    Weighted error — negative = line left, positive = line right:
        lo: -2   li: -1   ri: +1   ro: +2
    Centered (li+ri both True) → error = 0.
    all_on  = stop marker (all 4 triggered).
    all_off = line completely gone (none triggered).
    """
    lo, li, ri, ro = bot.read_line_sensors()
    error  = (-2 * lo) + (-1 * li) + (1 * ri) + (2 * ro)
    all_on  = lo and li and ri and ro
    all_off = not (lo or li or ri or ro)
    return lo, li, ri, ro, error, all_on, all_off


def steer(bot, error):
    """Differential steering — no in-place rotation during normal tracking.

    _apply_motors(lf, lr, rf, rr)
    Curve LEFT  → slow left wheels  (right side faster) → robot turns left
    Curve RIGHT → slow right wheels (left side faster)  → robot turns right
    """
    if error == 0:
        bot.forward(BASE_SPEED)

    elif error == -1:
        # Line slightly left → gentle curve left
        bot._apply_motors(
            BASE_SPEED - TURN_SPEED, BASE_SPEED - TURN_SPEED,  # LF, LR slow
            BASE_SPEED,              BASE_SPEED,                # RF, RR full
        )

    elif error == 1:
        # Line slightly right → gentle curve right
        bot._apply_motors(
            BASE_SPEED,              BASE_SPEED,                # LF, LR full
            BASE_SPEED - TURN_SPEED, BASE_SPEED - TURN_SPEED,  # RF, RR slow
        )

    elif error <= -2:
        # Line hard left → sharp curve left
        bot._apply_motors(
            BASE_SPEED - HARD_TURN_SPEED, BASE_SPEED - HARD_TURN_SPEED,
            BASE_SPEED,                   BASE_SPEED,
        )

    else:  # error >= 2
        # Line hard right → sharp curve right
        bot._apply_motors(
            BASE_SPEED,                   BASE_SPEED,
            BASE_SPEED - HARD_TURN_SPEED, BASE_SPEED - HARD_TURN_SPEED,
        )


def _log(msg):
    """Print a full line (clears the debug \r line first)."""
    print(f"\n{msg}")


def recover_line(bot, last_error, log):
    """Stop, then spin slowly toward last known direction to re-find the line.

    Returns True if re-acquired, False if timed out (stay stopped).
    """
    log("line lost — searching...")
    bot.set_all_leds_color(Color.YELLOW)

    spin_left = (last_error <= 0)
    deadline  = time.time() + SEARCH_TIMEOUT_S

    while time.time() < deadline:
        _, _, _, _, _, all_on, all_off = read_sensors(bot)
        if not all_off:
            bot.stop()
            time.sleep(RECOVER_SETTLE_S)   # settle so we don't overshoot
            log("line re-acquired")
            bot.set_all_leds_color(Color.GREEN)
            return True
        if spin_left:
            bot.rotate_left(SEARCH_SPEED)
        else:
            bot.rotate_right(SEARCH_SPEED)
        time.sleep(LOOP_PAUSE)

    bot.stop()
    log("could not re-acquire line — stopping. Check the tape path.")
    bot.set_all_leds_color(Color.RED)
    return False


def do_scan(bot, cam, log):
    """Run 360° scan at a stop marker (or just wait if SKIP_SCAN=True)."""
    bot.set_all_leds_color(Color.BLUE)
    if SKIP_SCAN:
        log("stop marker detected — waiting 1 s (SKIP_SCAN=True)")
        bot.beep(0.1)
        time.sleep(1.0)
        return

    log("stop marker → running 360° scan (do not touch the robot)")
    bot.beep(0.1)
    try:
        from pointcloud import scan360
        session, ply = scan360.scan_and_build(bot, cam, log=log)
        log(f"scan complete → {ply}")
        bot.beep(0.15)
    except Exception as exc:
        log(f"scan error (skipping): {exc}")


def run(bot, cam=None, log=_log):
    """Main line-following loop."""
    log("Line-following started. Ctrl-C to stop.")
    log(f"BASE={BASE_SPEED}  TURN={TURN_SPEED}  HARD={HARD_TURN_SPEED}  "
        f"CREEP={MISS_CREEP_SPEED}/{MISS_CREEP_TICKS}ticks  SKIP_SCAN={SKIP_SCAN}")
    bot.set_all_leds_color(Color.GREEN)

    last_error     = 0
    debounce_until = 0.0
    miss_count     = 0
    in_recovery    = False   # True once we enter Phase 2, until resolved
    tick           = 0

    while True:
        lo, li, ri, ro, error, all_on, all_off = read_sensors(bot)
        now = time.time()

        # ── live debug print ──────────────────────────────────────────────────
        if DEBUG_EVERY and tick % DEBUG_EVERY == 0:
            state = "STOP-MARKER" if all_on else (f"MISS×{miss_count}" if all_off else "tracking")
            print(f"lo={int(lo)} li={int(li)} ri={int(ri)} ro={int(ro)}"
                  f"  err={error:+d}  {state}          ", end="\r")
        tick += 1

        # ── stop marker ───────────────────────────────────────────────────────
        if all_on and now >= debounce_until:
            miss_count  = 0
            in_recovery = False
            bot.stop()
            do_scan(bot, cam, log)
            # drive forward to physically clear the cross-mark
            bot.forward(BASE_SPEED)
            time.sleep(CLEAR_MARKER_S)
            bot.stop()
            debounce_until = time.time() + STOP_DEBOUNCE_S
            last_error = 0
            bot.set_all_leds_color(Color.GREEN)
            continue

        # ── line lost ─────────────────────────────────────────────────────────
        if all_off:
            if not in_recovery:
                miss_count += 1

            if miss_count <= MISS_CREEP_TICKS:
                # Phase 1: creep slowly — handles small tape gaps.
                bot.forward(MISS_CREEP_SPEED)
                time.sleep(LOOP_PAUSE)
                continue

            # Phase 2: confirmed lost — stop and spin to search.
            if not in_recovery:
                in_recovery = True
                bot.stop()
                if not recover_line(bot, last_error, log):
                    break          # tape ended — stay stopped
                miss_count  = 0
                in_recovery = False
                last_error  = 0
            else:
                time.sleep(LOOP_PAUSE)
            continue

        # ── normal tracking ───────────────────────────────────────────────────
        miss_count  = 0
        in_recovery = False
        last_error  = error
        steer(bot, error)
        time.sleep(LOOP_PAUSE)


def calibrate():
    """Live sensor readout to verify polarity before a real run."""
    print("=== Calibration mode — Ctrl-C to exit ===")
    print("Move robot on/off tape: True = black, False = floor")
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
            print("\nCalibration done.")


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
            if cam is not None:
                cam.close()
            bot.stop()
            bot.set_all_leds_color(Color.RED)


if __name__ == "__main__":
    main()
