"""
Autonomous line-following movement (Mode 2 from the brief) — SCAFFOLD, NOT YET TESTED.

The robot follows a dark tape path using its 4 downward IR line sensors. A "stop
marker" (a perpendicular cross-mark that trips ALL four sensors at once) marks a
capture location: the robot halts, runs the 360 scan, then resumes to the next marker.

This file is a starting template that wires up the real RasBot API and the existing
scan routine the SAME way wasd/drive.py does. The control logic below is a first
draft — the thresholds, speeds, and turn gains MUST be tuned on the real robot, and
the stop-marker debounce is intentionally simple. Treat it as a skeleton to build on.

Run on the Pi (once implemented/tuned):
  python3 line_following/line_follow.py
"""

import os
import sys
import time

# project root = the folder that contains line_following/, camera/, pointcloud/, rasbot/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rasbot.api import RasBot, Color
from src.camera.rs_capture import StereoCapture
from src.pointcloud import scan360

# ── tunables (ALL need tuning on the real floor/tape) ───────────────────────────
BASE_SPEED      = 80     # forward speed while the line is centered
TURN_SPEED      = 70     # rotate speed used to steer back onto the line
LOOP_PAUSE      = 0.02   # seconds between sensor reads
LOST_LINE_STOP  = True   # stop if all four sensors go light (line lost) vs. keep searching


def at_stop_marker(sensors):
    """A stop marker trips all four sensors at once (the perpendicular cross-mark)."""
    left_outer, left_inner, right_inner, right_outer = sensors
    return left_outer and left_inner and right_inner and right_outer


def follow_step(bot, sensors):
    """One steering decision from the 4 line sensors.

    sensors = (left_outer, left_inner, right_inner, right_outer), True = over dark line.
    NOTE: first-draft logic — verify the sensor order/polarity on the robot and tune.
    """
    left_outer, left_inner, right_inner, right_outer = sensors

    if left_inner and right_inner:
        bot.forward(BASE_SPEED)                 # centered → go straight
    elif left_inner or left_outer:
        bot.rotate_left(TURN_SPEED)             # line drifted left → correct left
    elif right_inner or right_outer:
        bot.rotate_right(TURN_SPEED)            # line drifted right → correct right
    elif LOST_LINE_STOP:
        bot.stop()                              # line lost → stop (or implement a search)


def run(bot, cam, log=print):
    """Follow the line; at each stop marker, run a 360 scan, then resume.

    TODO before this works on hardware:
      - confirm read_line_sensors() order/polarity for OUR tape + lighting
      - tune BASE_SPEED / TURN_SPEED / the steering branches above
      - add proper stop-marker debounce (don't re-trigger on the same mark)
      - decide what to do when the line is lost (stop vs. sweep to re-find it)
    """
    log("line-following: starting (Ctrl-C to stop). UNTESTED scaffold — watch the robot.")
    bot.set_all_leds_color(Color.GREEN)
    try:
        while True:
            sensors = bot.read_line_sensors()

            if at_stop_marker(sensors):
                bot.stop()
                bot.set_all_leds_color(Color.BLUE)
                log("stop marker → running 360 scan (do not touch the robot)")
                session, ply = scan360.scan_and_build(bot, cam, log=log)
                log(f"scan done → {ply}")
                bot.set_all_leds_color(Color.GREEN)
                # drive forward briefly to clear the marker before resuming (tune this)
                bot.forward(BASE_SPEED)
                time.sleep(0.4)
                continue

            follow_step(bot, sensors)
            time.sleep(LOOP_PAUSE)
    finally:
        bot.stop()


def main():
    with RasBot() as bot:
        cam = StereoCapture()
        try:
            run(bot, cam)
        except KeyboardInterrupt:
            print("\nstopped.")
        finally:
            cam.close()


if __name__ == "__main__":
    main()
