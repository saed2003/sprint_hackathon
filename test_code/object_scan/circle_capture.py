"""
circle_capture.py — drive a 35 cm circle (mecanum geometry) + capture at KNOWN angles.
No vision. Object goes in the MIDDLE. Pi only.

This is the simple "make it work once" path: trust the geometry for the circle, so the
per-shot angle is just the geometric step (no camera needed for alignment). You can
nudge the robot/object by hand at each stop to keep it on the circle (recorded demo, so
retakes are free). Stop-and-shoot = sharp frames + a moment to hand-correct.

WORKFLOW
  1. First dial in the motion with circle_test.py (same --radius/--period/--dir/--strafe
     /--rot). When the marker stays centred, use those numbers here.
  2. Put the object on a small stand at the centre (~R in front), evenly lit.
  3. Run this. It captures shots at 0, 360/N, ... around the circle into
     captures/circle_<ts>/shot_NN/ with angle.txt = the geometric angle.
  4. Build on the LAPTOP:  run.py build captures/circle_<ts> --object db5   (try --dir -1)

USAGE (on the Pi)
  python3 circle_capture.py --radius 0.35 --shots 24            # default: ENTER + hand-nudge each stop
  python3 circle_capture.py --radius 0.35 --shots 24 --auto     # hands-off (no ENTER)
  python3 circle_capture.py --radius 0.35 --period 9 --strafe 45 --rot 0.08 --dir -1
"""
import os
import sys
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
sys.path.insert(0, SRC)

from capture_session import ColorStereoCapture, default_out_root
from circle_test import auto_params      # same geometry/calibration as the motion test

SETTLE = 0.5                              # pause after a move so the frame isn't blurred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.35, help="circle radius (m)")
    ap.add_argument("--shots", type=int, default=24, help="views around the circle")
    ap.add_argument("--period", type=float, default=9.0, help="seconds for a full circle")
    ap.add_argument("--strafe", type=float, default=None, help="override strafe speed")
    ap.add_argument("--rot", type=float, default=None, help="override drift rotation_rate")
    ap.add_argument("--dir", type=int, default=1, help="+1 / -1 circle direction (match the test)")
    ap.add_argument("--auto", action="store_true", help="don't pause for ENTER each stop")
    a = ap.parse_args()

    strafe, rot = auto_params(a.radius, a.period)
    if a.strafe is not None:
        strafe = a.strafe
    if a.rot is not None:
        rot = a.rot
    strafe = max(strafe, 40)                          # mecanum needs ~40 to not stall
    strafe_angle = 180 if a.dir >= 0 else 0           # strafe left for +dir
    step_time = a.period / a.shots                    # seconds of arc between shots
    step_deg = 360.0 / a.shots

    from setup_and_api.api import RasBot, Color
    cam = ColorStereoCapture()
    print(f"circle capture: R={a.radius*100:.0f}cm, {a.shots} shots ({step_deg:.0f} deg each), "
          f"strafe={strafe:.0f} rot={rot:.3f} dir={a.dir}, step {step_time:.2f}s")
    print("Object on a stand at the centre. " + ("(auto)" if a.auto else "(ENTER each stop; nudge by hand if needed)"))
    print("Opening D405..."); cam.start(); print(cam.info())

    session = os.path.join(default_out_root(), "circle_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)

    with RasBot() as bot:
        try:
            bot.set_all_leds_color(Color.BLUE)
        except Exception:
            pass
        try:
            for i in range(a.shots):
                if i > 0:                              # arc to the next position (geometry)
                    bot.drift(strafe, strafe_angle, a.dir * rot)
                    time.sleep(step_time)
                    bot.stop()
                time.sleep(SETTLE)                     # settle before the shot
                if not a.auto:
                    input(f"  shot {i+1}/{a.shots} (~{i*step_deg:.0f} deg): nudge if needed, ENTER to capture")
                folder = os.path.join(session, f"shot_{i:02d}")
                ok = cam.save_to(folder)
                with open(os.path.join(folder, "angle.txt"), "w") as f:
                    f.write(f"{i*step_deg:.3f}\n")
                print(f"    saved shot_{i:02d}" + ("" if ok else "  (FRAME DROPPED)"))
                try:
                    bot.beep(0.05)
                except Exception:
                    pass
            try:
                bot.set_all_leds_color(Color.GREEN); bot.beep(0.15)
            except Exception:
                pass
        finally:
            bot.stop()
            cam.close()

    print(f"\ndone -> {session}")
    print("On the LAPTOP, after git pull:")
    print(f"  ../../../.venv/bin/python run.py build {os.path.basename(session)} --object db5")
    print("  (if the model is mirrored/spread, rebuild with --dir -1)")


if __name__ == "__main__":
    main()
