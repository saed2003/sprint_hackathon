"""
circle_test.py — perfect the ORBIT MOTION first, BEFORE adding the car (Pi only).

Your idea: use the mecanum wheels' geometry to drive a smooth CIRCLE around a fixed
point, tune it until it's good, then put the car at the centre.

HOW (mecanum geometry): a mecanum base can translate AND rotate at the same time
(bot.drift). If it STRAFES sideways at speed v while ROTATING at rate w, its turn-
centre (the point it pivots around) sits a fixed distance in FRONT of it:

        turn-centre distance  R  =  v / w

Put the car at that point and the nose keeps pointing at it as the robot circles — the
car stays framed by geometry, no vision needed. This script drives that motion so you
can mark the floor and dial it in.

HONEST LIMIT: no encoders + mecanum slip => an open-loop circle won't be perfect; it
drifts over a lap. So use the geometry to keep the car FRAMED, but recover the real
per-frame ANGLE from the images later (dense overlap). Don't trust timing for angles.

USAGE  (put a marker / small box on the floor ~R in front, run, watch it circle that):
    python3 circle_test.py --radius 0.30 --period 9        # auto strafe/rot from calibration
    python3 circle_test.py --radius 0.30 --period 9 --watch # camera prints how framed it stays
    python3 circle_test.py --strafe 45 --rot 0.08 --seconds 9   # manual tune
    python3 circle_test.py --radius 0.30 --period 9 --dir -1     # circle the other way

TUNE:
    circle too BIG (robot too far from the marker) -> raise --rot or lower --strafe
    circle too TIGHT (robot too close)             -> lower --rot or raise --strafe
    doesn't return to start after one --period     -> adjust --period
    spirals in/out instead of circling             -> flip --dir, then re-tune
"""
import os
import sys
import math
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
sys.path.insert(0, SRC)

# ── calibration reference (from capture_orbit.py --calibrate-*, at motor speed 40) ──
SPIN_DPS_AT_40 = 180.0    # deg/s spinning in place at speed 40   (540 deg in 3.0 s)
FWD_MPS_AT_40  = 0.40     # m/s driving forward at speed 40       (1.2 m in 3.0 s)
STRAFE_EFF     = 0.7      # mecanum strafe covers ~70% of forward distance/sec (TUNE)
HALF_W         = 124.5    # CHASSIS_HALF_WIDTH from the API (drift rotation scaling)
MIN_MOVE_SPEED = 40       # below this the wheels tend to stall (mecanum needs some push)


def auto_params(radius, period):
    """Starting strafe-speed + drift rotation_rate for a circle of `radius` (m) done in
    `period` s, derived from the calibration above. A principled first guess — then tune."""
    omega = 360.0 / period                                  # deg/s the base must spin
    rot_speed_equiv = 40.0 * omega / SPIN_DPS_AT_40         # equivalent rotate() speed
    rot_rate = rot_speed_equiv / HALF_W                     # -> drift rotation_rate
    v = 2.0 * math.pi * radius / period                     # tangential speed (m/s)
    strafe = 40.0 * v / (FWD_MPS_AT_40 * STRAFE_EFF)        # -> strafe speed
    return strafe, rot_rate


def watch_framing(cam, radius):
    """One camera reading: (distance_m, horizontal_offset_px) of the closest blob — so you
    can SEE how steady the centre marker stays. Needs a textured marker/box at the centre."""
    depth = cam.grab_depth()
    if depth is None:
        return None
    m = (depth > 0.08) & (depth < radius + 0.25)            # the close thing = the marker
    if m.sum() < 500:
        return (None, None)
    ys, xs = np.where(m)
    return (float(np.median(depth[m])), float(xs.mean() - cam.intr.ppx))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.30, help="target turn-centre distance (m)")
    ap.add_argument("--period", type=float, default=9.0, help="seconds for one full circle")
    ap.add_argument("--seconds", type=float, default=None, help="run time (default = one period)")
    ap.add_argument("--strafe", type=float, default=None, help="override strafe speed")
    ap.add_argument("--rot", type=float, default=None, help="override drift rotation_rate")
    ap.add_argument("--dir", type=int, default=1, help="+1 / -1 circle direction")
    ap.add_argument("--watch", action="store_true", help="camera reports framing each ~0.5s")
    a = ap.parse_args()

    strafe, rot = auto_params(a.radius, a.period)
    if a.strafe is not None:
        strafe = a.strafe
    if a.rot is not None:
        rot = a.rot
    seconds = a.seconds if a.seconds is not None else a.period

    if strafe < MIN_MOVE_SPEED:
        print(f"NOTE: computed strafe speed {strafe:.0f} < {MIN_MOVE_SPEED} (may stall). "
              f"Use a shorter --period or smaller --radius for a brisker circle.")
        strafe = max(strafe, MIN_MOVE_SPEED)

    strafe_angle = 180 if a.dir >= 0 else 0                 # strafe left for +dir
    print(f"circle test: R~{a.radius*100:.0f}cm, period~{a.period:.0f}s, "
          f"strafe={strafe:.0f} rot={rot:.3f} dir={a.dir}, run {seconds:.0f}s")
    print("Mark the floor where the car will sit (~R in front). Watch it circle that point.\n")

    from setup_and_api.api import RasBot, Color
    cam = None
    if a.watch:
        from capture_session import ColorStereoCapture
        global np
        import numpy as np
        cam = ColorStereoCapture(); cam.start()

    with RasBot() as bot:
        try:
            bot.set_all_leds_color(Color.BLUE)
        except Exception:
            pass
        bot.drift(strafe, strafe_angle, a.dir * rot)       # start the continuous orbit
        t0 = time.time()
        try:
            while time.time() - t0 < seconds:
                if cam is not None:
                    r = watch_framing(cam, a.radius)
                    if r and r[0] is not None:
                        print(f"  t={time.time()-t0:4.1f}s  marker dist={r[0]*100:5.1f}cm  "
                              f"offset={r[1]:+5.0f}px  (steady dist + ~0 offset = good circle)")
                    elif r:
                        print(f"  t={time.time()-t0:4.1f}s  marker LOST (drifted off / out of range)")
                time.sleep(0.5)
        finally:
            bot.stop()
            try:
                bot.set_all_leds_color(Color.GREEN)
            except Exception:
                pass
            if cam is not None:
                cam.close()
    print("\ndone. If it circled the marker and came back near start, the motion is good — "
          "add the car at the marker next.")


if __name__ == "__main__":
    main()
