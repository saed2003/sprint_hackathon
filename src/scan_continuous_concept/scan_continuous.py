"""
CONTINUOUS-ROTATION 360 scan  (concept / experiment — isolated in this folder)
==============================================================================

This is the ALTERNATIVE idea from the Google-doc chat: instead of the shipped
scan360.py "stop -> shoot -> rotate" 10 times, the robot does ONE slow continuous
in-place spin while the D405 streams; we keep every Nth frame (default every 5th of
30 fps ~= 6 fps), save each as a shot_NN folder, and then reuse the EXISTING merge in
pointcloud/scan360.py to fuse them into one cloud.

It deliberately touches NOTHING outside scan_continuous_concept/. It uses its own copy
of the camera helper (rs_capture.py here) and only *reads* the unchanged merge functions
from pointcloud/scan360.py. Delete this folder to fully revert.

Can the D405 do this?  -> Yes. It streams depth + hardware-synced stereo IR at 30 fps,
so depth stays valid even while moving; the only motion cost is IR blur, which a short
locked exposure (CONT_EXPOSURE_US) keeps small at these speeds.

How alignment works (and why this can actually be GOOD here):
  We do NOT trust the spin timing for pose. We save the frames WITHOUT an angle.txt, so
  scan360's merge falls into its IMAGE-MEASURED mode: it recovers each step's yaw from the
  overlap between consecutive IR frames (ORB -> homography H = K R K^-1 -> yaw). With a
  continuous sweep, consecutive frames are only ~5 deg apart and overlap ~95%, which is
  exactly where that homography estimate is most robust (far better than the shipped scan's
  big 36 deg jumps). The spin timing is only used to decide WHEN to stop (~one full turn);
  it never enters the geometry.

Trade-off vs scan360.py (be honest):
  + faster, smoother, no start/stop shake, dense overlap -> robust per-step vision angle.
  - each frame has some motion blur -> per-frame depth a touch noisier than a dead-stop shot.
  - many frames (~50-70) -> the on-Pi merge takes longer (ORB on each consecutive pair).
  Net: at a slow spin with locked short exposure this should be competitive with, and
  smoother than, the discrete scan. At a fast spin it degrades (blur + bigger gaps).

Run it (on the robot, inside the Pi desktop/VNC or SSH — no display needed):
  python3 scan_continuous_concept/scan_continuous.py                 # spin, capture, build
  python3 scan_continuous_concept/scan_continuous.py --seconds 16    # slower full turn
  python3 scan_continuous_concept/scan_continuous.py --speed 30 --sub 6 --exposure 5000
  python3 scan_continuous_concept/scan_continuous.py --no-exposure   # let auto-exposure run

Rebuild a cloud from frames already captured (no robot needed):
  python3 scan_continuous_concept/scan_continuous.py captures/cscan_20260602_1200

Calibrate the one number that matters (CONT_FULL_ROTATION_SEC): tape the floor, start the
robot spinning at CONT_SPIN_SPEED, time one full 360 by eye/stopwatch, set it here. It only
controls when the spin stops, so it isn't critical — a little over a full turn is fine.
"""

import os
import sys
import time

import numpy as np

# import roots: this folder (for the local rs_capture copy) + src/ (for the unchanged
# pointcloud.scan360 merge and the setup_and_api robot API).
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, SRC)

import rs_capture                       # LOCAL concept copy (with save_arrays / exposure)
from pointcloud import scan360          # UNCHANGED shipped merge (build_from_session etc.)

# ── tunables ──────────────────────────────────────────────────────────────────
CONT_SPIN_SPEED      = 35       # motor speed for the continuous in-place spin (slow = sharp)
CONT_FULL_ROTATION_SEC = 12.0   # CALIBRATE: seconds for ONE full 360 at CONT_SPIN_SPEED
CONT_SUBSAMPLE       = 5        # keep every Nth streamed frame (5 -> ~6 fps from 30 fps)
CONT_EXPOSURE_US     = 6000     # locked exposure (us) to fight motion blur; None = auto
CONT_DIR             = scan360.SCAN_DIR    # +1 CCW (rotate_left), -1 CW — match the merge


def run_continuous_scan(bot, cam, full_rotation_sec=CONT_FULL_ROTATION_SEC,
                        spin_speed=CONT_SPIN_SPEED, subsample=CONT_SUBSAMPLE,
                        exposure_us=CONT_EXPOSURE_US, direction=CONT_DIR,
                        out_root=None, log=print):
    """Spin once in place, streaming the D405, and save every `subsample`-th frame as a
    shot_NN folder under captures/cscan_<ts>/. Returns the session folder.

    No angle.txt is written on purpose, so the merge measures each step's yaw from the
    image overlap (see module docstring). A `time_angle.txt` per frame records the rough
    timing-based angle for reference/debugging only — the geometry never reads it.
    """
    out_root = out_root or rs_capture.default_out_root()
    session = os.path.join(out_root, "cscan_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)

    if cam.pipeline is None:
        log("Opening D405 (warming up)...")
        cam.start()
        log(cam.info())
    if exposure_us is not None:
        cam.set_manual_exposure(exposure_us)
        log(f"  locked exposure {exposure_us} us (anti motion-blur)")
        for _ in range(5):                 # let the new exposure take effect
            cam.pipeline.wait_for_frames()

    spin = bot.rotate_left if direction >= 0 else bot.rotate_right
    sign = 1.0 if direction >= 0 else -1.0
    log(f"continuous 360: spin ~{full_rotation_sec:.1f}s at speed {spin_speed}, "
        f"keep every {subsample}th frame ({'CCW' if direction >= 0 else 'CW'})")

    saved = 0
    n = 0
    t0 = time.time()
    spin(spin_speed)
    try:
        while True:
            elapsed = time.time() - t0
            if elapsed >= full_rotation_sec:
                break
            frames = cam.pipeline.wait_for_frames()
            n += 1
            if n % subsample != 0:
                continue
            depth = frames.get_depth_frame()
            irl = frames.get_infrared_frame(1)
            irr = frames.get_infrared_frame(2)
            if not depth or not irl or not irr:
                continue
            folder = os.path.join(session, f"shot_{saved:02d}")
            cam.save_arrays(folder,
                            np.asanyarray(depth.get_data()),
                            np.asanyarray(irl.get_data()),
                            np.asanyarray(irr.get_data()))
            est = sign * elapsed / full_rotation_sec * 360.0    # reference only (not used by merge)
            with open(os.path.join(folder, "time_angle.txt"), "w") as f:
                f.write(f"{est:.3f}\n")
            saved += 1
    finally:
        bot.stop()
        if exposure_us is not None:
            cam.set_manual_exposure(None)   # restore auto-exposure for normal use

    log(f"  captured {saved} frames over ~{time.time() - t0:.1f}s -> {session}")
    if saved < 2:
        log("  WARNING: too few frames — increase --seconds or lower --sub")
    return session


def scan_and_build(bot, cam, log=print, measure=True, **kw):
    """Full concept pipeline: continuous sweep, then merge with the shipped vision-measured
    builder. save_shots=False keeps the Pi merge lean over the many continuous frames."""
    session = run_continuous_scan(bot, cam, log=log, **kw)
    ply = scan360.build_from_session(session, direction=CONT_DIR, measure=measure,
                                     save_shots=False, log=log)
    return session, ply


# ── standalone CLI ──────────────────────────────────────────────────────────────

def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        val = cast(args[i + 1])
        del args[i:i + 2]
        return val
    return None


def calibrate_spin(seconds, speed=CONT_SPIN_SPEED, direction=CONT_DIR):
    """Spin in place for `seconds` (NO camera) so you can dial in CONT_FULL_ROTATION_SEC.

    Mark the robot's start heading (tape on the floor), run this, and see how far past /
    short of one full turn it landed. Adjust:  new_sec = seconds * 360 / degrees_turned.
    """
    from setup_and_api.api import RasBot, Color
    print(f"Calibration spin: {seconds:.1f}s at speed {speed} "
          f"({'CCW' if direction >= 0 else 'CW'}). Mark the start heading now.")
    with RasBot() as bot:
        spin = bot.rotate_left if direction >= 0 else bot.rotate_right
        bot.set_all_leds_color(Color.BLUE)
        t0 = time.time()
        spin(speed)
        try:
            while time.time() - t0 < seconds:
                time.sleep(0.02)
        finally:
            bot.stop()
            bot.set_all_leds_color(Color.GREEN)
    print(f"done. If it turned D degrees, set CONT_FULL_ROTATION_SEC = "
          f"{seconds:.1f} * 360 / D   (e.g. turned 300 -> {seconds * 360 / 300:.1f}s).")


def main():
    args = sys.argv[1:]

    # rebuild-only: a session dir given and no robot needed
    if args and os.path.isdir(args[0]):
        scan360.build_from_session(args[0], direction=CONT_DIR, measure=True,
                                   save_shots=False)
        return

    cal = _pop(args, "--calibrate", float)        # spin only, no capture, to time one turn
    seconds   = _pop(args, "--seconds", float)
    speed     = _pop(args, "--speed", int)
    if cal is not None:
        calibrate_spin(cal, speed=speed or CONT_SPIN_SPEED)
        return
    sub       = _pop(args, "--sub", int)
    exposure  = _pop(args, "--exposure", int)
    if "--no-exposure" in args:
        args.remove("--no-exposure")
        exposure = None
    elif exposure is None:
        exposure = CONT_EXPOSURE_US

    from setup_and_api.api import RasBot, Color

    cam = rs_capture.StereoCapture()
    print("Connecting to robot board...")
    with RasBot() as bot:
        bot.set_all_leds_color(Color.BLUE)
        try:
            session, ply = scan_and_build(
                bot, cam,
                full_rotation_sec=seconds or CONT_FULL_ROTATION_SEC,
                spin_speed=speed or CONT_SPIN_SPEED,
                subsample=sub or CONT_SUBSAMPLE,
                exposure_us=exposure,
            )
            bot.set_all_leds_color(Color.GREEN)
            bot.beep(0.15)
            print(f"\ndone -> {ply}")
            print("view it with:  python3 pointcloud/view3d.py", ply)
        except Exception as e:
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            print("continuous scan error:", e)
            raise
        finally:
            bot.stop()
            cam.close()


if __name__ == "__main__":
    main()
