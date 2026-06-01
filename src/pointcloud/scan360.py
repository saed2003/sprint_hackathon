"""
On-the-Pi 360-degree scan: rotate in place, capture N views (8-10), and merge them into
ONE point cloud — entirely with numpy + OpenCV (NO Open3D, which has no Raspberry Pi
wheel). Everything runs on the robot; press R in drive.py to capture, T to build.

How the merge works (MEASURED angle — the fix for "the bot turns too far"):
  The RasBot has no IMU/encoders, so the rotation is open-loop and timed: each step is
  a short motor pulse, and the real angle it produces drifts from the nominal 360/N (it
  tends to OVERSHOOT — coast after stop, low-speed nonlinearity). So we don't trust the
  timer for the cloud. We back-project each view to 3D (X=(u-ppx)Z/fx, Y=(v-ppy)Z/fy,
  Z=depth) and rotate view i by the angle the CAMERA actually turned, recovered from the
  overlapping IR images (ORB -> essential matrix -> pose; see estimate_yaw). Anything we
  can't measure falls back to the nominal step. Pass --known to trust the timer instead.

  For a still-cleaner result, copy the raw shots to a laptop and run merge_clouds.py (ICP).

Standalone uses (no robot needed — rebuild from shots already captured):
  python3 pointcloud/scan360.py captures/scan_20260531_1700              # rebuild, MEASURED angle
  python3 pointcloud/scan360.py captures/scan_20260531_1700 --known      # trust the timed step
  python3 pointcloud/scan360.py captures/scan_20260531_1700 --angle 40   # force a fixed step angle
  python3 pointcloud/scan360.py captures/scan_20260531_1700 --dir -1     # flip rotation sign

Calibrate the rotation so each step lands near its nominal angle (do this once, on the
bot — it pulses EXACTLY like a scan, not one long spin, so the timing actually transfers):
  python3 pointcloud/scan360.py --calibrate                 # 8 scan-like pulses; measure total deg
  python3 pointcloud/scan360.py --calibrate --turned 470    # auto-prints the SCAN_SEC_PER_DEG to set
"""

import os
import sys
import glob
import time
import math

import numpy as np
import cv2

# project root = the folder that contains pointcloud/, camera/, rasbot/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera.rs_capture import StereoCapture, default_out_root

# ── scan tunables ───────────────────────────────────────────────────────────────
SCAN_SHOTS        = 9        # views per full turn (8-10): 9->40deg, 10->36deg, 8->45deg
SCAN_ROTATE_SPEED = 40       # motor speed used while rotating
# <<< CALIBRATE (pulsed): seconds of rotate-pulse per DEGREE, measured the way the scan
# actually moves (short start/stop pulses), NOT from one long continuous spin. See
# `python3 pointcloud/scan360.py --calibrate`. Default 6.0/360 just matches the old 6 s/rev guess.
SCAN_SEC_PER_DEG  = 6.0 / 360.0
SCAN_SETTLE_PAUSE = 0.4      # seconds to let the chassis stop shaking before a shot
SCAN_BRAKE_TAP    = 0.0      # seconds of reverse pulse after each step to kill coast (0=off)
SCAN_DIR          = 1        # +1 = rotate CCW (rotate_left); -1 = CW. Merge follows this.

# ── cloud tunables ──────────────────────────────────────────────────────────────
VOXEL = 0.01                 # 1 cm final resolution
ZMIN, ZMAX = 0.05, 3.0       # keep points in the D405's useful range (meters)

# ── visual angle-measurement tunables (Track B: don't trust the timer) ───────────
MEASURE_MIN_INLIERS = 30     # need at least this many pose inliers to trust a measured step
MEASURE_LO          = 0.5    # accept a measured step only if it is within
MEASURE_HI          = 1.8    #   [LO, HI] x the nominal step (else fall back to nominal)


# ── point-cloud math (pure numpy) ────────────────────────────────────────────────

def load_intrinsics(path):
    vals = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                vals[p[0]] = float(p[1])
    return vals


def intrinsics_K(folder):
    """3x3 camera matrix for a capture folder (left-IR / depth intrinsics)."""
    intr = load_intrinsics(os.path.join(folder, "intrinsics.txt"))
    return np.array([[intr["fx"], 0, intr["ppx"]],
                     [0, intr["fy"], intr["ppy"]],
                     [0, 0, 1]], dtype=np.float64)


def estimate_yaw(folder_a, folder_b, K):
    """Measure the rotation (deg, about the vertical/Y axis) between two shots from
    their overlapping IR images — so the merge can use the TRUE angle the robot turned
    instead of the assumed one. Pure OpenCV: ORB match -> essential matrix -> pose.

    Returns (yaw_deg or None, n_inliers). yaw_deg is signed; callers use its magnitude
    and apply the known rotation direction. None means "couldn't measure" (fall back).
    """
    a = cv2.imread(os.path.join(folder_a, "ir_left.png"), cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(os.path.join(folder_b, "ir_left.png"), cv2.IMREAD_GRAYSCALE)
    if a is None or b is None:
        return None, 0

    orb = cv2.ORB_create(2000)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None or len(ka) < 8 or len(kb) < 8:
        return None, 0

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(da, db)
    if len(matches) < 8:
        return None, 0
    pa = np.float64([ka[m.queryIdx].pt for m in matches])
    pb = np.float64([kb[m.trainIdx].pt for m in matches])

    E, mask = cv2.findEssentialMat(pa, pb, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None or E.shape != (3, 3):
        return None, 0
    n_in, R, _t, _mask = cv2.recoverPose(E, pa, pb, K, mask=mask)
    # R is dominated by the yaw component since the robot spins about the vertical axis;
    # extract it in the same convention as ry(): R[0,0]=cos, R[0,2]=sin.
    yaw = math.degrees(math.atan2(R[0, 2], R[0, 0]))
    return yaw, int(n_in)


def back_project(folder, zmin=ZMIN, zmax=ZMAX):
    """One capture folder -> (points Nx3 meters, colors Nx3 uint8) in camera frame."""
    depth_raw = np.load(os.path.join(folder, "depth.npy"))
    intr = load_intrinsics(os.path.join(folder, "intrinsics.txt"))
    Z = depth_raw.astype(np.float32) * intr["depth_scale"]
    H, W = depth_raw.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    X = (uu - intr["ppx"]) * Z / intr["fx"]
    Y = (vv - intr["ppy"]) * Z / intr["fy"]
    valid = (Z > zmin) & (Z < zmax)
    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=1).astype(np.float32)

    ir_path = os.path.join(folder, "ir_left.png")
    if os.path.exists(ir_path):
        g = cv2.imread(ir_path, cv2.IMREAD_GRAYSCALE)[valid]
        cols = np.stack([g, g, g], axis=1).astype(np.uint8)
    else:
        cols = np.full((len(pts), 3), 200, np.uint8)
    return pts, cols


def ry(angle_deg):
    """Rotation matrix about the camera Y (vertical) axis."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)


def voxel_downsample(pts, cols, voxel):
    """Keep one point per voxel cell (fast numpy downsample)."""
    if len(pts) == 0:
        return pts, cols
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx], cols[idx]


def write_ply(path, pts, cols):
    """Write a colored point cloud as a binary little-endian .ply."""
    pts = np.asarray(pts, np.float32)
    cols = np.asarray(cols, np.uint8)
    n = len(pts)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("r", "u1"), ("g", "u1"), ("b", "u1")])
    arr = np.empty(n, dtype=dt)
    arr["x"], arr["y"], arr["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    arr["r"], arr["g"], arr["b"] = cols[:, 0], cols[:, 1], cols[:, 2]
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(arr.tobytes())


def shot_dirs(session_dir):
    """All shot_* folders in a scan session that actually have a capture."""
    return sorted(os.path.dirname(p)
                  for p in glob.glob(os.path.join(session_dir, "shot_*", "depth.npy")))


def cumulative_angles(dirs, nominal_step, measure=True, log=print):
    """Angle (deg, about vertical) to rotate each view into view-0's frame.

    Known-angle mode (measure=False) just returns 0, step, 2*step, ... — the timer is
    trusted. Measured mode (the default) reads the TRUE per-step yaw from each pair of
    overlapping IR images (see estimate_yaw) and uses that, so the cloud is correct even
    when the open-loop rotation overshoots. We trust the camera for the step MAGNITUDE and
    keep the known turn direction (sign of nominal_step); anything outside a sane band, or
    with too few inliers, falls back to the nominal step.
    """
    n = len(dirs)
    if not measure or n < 2:
        return [nominal_step * i for i in range(n)]

    nominal = abs(nominal_step)
    lo, hi = MEASURE_LO * nominal, MEASURE_HI * nominal
    steps = []
    for i in range(n - 1):
        yaw, inliers = estimate_yaw(dirs[i], dirs[i + 1], intrinsics_K(dirs[i]))
        cand = abs(yaw) if yaw is not None else None
        if cand is not None and inliers >= MEASURE_MIN_INLIERS and lo <= cand <= hi:
            step = math.copysign(cand, nominal_step)
            log(f"  step {i}->{i+1}: measured {step:+.1f} deg ({inliers} inliers)")
        else:
            step = nominal_step
            why = "no match" if cand is None else f"{cand:.1f} deg / {inliers} inliers rejected"
            log(f"  step {i}->{i+1}: nominal {step:+.1f} deg  ({why})")
        steps.append(step)

    angles = [0.0]
    for s in steps:
        angles.append(angles[-1] + s)
    return angles


def build_from_session(session_dir, step_angle=None, direction=SCAN_DIR,
                       voxel=VOXEL, measure=True, log=print):
    """Merge all shots in a session into <session>/merged_360.ply.

    measure=True (default): use the camera-measured per-step yaw (robust to bad timing).
    measure=False or step_angle given: trust the known/timed step angle.
    """
    dirs = shot_dirs(session_dir)
    if not dirs:
        sys.exit(f"No shots found in {session_dir}")
    forced = step_angle is not None
    if step_angle is None:
        step_angle = 360.0 / len(dirs)
    nominal_step = direction * step_angle

    # an explicit --angle forces the known-angle path; otherwise measure by default
    angles = cumulative_angles(dirs, nominal_step, measure=measure and not forced, log=log)

    all_pts, all_cols = [], []
    for d, ang in zip(dirs, angles):
        pts, cols = back_project(d)
        pts = pts @ ry(ang).T          # rotate this view into view-0's frame
        all_pts.append(pts)
        all_cols.append(cols)

    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols)
    pts, cols = voxel_downsample(pts, cols, voxel)

    out = os.path.join(session_dir, "merged_360.ply")
    write_ply(out, pts, cols)
    mode = "known-angle" if (forced or not measure) else "measured-angle"
    log(f"  merged {len(dirs)} views ({mode}, swept {angles[-1]:.0f} deg) -> "
        f"{len(pts)} points -> {out}")
    return out


# ── robot sweep ──────────────────────────────────────────────────────────────────

def _rotate_step(bot, speed, secs, direction=SCAN_DIR, brake_tap=SCAN_BRAKE_TAP):
    """One timed rotation pulse. direction +1 = CCW (rotate_left), -1 = CW.
    An optional brake_tap drives the wheels the other way briefly to cancel coast."""
    spin = bot.rotate_left if direction >= 0 else bot.rotate_right
    brake = bot.rotate_right if direction >= 0 else bot.rotate_left
    spin(speed)
    time.sleep(secs)
    if brake_tap > 0:
        brake(speed)
        time.sleep(brake_tap)
    bot.stop()


def run_scan(bot, cam, shots=SCAN_SHOTS, rotate_speed=SCAN_ROTATE_SPEED,
             sec_per_deg=SCAN_SEC_PER_DEG, settle_pause=SCAN_SETTLE_PAUSE,
             brake_tap=SCAN_BRAKE_TAP, direction=SCAN_DIR, out_root=None, log=print):
    """Rotate in place and capture `shots` views. Returns the session folder."""
    out_root = out_root or default_out_root()
    session = os.path.join(out_root, "scan_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    step_angle = 360.0 / shots
    step_time = sec_per_deg * step_angle         # pulse length for one step

    if cam.pipeline is None:
        log("Opening D405 (warming up auto-exposure)...")
        cam.start()
        log(cam.info())

    log(f"360 scan: {shots} shots, {step_angle:.0f} deg each, "
        f"{step_time:.2f}s rotate/step (timed; the merge measures the real angle)")
    for i in range(shots):
        bot.stop()
        time.sleep(settle_pause)                 # let the chassis settle (less blur)
        folder = os.path.join(session, f"shot_{i:02d}")
        ok = cam.save_to(folder)
        log(f"  shot {i + 1}/{shots} -> {os.path.basename(folder)}"
            + ("" if ok else "  (FRAME DROPPED)"))
        bot.beep(0.05)
        if i < shots - 1:
            _rotate_step(bot, rotate_speed, step_time, direction, brake_tap)
    bot.stop()
    return session


def scan_and_build(bot, cam, log=print, shots=SCAN_SHOTS, measure=True, **kw):
    """Full pipeline: sweep, then build the 360 cloud (measured angle) — all on the Pi."""
    session = run_scan(bot, cam, shots=shots, log=log, **kw)
    ply = build_from_session(session, measure=measure, log=log)
    return session, ply


# ── standalone CLI ────────────────────────────────────────────────────────────────

def _calibrate(shots, speed, sec_per_deg, settle, brake_tap, turned=None):
    """Pulsed calibration: pulse-rotate EXACTLY like a scan, so the measured time->angle
    ratio is valid for the real (start/stop) motion — not a single continuous spin."""
    from setup_and_api.api import RasBot
    step_angle = 360.0 / shots
    step_time  = sec_per_deg * step_angle
    pulses     = shots - 1                       # a scan rotates shots-1 times
    total_drive = pulses * step_time
    print(f"Pulsed calibration: {pulses} identical {step_time:.2f}s pulses at speed {speed}")
    print(f"(this is exactly how a {shots}-shot scan moves between shots, minus the camera).")
    print("Mark the robot's start heading, let it run, then measure the TOTAL degrees turned.\n")
    with RasBot() as bot:
        for i in range(pulses):
            bot.stop()
            time.sleep(settle)
            _rotate_step(bot, speed, step_time, SCAN_DIR, brake_tap)
            print(f"  pulse {i + 1}/{pulses}")
        bot.stop()
    print(f"\ndone — total drive time was {total_drive:.2f}s over {pulses} pulses.")
    if turned:
        print(f"You measured {turned:.0f} deg turned. In scan360.py set:")
        print(f"  SCAN_SEC_PER_DEG = {total_drive / turned:.5f}    "
              f"# = {total_drive:.2f}s / {turned:.0f}deg")
        print("Re-run --calibrate once more to confirm it now turns ~360 total.")
    else:
        print("Set, in scan360.py:  SCAN_SEC_PER_DEG = "
              f"{total_drive:.2f} / (degrees you measured)")
        print("Or re-run with --turned <deg> to print the exact number.")


def _pop(args, flag, cast):
    """Pull '--flag value' out of args if present; return value or None."""
    if flag in args:
        i = args.index(flag)
        val = cast(args[i + 1])
        del args[i:i + 2]
        return val
    return None


def main():
    args = sys.argv[1:]

    if "--calibrate" in args:
        args.remove("--calibrate")
        shots  = _pop(args, "--shots", int) or SCAN_SHOTS
        speed  = _pop(args, "--speed", int) or SCAN_ROTATE_SPEED
        brake  = _pop(args, "--brake", float)
        turned = _pop(args, "--turned", float)
        _calibrate(shots, speed, SCAN_SEC_PER_DEG, SCAN_SETTLE_PAUSE,
                   SCAN_BRAKE_TAP if brake is None else brake, turned)
        return

    angle = _pop(args, "--angle", float)         # forces the known-angle merge
    direction = _pop(args, "--dir", int)
    direction = SCAN_DIR if direction is None else direction
    measure = "--known" not in args
    if "--known" in args:
        args.remove("--known")

    if not args:
        sys.exit("Usage:\n"
                 "  python3 pointcloud/scan360.py <session_dir>            # rebuild, measured angle\n"
                 "  python3 pointcloud/scan360.py <session_dir> --known    # trust the timed step\n"
                 "  python3 pointcloud/scan360.py <session_dir> --angle 40 # force a fixed step angle\n"
                 "  python3 pointcloud/scan360.py <session_dir> --dir -1   # flip rotation sign\n"
                 "  python3 pointcloud/scan360.py --calibrate [--shots N] [--speed S] [--turned DEG]")
    build_from_session(args[0], step_angle=angle, direction=direction, measure=measure)


if __name__ == "__main__":
    main()
