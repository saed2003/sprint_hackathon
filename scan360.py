"""
On-the-Pi 360-degree scan: rotate in place, capture N views, and merge them into
ONE point cloud — entirely with numpy + OpenCV (NO Open3D, which has no Raspberry
Pi wheel). Everything runs on the robot; press R in drive.py and you get a 3D map.

How the merge works (known-angle, no ICP):
  The robot rotates by a known step (360/N) between shots. We back-project each
  view to 3D (X=(u-ppx)Z/fx, Y=(v-ppy)Z/fy, Z=depth) and rotate view i by i*step
  about the vertical (camera Y) axis, then stack them into one cloud.

  This is OPEN-LOOP: the RasBot has no IMU/odometry, so alignment is only as good
  as the timed rotation. The raw shots are kept under captures/scan_<ts>/, so for a
  clean result you can copy them to the laptop and run merge_clouds.py (ICP).

Standalone uses (no robot needed — rebuild from shots already captured):
  python3 scan360.py captures/scan_20260531_1700              # rebuild, step=360/N
  python3 scan360.py captures/scan_20260531_1700 --angle 45   # force a step angle
  python3 scan360.py captures/scan_20260531_1700 --dir -1     # flip rotation sign

Calibrate the rotation timing (CRITICAL — do this once):
  python3 scan360.py --calibrate            # rotates for SECONDS_PER_REV, you watch
  python3 scan360.py --calibrate --secs 6   # rotate 6 s; see how far it actually turned
"""

import os
import sys
import glob
import time

import numpy as np
import cv2

from rs_capture import StereoCapture, default_out_root

# ── scan tunables ───────────────────────────────────────────────────────────────
SCAN_SHOTS        = 9        # views per full turn (40 deg each: 9 x 40 = 360)
SCAN_ROTATE_SPEED = 40      # motor speed used while rotating
SCAN_SECONDS_PER_REV = 6.0   # <<< CALIBRATE: seconds for a FULL 360 at the speed above
SCAN_SETTLE_PAUSE = 0.4      # seconds to let the chassis stop shaking before a shot
SCAN_DIR          = 1        # +1 / -1: flip if the merged cloud comes out "unwound"

# ── cloud tunables ──────────────────────────────────────────────────────────────
VOXEL = 0.01                 # 1 cm final resolution
ZMIN, ZMAX = 0.05, 3.0       # keep points in the D405's useful range (meters)


# ── point-cloud math (pure numpy) ────────────────────────────────────────────────

def load_intrinsics(path):
    vals = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                vals[p[0]] = float(p[1])
    return vals


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


def build_from_session(session_dir, step_angle=None, direction=SCAN_DIR,
                       voxel=VOXEL, log=print):
    """Merge all shots in a session into <session>/merged_360.ply (known-angle)."""
    dirs = shot_dirs(session_dir)
    if not dirs:
        sys.exit(f"No shots found in {session_dir}")
    if step_angle is None:
        step_angle = 360.0 / len(dirs)

    all_pts, all_cols = [], []
    for i, d in enumerate(dirs):
        pts, cols = back_project(d)
        pts = pts @ ry(direction * step_angle * i).T   # rotate view i into view-0 frame
        all_pts.append(pts)
        all_cols.append(cols)

    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols)
    pts, cols = voxel_downsample(pts, cols, voxel)

    out = os.path.join(session_dir, "merged_360.ply")
    write_ply(out, pts, cols)
    log(f"  merged {len(dirs)} views, step={step_angle:.1f} deg -> "
        f"{len(pts)} points -> {out}")
    return out


# ── robot sweep ──────────────────────────────────────────────────────────────────

def run_scan(bot, cam, shots=SCAN_SHOTS, rotate_speed=SCAN_ROTATE_SPEED,
             seconds_per_rev=SCAN_SECONDS_PER_REV, settle_pause=SCAN_SETTLE_PAUSE,
             out_root=None, log=print):
    """Rotate in place and capture `shots` views. Returns the session folder."""
    out_root = out_root or default_out_root()
    session = os.path.join(out_root, "scan_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    step_time = seconds_per_rev / shots

    if cam.pipeline is None:
        log("Opening D405 (warming up auto-exposure)...")
        cam.start()
        log(cam.info())

    log(f"360 scan: {shots} shots, {360.0/shots:.0f} deg each, "
        f"{step_time:.2f}s rotate/step")
    for i in range(shots):
        bot.stop()
        time.sleep(settle_pause)                 # let the chassis settle (less blur)
        folder = os.path.join(session, f"shot_{i:02d}")
        ok = cam.save_to(folder)
        log(f"  shot {i + 1}/{shots} -> {os.path.basename(folder)}"
            + ("" if ok else "  (FRAME DROPPED)"))
        bot.beep(0.05)
        if i < shots - 1:
            bot.rotate_left(rotate_speed)        # CCW pulse
            time.sleep(step_time)
            bot.stop()
    bot.stop()
    return session


def scan_and_build(bot, cam, log=print, shots=SCAN_SHOTS, **kw):
    """Full pipeline for the R key: sweep, then build the 360 cloud — all on the Pi."""
    session = run_scan(bot, cam, shots=shots, log=log, **kw)
    n = len(shot_dirs(session))
    ply = build_from_session(session, step_angle=360.0 / n, log=log)
    return session, ply


# ── standalone CLI ────────────────────────────────────────────────────────────────

def _calibrate(secs, speed):
    """Rotate in place for `secs` so you can measure the real turn rate."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rasbot.api import RasBot
    print(f"Calibrating: rotating CCW at speed {speed} for {secs:.1f}s.")
    print("Mark the robot's start heading, run this, then measure how many degrees")
    print("it actually turned. SECONDS_PER_REV = secs * 360 / (degrees turned).")
    with RasBot() as bot:
        bot.rotate_left(speed)
        time.sleep(secs)
        bot.stop()
    print("done — measure the angle now.")


def main():
    args = sys.argv[1:]

    if "--calibrate" in args:
        args.remove("--calibrate")
        secs = SCAN_SECONDS_PER_REV
        speed = SCAN_ROTATE_SPEED
        if "--secs" in args:
            i = args.index("--secs"); secs = float(args[i + 1]); del args[i:i + 2]
        if "--speed" in args:
            i = args.index("--speed"); speed = int(args[i + 1]); del args[i:i + 2]
        _calibrate(secs, speed)
        return

    angle = None
    direction = SCAN_DIR
    if "--angle" in args:
        i = args.index("--angle"); angle = float(args[i + 1]); del args[i:i + 2]
    if "--dir" in args:
        i = args.index("--dir"); direction = int(args[i + 1]); del args[i:i + 2]

    if not args:
        sys.exit("Usage: python3 scan360.py <session_dir> [--angle DEG] [--dir 1|-1]\n"
                 "       python3 scan360.py --calibrate [--secs N] [--speed S]")
    build_from_session(args[0], step_angle=angle, direction=direction)


if __name__ == "__main__":
    main()
