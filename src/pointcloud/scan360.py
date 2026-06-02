"""
On-the-Pi 360 scan: rotate in place, capture SCAN_SHOTS views, and merge them into one
point cloud with numpy + OpenCV (no Open3D on the Pi). Press R in drive.py to capture,
T to build.

Rotation is by CALIBRATED TIMING (the RasBot has no IMU/encoders): one motor pulse of
SCAN_SEC_PER_DEG * step between shots, shots-1 pulses for ~360 deg of coverage. Each shot
records its angle to shot_NN/angle.txt and the merge rotates each view by that.

Standalone (rebuild from already-captured shots, no robot):
  python3 pointcloud/scan360.py <session_dir>            # rebuild (measured angle)
  python3 pointcloud/scan360.py <session_dir> --known    # trust the timed step
  python3 pointcloud/scan360.py <session_dir> --angle 36 # force a fixed step angle
  python3 pointcloud/scan360.py <session_dir> --dir -1   # flip rotation sign
  python3 pointcloud/scan360.py --calibrate --turned <deg>   # print SCAN_SEC_PER_DEG to set
"""

import os
import sys
import glob
import time
import math

import numpy as np
import cv2

# project root = the folder that holds pointcloud/, camera/, rasbot/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera.rs_capture import default_out_root

# ── scan tunables ────────────────────────────────────────────────────────────────
SCAN_SHOTS        = 10       # views per turn: 10 -> 36 deg each
SCAN_ROTATE_SPEED = 40       # motor speed while rotating
# Calibrated seconds of rotate-pulse per DEGREE (pulsed, not one long spin). With
# SCAN_RETURN_MODE="forward" the full 10-step turn must total 360 deg to land back on start.
# Re-tune from one run: undershoots by d deg -> new = old*360/(360-d); overshoots -> 360/(360+d).
SCAN_SEC_PER_DEG  = 2.73 / 360.0
SCAN_SETTLE_PAUSE = 0.4      # seconds to let the chassis settle before a shot
SCAN_BRAKE_TAP    = 0.0      # seconds of reverse pulse after each step to kill coast (0=off)
SCAN_DIR          = 1        # +1 = CCW (rotate_left); -1 = CW. Merge follows this.
SCAN_RETURN_HOME  = False     # TRUE/FALSE toggle for the EXTRA rotation after the 10th photo:
                             #   True  = do the final turn back toward start (uses SCAN_RETURN_MODE);
                             #   False = stop right after the last shot, no extra turn.
SCAN_RETURN_MODE  = "forward"  # only used when SCAN_RETURN_HOME=True.
                             #   "forward" = one more step to finish the 360 circle (needs
                             #     SCAN_SEC_PER_DEG calibrated; can overshoot start if the scan does);
                             #   "rewind"  = spin back the exact steps just made (lands on start
                             #     regardless of calibration, but spins the wheels ~324 deg backward).

# ── cloud tunables ───────────────────────────────────────────────────────────────
VOXEL = 0.01                 # final resolution (m)
ZMIN, ZMAX = 0.1, 1.5        # keep depth in the D405's good range (m); past ~1.5 m is noise
MIN_NEIGHBORS = 4            # outlier filter: drop points with < this many filled neighbour cells

# ── angle-measurement tunables (merge: recover step yaw from IR images) ───────────
MEASURE_MIN_INLIERS = 20     # need >= this many homography inliers to trust a measured step
MEASURE_LO          = 0.5    # accept a measured step only within
MEASURE_HI          = 1.8    #   [LO, HI] x the nominal step


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


def read_angle(folder):
    """Recorded turn angle (deg magnitude) for a shot from angle.txt, or None."""
    p = os.path.join(folder, "angle.txt")
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            return abs(float(f.read().split()[0]))
    except (ValueError, IndexError):
        return None


def yaw_from_images(a, b, K):
    """Yaw (deg, about vertical) between two grayscale IR images: ORB matches -> RANSAC
    homography H = K R K^-1 -> R -> yaw. Returns (yaw_deg or None, n_inliers)."""
    if a is None or b is None:
        return None, 0

    orb = cv2.ORB_create(4000)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None or len(ka) < 12 or len(kb) < 12:
        return None, 0

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    good = [m for m, n in bf.knnMatch(da, db, k=2) if m.distance < 0.75 * n.distance]
    if len(good) < 12:
        return None, 0
    pa = np.float32([ka[m.queryIdx].pt for m in good])
    pb = np.float32([kb[m.trainIdx].pt for m in good])

    H, mask = cv2.findHomography(pa, pb, cv2.RANSAC, 3.0)
    if H is None:
        return None, 0
    inliers = int(mask.sum())

    # R ~ K^-1 H K, snapped to the nearest rotation (SVD); yaw read as R[0,0]=cos, R[0,2]=sin
    R = np.linalg.inv(K) @ H @ K
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt
    yaw = math.degrees(math.atan2(R[0, 2], R[0, 0]))
    return yaw, inliers


def estimate_yaw(folder_a, folder_b, K):
    """Per-step yaw from two folders' left-IR images (see yaw_from_images)."""
    a = cv2.imread(os.path.join(folder_a, "ir_left.png"), cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(os.path.join(folder_b, "ir_left.png"), cv2.IMREAD_GRAYSCALE)
    return yaw_from_images(a, b, K)


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


def remove_isolated(pts, cols, voxel, min_neighbors=MIN_NEIGHBORS):
    """Drop lone specks: keep a point only if >= min_neighbors of its 26 neighbouring voxel
    cells are occupied (a numpy stand-in for Open3D outlier removal). Call after downsample."""
    if len(pts) == 0 or min_neighbors <= 0:
        return pts, cols
    k = np.floor(pts / voxel).astype(np.int64)
    k = k - k.min(axis=0) + 1                      # shift into [1, max+1] (no negatives)
    dimx = int(k[:, 0].max()) + 3                  # pad so neighbours stay in [0, dim)
    dimy = int(k[:, 1].max()) + 3
    cell_hash = lambda a: a[:, 0] + a[:, 1] * dimx + a[:, 2] * dimx * dimy
    occupied = np.unique(cell_hash(k))
    counts = np.zeros(len(pts), np.int32)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx or dy or dz:
                    counts += np.isin(cell_hash(k + np.array([dx, dy, dz])), occupied)
    keep = counts >= min_neighbors
    return pts[keep], cols[keep]


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
    """Angle (deg) to rotate each view into view-0's frame.

    measure=False: trust the timer (0, step, 2*step, ...). measure=True: use the recorded
    angles if every shot has one, else measure per-step yaw from the IR images and fall back
    to the nominal step when a reading is missing or out of the [MEASURE_LO, MEASURE_HI] band.
    """
    n = len(dirs)
    if n < 2:
        return [0.0] * n

    if measure:
        recorded = [read_angle(d) for d in dirs]
        if all(a is not None for a in recorded):
            sign = math.copysign(1.0, nominal_step)
            log("  using recorded angles: " + ", ".join(f"{a:.0f}" for a in recorded) + " deg")
            return [sign * a for a in recorded]

    if not measure:
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
                       voxel=VOXEL, measure=True, save_shots=True, log=print):
    """Merge all shots in a session into <session>/merged_360.ply.

    measure=True: use the per-step yaw (recorded angles, else image-measured). measure=False
    or an explicit step_angle trusts the known/timed step. save_shots also writes each photo's
    own cloud to shot_NN/cloud.ply.
    """
    dirs = shot_dirs(session_dir)
    if not dirs:
        sys.exit(f"No shots found in {session_dir}")
    forced = step_angle is not None
    if step_angle is None:
        step_angle = 360.0 / len(dirs)
    nominal_step = direction * step_angle

    angles = cumulative_angles(dirs, nominal_step, measure=measure and not forced, log=log)

    all_pts, all_cols = [], []
    for d, ang in zip(dirs, angles):
        pts, cols = back_project(d)
        if save_shots:
            write_ply(os.path.join(d, "cloud.ply"), pts, cols)   # per-photo cloud, own frame
        pts = pts @ ry(ang).T          # rotate this view into view-0's frame
        all_pts.append(pts)
        all_cols.append(cols)

    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols)
    pts, cols = voxel_downsample(pts, cols, voxel)
    before = len(pts)
    pts, cols = remove_isolated(pts, cols, voxel)   # drop passive-stereo flyers

    out = os.path.join(session_dir, "merged_360.ply")
    write_ply(out, pts, cols)
    if forced or not measure:
        mode = "known-angle"
    elif all(read_angle(d) is not None for d in dirs):
        mode = "recorded angle"
    else:
        mode = "image-measured angle"
    log(f"  merged {len(dirs)} views ({mode}, swept {angles[-1]:.0f} deg) -> "
        f"{len(pts)} points (cleaned {before - len(pts)} flyers) -> {out}")

    # still preview of the cloud (headless — works with no display)
    try:
        from pointcloud import view3d
        prev = view3d.save_view(out, os.path.join(session_dir, "merged_360_preview.png"))
        if prev:
            log(f"  preview image -> {prev}")
    except Exception as e:
        log(f"  (preview render skipped: {e})")
    return out


# ── robot sweep ────────────────────────────────────────────────────────────────────

def _rotate_step(bot, speed, secs, direction=SCAN_DIR, brake_tap=SCAN_BRAKE_TAP):
    """One timed rotation pulse (+1 = CCW/rotate_left, -1 = CW). Optional brake_tap drives
    the wheels the other way briefly to cancel coast."""
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
             brake_tap=SCAN_BRAKE_TAP, direction=SCAN_DIR, return_home=SCAN_RETURN_HOME,
             return_mode=SCAN_RETURN_MODE, out_root=None, log=print):
    """Rotate in place and capture `shots` views (timed open-loop). Returns the session dir.

    return_home=True: after the last shot, rotate back to the starting heading (no photo, no
    extra shot folder, not merged). return_mode "forward" = one more step to complete the 360
    circle; "rewind" = spin back the exact shots-1 steps just made.
    """
    out_root = out_root or default_out_root()
    session = os.path.join(out_root, "scan_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    step_angle = 360.0 / shots

    if cam.pipeline is None:
        log("Opening D405 (warming up auto-exposure)...")
        cam.start()
        log(cam.info())

    log(f"360 scan: {shots} shots, {step_angle:.0f} deg each (timed open-loop)")
    cumulative = 0.0                              # turn so far (deg, from shot 0)
    for i in range(shots):
        bot.stop()
        time.sleep(settle_pause)                 # let the chassis settle (less blur)
        folder = os.path.join(session, f"shot_{i:02d}")
        ok = cam.save_to(folder)
        with open(os.path.join(folder, "angle.txt"), "w") as f:
            f.write(f"{cumulative:.3f}\n")        # this shot's angle vs shot 0 (for the merge)
        log(f"  shot {i + 1}/{shots} (~{cumulative:.0f} deg) -> {os.path.basename(folder)}"
            + ("" if ok else "  (FRAME DROPPED)"))
        bot.beep(0.05)
        if i < shots - 1:
            _rotate_step(bot, rotate_speed, sec_per_deg * step_angle, direction, brake_tap)
            cumulative += step_angle

    # return to the starting heading (no photo, no shot folder, not merged)
    if return_home and shots > 1:
        if return_mode == "rewind":
            # undo the exact rotations just made: shots-1 identical pulses in reverse, so it
            # lands on start regardless of how sec_per_deg is calibrated.
            log(f"  returning to start: rewinding the {shots - 1} steps it made (no photo)")
            for _ in range(shots - 1):
                bot.stop()
                time.sleep(settle_pause)
                _rotate_step(bot, rotate_speed, sec_per_deg * step_angle, -direction, brake_tap)
                cumulative -= step_angle
        else:
            # "forward": one more step to complete the full 360 deg circle.
            log("  returning to start: one more step forward (no photo)")
            _rotate_step(bot, rotate_speed, sec_per_deg * step_angle, direction, brake_tap)
            cumulative += step_angle

    bot.stop()
    homed = ("" if not (return_home and shots > 1)
             else ", then rewound to start" if return_mode == "rewind"
             else ", then stepped forward to start (full 360 turn)")
    log(f"  scan complete: {shots} shots over ~{step_angle * (shots - 1):.0f} deg" + homed)
    return session


def scan_and_build(bot, cam, log=print, shots=SCAN_SHOTS, measure=True, **kw):
    """Full pipeline: sweep, then build the 360 cloud — all on the Pi."""
    session = run_scan(bot, cam, shots=shots, log=log, **kw)
    ply = build_from_session(session, measure=measure, log=log)
    return session, ply


# ── standalone CLI ──────────────────────────────────────────────────────────────────

def _calibrate(shots, speed, sec_per_deg, settle, brake_tap, turned=None):
    """Pulse-rotate EXACTLY like a scan (shots-1 start/stop pulses) so the time->angle ratio
    transfers to the real motion. Measure the total degrees turned to set SCAN_SEC_PER_DEG."""
    from setup_and_api.api import RasBot
    step_angle = 360.0 / shots
    step_time  = sec_per_deg * step_angle
    pulses     = shots - 1
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
                 "  python3 pointcloud/scan360.py <session_dir> --angle 36 # force a fixed step angle\n"
                 "  python3 pointcloud/scan360.py <session_dir> --dir -1   # flip rotation sign\n"
                 "  python3 pointcloud/scan360.py --calibrate [--shots N] [--speed S] [--turned DEG]")
    build_from_session(args[0], step_angle=angle, direction=direction, measure=measure)


if __name__ == "__main__":
    main()
