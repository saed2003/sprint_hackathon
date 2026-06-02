"""
On-the-Pi 360-degree scan: rotate in place, capture N views (default 10 -> 36 deg each),
and merge them into ONE point cloud — entirely with numpy + OpenCV (NO Open3D, which has
no Raspberry Pi wheel). Everything runs on the robot; press R in drive.py to capture, T
to build.

How rotation + the merge work (the fix for "the bot turns too far"):
  The RasBot has no IMU/encoders, so we rotate by CALIBRATED TIMING: one motor pulse of
  SCAN_SEC_PER_DEG * 36 deg between each shot, 9 pulses for a 10-shot turn (~324 deg of
  rotation = a full 360 deg of coverage). Each shot records its nominal angle (0, 36, 72,
  ...) to shot_NN/angle.txt and the merge rotates each view by exactly that
  (back-projection: X=(u-ppx)Z/fx, Y=(v-ppy)Z/fy, Z=depth). Calibrate ONCE so 9 pulses
  land near 324 deg total: `python3 pointcloud/scan360.py --calibrate --turned <measured>`.

  There is also a VISION closed-loop (SCAN_CLOSED_LOOP / rotate_by_vision): pulse a bit,
  measure the turn from overlapping IR images (ORB -> homography -> yaw), stop at ~36 deg.
  It is OFF by default because at a 36 deg step the two IR images barely overlap, so the
  homography under-measures the yaw (it read ~20 deg when the bot really turned ~90); the
  loop then kept pulsing to its time budget and the bot spun ~2x too far (~820 deg for a
  360 scan), and the wrong recorded angles also skewed the merge. Calibrated timing is the
  reliable choice on this encoder-less chassis. Set SCAN_CLOSED_LOOP=True to try vision.

  For a still-cleaner result, copy the raw shots to a laptop and run merge_clouds.py (ICP).

Standalone uses (no robot needed — rebuild from shots already captured):
  python3 pointcloud/scan360.py captures/scan_20260531_1700              # rebuild, MEASURED angle
  python3 pointcloud/scan360.py captures/scan_20260531_1700 --known      # trust the timed step
  python3 pointcloud/scan360.py captures/scan_20260531_1700 --angle 36   # force a fixed step angle
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
SCAN_SHOTS        = 10       # views per full turn: 10 -> 36 deg each (9->40, 8->45)
SCAN_ROTATE_SPEED = 40       # motor speed used while rotating
# <<< CALIBRATE (pulsed): seconds of rotate-pulse per DEGREE, measured the way the scan
# actually moves (short start/stop pulses), NOT from one long continuous spin. This ONE knob
# sets how far the bot turns per step; the FULL turn (10 steps) must total 360 deg for the bot
# to come back to start. Bracketed from real runs of the full scan + return-home:
#   3.0/360 -> way over;  2.6/360 -> overshot start by ~10-20 deg;  2.5/360 -> undershot start.
# The value that closes on start is between 2.5 and 2.6, so start at the midpoint 2.55/360.
# Fine-tune from ONE full run by how far the bot ends from start:
#   undershoots by d deg: new = old * 360/(360 - d);  overshoots by d: new = old * 360/(360 + d).
SCAN_SEC_PER_DEG  = 2.55 / 360.0
SCAN_SETTLE_PAUSE = 0.4      # seconds to let the chassis stop shaking before a shot
SCAN_BRAKE_TAP    = 0.0      # seconds of reverse pulse after each step to kill coast (0=off)
SCAN_DIR          = 1        # +1 = rotate CCW (rotate_left); -1 = CW. Merge follows this.
SCAN_RETURN_HOME  = True     # after the last shot, do ONE more step (no photo) to close the
                             #   circle and bring the bot back to its starting heading.

# ── visual closed-loop rotation (stop when the CAMERA says ~step) ──
# OFF by default: at a 36 deg step the IR images barely overlap, so the homography
# UNDER-measures the yaw and the loop over-rotates (~820 deg for a 360 scan). Calibrated
# timing (closed_loop=False) is reliable on this encoder-less bot. See the module docstring.
SCAN_CLOSED_LOOP   = False   # rotate by vision (True) instead of calibrated timing (False)
SCAN_MIN_PULSE_DEG = 5       # smallest rotation pulse (enough to beat motor stiction)
SCAN_MAX_PULSE_DEG = 15      # largest single pulse (used early; pulses shrink near target)
SCAN_ANGLE_TOL     = 2       # stop once within this many deg of the target step
SCAN_STEP_BUDGET   = 1.25    # HARD cap on rotation per step = SCAN_STEP_BUDGET x target, so
                             #   a bad vision reading can't make one step spin past ~1.25x.
SCAN_YAW_GAIN      = 1.0     # correct a SYSTEMATIC vision bias: raise above 1.0 if the robot
                             #   consistently OVER-rotates, lower below 1.0 if it UNDER-rotates.

# ── cloud tunables ──────────────────────────────────────────────────────────────
VOXEL = 0.01                 # 1 cm final resolution
ZMIN, ZMAX = 0.1, 1.5        # keep points in the D405's GOOD range (meters). The D405 is
                             # short-range passive stereo: depth past ~1.5 m is mostly noise,
                             # so we drop it (raise ZMAX for a big room, lower it for cleaner).
MIN_NEIGHBORS = 4            # outlier filter: keep a point only if >= this many of its 26
                             # neighbour voxel cells are also filled (0 disables). Kills flyers.

# ── visual angle-measurement tunables (Track B: don't trust the timer) ───────────
MEASURE_MIN_INLIERS = 20     # need at least this many homography inliers to trust a step
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


def read_angle(folder):
    """Cumulative turn angle (deg magnitude) the closed-loop scan recorded for a shot,
    or None if the shot predates it (then the merge measures/assumes the angle)."""
    p = os.path.join(folder, "angle.txt")
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            return abs(float(f.read().split()[0]))
    except (ValueError, IndexError):
        return None


def yaw_from_images(a, b, K):
    """Yaw (deg, about the vertical axis) between two grayscale IR images.

    We model the robot's in-place spin as a (near) pure camera rotation, for which
    matched points are related by a HOMOGRAPHY  H = K R K^-1. So: ORB features matched
    with Lowe's ratio test -> RANSAC homography -> R = K^-1 H K -> read the yaw.

    Why not the essential matrix (the old way)? It needs camera TRANSLATION and
    degenerates for in-place rotation, so almost no inliers survived (the "7-20 inliers
    rejected" you saw). The homography is the correct, robust model here.

    Returns (yaw_deg or None, n_inliers). yaw is signed; callers use its magnitude and
    apply the known rotation direction. None means "couldn't measure" (fall back).
    """
    if a is None or b is None:
        return None, 0

    orb = cv2.ORB_create(4000)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None or len(ka) < 12 or len(kb) < 12:
        return None, 0

    # Lowe ratio test keeps many more good matches than crossCheck
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

    # H = K R K^-1  ->  R ~ K^-1 H K ; snap to the nearest rotation (SVD) and read the yaw
    # in the same convention as ry(): R[0,0]=cos, R[0,2]=sin.
    R = np.linalg.inv(K) @ H @ K
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt
    yaw = math.degrees(math.atan2(R[0, 2], R[0, 0]))
    return yaw, inliers


def estimate_yaw(folder_a, folder_b, K):
    """Shot-to-shot yaw: load the two folders' left-IR images and measure their
    relative rotation (see yaw_from_images). Used by the merge as a fallback when a
    shot has no recorded closed-loop angle."""
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
    """Drop lone specks: keep a point only if >= min_neighbors of its 26 neighbouring
    voxel cells are also occupied. A pure-numpy stand-in for Open3D's statistical
    outlier removal (the laptop merge_clouds.py uses Open3D; the Pi has only numpy).

    Call after voxel_downsample so each occupied cell holds exactly one point.
    """
    if len(pts) == 0 or min_neighbors <= 0:
        return pts, cols
    k = np.floor(pts / voxel).astype(np.int64)
    k = k - k.min(axis=0) + 1                      # shift into [1, max+1] (no negatives)
    dimx = int(k[:, 0].max()) + 3                  # pad so neighbours stay in [0, dim)
    dimy = int(k[:, 1].max()) + 3
    cell_hash = lambda a: a[:, 0] + a[:, 1] * dimx + a[:, 2] * dimx * dimy  # collision-free
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
    """Angle (deg, about vertical) to rotate each view into view-0's frame.

    Known-angle mode (measure=False) just returns 0, step, 2*step, ... — the timer is
    trusted. Measured mode (the default) reads the TRUE per-step yaw from each pair of
    overlapping IR images (see estimate_yaw) and uses that, so the cloud is correct even
    when the open-loop rotation overshoots. We trust the camera for the step MAGNITUDE and
    keep the known turn direction (sign of nominal_step); anything outside a sane band, or
    with too few inliers, falls back to the nominal step.
    """
    n = len(dirs)
    if n < 2:
        return [0.0] * n

    # closed-loop recorded angles win when present — they ARE the measured truth, so the
    # merge needs no image re-measurement and the rebuild is reproducible.
    if measure:
        recorded = [read_angle(d) for d in dirs]
        if all(a is not None for a in recorded):
            sign = math.copysign(1.0, nominal_step)
            log("  using closed-loop recorded angles: "
                + ", ".join(f"{a:.0f}" for a in recorded) + " deg")
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

    measure=True (default): use the camera-measured per-step yaw (robust to bad timing).
    measure=False or step_angle given: trust the known/timed step angle.
    save_shots=True: also write each photo's own point cloud to shot_NN/cloud.ply.
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
        if save_shots:
            # the per-photo point cloud, in this shot's own camera frame
            write_ply(os.path.join(d, "cloud.ply"), pts, cols)
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
        mode = "closed-loop angle"
    else:
        mode = "image-measured angle"
    log(f"  merged {len(dirs)} views ({mode}, swept {angles[-1]:.0f} deg) -> "
        f"{len(pts)} points (cleaned {before - len(pts)} flyers) -> {out}")

    # also save a still preview image of the cloud (headless — works with no display)
    try:
        from pointcloud import view3d
        prev = view3d.save_view(out, os.path.join(session_dir, "merged_360_preview.png"))
        if prev:
            log(f"  preview image -> {prev}")
    except Exception as e:
        log(f"  (preview render skipped: {e})")
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


def _K_from_cam(cam):
    """3x3 camera matrix from the live D405 intrinsics (valid after cam.start())."""
    return np.array([[cam.intr.fx, 0, cam.intr.ppx],
                     [0, cam.intr.fy, cam.intr.ppy],
                     [0, 0, 1]], dtype=np.float64)


def rotate_by_vision(bot, cam, target_deg, K, direction=SCAN_DIR,
                     rotate_speed=SCAN_ROTATE_SPEED, sec_per_deg=SCAN_SEC_PER_DEG,
                     min_pulse=SCAN_MIN_PULSE_DEG, max_pulse=SCAN_MAX_PULSE_DEG,
                     tol=SCAN_ANGLE_TOL, budget=SCAN_STEP_BUDGET, gain=SCAN_YAW_GAIN,
                     brake_tap=SCAN_BRAKE_TAP, settle=0.2, log=print):
    """Rotate in pulses until the CAMERA says we have turned ~target_deg about the vertical
    axis (measure cumulative yaw vs the pre-rotation frame). Returns the achieved yaw (deg).

    Anti-overshoot, in three layers:
      * pulses SHRINK as we approach the target (each is sized to the remaining angle), so
        the last pulse barely overshoots instead of a fixed big jump;
      * a HARD per-step time budget (budget x target) ends the step even if vision keeps
        reading low, so one bad step can never spin past ~budget x the target;
      * an optional brake_tap drives the wheels back briefly to cancel coast.
    `gain` corrects a systematic vision bias. A failed/absurd read dead-reckons from the timer.
    """
    spin  = bot.rotate_left  if direction >= 0 else bot.rotate_right
    brake = bot.rotate_right if direction >= 0 else bot.rotate_left
    ref = cam.grab_ir()                              # the view before this step's rotation
    achieved = 0.0
    spent = 0.0
    max_spin = sec_per_deg * target_deg * budget     # hard cap on total spin TIME per step
    for _ in range(30):                              # final safety bound on iterations
        if achieved >= target_deg - tol or spent >= max_spin:
            break
        pulse_deg = max(min_pulse, min(max_pulse, target_deg - achieved))   # shrink near target
        pulse_time = sec_per_deg * pulse_deg
        spin(rotate_speed)
        time.sleep(pulse_time)
        if brake_tap > 0:
            bot.stop(); brake(rotate_speed); time.sleep(brake_tap)
        bot.stop()
        spent += pulse_time
        time.sleep(settle)                           # let it stop shaking before measuring
        yaw, inliers = yaw_from_images(ref, cam.grab_ir(), K)
        if yaw is not None and inliers >= MEASURE_MIN_INLIERS and abs(yaw) <= target_deg * 1.6:
            achieved = abs(yaw) * gain               # absolute angle from the reference frame
        else:
            achieved += pulse_deg                    # vision failed/absurd -> dead-reckon
    bot.stop()
    log(f"    turned ~{achieved:.0f} deg (target {target_deg:.0f})"
        + ("  [time-capped]" if spent >= max_spin else ""))
    return achieved


def run_scan(bot, cam, shots=SCAN_SHOTS, rotate_speed=SCAN_ROTATE_SPEED,
             sec_per_deg=SCAN_SEC_PER_DEG, settle_pause=SCAN_SETTLE_PAUSE,
             brake_tap=SCAN_BRAKE_TAP, direction=SCAN_DIR,
             closed_loop=SCAN_CLOSED_LOOP, return_home=SCAN_RETURN_HOME,
             out_root=None, log=print):
    """Rotate in place and capture `shots` views. Returns the session folder.

    closed_loop=False (default): blind, calibrated timed pulses (no camera used to steer).
    closed_loop=True: each step rotates by VISION (rotate_by_vision) until the camera
    reports ~360/shots deg, and the achieved angle is written to each shot's angle.txt.

    return_home=True (default): after the last shot, do ONE more identical rotation step
    (no photo) so the 10 shots over 0..324 deg are followed by a final 36 deg turn that
    completes the full 360 deg circle and leaves the bot back on its starting heading.
    """
    out_root = out_root or default_out_root()
    session = os.path.join(out_root, "scan_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    step_angle = 360.0 / shots

    if cam.pipeline is None:
        log("Opening D405 (warming up auto-exposure)...")
        cam.start()
        log(cam.info())

    K = _K_from_cam(cam)
    log(f"360 scan: {shots} shots, {step_angle:.0f} deg each "
        f"({'vision closed-loop' if closed_loop else 'timed open-loop'} rotation)")
    cumulative = 0.0                              # achieved turn so far (deg, from shot 0)
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
            if closed_loop:
                turned = rotate_by_vision(bot, cam, step_angle, K, direction=direction,
                                          rotate_speed=rotate_speed, sec_per_deg=sec_per_deg,
                                          brake_tap=brake_tap, log=log)
            else:
                _rotate_step(bot, rotate_speed, sec_per_deg * step_angle, direction, brake_tap)
                turned = step_angle
            cumulative += turned

    # close the circle: one more identical rotation (no photo, no shot folder, not merged)
    # so the bot ends on its starting heading. The 10 shots sit 36 deg apart over 0..324 deg;
    # this final step adds the last 36 deg to make a full 360 deg turn.
    if return_home and shots > 1:
        log("  returning to start heading (one more step, no photo)")
        if closed_loop:
            rotate_by_vision(bot, cam, step_angle, K, direction=direction,
                             rotate_speed=rotate_speed, sec_per_deg=sec_per_deg,
                             brake_tap=brake_tap, log=log)
        else:
            _rotate_step(bot, rotate_speed, sec_per_deg * step_angle, direction, brake_tap)
        cumulative += step_angle

    bot.stop()
    log(f"  scan complete: {shots} shots over ~{step_angle * (shots - 1):.0f} deg"
        + (f", then returned to start (~{cumulative:.0f} deg full turn)"
           if return_home and shots > 1 else ""))
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
                 "  python3 pointcloud/scan360.py <session_dir> --angle 36 # force a fixed step angle\n"
                 "  python3 pointcloud/scan360.py <session_dir> --dir -1   # flip rotation sign\n"
                 "  python3 pointcloud/scan360.py --calibrate [--shots N] [--speed S] [--turned DEG]")
    build_from_session(args[0], step_angle=angle, direction=direction, measure=measure)


if __name__ == "__main__":
    main()
