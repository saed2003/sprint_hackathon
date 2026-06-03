"""
geometry.py -- the math layer for the feature-based 360 reconstruction.

No feature matching here, no Open3D here. Just the pure-NumPy pieces every step
needs:

    load_intrinsics / load_depth_m   read a capture folder into usable arrays
    back_project_dense               full depth image  -> dense 3D cloud + color
    lift_keypoints                   a few pixel coords -> 3D (robust, windowed)
    kabsch / kabsch_ransac           3D<->3D correspondences -> rigid (R, t)
    save_ply / save_preview          write the result + a quick PNG to eyeball

Camera frame used everywhere below:  X right, Y down, Z forward (the way the lens
looks). We keep clouds in this raw frame the whole pipeline and only flip upright
once, at the very end, for viewing (see flip_upright).
"""

import os

import cv2
import numpy as np


# ── reading a capture folder ──────────────────────────────────────────────────────

def load_intrinsics(path):
    """Read 'key value' lines from an intrinsics.txt into a dict of floats.

    Expected keys: width height fx fy ppx ppy depth_scale baseline_m
    """
    vals = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                try:
                    vals[p[0]] = float(p[1])
                except ValueError:
                    pass
    return vals


def load_depth_m(shot_dir, intr):
    """depth.npy (uint16 raw units) -> float32 depth in METERS.

    depth_m = raw * depth_scale. Zeros stay zero (invalid / no return).
    """
    raw = np.load(os.path.join(shot_dir, "depth.npy"))
    scale = intr.get("depth_scale", 0.0001)
    return raw.astype(np.float32) * scale


def load_ir_gray(shot_dir):
    """Left IR image as uint8 grayscale (depth is aligned to the LEFT camera)."""
    return cv2.imread(os.path.join(shot_dir, "ir_left.png"), cv2.IMREAD_GRAYSCALE)


# ── depth cleanup ─────────────────────────────────────────────────────────────────

def drop_depth_edges(depth_m, max_step=0.04):
    """Zero out 'flying pixels' -- the stereo artifact that smears points along the
    view ray at object edges. A flyer differs a lot in depth from its neighbors;
    detect (jump > max_step m vs any 4-neighbor) and drop it. Returns a copy."""
    z = depth_m
    bad = np.zeros(z.shape, bool)
    for ax, sh in ((0, 1), (0, -1), (1, 1), (1, -1)):
        nb = np.roll(z, sh, axis=ax)
        both = (z > 0) & (nb > 0)
        bad |= both & (np.abs(z - nb) > max_step)
    out = z.copy()
    out[bad] = 0.0
    return out


# ── back-projection (depth -> 3D) ─────────────────────────────────────────────────

def back_project_dense(depth_m, color, intr, zmin, zmax):
    """Whole depth image -> (points Nx3 camera frame, colors Nx3 RGB uint8).

    Z = depth(u,v);  X = (u-ppx)Z/fx;  Y = (v-ppy)Z/fy.  Keep zmin < Z < zmax.

    color : (H,W) gray  -> point colored gray
            (H,W,3) BGR  -> real color (stored RGB)
    """
    fx, fy = intr["fx"], intr["fy"]
    ppx, ppy = intr["ppx"], intr["ppy"]
    h, w = depth_m.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    Z = depth_m
    X = (u - ppx) * Z / fx
    Y = (v - ppy) * Z / fy

    keep = (Z > zmin) & (Z < zmax)
    pts = np.stack([X[keep], Y[keep], Z[keep]], axis=1).astype(np.float32)

    if color is not None and color.ndim == 3:
        cols = color[keep][:, ::-1].astype(np.uint8)            # BGR -> RGB
    else:
        c = (color[keep] if color is not None
             else np.full(keep.sum(), 200, np.uint8)).astype(np.uint8)
        cols = np.stack([c, c, c], axis=1)
    return pts, cols


def lift_keypoints(xy, depth_m, intr, win=5, max_std=0.05, zmin=0.1, zmax=6.0):
    """Pixel coords -> 3D points, robustly. The key to good correspondences.

    Feature corners often land on depth discontinuities where a single-pixel depth
    is a flyer. So for each keypoint we take the MEDIAN depth in a win x win window
    and REJECT the point if that window is depth-inconsistent (std > max_std) or has
    no valid depth or falls outside [zmin, zmax].

    xy      : (M,2) float keypoint pixel coords (x=col, y=row), subpixel ok.
    returns : (pts Kx3, mask) where mask is the (M,) bool of which inputs survived.
              pts are the K survivors, in camera frame, in the SAME order as the
              True entries of mask.
    """
    fx, fy = intr["fx"], intr["fy"]
    ppx, ppy = intr["ppx"], intr["ppy"]
    h, w = depth_m.shape
    r = win // 2

    pts = []
    mask = np.zeros(len(xy), bool)
    for i, (x, y) in enumerate(xy):
        xi, yi = int(round(x)), int(round(y))
        if xi < r or yi < r or xi >= w - r or yi >= h - r:
            continue
        patch = depth_m[yi - r:yi + r + 1, xi - r:xi + r + 1]
        valid = patch[patch > 0]
        if valid.size < max(3, (win * win) // 4):
            continue
        z = float(np.median(valid))
        if z < zmin or z > zmax or float(valid.std()) > max_std:
            continue
        X = (x - ppx) * z / fx
        Y = (y - ppy) * z / fy
        pts.append((X, Y, z))
        mask[i] = True
    pts = np.asarray(pts, np.float32).reshape(-1, 3)
    return pts, mask


# ── rigid solve from 3D<->3D correspondences ──────────────────────────────────────

def kabsch(P, Q):
    """Best rigid (R, t) mapping P onto Q:  minimize ||(R @ P^T + t) - Q^T||.

    P, Q : (N,3) corresponding points (P in SOURCE frame, Q in TARGET frame).
    returns R (3x3), t (3,). So  Q ~= P @ R.T + t  (i.e. source -> target).
    """
    Pc = P - P.mean(0)
    Qc = Q - Q.mean(0)
    H = Pc.T @ Qc
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])                       # reflection guard
    R = Vt.T @ D @ U.T
    t = Q.mean(0) - R @ P.mean(0)
    return R, t


def _to_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def kabsch_ransac(P, Q, thresh=0.02, iters=2000, min_sample=3, seed=0):
    """RANSAC-robust rigid fit (source P -> target Q) from noisy 3D correspondences.

    P, Q : (N,3) putative matches (same order). Many are wrong -> RANSAC.
    thresh: inlier distance (m) between R@P+t and Q.
    returns (T 4x4 source->target, inlier_idx int array). T is refit on all inliers.
    Returns (None, empty) if too few points.
    """
    n = len(P)
    if n < min_sample:
        return None, np.empty(0, int)
    rng = np.random.default_rng(seed)
    best_inl = np.empty(0, int)

    for _ in range(iters):
        idx = rng.choice(n, min_sample, replace=False)
        try:
            R, t = kabsch(P[idx], Q[idx])
        except np.linalg.LinAlgError:
            continue
        resid = np.linalg.norm(P @ R.T + t - Q, axis=1)
        inl = np.where(resid < thresh)[0]
        if len(inl) > len(best_inl):
            best_inl = inl
            if len(inl) > 0.8 * n:                   # early out on a clear winner
                break

    if len(best_inl) < min_sample:
        return None, best_inl
    R, t = kabsch(P[best_inl], Q[best_inl])          # final refit on all inliers
    resid = np.linalg.norm(P @ R.T + t - Q, axis=1)
    best_inl = np.where(resid < thresh)[0]
    R, t = kabsch(P[best_inl], Q[best_inl])
    return _to_T(R, t), best_inl


# ── output ────────────────────────────────────────────────────────────────────────

def flip_upright(pts):
    """Stand a camera-frame cloud upright for MeshLab/CloudCompare.

    Image rows grow DOWN and the lens looks along +Z; viewers want +Y up, +Z toward
    you. Negating Y and Z fixes the upside-down/mirrored look. Returns a new array.
    """
    out = pts.copy()
    out[:, 1] *= -1
    out[:, 2] *= -1
    return out


def save_ply(path, pts, cols):
    """Write a colored cloud as binary little-endian .ply (MeshLab-friendly)."""
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
    print(f"  saved {n} points -> {path}")


def save_preview(path, pts, cols, max_pts=80000):
    """Top-down (X-Z) + front (X-Y) scatter PNG to sanity-check without a 3D viewer."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:                                       # noqa: BLE001
        print(f"  (preview skipped: {e})")
        return
    if len(pts) == 0:
        print("  (preview skipped: no points)")
        return
    i = np.random.choice(len(pts), min(len(pts), max_pts), replace=False)
    s, c = pts[i], np.clip(cols[i] / 255.0, 0, 1)
    fig = plt.figure(figsize=(11, 5))
    ax = fig.add_subplot(121); ax.set_aspect("equal")
    ax.scatter(s[:, 0], s[:, 2], c=c, s=1, linewidths=0)
    ax.set_title("TOP-DOWN (X-Z) -- the 360 ring"); ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)")
    ax2 = fig.add_subplot(122); ax2.set_aspect("equal")
    ax2.scatter(s[:, 0], s[:, 1], c=c, s=1, linewidths=0)
    ax2.set_title("FRONT (X-Y)"); ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)
    print(f"  preview -> {path}")
