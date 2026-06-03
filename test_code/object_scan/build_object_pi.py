"""build_object_pi.py -- feature-based 360 OBJECT reconstruction that runs ON THE PI.

Same idea as src/new_point_cloud/register_360.py (CLAHE -> ORB/SIFT, lift to 3D, match,
Kabsch+RANSAC rigid fit -- the pose is MEASURED from correspondences, not guessed) but
with NO Open3D, so the whole orbit -> 3D model can finish on the robot:

    register_360 (laptop)            build_object_pi (Pi)
    ----------------------           ---------------------------------
    pose-graph global optimisation   BFS pose chain + fractional loop closure   (numpy)
    o3d voxel_down_sample            voxel_downsample                           (numpy)
    o3d remove_statistical_outlier   remove_isolated (voxel-neighbour count)    (numpy)
    matplotlib/o3d preview           cv2 two-panel scatter                      (cv2)

It REUSES the proven, pure-numpy/cv2 pieces directly:
  * register_360.py  -> _detect/get_feat/match/register_pair_robust/components/bfs_init_poses
    (these never import Open3D; only shot_down/optimize_poses/finalize do, and we don't call them)
  * geometry.py      -> load_intrinsics/load_depth_m/load_ir_gray/back_project_dense/
                        kabsch_ransac/drop_depth_edges/save_ply

The only object-specific addition is per-shot SEGMENTATION (depth gate + RANSAC floor
removal + largest blob + crop) -- the same trick that fixed the orbit tracker -- so the
features and the cloud are the OBJECT, not the room/floor. Blacking out the background in
each shot's IR also means CLAHE+ORB naturally finds keypoints only on the object.

RUN (Pi or laptop; needs cv2 + numpy, NOT open3d):
    python3 build_object_pi.py                              # newest orbit_/obj_ session
    python3 build_object_pi.py captures/orbit_<ts>          # a specific session
    python3 build_object_pi.py captures/orbit_<ts> --object db5 --sift
    python3 build_object_pi.py captures/orbit_<ts> --voxel 0.002 --zmax 0.6 --win 3
Writes <session>/object_pi.ply (+ object_pi_preview.png). View on the Pi:
    python3 ../../src/pointcloud/view3d.py captures/orbit_<ts>/object_pi.ply
"""
import os
import sys
import glob
import math

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "src"))
sys.path.insert(0, os.path.join(SRC, "new_point_cloud"))   # for `import geometry` / `register_360`
sys.path.insert(0, HERE)                                   # for `import config`

import geometry as G            # noqa: E402  pure numpy/cv2
import register_360 as R        # noqa: E402  Open3D imported lazily inside funcs we don't call
import config                   # noqa: E402

CAPTURES = os.path.join(HERE, "captures")

# object-tuned registration (closer + less noisy than the room scan register_360 targets)
KP_MAX_STD    = 0.04      # tighter depth-window std (object depth is cleaner at ~35 cm)
RANSAC_THRESH = 0.02      # 2 cm 3D-3D inlier band
MAX_RMSE      = 0.02
MIN_INLIERS   = 6
# floor removal (same constants as capture_orbit.py)
FLOOR_THRESH, FLOOR_MIN_FRAC, FLOOR_VERT = 0.015, 0.30, 0.6


# ── per-shot object segmentation (depth gate + floor removal + largest blob + crop) ──

def _remove_floor(pts):
    """Boolean mask of points NOT on the dominant ~horizontal plane (the floor). Pure
    numpy RANSAC; keeps everything if no big horizontal plane is found (object fills frame)."""
    n = len(pts)
    if n < 200:
        return np.ones(n, bool)
    rng = np.random.default_rng(0)
    s = pts if n <= 4000 else pts[rng.choice(n, 4000, replace=False)]
    best_cnt, best_nrm, best_p0 = 0, None, None
    for _ in range(80):
        p0, p1, p2 = s[rng.choice(len(s), 3, replace=False)]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm = nrm / ln
        cnt = int((np.abs((s - p0) @ nrm) < FLOOR_THRESH).sum())
        if cnt > best_cnt:
            best_cnt, best_nrm, best_p0 = cnt, nrm, p0
    if best_nrm is None:
        return np.ones(n, bool)
    if best_cnt >= FLOOR_MIN_FRAC * len(s) and abs(best_nrm[1]) >= FLOOR_VERT:
        return np.abs((pts - best_p0) @ best_nrm) >= FLOOR_THRESH
    return np.ones(n, bool)


def _largest_blob(mask_img):
    """Largest connected component of a HxW bool mask (the object body)."""
    num, lab, stats, _ = cv2.connectedComponentsWithStats(mask_img.astype(np.uint8), 8)
    if num <= 1:
        return mask_img
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return lab == best


def object_mask(depth_m, intr, zmin, zmax, crop):
    """HxW bool mask of just the object: depth gate -> drop the floor plane -> largest blob
    -> crop to a box of half-size `crop` (m) around the object's 3D centroid. None if empty."""
    gate = (depth_m > zmin) & (depth_m < zmax)
    if int(gate.sum()) < 500:
        return None
    vs, us = np.where(gate)
    z = depth_m[gate]
    x = (us - intr["ppx"]) * z / intr["fx"]
    y = (vs - intr["ppy"]) * z / intr["fy"]
    pts = np.stack([x, y, z], axis=1)
    keep = _remove_floor(pts)
    if int(keep.sum()) < 300:
        keep = np.ones(len(z), bool)
    m = np.zeros(depth_m.shape, bool)
    m[vs[keep], us[keep]] = True
    m = _largest_blob(m)
    sel = m[vs, us]
    if int(sel.sum()) < 200:
        return m
    c = np.median(pts[sel], axis=0)                       # object centroid (3D)
    near = np.all(np.abs(pts - c) < crop, axis=1)
    final = sel & near
    if int(final.sum()) < 200:
        return m
    m2 = np.zeros(depth_m.shape, bool)
    m2[vs[final], us[final]] = True
    return m2


def load_shot_masked(d, zmin, zmax, crop):
    """Capture folder -> shot dict with IR + depth MASKED to the object (background -> 0),
    plus the colour image for the final cloud. Shape matches what register_360 expects."""
    intr = G.load_intrinsics(os.path.join(d, "intrinsics.txt"))
    depth = G.load_depth_m(d, intr)
    gray = G.load_ir_gray(d)
    m = object_mask(depth, intr, zmin, zmax, crop)
    if m is None:
        return None
    color = cv2.imread(os.path.join(d, "color.png"))
    if color is None:
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return {
        "dir": d, "name": os.path.basename(d), "intr": intr,
        "gray": np.where(m, gray, 0).astype(gray.dtype),
        "depth_m": np.where(m, depth, 0).astype(depth.dtype),
        "color": color, "mask": m,
    }


# ── pairwise registration over a ring window (reuses register_360) ──────────────────

def build_edges(shots, force_sift, window, log=print):
    """Register each shot to its next `window` neighbours (wrapping), giving ring odometry
    + short loop closures. Returns {(a,b): result} with a<b. Reuses register_360's matcher."""
    n = len(shots)
    pairs = sorted({(min(i, (i + dj) % n), max(i, (i + dj) % n))
                    for i in range(n) for dj in range(1, window + 1) if (i + dj) % n != i})
    edges = {}
    for a, b in pairs:
        try:
            r, det = R.register_pair_robust(shots[a], shots[b], force_sift)
        except Exception as e:                                       # noqa: BLE001
            r, det = None, None
            log(f"  {a:2d}-{b:2d} register error: {e}")
        if r is not None:
            edges[(a, b)] = r
            tag = "odom" if (abs(a - b) == 1 or {a, b} == {0, n - 1}) else "loop"
            log(f"  {a:2d}-{b:2d} [{tag}] {det}: {r['n']:3d} inliers, rmse {r['rmse'] * 1000:4.0f} mm")
    return edges


# ── poses: BFS chain + fractional loop-closure (the numpy stand-in for global opt) ──

def _frac_T(T, f):
    """Fraction `f` of a rigid transform: rotation via scaled axis-angle, translation
    linear. An approximation of the SE3 power, fine for spreading a small loop drift."""
    rvec, _ = cv2.Rodrigues(T[:3, :3].astype(np.float64))
    Rf, _ = cv2.Rodrigues(rvec * f)
    out = np.eye(4)
    out[:3, :3] = Rf
    out[:3, 3] = T[:3, 3] * f
    return out


def close_loop(poses, n, edges, log=print):
    """Spread the ring's accumulated drift around the loop. Uses the 0<->(n-1) edge: it
    predicts shot (n-1)'s pose a second way; the mismatch with the BFS chain is the drift,
    applied progressively (0 at the root, full at the far side) so the seam closes."""
    key = (0, n - 1)
    if key not in edges or 0 not in poses or (n - 1) not in poses:
        return poses
    try:
        pose_last_loop = poses[0] @ np.linalg.inv(edges[key]["T"])
        D = pose_last_loop @ np.linalg.inv(poses[n - 1])          # world-frame drift to spread
        ang = math.degrees(np.linalg.norm(cv2.Rodrigues(D[:3, :3].astype(np.float64))[0]))
        log(f"  loop drift: {ang:.1f} deg, {np.linalg.norm(D[:3, 3]) * 100:.1f} cm -> spreading")
        placed = sorted(poses)
        last = placed[-1]
        for i in placed:
            poses[i] = _frac_T(D, i / last if last else 0.0) @ poses[i]
    except Exception as e:                                          # noqa: BLE001
        log(f"  loop closure skipped: {e}")
    return poses


def solve_poses(n, edges, log=print):
    comps = R.components(n, edges)
    main = comps[0]
    log(f"  components: {[sorted(c) for c in comps]}")
    if len(main) < 2:
        sys.exit("registration failed: no two shots share enough object texture+depth. "
                 "Recapture with more overlap / better light (see object_scan README).")
    if len(main) < n:
        log(f"  placing largest component ({len(main)}/{n}); rest dropped: "
            f"{sorted(set(range(n)) - main)}")
    poses = R.bfs_init_poses(min(main), edges)
    return close_loop(poses, n, edges, log)


# ── render + clean + save (all numpy/cv2) ───────────────────────────────────────────

def render(shots, poses, zmin, zmax, log=print):
    allp, allc = [], []
    for i in sorted(poses):
        s = shots[i]
        depth = G.drop_depth_edges(s["depth_m"])
        pts, cols = G.back_project_dense(depth, s["color"], s["intr"], zmin, zmax)
        if len(pts) == 0:
            continue
        T = poses[i]
        pts = pts @ T[:3, :3].T + T[:3, 3]
        allp.append(pts.astype(np.float32))
        allc.append(cols)
        log(f"  {s['name']}: {len(pts):6d} pts")
    if not allp:
        sys.exit("nothing to render (no placed shots had object depth).")
    return np.concatenate(allp), np.concatenate(allc)


def voxel_downsample(pts, cols, voxel):
    """One point per voxel cell (numpy)."""
    if len(pts) == 0 or voxel <= 0:
        return pts, cols
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx], cols[idx]


def remove_isolated(pts, cols, voxel, min_neighbors=3):
    """Drop specks: keep a point only if >= min_neighbors of its 26 voxel neighbours are
    occupied (numpy stand-in for statistical outlier removal). Call after downsample."""
    if len(pts) == 0 or min_neighbors <= 0:
        return pts, cols
    k = np.floor(pts / voxel).astype(np.int64)
    k = k - k.min(axis=0) + 1
    dimx = int(k[:, 0].max()) + 3
    dimy = int(k[:, 1].max()) + 3
    h = lambda a: a[:, 0] + a[:, 1] * dimx + a[:, 2] * dimx * dimy
    occupied = np.unique(h(k))
    counts = np.zeros(len(pts), np.int32)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx or dy or dz:
                    counts += np.isin(h(k + np.array([dx, dy, dz])), occupied)
    keep = counts >= min_neighbors
    return pts[keep], cols[keep]


def save_preview(path, pts, cols, px=900):
    """Two-panel scatter PNG (top-down X-Z + front X-Y), cv2 only (no matplotlib)."""
    if len(pts) == 0:
        return

    def panel(ax0, ax1, flip1, title):
        a, b = pts[:, ax0].copy(), pts[:, ax1].copy()
        if flip1:
            b = -b
        lo = np.array([a.min(), b.min()])
        span = max((a.max() - a.min()), (b.max() - b.min()), 1e-3)
        ia = ((a - lo[0]) / span * (px - 40) + 20).astype(np.int32)
        ib = ((b - lo[1]) / span * (px - 40) + 20).astype(np.int32)
        img = np.full((px, px, 3), 30, np.uint8)
        ib = px - 1 - ib                                           # image y is down
        ok = (ia >= 0) & (ia < px) & (ib >= 0) & (ib < px)
        img[ib[ok], ia[ok]] = cols[ok][:, ::-1]                   # RGB -> BGR
        cv2.putText(img, title, (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return img

    top = panel(0, 2, False, "TOP-DOWN (X-Z) -- the orbit ring")
    front = panel(0, 1, True, "FRONT (X-Y)")
    cv2.imwrite(path, np.hstack([top, front]))
    print(f"  preview -> {path}")


# ── session discovery + main ────────────────────────────────────────────────────────

def newest_session():
    cand = []
    for pat in ("orbit_*", "obj_*", "circle_*", "cscan_*"):
        cand += glob.glob(os.path.join(CAPTURES, pat))
    cand = [c for c in cand if os.path.isdir(c)]
    return max(cand, key=os.path.getmtime) if cand else None


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        v = cast(args[i + 1])
        del args[i:i + 2]
        return v
    return None


def main():
    args = sys.argv[1:]
    force_sift = "--sift" in args
    if force_sift:
        args.remove("--sift")
    obj = _pop(args, "--object", str)
    zmin = _pop(args, "--zmin", float)
    zmax = _pop(args, "--zmax", float)
    voxel = _pop(args, "--voxel", float)
    crop = _pop(args, "--crop", float)
    window = _pop(args, "--win", int) or 3

    cfg = config.select(obj) if obj else config.DEFAULT
    zmin = cfg["zmin"] if zmin is None else zmin
    zmax = cfg["zmax"] if zmax is None else zmax
    voxel = cfg["voxel"] if voxel is None else voxel
    crop = cfg["crop"] if crop is None else crop

    session = args[0] if args else newest_session()
    if not session or not os.path.isdir(session):
        sys.exit(f"no session given and none found under {CAPTURES}")
    dirs = R.shot_dirs(session)                               # shot_*/ir_left.png, sorted
    if not dirs:
        sys.exit(f"no shot_*/ir_left.png in {session}")

    # object-tuned registration knobs into the reused module
    R.ZMIN, R.ZMAX = zmin, zmax
    R.KP_MAX_STD, R.RANSAC_THRESH = KP_MAX_STD, RANSAC_THRESH
    R.MAX_RMSE, R.MIN_INLIERS = MAX_RMSE, MIN_INLIERS

    print(f"build_object_pi: {cfg['name']}")
    print(f"  session {session}: {len(dirs)} shots, object band [{zmin}, {zmax}] m, "
          f"crop {crop * 100:.0f}cm, voxel {voxel * 1000:.1f}mm, "
          f"detector {'SIFT' if force_sift else 'ORB->SIFT'}, win {window}")

    shots, kept = [], []
    for d in dirs:
        s = load_shot_masked(d, zmin, zmax, crop)
        if s is None:
            print(f"  {os.path.basename(d)}: object not found (skipped)")
            continue
        shots.append(s)
        kept.append(d)
    n = len(shots)
    if n < 2:
        sys.exit("fewer than 2 shots have a visible object -- check the depth band / scan.")
    print(f"  {n} shots with a segmented object")

    print("Registering ring pairs:")
    edges = build_edges(shots, force_sift, window)
    if not edges:
        sys.exit("no pair registered -- object too smooth/dark or too little overlap.")
    poses = solve_poses(n, edges)

    print("Rendering:")
    pts, cols = render(shots, poses, zmin, zmax)
    before = len(pts)
    pts, cols = voxel_downsample(pts, cols, voxel)
    pts, cols = remove_isolated(pts, cols, voxel, min_neighbors=3)
    print(f"  {before} -> {len(pts)} points after voxel/cull")

    ply = os.path.join(session, "object_pi.ply")
    G.save_ply(ply, pts, cols)
    save_preview(os.path.join(session, "object_pi_preview.png"), pts, cols)
    print(f"Done -> {ply}")
    print(f"  view on the Pi:  python3 {os.path.join('..', '..', 'src', 'pointcloud', 'view3d.py')} {ply}")


if __name__ == "__main__":
    main()
