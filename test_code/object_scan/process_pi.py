"""process_pi.py -- turn an orbit scan into a CLEAN 3D model of the object (runs ON THE PI).

This is the post-processing pipeline that sits after run_pi.py. run_pi merges the orbit
with image features; this file takes the same captures and pushes them further:

    [1] LOAD + SEGMENT      reuse build_object_pi's depth-gate + floor-RANSAC + largest-blob,
                            then ALSO cut the floor "stalk" under the car with a height band
                            (the silver car is ~5 cm tall but the raw segment keeps ~15 cm of
                            floor/reflection below it -- that column is most of the old smear).
    [2] REGISTER            reuse the PROVEN feature poses (build_object_pi.build_edges +
                            solve_poses: CLAHE->ORB/SIFT, Kabsch+RANSAC, BFS + loop closure).
                            These are the most accurate poses we can measure from this scan.
    [3] CONNECT THE GAPS    the smooth-sided car leaves whole arcs with no matchable texture,
                            so feature registration drops those shots. --full estimates one
                            average per-step motion (an SE(3) "screw", in closed form) from the
                            feature edges and places each dropped shot at the screw-power of its
                            nearest placed neighbour -> fuller coverage (softer; off by default).
    [4] FUSE (dup-aware)    the fast orbit shoots the same surface many times. Instead of piling
                            the duplicates up (smear) every voxel AVERAGES all observations that
                            land in it -> redundant/duplicate views become DENOISING, and the
                            per-voxel hit count is a confidence we threshold.
    [5] DENOISE             statistical-outlier removal (k-NN mean distance) drops stereo flyers;
                            keeping the largest Euclidean cluster drops detached debris.
    [6] SAVE                object_clean.ply + a 3-view (top/front/side) preview PNG.

Pure numpy + cv2 (scipy used for the k-NN if present, else a numpy fallback) so it finishes
on the Pi 5. Reuses the teammate's feature/geometry code in src/new_point_cloud
(register_360.py as R, geometry.py as G) and build_object_pi.py (as PI).

RUN (Pi or laptop):
    python3 process_pi.py                                # newest orbit_/obj_ session, clean mode
    python3 process_pi.py captures/orbit_<ts>            # a specific session
    python3 process_pi.py captures/orbit_<ts> --full     # add screw-bridged shots (more coverage)
    python3 process_pi.py captures/orbit_<ts> --object db5 --sift --win 3 --voxel 0.003
    python3 process_pi.py captures/orbit_<ts> --keep-stand   # don't cut the floor stalk
Writes <session>/object_clean.ply (+ object_clean_preview.png). View on the Pi:
    python3 ../../src/pointcloud/view3d.py captures/orbit_<ts>/object_clean.ply
"""
import os
import sys
import glob

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config                       # noqa: E402
import build_object_pi as PI        # noqa: E402  (pulls register_360 as PI.R, geometry as PI.G)

R = PI.R
G = PI.G

try:
    from scipy.spatial import cKDTree
    _HAVE_KD = True
except Exception:                   # noqa: BLE001
    _HAVE_KD = False

CAPTURES = os.path.join(HERE, "captures")

# pipeline defaults (overridable on the CLI)
FEAT_WINDOW   = 3       # register each shot to its next k neighbours (wrapping)
FUSE_VOXEL    = 0.0025  # voxel-mean fusion cell (m): a touch coarser than capture voxel = denoise
MIN_VOX_HITS  = 2       # drop voxels seen by fewer than this many observations (duplicate gate)
SOR_K         = 12      # statistical-outlier removal: neighbours per point
SOR_STD       = 2.0     # ...drop points whose mean-NN distance > mean + STD*sigma
CROP_MARGIN   = 0.03    # extra metres kept below the object's top in the height-band crop
MAX_STEP_DEG  = 30.0    # ignore a feature edge implying > this per shot-index when learning the screw


# ── [1] load + segment (reuse build_object_pi) + cut the floor stalk ────────────────

def crop_stalk(shot, obj_height, margin=CROP_MARGIN):
    """Keep only the band from the object's top down to top + obj_height (+margin), in 3D.
    The depth gate + floor RANSAC still leave the floor patch / reflection directly under
    the car (a vertical 'stalk' ~10 cm tall) because it sits at the same depth; this trims
    it so the cloud is the car body, which both tightens it and sharpens the features."""
    depth = shot["depth_m"]
    ys, xs = np.where(depth > 0)
    if len(ys) < 60:
        return shot
    intr = shot["intr"]
    z = depth[ys, xs]
    y3d = (ys - intr["ppy"]) * z / intr["fy"]          # metric height (camera Y, +down)
    ytop = float(np.percentile(y3d, 2))                # robust topmost object point
    band = y3d <= ytop + obj_height + margin
    if int(band.sum()) < 60:
        return shot
    m = np.zeros(depth.shape, bool)
    m[ys[band], xs[band]] = True
    out = dict(shot)
    out["gray"] = np.where(m, shot["gray"], 0).astype(shot["gray"].dtype)
    out["depth_m"] = np.where(m, depth, 0).astype(depth.dtype)
    out["mask"] = m
    return out


def load_session(session, cfg, crop_stand, log=print):
    """All shot_*/ folders -> list of object-segmented shot dicts (ready for register_360),
    height-cropped to the object body unless --keep-stand."""
    zmin, zmax, crop = cfg["zmin"], cfg["zmax"], cfg["crop"]
    obj_height = cfg.get("car_height") or cfg.get("fig_height") or 0.08
    dirs = R.shot_dirs(session)
    if not dirs:
        sys.exit(f"no shot_*/ir_left.png in {session}")
    shots = []
    for d in dirs:
        s = PI.load_shot_masked(d, zmin, zmax, crop)
        if s is None:
            log(f"  {os.path.basename(d)}: object not found (skipped)")
            continue
        if crop_stand:
            s = crop_stalk(s, obj_height)
        shots.append(s)
    n = len(shots)
    if n < 2:
        sys.exit("fewer than 2 usable shots -- check the depth band / lighting / scan.")
    how = (f"stalk-cropped to ~{(obj_height + CROP_MARGIN) * 100:.0f} cm body"
           if crop_stand else "full segment kept")
    log(f"  {n} usable shots ({how})")
    return shots


# ── [2] register: the proven feature poses (reuse build_object_pi) ───────────────────

def feature_poses(shots, window, force_sift, log=print):
    """Reuse build_object_pi's proven registration: CLAHE->ORB/SIFT 3D<->3D Kabsch+RANSAC
    over a ring window, BFS chain + fractional loop closure. Returns (poses, edges) where
    poses maps placed-shot index -> 4x4 local->world for the largest connected component."""
    edges = PI.build_edges(shots, force_sift, window, log=log)
    if not edges:
        sys.exit("no pair registered -- object too smooth/dark or too little overlap (try --sift).")
    poses = PI.solve_poses(len(shots), edges, log=log)
    return poses, edges


# ── [3] connect the gaps: one average per-step SE(3) screw, in closed form ───────────

def _skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], float)


def se3_log(T):
    """SE(3) -> 6-vector (omega, upsilon). Closed form (no scipy) so it runs on the Pi."""
    Rm, t = T[:3, :3], T[:3, 3]
    c = (np.trace(Rm) - 1.0) / 2.0
    th = float(np.arccos(np.clip(c, -1.0, 1.0)))
    if th < 1e-8:
        return np.concatenate([np.zeros(3), t])
    w = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]]) / (2 * np.sin(th))
    omega = w * th
    W = _skew(omega)
    Vinv = np.eye(3) - 0.5 * W + (1.0 / th**2 - (1 + np.cos(th)) / (2 * th * np.sin(th))) * (W @ W)
    return np.concatenate([omega, Vinv @ t])


def se3_exp(xi):
    """6-vector (omega, upsilon) -> SE(3). Closed form."""
    omega, ups = xi[:3], xi[3:]
    th = float(np.linalg.norm(omega))
    T = np.eye(4)
    if th < 1e-8:
        T[:3, 3] = ups
        return T
    W = _skew(omega)
    Rm = np.eye(3) + np.sin(th) / th * W + (1 - np.cos(th)) / th**2 * (W @ W)
    V = np.eye(3) + (1 - np.cos(th)) / th**2 * W + (th - np.sin(th)) / th**3 * (W @ W)
    T[:3, :3] = Rm
    T[:3, 3] = V @ ups
    return T


def learn_screw(edges, log=print):
    """Robust average per-step motion S (frame k -> k+1) from the feature edges: take each
    edge's SE(3) log, scale to per-index, drop wild ones, median in Lie coords. The orbit is
    near-constant-velocity, so S^k bridges an index gap of k that has no matchable texture."""
    xis = []
    for (a, b), r in edges.items():
        try:
            xi = se3_log(r["T"].astype(float)) / (b - a)
        except Exception:                              # noqa: BLE001
            continue
        if np.degrees(np.linalg.norm(xi[:3])) <= MAX_STEP_DEG:
            xis.append(xi)
    if not xis:
        return None
    S = se3_exp(np.median(np.array(xis), axis=0))
    rot = np.degrees(np.linalg.norm(se3_log(S)[:3]))
    log(f"  learned screw: {rot:.1f} deg/shot, {np.linalg.norm(S[:3, 3])*100:.1f} cm/shot")
    return S


def screw_fill(poses, n, S, log=print):
    """Place every shot missing from `poses` at the screw-power of its nearest placed
    neighbour (M_i = M_j @ inv(S)^(i-j)). Returns the extended poses dict."""
    if S is None:
        return poses
    Sinv = np.linalg.inv(S)
    placed = sorted(poses)
    added = 0
    for i in range(n):
        if i in poses:
            continue
        j = min(placed, key=lambda p: abs(p - i))
        poses[i] = poses[j] @ np.linalg.matrix_power(Sinv, i - j)
        added += 1
    log(f"  screw-bridged {added} feature-less shots (coverage up, edges softer)")
    return poses


# ── [4] render + duplicate-aware voxel-MEAN fusion ──────────────────────────────────

def render_world(shots, poses, zmin, zmax, log=print):
    """Back-project each placed shot's full (cropped) depth and apply its pose. Uses the
    real colour image so the car keeps its colour. Returns stacked (pts, cols)."""
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
    if not allp:
        sys.exit("nothing to render (no placed shot had object depth).")
    log(f"  rendered {sum(len(p) for p in allp)} pts from {len(allp)} placed shots")
    return np.concatenate(allp), np.concatenate(allc)


def voxel_mean_fuse(pts, cols, voxel):
    """One point per voxel = the MEAN of every observation in that cell (position AND
    colour), plus the hit count. Averaging the overlapping/duplicate views is what turns
    redundant coverage into denoising instead of smear. Returns (pts, cols, hits)."""
    if len(pts) == 0 or voxel <= 0:
        return pts, cols, np.ones(len(pts), np.int32)
    keys = np.floor(pts / voxel).astype(np.int64)
    _, inv, hits = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    m = len(hits)
    sums = np.zeros((m, 3), np.float64)
    csum = np.zeros((m, 3), np.float64)
    np.add.at(sums, inv, pts)
    np.add.at(csum, inv, cols.astype(np.float64))
    mean_pts = (sums / hits[:, None]).astype(np.float32)
    mean_cols = (csum / hits[:, None]).round().astype(np.uint8)
    return mean_pts, mean_cols, hits.astype(np.int32)


# ── [5] denoise: statistical-outlier removal + largest Euclidean cluster ────────────

def statistical_outlier_mask(pts, k=SOR_K, std_ratio=SOR_STD):
    """Keep points whose mean distance to their k nearest neighbours is within
    mean + std_ratio*sigma of the global average (classic SOR flyer cull). scipy KDTree if
    available, else a blocked numpy fallback (fine for tens of thousands)."""
    n = len(pts)
    if n <= k + 1:
        return np.ones(n, bool)
    if _HAVE_KD:
        d, _ = cKDTree(pts).query(pts, k=k + 1)        # +1: self is the nearest
        mean_d = d[:, 1:].mean(axis=1)
    else:
        mean_d = np.empty(n)
        P = pts.astype(np.float32)
        for s in range(0, n, 2048):
            blk = P[s:s + 2048]
            d2 = ((blk[:, None, :] - P[None, :, :]) ** 2).sum(-1)
            part = np.partition(d2, k, axis=1)[:, 1:k + 1]
            mean_d[s:s + len(blk)] = np.sqrt(part).mean(axis=1)
    return mean_d <= mean_d.mean() + std_ratio * mean_d.std()


def largest_cluster_mask(pts, voxel):
    """Boolean mask of the biggest connected blob (26-neighbour flood fill over occupied
    voxels). Drops detached debris / flyer clusters so only the car body survives."""
    from collections import deque
    n = len(pts)
    if n == 0:
        return np.zeros(0, bool)
    keys = np.floor(pts / voxel).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    index = {tuple(k): i for i, k in enumerate(uniq)}
    nbr = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
           if dx or dy or dz]
    label = np.full(len(uniq), -1, np.int64)
    best_lbl, best_sz, cur = -1, 0, 0
    for start in range(len(uniq)):
        if label[start] != -1:
            continue
        q = deque([start]); label[start] = cur; sz = 0
        while q:
            x = q.popleft(); sz += 1
            kx = uniq[x]
            for dd in nbr:
                j = index.get((kx[0] + dd[0], kx[1] + dd[1], kx[2] + dd[2]))
                if j is not None and label[j] == -1:
                    label[j] = cur; q.append(j)
        if sz > best_sz:
            best_sz, best_lbl = sz, cur
        cur += 1
    return (label == best_lbl)[inv]


# ── [6] preview ─────────────────────────────────────────────────────────────────────

def _autolevel(cols):
    """Display-only brightness stretch: the silver car on a dark scene is dim (~mean 60),
    so percentile-stretch the colours just for the preview. The .ply keeps true colour."""
    c = cols.astype(np.float32)
    lo, hi = np.percentile(c, 2), np.percentile(c, 98)
    if hi - lo < 1e-3:
        return cols
    return np.clip((c - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)


def save_preview(path, pts, cols, px=600):
    """Three orthographic panels (top / front / side) so the cleaned car reads as 3D
    without a viewer. cv2 only (no matplotlib) -> Pi-safe. Points are drawn as small blocks
    and the colours auto-levelled so the dim silver body is visible in a thumbnail."""
    if len(pts) == 0:
        return
    disp = _autolevel(cols)[:, ::-1]                    # RGB -> BGR, brightened

    def panel(ax0, ax1, flip1, title):
        a, b = pts[:, ax0].copy(), pts[:, ax1].copy()
        if flip1:
            b = -b
        span = max(a.max() - a.min(), b.max() - b.min(), 1e-3)
        ia = ((a - a.min()) / span * (px - 40) + 20).astype(np.int32)
        ib = px - 1 - ((b - b.min()) / span * (px - 40) + 20).astype(np.int32)
        img = np.full((px, px, 3), 25, np.uint8)
        for dx in (0, 1):                               # 2x2 block per point => denser look
            for dy in (0, 1):
                xa, yb = ia + dx, ib + dy
                ok = (xa >= 0) & (xa < px) & (yb >= 0) & (yb < px)
                img[yb[ok], xa[ok]] = disp[ok]
        cv2.putText(img, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return img

    out = np.hstack([panel(0, 2, False, "TOP (X-Z)"),
                     panel(0, 1, True, "FRONT (X-Y)"),
                     panel(2, 1, True, "SIDE (Z-Y)")])
    cv2.imwrite(path, out)
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


def process(session, cfg, force_sift=False, window=FEAT_WINDOW, voxel=FUSE_VOXEL,
            full=False, crop_stand=True, log=print):
    """The whole pipeline on one session. Returns the cleaned .ply path."""
    zmin, zmax = cfg["zmin"], cfg["zmax"]
    # match build_object_pi's object-tuned registration knobs
    R.ZMIN, R.ZMAX = zmin, zmax
    R.KP_MAX_STD, R.RANSAC_THRESH = PI.KP_MAX_STD, PI.RANSAC_THRESH
    R.MAX_RMSE, R.MIN_INLIERS = PI.MAX_RMSE, PI.MIN_INLIERS

    log(f"process_pi: {cfg['name']}")
    log(f"  session {session}: band [{zmin}, {zmax}] m, fuse-voxel {voxel*1000:.1f} mm, "
        f"win {window}, {'SIFT' if force_sift else 'ORB->SIFT'}, "
        f"{'FULL (screw-bridged)' if full else 'clean (feature only)'}, "
        f"NN {'kdtree' if _HAVE_KD else 'numpy'}")

    log("[1/6] load + segment (+ stalk crop)")
    shots = load_session(session, cfg, crop_stand, log=log)
    n = len(shots)

    log("[2/6] register: proven feature poses")
    poses, edges = feature_poses(shots, window, force_sift, log=log)
    log(f"  placed {len(poses)}/{n} shots by features")

    if full:
        log("[3/6] connect the gaps: screw-bridge the feature-less shots")
        poses = screw_fill(poses, n, learn_screw(edges, log=log), log=log)
    else:
        log(f"[3/6] gap-bridge skipped (clean mode); {n - len(poses)} feature-less shots "
            f"left out -- add --full for fuller, softer coverage")

    log("[4/6] render + duplicate-aware voxel-mean fusion")
    pts, cols = render_world(shots, poses, zmin, zmax, log=log)
    raw = len(pts)
    pts, cols, hits = voxel_mean_fuse(pts, cols, voxel)
    log(f"  {raw} pts -> {len(pts)} voxels (mean hits {hits.mean():.1f}, max {hits.max()})")
    if MIN_VOX_HITS > 1:
        keep = hits >= MIN_VOX_HITS
        if keep.sum() > 50:
            pts, cols = pts[keep], cols[keep]
            log(f"  kept {int(keep.sum())} voxels seen by >= {MIN_VOX_HITS} views")

    log("[5/6] denoise: statistical-outlier removal + largest cluster")
    m = statistical_outlier_mask(pts)
    pts, cols = pts[m], cols[m]
    log(f"  SOR kept {int(m.sum())}/{len(m)}")
    if len(pts) > 50:
        m = largest_cluster_mask(pts, max(voxel * 2, 0.005))
        if m.sum() > 50:
            pts, cols = pts[m], cols[m]
            log(f"  largest cluster kept {int(m.sum())} (debris dropped)")

    bb = (pts.max(0) - pts.min(0)) * 100 if len(pts) else np.zeros(3)
    log(f"  final cloud: {len(pts)} pts, bbox {bb[0]:.0f}x{bb[1]:.0f}x{bb[2]:.0f} cm")

    log("[6/6] save")
    ply = os.path.join(session, "object_clean.ply")
    G.save_ply(ply, pts, cols)
    try:
        save_preview(os.path.join(session, "object_clean_preview.png"), pts, cols)
    except Exception as e:                              # noqa: BLE001
        log(f"  (preview skipped: {e})")
    return ply


def main():
    args = sys.argv[1:]
    force_sift = "--sift" in args
    if force_sift:
        args.remove("--sift")
    full = "--full" in args
    if full:
        args.remove("--full")
    crop_stand = "--keep-stand" not in args
    if not crop_stand:
        args.remove("--keep-stand")
    obj = _pop(args, "--object", str)
    voxel = _pop(args, "--voxel", float) or FUSE_VOXEL
    window = _pop(args, "--win", int) or FEAT_WINDOW

    cfg = config.select(obj) if obj else config.DEFAULT
    session = args[0] if args else newest_session()
    if not session or not os.path.isdir(session):
        sys.exit(f"no session given and none found under {CAPTURES}")

    ply = process(session, cfg, force_sift=force_sift, window=window, voxel=voxel,
                  full=full, crop_stand=crop_stand)
    print(f"\nDONE -> {ply}")
    print(f"  view on the Pi:  python3 {os.path.join('..', '..', 'src', 'pointcloud', 'view3d.py')} {ply}")


if __name__ == "__main__":
    main()
