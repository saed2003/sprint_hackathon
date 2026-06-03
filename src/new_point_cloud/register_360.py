"""
register_360.py -- build a 360 point cloud from a 10-shot spin, by FEATURE MATCHING.

Why this exists (the old combine_360.py failed):
    The robot spins ~400 deg in 10 stops with NO IMU/encoder, and the camera sits
    off the spin axis (so each step is rotation + a little translation = an arc).
        - "trust 36 deg/step" smears: the real step is ~40 deg and it drifts.
        - ICP locks onto noisy passive-stereo geometry and returns garbage.
    Both guess the pose. This file MEASURES it.

The method (3D-3D feature registration + pose-graph loop closure):
    1. Each shot already has a hardware depth map (depth.npy). Detect ORB features
       in its left IR image and lift each one to a 3D point via that depth.
    2. For EVERY pair of shots, match features, then solve the rigid transform that
       takes one shot's matched 3D points onto the other's (Kabsch inside RANSAC).
       This needs no angle guess -- the geometry is measured from correspondences.
       Weak ORB pairs are retried with SIFT (robust to the big ~40 deg viewpoint jump).
    3. The surviving pairwise transforms form a pose graph: consecutive steps are
       "odometry" (trusted), every other accepted pair is a "loop closure". Open3D's
       global optimization spreads the accumulated drift evenly around the ring
       (the ~400 deg over-rotation means shot 9 also overlaps shots 0 AND 1 -> extra
       loop constraints, which we get for free from the exhaustive matching).
    4. Re-project every shot's full depth into the one optimized world frame, stack,
       voxel-downsample, cull flyers, save the .ply + a preview PNG.

Run (laptop or Pi -- needs cv2, numpy, open3d):
    python register_360.py                          # newest scan under ../captures
    python register_360.py <session_dir>
    python register_360.py <session_dir> --sift     # SIFT for every pair (slower, robust)
    python register_360.py <session_dir> --byshot   # tint each shot -> see seams
    python register_360.py <session_dir> --gray     # raw IR texture (default = depth color)
    python register_360.py <session_dir> --zmax 3.0 # tighten the far cutoff

Default color: RED = close, BLUE = far (by distance from camera).
"""

import os
import sys
import glob
from collections import deque

import cv2
import numpy as np

# allow running from anywhere: this folder is on the path for `import geometry`
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import geometry as G  # noqa: E402


# ── tunables ──────────────────────────────────────────────────────────────────────

N_ORB = 3000             # ORB features per image
CLAHE_CLIP = 3.0         # contrast boost before detection. The IR is dim/low-contrast
                         # (mean ~55/255) so ORB's corner threshold found almost nothing
                         # on some shots (~30 keypoints). CLAHE reveals the real texture
                         # (~30x more keypoints) -- this is what makes every shot register.
RATIO = 0.75             # Lowe ratio test
KP_MAX_STD = 0.20        # reject a keypoint if its 5x5 depth window std > this (m).
                         # 0.05 was far too tight -- it killed exactly the textured
                         # edge corners we match on (both3D collapsed 50 -> 3).
FUND_PX = 2.0            # 2D fundamental-matrix RANSAC reprojection px: a geometric
                         # pre-filter that throws ORB's false matches BEFORE we lift
                         # to 3D. Big robustness win across the ~40 deg viewpoint jump.

ZMIN, ZMAX = 0.15, 2.0   # keep depth in this band (m). 0 = invalid; the D405 is short
                         # range so beyond ~2 m is mostly noise (raise --zmax for big
                         # rooms, lower it to ~1.5 for a cleaner cloud).

MIN_MATCHES = 8          # need this many putative matches to even attempt a pair
RANSAC_THRESH = 0.06     # 6 cm inlier distance for the 3D-3D fit. Passive-stereo depth
                         # noise is ~5-12 cm at 1-2 m, so 3 cm rejected good inliers.
MIN_INLIERS = 6          # accept an edge only with at least this many 3D inliers.
                         # 6 is safe because the 2D fundamental pre-filter has already
                         # removed geometric outliers; a rigid fit needs only 3.
MAX_RMSE = 0.05          # ...and inlier RMSE at most this (m)

INFO_CORR = 0.06         # correspondence dist for the pose-graph information matrix
OPT_MAX_CORR = 0.06      # global-optimization max correspondence distance
EDGE_PRUNE = 0.25        # global-optimization edge prune threshold

OUT_VOXEL = 0.005        # 5 mm final voxel (dedup the heavy view overlap)
CULL_NN, CULL_STD = 20, 2.0   # statistical outlier removal (flyer cull)

# distinct colors for --byshot debug tinting (RGB)
PALETTE = np.array([
    [230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48],
    [145, 30, 180], [70, 240, 240], [240, 50, 230], [210, 245, 60], [250, 190, 190],
], np.uint8)


# ── session / shots ───────────────────────────────────────────────────────────────

def newest_session():
    """Newest captures/scan_* folder (so a bare run picks the last scan)."""
    caps = os.path.join(HERE, "..", "captures")
    scans = sorted(glob.glob(os.path.join(caps, "scan_*")))
    return scans[-1] if scans else os.path.join(caps, "scan_unknown")


def shot_dirs(session_dir):
    """All shot_* folders holding an IR pair, in order."""
    return sorted(os.path.dirname(p)
                  for p in glob.glob(os.path.join(session_dir, "shot_*", "ir_left.png")))


def load_shot(d):
    """A capture folder -> the per-shot dict the pipeline carries around."""
    intr = G.load_intrinsics(os.path.join(d, "intrinsics.txt"))
    return {
        "dir": d,
        "name": os.path.basename(d),
        "intr": intr,
        "gray": G.load_ir_gray(d),
        "depth_m": G.load_depth_m(d, intr),
        # feature caches filled lazily by get_feat(): "<kind>_feat"
    }


# ── features ──────────────────────────────────────────────────────────────────────

_CLAHE = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=(8, 8))


def _detect(gray, kind):
    """Detect + describe on the CONTRAST-ENHANCED IR. Returns (xy Nx2, desc, norm).

    CLAHE first: the raw IR is dim, so detecting on it misses most of the texture.
    We enhance only for detection -- the original gray still colors the cloud.
    """
    g = _CLAHE.apply(gray)
    if kind == "orb":
        det = cv2.ORB_create(nfeatures=N_ORB)
        norm = cv2.NORM_HAMMING
    else:
        det = cv2.SIFT_create()
        norm = cv2.NORM_L2
    kps, desc = det.detectAndCompute(g, None)
    xy = np.array([k.pt for k in kps], np.float32).reshape(-1, 2)
    return xy, desc, norm


def _lift_all(xy, depth_m, intr):
    """Lift every keypoint to 3D; full Nx3 array (NaN where invalid) + valid mask."""
    pts, mask = G.lift_keypoints(xy, depth_m, intr, win=5,
                                 max_std=KP_MAX_STD, zmin=ZMIN, zmax=ZMAX)
    full = np.full((len(xy), 3), np.nan, np.float32)
    full[mask] = pts
    return full, mask


def get_feat(shot, kind):
    """Cached (xy, desc, p3d Nx3, valid mask, norm) for a shot and detector kind."""
    key = f"{kind}_feat"
    if key not in shot:
        xy, desc, norm = _detect(shot["gray"], kind)
        if len(xy) == 0:
            shot[key] = (xy, desc, np.zeros((0, 3), np.float32), np.zeros(0, bool), norm)
        else:
            p3d, valid = _lift_all(xy, shot["depth_m"], shot["intr"])
            shot[key] = (xy, desc, p3d, valid, norm)
    return shot[key]


def match(descA, descB, norm):
    """Mutual + Lowe-ratio match. Returns list of (iA, iB) index pairs."""
    if descA is None or descB is None or len(descA) < 2 or len(descB) < 2:
        return []
    bf = cv2.BFMatcher(norm)
    knn_ab = bf.knnMatch(descA, descB, k=2)
    knn_ba = bf.knnMatch(descB, descA, k=2)

    def ratio_good(knn):
        out = {}
        for m in knn:
            if len(m) == 2 and m[0].distance < RATIO * m[1].distance:
                out[m[0].queryIdx] = m[0].trainIdx
        return out

    ab = ratio_good(knn_ab)
    ba = ratio_good(knn_ba)
    return [(i, j) for i, j in ab.items() if ba.get(j, -1) == i]   # mutual best


# ── pairwise registration (the core: matched 3D -> rigid transform) ───────────────

def register_pair(shotA, shotB, kind):
    """Match shot A to shot B and solve A's matched points onto B's.

    Returns dict(T = 4x4 A->B, n = inliers, rmse) or None if the pair won't lock.
    """
    xyA, dA, p3dA, vA, norm = get_feat(shotA, kind)
    xyB, dB, p3dB, vB, _ = get_feat(shotB, kind)

    pairs = match(dA, dB, norm)
    if len(pairs) < MIN_MATCHES:
        return None
    iA = np.array([p[0] for p in pairs])
    iB = np.array([p[1] for p in pairs])

    # 2D geometric pre-filter: drop matches that can't fit one rigid scene view
    # (epipolar geometry). Removes ORB's false pairings before they poison the 3D fit.
    F, fmask = cv2.findFundamentalMat(xyA[iA], xyB[iB], cv2.FM_RANSAC, FUND_PX, 0.99)
    if fmask is None:
        return None
    fmask = fmask.ravel().astype(bool)
    iA, iB = iA[fmask], iB[fmask]

    keep = vA[iA] & vB[iB]                       # both keypoints had good depth
    if keep.sum() < MIN_INLIERS:
        return None

    P = p3dA[iA[keep]]                            # source (A) 3D points
    Q = p3dB[iB[keep]]                            # target (B) 3D points
    T, inl = G.kabsch_ransac(P, Q, thresh=RANSAC_THRESH)
    if T is None or len(inl) < MIN_INLIERS:
        return None

    proj = P[inl] @ T[:3, :3].T + T[:3, 3]        # A inliers mapped into B frame
    rmse = float(np.sqrt(np.mean(np.sum((proj - Q[inl]) ** 2, axis=1))))
    if rmse > MAX_RMSE:
        return None
    return {"T": T, "n": int(len(inl)), "rmse": rmse}


def register_pair_robust(shotA, shotB, force_sift):
    """ORB first, SIFT fallback (or SIFT-only if forced). Returns (result, detector)."""
    if not force_sift:
        r = register_pair(shotA, shotB, "orb")
        if r is not None:
            return r, "orb"
    r = register_pair(shotA, shotB, "sift")
    return (r, "sift") if r is not None else (None, None)


# ── pose graph (chain + loop closure -> one optimized frame) ──────────────────────

def _adjacency(edges):
    """node -> list of (other, T_self2other), both directions, from accepted edges."""
    adj = {}
    for (a, b), r in edges.items():
        T = r["T"]
        adj.setdefault(a, []).append((b, T))                       # a -> b
        adj.setdefault(b, []).append((a, np.linalg.inv(T)))        # b -> a
    return adj


def components(n, edges):
    """Connected components of the registration graph, as a list of node-id sets,
    largest first. Isolated shots (no accepted edge) come back as singletons."""
    adj = _adjacency(edges)
    seen, comps = set(), []
    for start in range(n):
        if start in seen:
            continue
        comp, q = set(), deque([start])
        seen.add(start)
        while q:
            x = q.popleft(); comp.add(x)
            for y, _ in adj.get(x, []):
                if y not in seen:
                    seen.add(y); q.append(y)
        comps.append(comp)
    return sorted(comps, key=len, reverse=True)


def bfs_init_poses(root, edges):
    """Seed each node's local->world pose by walking accepted edges from `root`.

    edges: dict (a,b)->result with result['T'] = T_a2b (a-local -> b-local), a<b.
    Returns dict node_id -> 4x4 (local->world, world == root's frame) for every node
    reachable from `root`. The world frame is anchored at `root`.
    """
    adj = _adjacency(edges)
    pose = {root: np.eye(4)}
    q = deque([root])
    while q:
        x = q.popleft()
        for y, T_x2y in adj.get(x, []):
            if y in pose:
                continue
            # p_world = pose[x] @ p_x ; p_x = inv(T_x2y) @ p_y  ->  pose[y] = pose[x] @ inv(T_x2y)
            pose[y] = pose[x] @ np.linalg.inv(T_x2y)
            q.append(y)
    return pose


def shot_down(shot, voxel=0.03):
    """Downsampled Open3D cloud of a shot (for the edge information matrices)."""
    import open3d as o3d
    depth = G.drop_depth_edges(shot["depth_m"])
    pts, _ = G.back_project_dense(depth, shot["gray"], shot["intr"], ZMIN, ZMAX)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    return pc.voxel_down_sample(voxel)


def optimize_poses(n, edges, root, log=print):
    """Build + globally optimize the pose graph. Returns {orig_id: 4x4 local->world}.

    Operates on the component reachable from `root` (the largest connected component;
    a broken capture can leave shots unplaceable). Consecutive ring neighbors are
    trusted odometry edges; all other accepted pairs are uncertain loop closures.
    """
    import open3d as o3d
    reg = o3d.pipelines.registration

    init = bfs_init_poses(root, edges)
    visited = sorted(init.keys())
    if len(visited) < n:
        missing = sorted(set(range(n)) - set(visited))
        log(f"  WARN: {len(missing)} shots not in this component, dropped: {missing}")

    old2new = {old: i for i, old in enumerate(visited)}
    downs = {old: shot_down_cache[old] for old in visited}   # filled by caller

    pg = reg.PoseGraph()
    for old in visited:
        pg.nodes.append(reg.PoseGraphNode(init[old].copy()))

    def is_consecutive(a, b):
        return abs(a - b) == 1 or {a, b} == {0, n - 1}        # ring neighbors

    for (a, b), r in edges.items():
        if a not in old2new or b not in old2new:
            continue
        try:
            info = reg.get_information_matrix_from_point_clouds(
                downs[a], downs[b], INFO_CORR, r["T"])
        except Exception:                                     # noqa: BLE001
            info = np.eye(6) * r["n"]
        pg.edges.append(reg.PoseGraphEdge(
            old2new[a], old2new[b], r["T"], info,
            uncertain=not is_consecutive(a, b)))

    option = reg.GlobalOptimizationOption(
        max_correspondence_distance=OPT_MAX_CORR,
        edge_prune_threshold=EDGE_PRUNE,
        reference_node=old2new[root])
    log("  optimizing pose graph ...")
    reg.global_optimization(
        pg, reg.GlobalOptimizationLevenbergMarquardt(),
        reg.GlobalOptimizationConvergenceCriteria(), option)

    return {old: np.asarray(pg.nodes[new].pose) for old, new in old2new.items()}


# caller-populated cache so optimize_poses can read per-shot downsampled clouds
shot_down_cache = {}


# ── render (apply optimized poses, stack, clean, save) ────────────────────────────

def shot_world_cloud(shot, T_local2world, mode):
    """Re-project a shot's full depth into the world frame. Returns (pts, cols).

    mode "depth" (default): color by distance -- RED = close, BLUE = far.
    mode "gray": raw IR texture.  mode "byshot": per-shot tint (applied by caller).
    """
    depth = G.drop_depth_edges(shot["depth_m"])
    if mode == "depth":
        norm = np.clip((depth - ZMIN) / max(ZMAX - ZMIN, 1e-6), 0, 1)   # 0 near .. 1 far
        idx = ((1.0 - norm) * 255).astype(np.uint8)     # invert so near -> 255 -> red
        color = cv2.applyColorMap(idx, cv2.COLORMAP_JET)  # JET: 255=red(near) 0=blue(far)
        color[depth <= 0] = 0
    else:                                               # "gray" and "byshot" use IR
        color = shot["gray"]
    pts, cols = G.back_project_dense(depth, color, shot["intr"], ZMIN, ZMAX)
    pts = pts @ T_local2world[:3, :3].T + T_local2world[:3, 3]
    return pts.astype(np.float32), cols


def finalize(pts, cols, out_dir, voxel, log=print):
    """Voxel-dedup overlap, cull flyers, stand upright, save .ply + preview."""
    import open3d as o3d
    pts = G.flip_upright(pts)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pc.colors = o3d.utility.Vector3dVector(cols.astype(np.float64) / 255.0)
    before = len(pc.points)
    if voxel > 0:
        pc = pc.voxel_down_sample(voxel)
    pc, _ = pc.remove_statistical_outlier(nb_neighbors=CULL_NN, std_ratio=CULL_STD)
    p = np.asarray(pc.points, np.float32)
    c = (np.asarray(pc.colors) * 255.0).astype(np.uint8)
    log(f"  {before} -> {len(p)} points after voxel/cull")

    ply = os.path.join(out_dir, "pointcloud_360.ply")
    png = os.path.join(out_dir, "pointcloud_360_preview.png")
    G.save_ply(ply, p, c)
    G.save_preview(png, p, c)
    return ply


# ── main ──────────────────────────────────────────────────────────────────────────

def _pop_val(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        v = cast(args[i + 1])
        del args[i:i + 2]
        return v
    return None


def main():
    global ZMIN, ZMAX
    args = sys.argv[1:]
    force_sift = "--sift" in args and not args.remove("--sift")
    byshot = "--byshot" in args and not args.remove("--byshot")
    gray = "--gray" in args and not args.remove("--gray")
    zmin = _pop_val(args, "--zmin", float)
    zmax = _pop_val(args, "--zmax", float)
    voxel = _pop_val(args, "--voxel", float)
    if zmin is not None:
        ZMIN = zmin
    if zmax is not None:
        ZMAX = zmax
    voxel = OUT_VOXEL if voxel is None else voxel
    mode = "byshot" if byshot else ("gray" if gray else "depth")   # default: red=near

    session = args[0] if args else newest_session()
    dirs = shot_dirs(session)
    if not dirs:
        sys.exit(f"No shots (shot_*/ir_left.png) found in {session}")
    n = len(dirs)
    print(f"Session {session}: {n} shots, depth band [{ZMIN}, {ZMAX}] m, "
          f"detector {'SIFT' if force_sift else 'ORB->SIFT'}, color={mode}")

    shots = [load_shot(d) for d in dirs]

    # --- exhaustive pairwise registration ---
    print("Registering all pairs:")
    edges = {}
    for a in range(n):
        for b in range(a + 1, n):
            r, det = register_pair_robust(shots[a], shots[b], force_sift)
            if r is not None:
                edges[(a, b)] = r
                tag = "odom" if (abs(a - b) == 1 or {a, b} == {0, n - 1}) else "loop"
                print(f"  {a:2d}-{b:2d} [{tag}] {det}: {r['n']:3d} inliers, "
                      f"rmse {r['rmse']*1000:4.0f} mm")

    if not edges:
        sys.exit("No pair registered -- check depth band / images.")

    # diagnostics: ring coverage + connected components of the registration graph
    ring = [(i, i + 1) for i in range(n - 1)] + [(0, n - 1)]
    present = sum(e in edges for e in ring)
    comps = components(n, edges)
    print(f"  {len(edges)} edges accepted; {present}/{len(ring)} ring edges present")
    print(f"  components: {[sorted(c) for c in comps]}")
    main_comp = comps[0]
    if len(main_comp) < 2:
        sys.exit("No two shots share enough texture+depth to register -- recapture "
                 "(see README: keep ~1-3 m from walls, ensure texture, full overlap).")
    if len(main_comp) < n:
        rest = sorted(set(range(n)) - main_comp)
        print(f"  -> placing the largest component ({len(main_comp)}/{n} shots); "
              f"shots {rest} can't be placed (no shared depth-bearing texture).")
    root = min(main_comp)

    # --- build per-shot downsampled clouds, then optimize the pose graph ---
    shot_down_cache.clear()
    for i in main_comp:
        shot_down_cache[i] = shot_down(shots[i])
    poses = optimize_poses(n, edges, root)

    # --- render every placed shot into the shared world frame ---
    print("Rendering:")
    all_pts, all_cols = [], []
    for i, T in sorted(poses.items()):
        pts, cols = shot_world_cloud(shots[i], T, mode)
        if mode == "byshot":
            cols = np.tile(PALETTE[i % len(PALETTE)], (len(pts), 1))
        all_pts.append(pts)
        all_cols.append(cols)
        print(f"  {shots[i]['name']}: {len(pts):6d} pts")
    pts = np.concatenate(all_pts)
    cols = np.concatenate(all_cols)

    finalize(pts, cols, HERE, voxel)
    print("Done. Open src/new_point_cloud/pointcloud_360.ply in MeshLab / CloudCompare.")


if __name__ == "__main__":
    main()
