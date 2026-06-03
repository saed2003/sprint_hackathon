"""
Merge many object views into ONE clean point cloud  (the heart of the object scan).

Input  : a session folder of shot_NN capture folders (turntable, hand-stepped, or the
         robot orbit — they all write the standard folder).
Output : <session>/merged_object.ply  (+ optionally an object_mesh.ply via mesh.py)

WHY THIS VERSION IS ROBUST (and the old one shattered)
------------------------------------------------------
The old merge built a FRAGILE ring: only neighbour edges i->i+1 plus ONE loop edge,
each a blind point-to-plane ICP started from the angle prior and TRUSTED no matter how
bad. A single wrong edge (a low-overlap / drifted view) broke the chain and the model
exploded into scattered fragments (e.g. the 48-shot take orbit_20260603_150159).

This version measures + verifies instead of guessing, reusing the repo's PROVEN
feature-based machinery (the same idea that makes new_point_cloud/register_360.py the
working room-360 — see CLAUDE.md "why register_360 wins"):

  1. SEGMENT each view to the object only                       (segment.py)
  2. Per view, MASK the IR+depth to the object so features describe the OBJECT, not the
     floor/camera motion                                        (build_object_pi.load_shot_masked)
  3. WINDOWED edges: register each view to its next W neighbours (wrapping) -> a redundant
     ring, not a single fragile chain.
       - coarse pose MEASURED from features (CLAHE->ORB/SIFT->Kabsch+RANSAC, no angle
         guess); falls back to the angle prior only when features are too weak
       - REFINE with Open3D point-to-plane ICP
       - GATE: keep an edge only if ICP fitness >= FIT_MIN and rmse is small (reject garbage)
  4. Keep the LARGEST CONNECTED COMPONENT (drop unplaceable views — a clean partial beats a
     shattered whole), init poses by BFS, then Open3D global pose-graph optimization (the line
     process on uncertain loop edges down-weights any remaining bad closure).
  5. Fuse the placed views -> voxel -> outlier cull -> merged_object.ply.

Laptop-only (Open3D). Run from this folder:
    python build_object.py <session_dir>
    python build_object.py <session_dir> --object db5            # use a config preset
    python build_object.py <session_dir> --no-loop               # partial scan (not full 360)
    python build_object.py <session_dir> --win 4 --fit-min 0.25  # more redundancy / looser gate
    python build_object.py <session_dir> --sift                  # SIFT every pair (slow, robust)
    python build_object.py <session_dir> --mesh                  # also build a surface mesh
"""
import os
import sys
import glob

import numpy as np
import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import segment as seg

# Reuse the repo's PROVEN robust registration machinery. None of these pull smbus/RasBot,
# and register_360 imports Open3D only lazily inside functions we don't call, so importing
# is laptop-safe:
#   register_360 : CLAHE->ORB/SIFT->Kabsch+RANSAC pair pose, components, bfs_init_poses
#   geometry     : back-project / kabsch / ply IO
#   build_object_pi : per-view object MASKING + object-scale registration constants
import build_object_pi as PI
R = PI.R          # register_360 module
G = PI.G          # geometry module

reg = o3d.pipelines.registration

WINDOW = 3        # register each view to its next W neighbours (wrapping) -> redundant ring
FIT_MIN = 0.30    # ICP fitness floor to ACCEPT an edge (rejects fits locked onto noise)


# ── geometry: the object-centred angle prior (now only a FALLBACK init) ───────────

def ry_about(center, angle_deg):
    """4x4 transform: rotate `angle_deg` about the vertical (Y) axis THROUGH `center`.

    p' = T(center) @ Ry @ T(-center) @ p   — rotate the world around the object, not the
    camera. Used only to seed an edge when feature matching fails.
    """
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    Rm = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = center - Rm @ center
    return T


def read_angle(folder):
    """Recorded turn angle (deg) for a shot from angle.txt, or None."""
    p = os.path.join(folder, "angle.txt")
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            return float(f.read().split()[0])
    except (ValueError, IndexError):
        return None


def shot_dirs(session_dir):
    """All shot_* folders in a session that actually hold a capture."""
    return sorted(os.path.dirname(p)
                  for p in glob.glob(os.path.join(session_dir, "shot_*", "depth.npy")))


def estimate_rotation_axis(pcds, angles=None, log=print, iters=3):
    """Find the (x,z) of the vertical rotation axis through the OBJECT CENTRE.

    The object stays in ~the same place in the camera frame across views (turntable: the
    camera is fixed and the object spins; orbit: the camera re-aims at it), so the axis is
    simply the MEDIAN of each view's centroid. Robust and cheap. (An earlier bootstrap that
    pre-merged with the angle prior could run the axis away — that produced 80 cm blow-ups.)
    """
    cents = np.array([np.median(np.asarray(p.points), axis=0) for p in pcds])
    center = np.median(cents, axis=0)
    log(f"  rotation axis (x,z) = ({center[0]:.3f}, {center[2]:.3f}) m")
    return center


# ── pairwise: feature coarse pose -> ICP refine -> gate ───────────────────────────

def _icp_refine(source, target, init, coarse, fine):
    """Point-to-plane ICP source->target, coarse then fine, starting from `init`.
    Returns the fine RegistrationResult (.transformation, .fitness, .inlier_rmse)."""
    c = reg.registration_icp(source, target, coarse, init,
                             reg.TransformationEstimationPointToPlane())
    return reg.registration_icp(source, target, fine, c.transformation,
                                reg.TransformationEstimationPointToPlane())


def _relative_prior(center, angles, i, j):
    """Relative transform local_i -> local_j implied by the angle prior (fallback only).
    A_k = ry_about(center, angles[k]) maps local_k into the common frame, so
    local_i -> local_j is inv(A_j) @ A_i."""
    return np.linalg.inv(ry_about(center, angles[j])) @ ry_about(center, angles[i])


def build_edges(seg_pcds, shots, center, angles, voxel, window, fit_min,
                loop=True, force_sift=False, log=print):
    """Windowed, gated edges. For each view i and dj in 1..window, register i to (i+dj)%n:
    coarse pose from features (register_pair_robust), else the angle prior; refine with ICP;
    ACCEPT only fits with fitness >= fit_min and a small rmse. Adjacent (and the 0<->n-1 wrap)
    are odometry edges; the rest are uncertain loop closures.

    Returns {(a, b): {T (a->b), info, fit, rmse, uncertain}} with a < b.
    """
    n = len(seg_pcds)
    coarse, fine = voxel * 15, voxel * 1.5
    pairs = sorted({(min(i, (i + dj) % n), max(i, (i + dj) % n))
                    for i in range(n) for dj in range(1, window + 1) if (i + dj) % n != i})
    edges = {}
    n_feat = 0
    for a, b in pairs:
        if not loop and (b - a) > window:                 # a wrap pair across the 360 seam
            continue
        res, det = R.register_pair_robust(shots[a], shots[b], force_sift)
        if res is not None:
            init, src = res["T"], "feat-" + det
            n_feat += 1
        else:
            init, src = _relative_prior(center, angles, a, b), "prior"
        icp = _icp_refine(seg_pcds[a], seg_pcds[b], init, coarse, fine)
        if icp.fitness < fit_min or not (0.0 < icp.inlier_rmse <= fine):
            log(f"  {a:2d}-{b:2d} [{src}] reject  (fit {icp.fitness:.2f}, "
                f"rmse {icp.inlier_rmse * 1000:.0f}mm)")
            continue
        try:
            info = reg.get_information_matrix_from_point_clouds(
                seg_pcds[a], seg_pcds[b], fine, icp.transformation)
        except Exception:                                  # noqa: BLE001
            info = np.eye(6)
        uncertain = not (abs(a - b) == 1 or {a, b} == {0, n - 1})
        edges[(a, b)] = {"T": icp.transformation, "info": info,
                         "fit": icp.fitness, "rmse": icp.inlier_rmse, "uncertain": uncertain}
        log(f"  {a:2d}-{b:2d} [{src}/{'loop' if uncertain else 'odom'}] "
            f"fit {icp.fitness:.2f}, rmse {icp.inlier_rmse * 1000:.0f}mm")
    log(f"  {len(edges)} edges accepted ({n_feat} feature-measured)")
    return edges


# ── robust global solve: largest component -> BFS init -> global optimization ──────

def solve_and_fuse(seg_pcds, edges, voxel, log=print):
    """Keep the largest connected component, init poses by BFS over the accepted edges,
    globally optimize the Open3D pose graph, then fuse the placed views into one cloud."""
    n = len(seg_pcds)
    fine = voxel * 1.5
    comps = R.components(n, edges)
    main = comps[0]
    log(f"  components: {[sorted(c) for c in comps]}")
    if len(main) < 2:
        sys.exit("registration failed: no two views share enough object texture + overlap. "
                 "Recapture with more overlap / better light (see README).")
    if len(main) < n:
        log(f"  placing largest component ({len(main)}/{n} views); dropped: "
            f"{sorted(set(range(n)) - main)}")
    root = min(main)

    init = R.bfs_init_poses(root, {k: {"T": v["T"]} for k, v in edges.items()})
    placed = sorted(init.keys())
    old2new = {old: i for i, old in enumerate(placed)}

    pg = reg.PoseGraph()
    for old in placed:
        pg.nodes.append(reg.PoseGraphNode(init[old].copy()))
    for (a, b), e in edges.items():
        if a in old2new and b in old2new:
            pg.edges.append(reg.PoseGraphEdge(
                old2new[a], old2new[b], e["T"], e["info"], uncertain=e["uncertain"]))

    option = reg.GlobalOptimizationOption(
        max_correspondence_distance=fine, edge_prune_threshold=0.25,
        reference_node=old2new[root])
    log("  optimizing pose graph ...")
    reg.global_optimization(
        pg, reg.GlobalOptimizationLevenbergMarquardt(),
        reg.GlobalOptimizationConvergenceCriteria(), option)

    merged = o3d.geometry.PointCloud()
    for old in placed:
        q = o3d.geometry.PointCloud(seg_pcds[old])
        q.transform(pg.nodes[old2new[old]].pose)
        merged += q
    return merged


# ── top-level build ──────────────────────────────────────────────────────────────

def build(session_dir, voxel=0.003, zmin=seg.ZMIN_DEFAULT, zmax=seg.ZMAX_DEFAULT,
          remove_plane=True, crop=None, loop=True, direction=1, window=WINDOW,
          fit_min=FIT_MIN, force_sift=False, log=print):
    """Segment, measure+verify windowed edges, robustly optimize, fuse -> merged_object.ply.

    direction: sign of the turn between shots, used only for the prior FALLBACK init.
    window   : register each view to its next `window` neighbours (redundant ring).
    fit_min  : ICP fitness floor to accept an edge.
    """
    dirs = shot_dirs(session_dir)
    if len(dirs) < 2:
        sys.exit(f"need >=2 shots in {session_dir} (found {len(dirs)})")
    log(f"object build (robust feature+ICP): {len(dirs)} views from {session_dir}")

    # object-scale registration knobs into the reused register_360 module
    R.ZMIN, R.ZMAX = zmin, zmax
    R.KP_MAX_STD, R.RANSAC_THRESH = PI.KP_MAX_STD, PI.RANSAC_THRESH
    R.MAX_RMSE, R.MIN_INLIERS = PI.MAX_RMSE, PI.MIN_INLIERS
    mask_crop = crop if crop is not None else 0.15

    # 1. per view: dense segmented cloud (what we fuse) + masked shot (for features)
    seg_pcds, shots, kept = [], [], []
    for d in dirs:
        pcd = seg.segment_object(d, zmin=zmin, zmax=zmax, remove_plane=remove_plane,
                                 crop=crop, voxel=voxel)
        if len(pcd.points) < 50:
            log(f"  skip {os.path.basename(d)} (only {len(pcd.points)} pts)")
            continue
        s = PI.load_shot_masked(d, zmin, zmax, mask_crop)
        if s is None:
            log(f"  skip {os.path.basename(d)} (no object for feature masking)")
            continue
        seg_pcds.append(pcd)
        shots.append(s)
        kept.append(d)
    n = len(seg_pcds)
    if n < 2:
        sys.exit("too few usable views after segmentation — check zmax / lighting / texture")
    log(f"  {n} usable views")

    # 2. angle prior (fallback init only) + robust object axis
    recorded = [read_angle(d) for d in kept]
    have_all = all(a is not None for a in recorded)
    angles = [direction * (recorded[i] if have_all else i * 360.0 / n) for i in range(n)]
    log("  prior angles: " + ("recorded" if have_all else "uniform") + " (fallback only)")
    center = estimate_rotation_axis(seg_pcds, log=log)

    # 3. windowed, gated, feature-first edges
    log(f"  registering (window={window}, fit>={fit_min}, "
        f"{'ring + loop' if loop else 'open chain'})...")
    edges = build_edges(seg_pcds, shots, center, angles, voxel, window, fit_min,
                        loop=loop, force_sift=force_sift, log=log)
    if not edges:
        sys.exit("no edges survived gating — object too smooth/dark, or too little overlap. "
                 "Try --sift, a tighter depth gate (--object), or recapture with more overlap.")

    # 4. largest component -> global optimization -> fuse
    merged = solve_and_fuse(seg_pcds, edges, voxel, log=log)
    merged = merged.voxel_down_sample(voxel)
    if len(merged.points) > 20:
        merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    out = os.path.join(session_dir, "merged_object.ply")
    o3d.io.write_point_cloud(out, merged)
    log(f"  merged -> {len(merged.points)} points -> {out}")
    return out


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        v = cast(args[i + 1])
        del args[i:i + 2]
        return v
    return None


def _main():
    args = sys.argv[1:]
    do_mesh = "--mesh" in args
    if do_mesh:
        args.remove("--mesh")
    loop = "--no-loop" not in args
    if "--no-loop" in args:
        args.remove("--no-loop")
    force_sift = "--sift" in args
    if force_sift:
        args.remove("--sift")

    obj = _pop(args, "--object", str)
    voxel = _pop(args, "--voxel", float)
    crop = _pop(args, "--crop", float)
    zmax = _pop(args, "--zmax", float)
    zmin = _pop(args, "--zmin", float)
    direction = _pop(args, "--dir", int) or 1
    window = _pop(args, "--win", int) or WINDOW
    fit_min = _pop(args, "--fit-min", float) or FIT_MIN

    if obj:                                   # --object preset fills any unset knob
        import config
        cfg = config.select(obj)
        voxel = cfg["voxel"] if voxel is None else voxel
        zmin = cfg["zmin"] if zmin is None else zmin
        zmax = cfg["zmax"] if zmax is None else zmax
        crop = cfg["crop"] if crop is None else crop
    voxel = voxel or 0.003
    zmin = seg.ZMIN_DEFAULT if zmin is None else zmin
    zmax = seg.ZMAX_DEFAULT if zmax is None else zmax

    if not args:
        sys.exit("usage: python build_object.py <session_dir> [--object db5] "
                 "[--voxel 0.003] [--crop 0.15] [--zmax 0.45] [--dir -1] [--win 3] "
                 "[--fit-min 0.3] [--sift] [--no-loop] [--mesh]")
    out = build(args[0], voxel=voxel, zmin=zmin, zmax=zmax, crop=crop, loop=loop,
                direction=direction, window=window, fit_min=fit_min, force_sift=force_sift)
    if do_mesh:
        import mesh
        mesh.poisson_mesh(out)


if __name__ == "__main__":
    _main()
