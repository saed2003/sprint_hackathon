"""
Merge many object views into ONE clean point cloud  (the heart of the object scan).

Input  : a session folder of shot_NN capture folders. Works the same whether the
         shots came from a turntable (capture_session.py), a hand-stepped scan, or
         the robot orbit (capture_orbit.py) — they all write the standard folder.
Output : <session>/merged_object.ply  (+ optionally an object_mesh.ply via mesh.py)

WHY THIS REUSES YOUR ROOM-SCAN IDEA
-----------------------------------
scan360.py merges by rotating each view about a vertical axis through the CAMERA.
An object orbit is the same operation about a vertical axis through the OBJECT
CENTRE (a point ~d in front of the camera). So we:

  1. segment each view to the object only            (segment.py)
  2. PRE-ALIGN with the angle prior: rotate each view about the object centre by
     its turn angle, into view-0's frame                (a good starting guess)
  3. REFINE + close the loop with Open3D MULTIWAY REGISTRATION:
     point-to-plane ICP on neighbours -> pose graph -> global optimization.
     This is what absorbs the robot's open-loop motion error.

The angle prior comes from each shot's angle.txt (written by the capturers); with
no angle.txt it assumes a uniform 360/N step. Either way ICP fixes the residual.

Laptop-only (Open3D). Run from this folder:
    python build_object.py <session_dir>
    python build_object.py <session_dir> --no-loop      # partial scan (not full 360)
    python build_object.py <session_dir> --voxel 0.004 --crop 0.15
    python build_object.py <session_dir> --mesh         # also build a surface mesh
"""
import os
import sys
import glob

import numpy as np
import open3d as o3d

import segment as seg

reg = o3d.pipelines.registration


# ── geometry: the object-centred angle prior ─────────────────────────────────────

def ry_about(center, angle_deg):
    """4x4 transform: rotate `angle_deg` about the vertical (Y) axis THROUGH `center`.

    p' = T(center) @ Ry @ T(-center) @ p   — i.e. rotate the world around the object,
    not around the camera. This is the only geometric difference from scan360.
    """
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = center - R @ center
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


def object_center(pcd):
    """Robust centre of an object cloud, in the camera frame (median, not mean)."""
    return np.median(np.asarray(pcd.points), axis=0)


def estimate_rotation_axis(pcds, angles=None, log=print, iters=3):
    """Find the (x,z) of the vertical rotation axis through the OBJECT CENTRE.

    The object stays in ~the same place in the camera frame across views (turntable:
    the camera is fixed and the object spins in place; orbit: the camera re-aims at it),
    so the axis is simply the MEDIAN of each view's centroid. Robust and cheap. (An
    earlier bootstrap that pre-merged with the angle prior could run the axis away when
    the prior was even slightly off — that produced the 80 cm blow-ups.)
    """
    cents = np.array([np.median(np.asarray(p.points), axis=0) for p in pcds])
    center = np.median(cents, axis=0)
    log(f"  rotation axis (x,z) = ({center[0]:.3f}, {center[2]:.3f}) m")
    return center


# ── multiway registration (Open3D pose graph) ────────────────────────────────────

def _pairwise(source, target, coarse, fine):
    """Point-to-plane ICP source->target: coarse then fine. Returns (T, info matrix).

    Clouds are already pre-aligned by the angle prior, so we start ICP from identity
    and it only needs to clean up the residual.
    """
    icp_c = reg.registration_icp(source, target, coarse, np.eye(4),
                                 reg.TransformationEstimationPointToPlane())
    icp_f = reg.registration_icp(source, target, fine, icp_c.transformation,
                                 reg.TransformationEstimationPointToPlane())
    info = reg.get_information_matrix_from_point_clouds(source, target, fine,
                                                        icp_f.transformation)
    return icp_f.transformation, info


def build_pose_graph(pcds, coarse, fine, loop=True):
    """Ring pose graph: neighbours are odometry edges; last->first is the loop closure.

    For an object orbit, only ADJACENT views overlap (opposite sides share nothing),
    so we connect neighbours + the 360 wrap — not all pairs. That wrap edge is what
    ties the ring together and distributes the accumulated error.
    """
    pose_graph = reg.PoseGraph()
    odometry = np.eye(4)
    pose_graph.nodes.append(reg.PoseGraphNode(odometry))
    n = len(pcds)
    for i in range(n - 1):
        T, info = _pairwise(pcds[i], pcds[i + 1], coarse, fine)
        odometry = T @ odometry
        pose_graph.nodes.append(reg.PoseGraphNode(np.linalg.inv(odometry)))
        pose_graph.edges.append(reg.PoseGraphEdge(i, i + 1, T, info, uncertain=False))
    if loop and n > 2:
        T, info = _pairwise(pcds[n - 1], pcds[0], coarse, fine)
        pose_graph.edges.append(reg.PoseGraphEdge(n - 1, 0, T, info, uncertain=True))
    return pose_graph


def optimize(pose_graph, fine):
    """Global pose-graph optimization (Levenberg-Marquardt) — fixes the ring."""
    option = reg.GlobalOptimizationOption(
        max_correspondence_distance=fine, edge_prune_threshold=0.25, reference_node=0)
    reg.global_optimization(
        pose_graph, reg.GlobalOptimizationLevenbergMarquardt(),
        reg.GlobalOptimizationConvergenceCriteria(), option)
    return pose_graph


# ── top-level build ──────────────────────────────────────────────────────────────

def build(session_dir, voxel=0.003, zmin=seg.ZMIN_DEFAULT, zmax=seg.ZMAX_DEFAULT,
          remove_plane=True, crop=None, loop=True, direction=1, log=print):
    """Segment, pre-align, multiway-register, fuse -> <session>/merged_object.ply.

    direction: +1 or -1, the sign of the turn between shots. If the model comes out
    smeared/mirrored, flip it (--dir -1) — the turntable/robot may spin the other way.
    """
    dirs = shot_dirs(session_dir)
    if len(dirs) < 2:
        sys.exit(f"need >=2 shots in {session_dir} (found {len(dirs)})")
    log(f"object build: {len(dirs)} views from {session_dir}")

    # 1. segment every view to the object only
    raw = []
    for d in dirs:
        pcd = seg.segment_object(d, zmin=zmin, zmax=zmax, remove_plane=remove_plane,
                                 crop=crop, voxel=voxel)
        if len(pcd.points) < 50:
            log(f"  skip {os.path.basename(d)} (only {len(pcd.points)} pts)")
            continue
        raw.append((d, pcd))
    if len(raw) < 2:
        sys.exit("too few usable views after segmentation — check zmax / lighting / texture")

    # 2. angle prior: rotate each view about the object's vertical axis into view-0's
    #    frame. Angles come from angle.txt (else a uniform 360/N step); ICP fixes the
    #    residual. The axis (x,z) is found by estimate_rotation_axis (see why there).
    n = len(raw)
    recorded = [read_angle(d) for d, _ in raw]
    have_all = all(a is not None for a in recorded)
    angles = [direction * (recorded[i] if have_all else i * 360.0 / n) for i in range(n)]
    log("  prior angles: " + ("recorded " if have_all else "uniform ")
        + ", ".join(f"{a:.0f}" for a in angles) + " deg")

    seg_pcds = [pcd for _, pcd in raw]
    center = estimate_rotation_axis(seg_pcds, angles, log=log)

    pcds = []
    for pcd, ang in zip(seg_pcds, angles):
        pcd.transform(ry_about(center, ang))     # pre-align into the common frame
        pcds.append(pcd)

    # 3. multiway registration to refine + close the loop
    coarse, fine = voxel * 15, voxel * 1.5
    log(f"  registering ({n} nodes, ring{' + loop close' if loop else ''})...")
    pose_graph = build_pose_graph(pcds, coarse, fine, loop=loop)
    optimize(pose_graph, fine)

    # 4. apply corrected poses and fuse
    merged = o3d.geometry.PointCloud()
    for i in range(n):
        pcds[i].transform(pose_graph.nodes[i].pose)
        merged += pcds[i]
    merged = merged.voxel_down_sample(voxel)
    if len(merged.points) > 20:
        merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    out = os.path.join(session_dir, "merged_object.ply")
    o3d.io.write_point_cloud(out, merged)
    log(f"  merged {n} views -> {len(merged.points)} points -> {out}")
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
    voxel = _pop(args, "--voxel", float) or 0.003
    crop = _pop(args, "--crop", float)
    zmax = _pop(args, "--zmax", float) or seg.ZMAX_DEFAULT
    zmin = _pop(args, "--zmin", float) or seg.ZMIN_DEFAULT
    direction = _pop(args, "--dir", int) or 1
    if not args:
        sys.exit("usage: python build_object.py <session_dir> "
                 "[--voxel 0.003] [--crop 0.15] [--zmax 0.45] [--dir -1] [--no-loop] [--mesh]")
    out = build(args[0], voxel=voxel, zmin=zmin, zmax=zmax, crop=crop, loop=loop,
                direction=direction)
    if do_mesh:
        import mesh
        mesh.poisson_mesh(out)


if __name__ == "__main__":
    _main()
