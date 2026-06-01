"""
Merge several captures into ONE point cloud using Open3D registration (ICP).

This is milestone 2: take clouds taken from different angles of the SAME scene and
align them into a single shared coordinate frame, then combine them.

For each consecutive pair it does:
  1. coarse alignment  -> RANSAC on FPFH features  (a rough first guess)
  2. fine alignment    -> point-to-plane ICP        (snaps them together)
It chains the transforms so every cloud ends up in the FIRST cloud's frame, then merges.

Usage:
  # merge specific captures (give them in the order they were taken):
  .venv/bin/python pointcloud/merge_clouds.py captures/A captures/B captures/C

  # or merge ALL captures (oldest -> newest):
  .venv/bin/python pointcloud/merge_clouds.py

  # robot case: if you know the rotation step between shots, seed it (degrees about vertical):
  .venv/bin/python pointcloud/merge_clouds.py --angle 30 captures/A captures/B captures/C

Tips for it to work:
  - capture the SAME scene, rotating only a LITTLE between shots (lots of overlap).
  - textured, well-lit scene (D405 is passive stereo, remember).

Outputs (in the project root):
  merged.ply          the combined cloud
  merged_views.png    front + top-down preview
"""

import sys, os, glob, copy
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# captures/ + merged output live at the project root (one level up from pointcloud/)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

VOXEL = 0.01            # 1 cm: final cloud resolution
REG_VOXEL = 0.02        # 2 cm: coarser, used for registration (faster, more robust)


def load_cloud(folder):
    """Build a clean point cloud from one capture's depth.npy + intrinsics.txt."""
    depth = np.load(os.path.join(folder, "depth.npy")).astype(np.float32)
    intr = {}
    with open(os.path.join(folder, "intrinsics.txt")) as f:
        for line in f:
            k = line.split()
            if len(k) == 2:
                intr[k[0]] = float(k[1])
    Z = depth * intr["depth_scale"]
    H, W = depth.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    X = (uu - intr["ppx"]) * Z / intr["fx"]
    Y = (vv - intr["ppy"]) * Z / intr["fy"]
    valid = (Z > 0.05) & (Z < 3.0)
    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd = pcd.voxel_down_sample(VOXEL)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL * 2, max_nn=30))
    return pcd


def prep(pcd):
    """Downsample for registration and compute FPFH features."""
    down = pcd.voxel_down_sample(REG_VOXEL)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=REG_VOXEL * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=REG_VOXEL * 5, max_nn=100))
    return down, fpfh


def global_align(src, dst):
    """Coarse alignment of src onto dst using FPFH features (Fast Global Registration).

    FGR is much faster than RANSAC and works well for overlapping views.
    """
    s_down, s_fpfh = prep(src)
    d_down, d_fpfh = prep(dst)
    opt = o3d.pipelines.registration.FastGlobalRegistrationOption(
        maximum_correspondence_distance=REG_VOXEL * 1.5)
    res = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        s_down, d_down, s_fpfh, d_fpfh, opt)
    return res.transformation


def yaw_seed(angle_deg):
    """Initial guess = rotation about the vertical (camera Y) axis by angle_deg."""
    a = np.radians(angle_deg)
    return np.array([[ np.cos(a), 0, np.sin(a), 0],
                     [ 0,         1, 0,         0],
                     [-np.sin(a), 0, np.cos(a), 0],
                     [ 0,         0, 0,         1]])


def refine_icp(src, dst, init):
    """Fine point-to-plane ICP starting from init transform."""
    res = o3d.pipelines.registration.registration_icp(
        src, dst, VOXEL * 2, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    return res


def main():
    args = sys.argv[1:]
    angle = None
    if "--angle" in args:
        i = args.index("--angle")
        angle = float(args[i + 1])
        del args[i:i + 2]

    folders = args if args else sorted(
        os.path.dirname(p) for p in glob.glob(os.path.join(ROOT, "captures", "*", "depth.npy")))
    if len(folders) < 2:
        sys.exit("Need at least 2 captures. Run camera/capture.py a few times "
                 "(rotate the camera a little between shots).")

    print(f"Merging {len(folders)} captures:")
    for f in folders:
        print("  -", f)

    clouds = [load_cloud(f) for f in folders]

    merged = copy.deepcopy(clouds[0])
    cumulative = np.eye(4)                      # maps cloud i -> cloud 0 frame
    for i in range(1, len(clouds)):
        init = yaw_seed(angle) if angle is not None else global_align(clouds[i], clouds[i - 1])
        res = refine_icp(clouds[i], clouds[i - 1], init)
        print(f"  pair {i-1}->{i}: fitness={res.fitness:.3f} "
              f"rmse={res.inlier_rmse*1000:.1f}mm  (higher fitness = better overlap)")
        cumulative = cumulative @ res.transformation
        merged += copy.deepcopy(clouds[i]).transform(cumulative)

    merged = merged.voxel_down_sample(VOXEL)
    # final cleaning pass: drop floating noise specks left over after merging
    before = len(merged.points)
    merged, _ = merged.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
    out_ply = os.path.join(ROOT, "merged.ply")
    o3d.io.write_point_cloud(out_ply, merged)
    print(f"merged cloud: {len(merged.points)} points "
          f"(cleaned {before - len(merged.points)} outliers) -> {out_ply}")

    # preview (front + top-down)
    p = np.asarray(merged.points)
    idx = np.random.choice(len(p), size=min(len(p), 50000), replace=False)
    p = p[idx]
    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(121); ax1.set_aspect("equal")
    ax1.scatter(p[:, 0], -p[:, 1], c=p[:, 2], cmap="turbo", s=1, linewidths=0)
    ax1.set_title("MERGED — front view"); ax1.set_xlabel("X (m)"); ax1.set_ylabel("-Y (m)")
    ax2 = fig.add_subplot(122); ax2.set_aspect("equal")
    ax2.scatter(p[:, 0], p[:, 2], c=p[:, 2], cmap="turbo", s=1, linewidths=0)
    ax2.set_title("MERGED — top-down"); ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z (m)")
    out_png = os.path.join(ROOT, "merged_views.png")
    plt.tight_layout(); plt.savefig(out_png, dpi=110); plt.close()
    print(f"saved -> {out_png}")


if __name__ == "__main__":
    main()
