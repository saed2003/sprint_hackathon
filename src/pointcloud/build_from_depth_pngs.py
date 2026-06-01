"""
Build ONE merged point cloud from 10 depth-map PNGs.

Pipeline (one shot):
    data/depth_map/<NN>/depth.png   ->   per-frame point cloud
                                    ->   ICP-merged into a single cloud
                                    ->   data/depth_map/merged.ply  (+ preview)

Per-frame back-projection (D405 intrinsics by default, override per folder):
    Z = depth(u,v) [meters]
    X = (u - ppx) * Z / fx
    Y = (v - ppy) * Z / fy

Folder layout this script expects:
    data/depth_map/
        01/depth.png            (uint16 depth PNG)
        02/depth.png
        ...
        10/depth.png
    Optional per-folder override:  intrinsics.txt  (same format as captures/)
    Optional per-folder grayscale: ir.png          (used only for colorizing)

Depth-PNG units:
    - Auto-detected: uint16 -> millimeters; float / uint8 -> meters.
    - Override with --units mm | m.

Usage:
    .venv/bin/python src/pointcloud/build_from_depth_pngs.py
    .venv/bin/python src/pointcloud/build_from_depth_pngs.py --units mm
    .venv/bin/python src/pointcloud/build_from_depth_pngs.py --angle 36   # seed yaw between shots
"""

import os
import sys
import glob
import copy
import argparse

import cv2
import numpy as np
import open3d as o3d

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# repo root = three levels up from this file (src/pointcloud/build_from_depth_pngs.py)
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_DIR = os.path.join(ROOT, "data", "depth_map")

# D405 IR @ 1280x720 — used when a folder has no intrinsics.txt
DEFAULT_INTR = {
    "width": 1280, "height": 720,
    "fx": 637.0789794921875, "fy": 637.0789794921875,
    "ppx": 632.6076049804688, "ppy": 363.996337890625,
    "depth_scale": 1.0,   # not used here; we convert PNG units ourselves
}

VOXEL = 0.01            # 1 cm: final cloud resolution
REG_VOXEL = 0.02        # 2 cm: coarser, used for registration
Z_MIN, Z_MAX = 0.05, 3.0  # D405's useful range, meters


def load_intrinsics(folder):
    """Read folder's intrinsics.txt, or fall back to DEFAULT_INTR."""
    path = os.path.join(folder, "intrinsics.txt")
    if not os.path.exists(path):
        return dict(DEFAULT_INTR)
    intr = dict(DEFAULT_INTR)
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 2:
                try:
                    intr[parts[0]] = float(parts[1])
                except ValueError:
                    pass
    return intr


def find_depth_png(folder):
    """First PNG that looks like a depth map (depth.png preferred)."""
    pref = os.path.join(folder, "depth.png")
    if os.path.exists(pref):
        return pref
    pngs = sorted(glob.glob(os.path.join(folder, "*.png")))
    pngs = [p for p in pngs if "ir" not in os.path.basename(p).lower()]
    return pngs[0] if pngs else None


def to_meters(depth_raw, units):
    """Return depth in meters as float32."""
    if units == "mm":
        return depth_raw.astype(np.float32) / 1000.0
    if units == "m":
        return depth_raw.astype(np.float32)
    # auto
    if depth_raw.dtype == np.uint16:
        return depth_raw.astype(np.float32) / 1000.0
    if np.issubdtype(depth_raw.dtype, np.floating):
        return depth_raw.astype(np.float32)
    # uint8 (rare): treat as meters * scale guess won't be right; assume meters
    return depth_raw.astype(np.float32)


def cloud_from_folder(folder, units):
    """Back-project this folder's depth PNG into a cleaned Open3D cloud."""
    depth_path = find_depth_png(folder)
    if depth_path is None:
        raise SystemExit(f"No depth PNG found in {folder}")

    raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise SystemExit(f"Could not read {depth_path}")
    if raw.ndim == 3:                 # accidental 3-channel — collapse
        raw = raw[..., 0]

    Z = to_meters(raw, units)
    H, W = Z.shape
    intr = load_intrinsics(folder)
    fx, fy, ppx, ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

    # If intrinsics were captured at a different resolution, scale to this image.
    iw, ih = int(intr.get("width", W)), int(intr.get("height", H))
    if (iw, ih) != (W, H) and iw > 0 and ih > 0:
        sx, sy = W / iw, H / ih
        fx, fy = fx * sx, fy * sy
        ppx, ppy = ppx * sx, ppy * sy

    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    X = (uu - ppx) * Z / fx
    Y = (vv - ppy) * Z / fy

    valid = (Z > Z_MIN) & (Z < Z_MAX)
    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=1)

    # optional IR grayscale alongside the depth, for coloring
    ir_path = os.path.join(folder, "ir.png")
    colors = None
    if os.path.exists(ir_path):
        ir = cv2.imread(ir_path, cv2.IMREAD_GRAYSCALE)
        if ir is not None and ir.shape == Z.shape:
            g = ir[valid].astype(np.float32) / 255.0
            colors = np.stack([g, g, g], axis=1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    pcd = pcd.voxel_down_sample(VOXEL)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL * 2, max_nn=30))

    print(f"  {os.path.basename(folder)}: {len(pcd.points)} points "
          f"(depth {depth_path.split(os.sep)[-1]}, {W}x{H})")
    return pcd


def prep_for_reg(pcd):
    down = pcd.voxel_down_sample(REG_VOXEL)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=REG_VOXEL * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=REG_VOXEL * 5, max_nn=100))
    return down, fpfh


def global_align(src, dst):
    s_down, s_fpfh = prep_for_reg(src)
    d_down, d_fpfh = prep_for_reg(dst)
    opt = o3d.pipelines.registration.FastGlobalRegistrationOption(
        maximum_correspondence_distance=REG_VOXEL * 1.5)
    res = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        s_down, d_down, s_fpfh, d_fpfh, opt)
    return res.transformation


def yaw_seed(angle_deg):
    a = np.radians(angle_deg)
    return np.array([[ np.cos(a), 0, np.sin(a), 0],
                     [ 0,         1, 0,         0],
                     [-np.sin(a), 0, np.cos(a), 0],
                     [ 0,         0, 0,         1]])


def refine_icp(src, dst, init):
    return o3d.pipelines.registration.registration_icp(
        src, dst, VOXEL * 2, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))


def save_preview(merged, out_png):
    p = np.asarray(merged.points)
    if len(p) == 0:
        print("  (empty cloud — skipping preview)")
        return
    idx = np.random.choice(len(p), size=min(len(p), 60000), replace=False)
    p = p[idx]
    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(121); ax1.set_aspect("equal")
    ax1.scatter(p[:, 0], -p[:, 1], c=p[:, 2], cmap="turbo", s=1, linewidths=0)
    ax1.set_title("MERGED — front view"); ax1.set_xlabel("X (m)"); ax1.set_ylabel("-Y (m)")
    ax2 = fig.add_subplot(122); ax2.set_aspect("equal")
    ax2.scatter(p[:, 0], p[:, 2], c=p[:, 2], cmap="turbo", s=1, linewidths=0)
    ax2.set_title("MERGED — top-down"); ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z (m)")
    plt.tight_layout(); plt.savefig(out_png, dpi=110); plt.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--units", choices=["auto", "mm", "m"], default="auto",
                    help="Depth-PNG units (default: auto — uint16=mm, float=m).")
    ap.add_argument("--angle", type=float, default=None,
                    help="Seed yaw between consecutive frames (deg). "
                         "E.g. 36 for 10 evenly spaced shots around 360°.")
    ap.add_argument("--folders", nargs="*", default=None,
                    help="Explicit folders (default: every subfolder of "
                         "data/depth_map/ that has a depth PNG, sorted by name).")
    args = ap.parse_args()

    if args.folders:
        folders = [os.path.abspath(f) for f in args.folders]
    else:
        folders = sorted(
            d for d in glob.glob(os.path.join(DATA_DIR, "*"))
            if os.path.isdir(d) and find_depth_png(d))

    if len(folders) < 2:
        sys.exit(f"Need at least 2 depth PNGs under {DATA_DIR}/<NN>/depth.png "
                 f"(found {len(folders)}).")

    print(f"Building point cloud from {len(folders)} depth maps "
          f"(units={args.units}):")
    clouds = [cloud_from_folder(f, args.units) for f in folders]

    print("\nMerging:")
    merged = copy.deepcopy(clouds[0])
    cumulative = np.eye(4)
    for i in range(1, len(clouds)):
        init = (yaw_seed(args.angle) if args.angle is not None
                else global_align(clouds[i], clouds[i - 1]))
        res = refine_icp(clouds[i], clouds[i - 1], init)
        print(f"  pair {i-1}->{i}: fitness={res.fitness:.3f} "
              f"rmse={res.inlier_rmse*1000:.1f}mm")
        cumulative = cumulative @ res.transformation
        merged += copy.deepcopy(clouds[i]).transform(cumulative)

    merged = merged.voxel_down_sample(VOXEL)
    before = len(merged.points)
    merged, _ = merged.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)

    out_ply = os.path.join(DATA_DIR, "merged.ply")
    out_png = os.path.join(DATA_DIR, "merged_views.png")
    o3d.io.write_point_cloud(out_ply, merged)
    save_preview(merged, out_png)

    print(f"\nmerged: {len(merged.points)} points "
          f"(cleaned {before - len(merged.points)} outliers)")
    print(f"  -> {out_ply}")
    print(f"  -> {out_png}")


if __name__ == "__main__":
    main()
