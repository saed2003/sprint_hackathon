"""
Turn one capture (depth + intrinsics) into a 3D point cloud (.ply).

This is the back-projection step from D405_Depth_Point_Clouds.md:
    Z = depth(u,v) in meters
    X = (u - ppx) * Z / fx
    Y = (v - ppy) * Z / fy

It does ONE point cloud per capture folder. By default that is just the newest
capture; use --all (or list folders) to do several in a single command.

Usage:
    .venv/bin/python make_pointcloud.py                       # newest capture only
    .venv/bin/python make_pointcloud.py captures/2026..       # one specific capture
    .venv/bin/python make_pointcloud.py captures/A captures/B # several captures
    .venv/bin/python make_pointcloud.py --all                 # EVERY capture folder

Outputs (inside each capture folder):
    cloud.ply          the 3D point cloud (open in MeshLab, CloudCompare, or Open3D)
    cloud_preview.png  a quick rendered preview so you can see it without a 3D viewer
"""

import os
import sys
import glob
import numpy as np
import open3d as o3d

# don't require a display for the preview image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_intrinsics(path):
    """Read the key=value lines from intrinsics.txt into a dict of floats."""
    vals = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 2:
                vals[parts[0]] = float(parts[1])
    return vals


def all_captures():
    """Every capture folder (one that has a depth.npy), oldest -> newest."""
    folders = sorted(
        os.path.dirname(p) for p in glob.glob("captures/*/depth.npy"))
    if not folders:
        sys.exit("No captures found. Run capture.py, or drive.py and press C.")
    return folders


def newest_capture():
    folders = all_captures()
    return max(folders, key=os.path.getmtime)


def process(folder):
    print(f"Using capture: {folder}")

    depth_raw = np.load(os.path.join(folder, "depth.npy"))          # uint16
    ir = None
    ir_path = os.path.join(folder, "ir_left.png")
    if os.path.exists(ir_path):
        import cv2
        ir = cv2.imread(ir_path, cv2.IMREAD_GRAYSCALE)              # aligned to depth

    intr = load_intrinsics(os.path.join(folder, "intrinsics.txt"))
    fx, fy = intr["fx"], intr["fy"]
    ppx, ppy = intr["ppx"], intr["ppy"]
    scale = intr["depth_scale"]                                     # raw -> meters

    H, W = depth_raw.shape
    Z = depth_raw.astype(np.float32) * scale                       # meters

    # pixel grid
    u = np.arange(W)
    v = np.arange(H)
    uu, vv = np.meshgrid(u, v)

    # back-projection (the core equation)
    X = (uu - ppx) * Z / fx
    Y = (vv - ppy) * Z / fy

    # keep only valid points: real depth, and within the D405's useful range
    valid = (Z > 0.05) & (Z < 3.0)
    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=1)

    print(f"  {pts.shape[0]} valid 3D points (from {W*H} pixels)")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if ir is not None:
        g = ir[valid].astype(np.float32) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(np.stack([g, g, g], axis=1))

    out_ply = os.path.join(folder, "cloud.ply")
    o3d.io.write_point_cloud(out_ply, pcd)
    print(f"  saved -> {out_ply}")

    # quick preview render (downsample so it's fast)
    n = pts.shape[0]
    idx = np.random.choice(n, size=min(n, 25000), replace=False)
    s = pts[idx]
    col = (ir[valid][idx] / 255.0) if ir is not None else s[:, 2]
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    # view down the +Z axis (looking out the way the camera looks)
    ax.scatter(s[:, 0], s[:, 2], -s[:, 1], c=col, cmap="gray", s=1, linewidths=0)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Z depth (m)"); ax.set_zlabel("-Y (m)")
    ax.set_title(f"{os.path.basename(folder)}  —  {n} points")
    out_png = os.path.join(folder, "cloud_preview.png")
    plt.tight_layout(); plt.savefig(out_png, dpi=110); plt.close()
    print(f"  saved -> {out_png}")


def main():
    args = sys.argv[1:]
    if "--all" in args:
        folders = all_captures()
    elif args:
        folders = args                  # one or more folders given explicitly
    else:
        folders = [newest_capture()]    # default: just the newest

    print(f"Making {len(folders)} point cloud(s).\n")
    for folder in folders:
        process(folder)
        print()


if __name__ == "__main__":
    main()
