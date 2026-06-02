"""
Headless multi-angle preview of a point cloud or mesh -> one PNG.

No 3D viewer / display needed (matplotlib Agg), so it works over SSH and is handy
for checking a scan on the Pi or saving a figure for the presentation. Renders the
geometry from 4 viewpoints around it, using the cloud's own colours.

    python preview.py <merged_object.ply>
    python preview.py <merged_object.ply> -o my_preview.png
"""
import os
import sys

import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _load_xyz_rgb(path):
    """Read a PLY (cloud or mesh) -> (Nx3 xyz, Nx3 rgb 0..1). Meshes use their vertices."""
    pcd = o3d.io.read_point_cloud(path)
    pts = np.asarray(pcd.points)
    cols = np.asarray(pcd.colors)
    if len(pts) == 0:                                   # it's probably a mesh
        m = o3d.io.read_triangle_mesh(path)
        pts = np.asarray(m.vertices)
        cols = np.asarray(m.vertex_colors)
    if len(cols) != len(pts):
        cols = np.tile([0.6, 0.6, 0.65], (len(pts), 1))
    return pts, cols


def preview(path, out_path=None, max_points=40000):
    pts, cols = _load_xyz_rgb(path)
    if len(pts) == 0:
        sys.exit(f"no points in {path}")
    if len(pts) > max_points:                           # thin for a fast, light figure
        idx = np.random.choice(len(pts), max_points, replace=False)
        pts, cols = pts[idx], cols[idx]

    # camera frame: y is DOWN, so flip y to show the object upright in the plots
    x, y, z = pts[:, 0], -pts[:, 1], pts[:, 2]
    views = [(20, -60), (20, 30), (20, 120), (89, -90)]  # (elev, azim): 3 sides + top
    titles = ["front-left", "front-right", "back", "top"]

    fig = plt.figure(figsize=(12, 3.4))
    for i, ((elev, azim), title) in enumerate(zip(views, titles)):
        ax = fig.add_subplot(1, 4, i + 1, projection="3d")
        ax.scatter(x, z, y, c=np.clip(cols, 0, 1), s=1.2, depthshade=False)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, fontsize=9)
        ax.set_box_aspect((np.ptp(x) or 1, np.ptp(z) or 1, np.ptp(y) or 1))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    fig.suptitle(f"{os.path.basename(path)}   ({len(pts)} pts shown)", fontsize=10)
    fig.tight_layout()

    out = out_path or path.replace(".ply", "_preview.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"preview -> {out}")
    return out


def _main():
    args = sys.argv[1:]
    if not args:
        sys.exit("usage: python preview.py <cloud_or_mesh.ply> [-o out.png]")
    out = None
    if "-o" in args:
        out = args[args.index("-o") + 1]
    preview(args[0], out_path=out)


if __name__ == "__main__":
    _main()
