"""
view_ply.py -- open a .ply in an interactive 3D window using Open3D (no MeshLab).

Controls:
    drag        = rotate
    scroll      = zoom
    right-drag  = pan
    + / -       = bigger / smaller points   (live)
    Q or close  = quit

Run:
    python view_ply.py                      # shows pointcloud_360.ply in this folder
    python view_ply.py path/to/cloud.ply
    python view_ply.py --size 3             # initial point size

Needs a display. On your laptop it just works. On the headless Pi you'd need VNC / X
(or use the preview PNG). Colours stored in the .ply (red=near, blue=far) show as-is.
"""

import os
import sys

import numpy as np
import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    args = sys.argv[1:]
    size = 2.0
    if "--size" in args:
        i = args.index("--size")
        size = float(args[i + 1])
        del args[i:i + 2]

    path = args[0] if args else os.path.join(HERE, "pointcloud_360.ply")
    if not os.path.exists(path):
        sys.exit(f"Not found: {path}")

    pcd = o3d.io.read_point_cloud(path)
    n = len(pcd.points)
    if n == 0:
        sys.exit(f"{path} has no points.")
    print(f"{os.path.basename(path)}: {n} points, colors={'yes' if pcd.has_colors() else 'no'}")
    print("drag=rotate  scroll=zoom  right-drag=pan  +/- =point size  Q=quit")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"360 cloud - {os.path.basename(path)}", width=1100, height=750)
    vis.add_geometry(pcd)
    opt = vis.get_render_option()
    opt.point_size = size
    opt.background_color = np.array([0.05, 0.06, 0.09])

    def bump(delta):
        def cb(v):
            o = v.get_render_option()
            o.point_size = float(np.clip(o.point_size + delta, 1.0, 15.0))
            v.update_renderer()
            print(f"point size: {o.point_size:.0f}")
            return False
        return cb

    # '=' is the unshifted '+' key; bind both, plus ']'/'[' as alternates
    for k in ("=", "+", "]"):
        vis.register_key_callback(ord(k), bump(+1))
    for k in ("-", "_", "["):
        vis.register_key_callback(ord(k), bump(-1))

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
