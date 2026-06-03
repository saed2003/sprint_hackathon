"""
Trim the STAND/PEDESTAL off a finished object scan, leaving just the object.

Both demo setups put the object on a support that ends up fused into the merged
cloud: the turntable spins the car on a white CUP; the robot orbit films it on a
tall STAND. In the merged cloud that support is a narrower column BELOW the (wider)
object, joined at a 'neck'. This reads merged_object.ply, cuts at that neck, keeps
only the object on top, and (optionally) re-meshes it -> a clean car-only model.

It's a pure POST-PROCESS of an already-built cloud, so it's independent of the merge
pipeline (build_object.py) and safe to run on any session's merged_object.ply.

    python trim_car.py captures/<session>            # auto neck-detect, points + mesh
    python trim_car.py captures/<session> --keep-cm 6.5   # force: keep top 6.5 cm
    python trim_car.py captures/<session>/merged_object.ply --no-mesh
    python trim_car.py captures/<session> --frac 0.65    # tweak neck sensitivity
"""
import os
import sys

import numpy as np
import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import segment as seg          # keep_largest_cluster
import mesh as mesh_mod
import preview as preview_mod


def trim_pedestal(pcd, frac=0.6, keep_cm=None, log=print):
    """Keep the object on top, drop the support column below.

    Camera Y points DOWN, so the object sits at the SMALLEST y. We slice the cloud
    vertically and find the 'neck' where the object meets its narrower support:

      keep_cm given -> just keep the top `keep_cm` cm.
      else (auto)   -> walk down from the top; once we've entered the wide object
        (a slice >= 0.85*peak width), cut at the first slice that necks in below
        `frac`*peak. Everything above (incl. a narrow roof) is the object.

    Returns (trimmed_cloud, y_cut_cm_from_top).
    """
    pts = np.asarray(pcd.points)
    if len(pts) < 50:
        return pcd, None
    y = pts[:, 1]
    ylo, yhi = y.min(), y.max()

    if keep_cm is not None:
        ycut = ylo + keep_cm / 100.0
    else:
        nb = 24
        edges = np.linspace(ylo, yhi, nb + 1)
        widths = np.zeros(nb)
        for i in range(nb):
            m = (y >= edges[i]) & (y < edges[i + 1])
            if m.sum() >= 5:
                sl = pts[m]
                widths[i] = max(np.ptp(sl[:, 0]), np.ptp(sl[:, 2]))
        peak = widths.max()
        ycut, entered = None, False
        for i in range(nb):
            if widths[i] >= 0.85 * peak:
                entered = True
            elif entered and 0 < widths[i] < frac * peak:
                ycut = edges[i]                      # top edge of the necking slice
                break
        if ycut is None:
            log("  trim: no clear neck found — leaving cloud as-is")
            return pcd, None

    keep = np.where(y <= ycut)[0]
    trimmed = pcd.select_by_index(keep)
    trimmed = seg.keep_largest_cluster(trimmed, eps=0.012, min_points=15)
    if len(trimmed.points) > 20:
        trimmed, _ = trimmed.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    log(f"  trim: kept top {(ycut - ylo) * 100:.1f} cm -> "
        f"{len(trimmed.points)} pts (was {len(pts)})")
    return trimmed, (ycut - ylo) * 100


def _resolve(path):
    """Accept a session dir or a direct .ply path; return the merged_object.ply."""
    if os.path.isdir(path):
        p = os.path.join(path, "merged_object.ply")
        if not os.path.exists(p):
            sys.exit(f"no merged_object.ply in {path} — build it first")
        return p
    if not os.path.exists(path):
        sys.exit(f"not found: {path}")
    return path


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag); v = cast(args[i + 1]); del args[i:i + 2]; return v
    return None


def main():
    args = sys.argv[1:]
    do_mesh = "--no-mesh" not in args
    if not do_mesh:
        args.remove("--no-mesh")
    keep_cm = _pop(args, "--keep-cm", float)
    frac = _pop(args, "--frac", float) or 0.6
    depth = _pop(args, "--depth", int) or 10
    if not args:
        sys.exit("usage: python trim_car.py <session_dir | merged_object.ply> "
                 "[--keep-cm N] [--frac 0.6] [--depth 10] [--no-mesh]")

    src = _resolve(args[0])
    pcd = o3d.io.read_point_cloud(src)
    print(f"trim_car: {len(pcd.points)} pts from {src}")
    car, _ = trim_pedestal(pcd, frac=frac, keep_cm=keep_cm)

    pts = np.asarray(car.points)
    print("  car bbox (cm):", np.round((pts.max(0) - pts.min(0)) * 100, 1))

    out_pts = src.replace("merged_object.ply", "merged_car.ply")
    o3d.io.write_point_cloud(out_pts, car)
    print(f"  points -> {out_pts}")

    outputs = [out_pts]
    if do_mesh:
        # a little higher density cut than the default: a half-scanned car (no
        # underside) makes Poisson balloon below, so trim weak vertices harder.
        outputs.append(mesh_mod.poisson_mesh(out_pts, depth=depth, density_quantile=0.06))

    for f in outputs:
        try:
            print("  preview ->", preview_mod.preview(f))
        except Exception as e:
            print(f"  (preview skipped for {os.path.basename(f)}: {e})")

    print("\ndone. view interactively (laptop):")
    print(f"  env -u WAYLAND_DISPLAY DISPLAY=:0 {sys.executable} -c "
          f"\"import open3d as o3d; o3d.visualization.draw_geometries("
          f"[o3d.io.read_point_cloud('{out_pts}')])\"")


if __name__ == "__main__":
    main()
