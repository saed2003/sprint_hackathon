"""
Object segmentation for the close-range D405 object-scan pipeline.

Reads a STANDARD capture folder (the same contract the whole robot uses) ...

    capture/<...>/
        depth.npy        uint16 raw depth (× depth_scale -> meters)
        intrinsics.txt   width height fx fy ppx ppy depth_scale baseline_m
        ir_left.png      gray IR (used as point colour if no color.png)
        color.png        OPTIONAL real RGB (capture_session.py saves this) -> colored model

... back-projects it to a 3D point cloud and ISOLATES THE OBJECT from the floor and
background. For an object scan this is the single biggest quality lever: the object
is the closest thing to the D405, so a tight depth gate + plane removal cleans it up.

Three stages, each optional:
  1. depth gate    keep only points in [zmin, zmax] m  (D405 is sharp 0.07-0.50 m)
  2. plane removal RANSAC-drop the dominant plane (the table the object sits on)
  3. centroid crop keep a cube of half-size `crop` m around the object's middle

Laptop-only (needs open3d). It only READS capture folders, so it can't affect the
running robot. Used by build_object.py; also runnable standalone to eyeball one view.

    python segment.py <capture_folder>            # view the segmented object
    python segment.py <capture_folder> --no-plane # skip table removal
"""
import os
import sys

import numpy as np
import cv2
import open3d as o3d

# D405 close-range sweet spot. Tighten zmax to your object's distance for a cleaner cut.
ZMIN_DEFAULT = 0.07
ZMAX_DEFAULT = 0.50


def load_intrinsics(path):
    """Parse intrinsics.txt (key value per line) into a dict of floats."""
    vals = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                vals[p[0]] = float(p[1])
    return vals


def back_project(folder, zmin=ZMIN_DEFAULT, zmax=ZMAX_DEFAULT):
    """capture folder -> (points Nx3 metres, colors Nx3 uint8 RGB) in the camera frame.

    Same back-projection math as pointcloud/scan360.py (so results are consistent):
        Z = depth * depth_scale,  X = (u-ppx)Z/fx,  Y = (v-ppy)Z/fy
    Colour comes from color.png if present, else the left-IR image (gray).
    """
    depth_raw = np.load(os.path.join(folder, "depth.npy"))
    intr = load_intrinsics(os.path.join(folder, "intrinsics.txt"))
    Z = depth_raw.astype(np.float32) * intr["depth_scale"]
    H, W = depth_raw.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    X = (uu - intr["ppx"]) * Z / intr["fx"]
    Y = (vv - intr["ppy"]) * Z / intr["fy"]
    valid = (Z > zmin) & (Z < zmax)

    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=1).astype(np.float32)

    color_path = os.path.join(folder, "color.png")
    if os.path.exists(color_path):
        bgr = cv2.imread(color_path)
        if bgr.shape[:2] != depth_raw.shape:        # match color to depth grid if sizes differ
            bgr = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)[valid]
        cols = rgb.astype(np.uint8)
    else:
        ir_path = os.path.join(folder, "ir_left.png")
        if os.path.exists(ir_path):
            g = cv2.imread(ir_path, cv2.IMREAD_GRAYSCALE)
            if g.shape != depth_raw.shape:
                g = cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA)
            g = g[valid]
            cols = np.stack([g, g, g], axis=1).astype(np.uint8)
        else:
            cols = np.full((len(pts), 3), 200, np.uint8)
    return pts, cols


def to_o3d(points, colors):
    """numpy points (m) + colours (uint8 RGB) -> Open3D point cloud (colours 0..1)."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
    return pcd


def remove_dominant_plane(pcd, distance_threshold=0.006, ransac_n=3,
                          num_iterations=400, min_inlier_frac=0.15):
    """Drop the biggest flat surface (the table) with RANSAC plane fitting.

    Only removes it if the plane is a meaningful chunk of the cloud (>= min_inlier_frac),
    so we don't accidentally carve a flat FACE off the object. Returns the cloud minus
    the plane.
    """
    if len(pcd.points) < ransac_n + 1:
        return pcd
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)
    if len(inliers) < min_inlier_frac * len(pcd.points):
        return pcd                                   # no dominant plane -> leave as-is
    return pcd.select_by_index(inliers, invert=True)


def keep_largest_cluster(pcd, eps=0.012, min_points=15):
    """Keep only the biggest DBSCAN cluster — the object — and drop everything else
    (floor remnants, background patches, stray flyers). This is the robust way to
    isolate a SMALL object that may be off-centre in the frame: after the floor plane
    is removed, the object is the largest compact blob left. Much better than cropping
    around a centroid, which gets dragged off by background points."""
    if len(pcd.points) < min_points:
        return pcd
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))
    if labels.max() < 0:                              # no cluster found -> leave as-is
        return pcd
    biggest = np.bincount(labels[labels >= 0]).argmax()
    return pcd.select_by_index(np.where(labels == biggest)[0])


def crop_around_centroid(pcd, half):
    """Keep only points within `half` metres (per axis) of the cloud's robust centre.

    A cheap, orientation-free 'is this the object?' box. Use it when plane removal
    leaves background clutter. `half` ~ object half-size + a margin (e.g. 0.15).
    """
    if len(pcd.points) == 0 or half is None:
        return pcd
    pts = np.asarray(pcd.points)
    c = np.median(pts, axis=0)
    keep = np.all(np.abs(pts - c) <= half, axis=1)
    return pcd.select_by_index(np.where(keep)[0])


def estimate_normals(pcd, radius=0.01, max_nn=30):
    """Normals are required for point-to-plane ICP and Poisson meshing."""
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    return pcd


def segment_object(folder, zmin=ZMIN_DEFAULT, zmax=ZMAX_DEFAULT, remove_plane=True,
                   crop=None, voxel=0.003, sor=True, normals=True, cluster=True):
    """Full per-view segmentation: capture folder -> clean object point cloud.

    Order: depth gate -> voxel -> remove floor plane -> KEEP LARGEST CLUSTER (the
    object) -> optional centroid crop -> outlier removal -> normals.

    voxel  : downsample resolution (m); 0.0015-0.004 keeps detail for small objects.
    cluster: keep only the biggest blob (isolates a small/off-centre object from
             background). Turn off for objects that fill the frame.
    crop   : half-size (m) of a centroid box, or None to skip.
    sor    : statistical outlier removal (drops passive-stereo flyers).
    """
    pts, cols = back_project(folder, zmin, zmax)
    pcd = to_o3d(pts, cols)
    if voxel:
        pcd = pcd.voxel_down_sample(voxel)
    if remove_plane:
        pcd = remove_dominant_plane(pcd)
    if cluster:
        pcd = keep_largest_cluster(pcd, eps=max(0.01, (voxel or 0.003) * 8))
    pcd = crop_around_centroid(pcd, crop)
    if sor and len(pcd.points) > 20:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    if normals and len(pcd.points) > 10:
        pcd = estimate_normals(pcd)
    return pcd


def _main():
    args = sys.argv[1:]
    if not args:
        sys.exit("usage: python segment.py <capture_folder> [--no-plane] [--crop 0.15]")
    folder = args[0]
    remove_plane = "--no-plane" not in args
    crop = None
    if "--crop" in args:
        crop = float(args[args.index("--crop") + 1])
    pcd = segment_object(folder, remove_plane=remove_plane, crop=crop)
    print(f"segmented {len(pcd.points)} points from {folder}")
    try:
        o3d.visualization.draw_geometries([pcd], window_name="segmented object")
    except Exception as e:
        out = os.path.join(folder, "segmented.ply")
        o3d.io.write_point_cloud(out, pcd)
        print(f"(no display: {e})\nwrote {out}")


if __name__ == "__main__":
    _main()
