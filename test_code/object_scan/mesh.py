"""
Point cloud -> watertight, textured SURFACE MESH  (the presentation 'wow').

A merged point cloud already looks good, but a solid mesh is what makes an audience
react. This wraps Open3D's Poisson surface reconstruction (best for closed objects
scanned all the way around) with a Ball-Pivoting fallback (better for open/partial
scans). Laptop-only.

    python mesh.py <merged_object.ply>
    python mesh.py <merged_object.ply> --bpa        # force ball-pivoting
    python mesh.py <merged_object.ply> --depth 10   # finer Poisson (slower)
"""
import os
import sys

import numpy as np
import open3d as o3d


def _prep(pcd):
    """Ensure normals exist and are CONSISTENTLY oriented (Poisson needs this)."""
    if not pcd.has_normals():
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    try:
        pcd.orient_normals_consistent_tangent_plane(30)   # global, for all-around scans
    except Exception:
        pcd.orient_normals_towards_camera_location(pcd.get_center())
    return pcd


def poisson_mesh(ply_path, depth=9, density_quantile=0.03, out_path=None):
    """Poisson reconstruction; trims the lowest-density (least-supported) vertices
    so the mesh doesn't balloon into blobs where there were no points."""
    pcd = _prep(o3d.io.read_point_cloud(ply_path))
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth)
    densities = np.asarray(densities)
    keep = densities > np.quantile(densities, density_quantile)
    mesh.remove_vertices_by_mask(~keep)
    mesh.compute_vertex_normals()
    out = out_path or ply_path.replace(".ply", "_mesh.ply")
    o3d.io.write_triangle_mesh(out, mesh)
    print(f"poisson mesh: {len(mesh.triangles)} triangles -> {out}")
    return out


def bpa_mesh(ply_path, radii=None, out_path=None):
    """Ball-Pivoting reconstruction — keeps to the actual points (no invented blobs),
    good for partial/open scans. radii in metres scaled to the cloud's point spacing."""
    pcd = _prep(o3d.io.read_point_cloud(ply_path))
    if radii is None:
        d = np.mean(pcd.compute_nearest_neighbor_distance())
        radii = [d * 1.5, d * 3.0, d * 6.0]
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))
    mesh.compute_vertex_normals()
    out = out_path or ply_path.replace(".ply", "_mesh.ply")
    o3d.io.write_triangle_mesh(out, mesh)
    print(f"ball-pivoting mesh: {len(mesh.triangles)} triangles -> {out}")
    return out


def _main():
    args = sys.argv[1:]
    if not args:
        sys.exit("usage: python mesh.py <merged_object.ply> [--bpa] [--depth 9]")
    ply = args[0]
    if "--bpa" in args:
        bpa_mesh(ply)
    else:
        depth = int(args[args.index("--depth") + 1]) if "--depth" in args else 9
        poisson_mesh(ply, depth=depth)


if __name__ == "__main__":
    _main()
