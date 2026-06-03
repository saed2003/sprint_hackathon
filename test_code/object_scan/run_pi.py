#!/usr/bin/env python3
"""
run_pi.py — ONE command, ALL on the Pi: orbit-capture an object AND merge it into a single
coloured point-cloud .ply.  No laptop step, no Open3D, no mesh.

WHY THIS CAN RUN ON THE PI
--------------------------
The normal flow needs Open3D (laptop-only) for the merge. But your friend's
`build_object_pi.py` does the merge in PURE numpy/cv2 — it reuses
`src/new_point_cloud/register_360.py` (CLAHE -> ORB/SIFT -> Kabsch+RANSAC: the pose is
MEASURED from features, not guessed) + a BFS pose chain + fractional loop closure, and
`register_360` only imports Open3D lazily inside functions we never call. So the whole
pipeline finishes on the robot.

This script changes NOTHING — it just glues two pieces that already exist:
  * capture_orbit.capture()  -> the robot orbits the object and writes shot_NN folders
                                (the vision aim + radius-hold + movement you tuned)
  * build_object_pi.*        -> the friend's pure-numpy/cv2 feature merge -> one .ply

NO mesh (you said you don't need it). Output is the merged point cloud only.

RUN (on the Pi — needs RasBot + pyrealsense2 + cv2 + numpy; NO Open3D):
    python3 run_pi.py                         # orbit-scan the default object (db5) -> .ply
    python3 run_pi.py --object teemo
    python3 run_pi.py --shots 36 --radius 0.35
    python3 run_pi.py --sift --win 4          # merge knobs (passed to the friend's merge)
    python3 run_pi.py --build captures/orbit_<ts>   # SKIP capture, just merge an existing scan

Output:  <session>/object_pi.ply   (+ object_pi_preview.png, the friend's cv2 two-panel view)
View it on the Pi:  python3 ../../src/pointcloud/view3d.py <session>/object_pi.ply
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config                       # noqa: E402  the object presets (db5 / teemo)
import build_object_pi as PI        # noqa: E402  the friend's pure-numpy/cv2 merge (no Open3D)

R = PI.R                            # register_360 (feature matcher, pose graph helpers)
G = PI.G                            # geometry (back-project, kabsch, ply IO)


def merge_pi(session, cfg, force_sift=False, window=3, log=print):
    """Merge a captured session into ONE coloured .ply — all numpy/cv2, NO Open3D.

    This mirrors build_object_pi.main() but as a plain callable, so it leaves that file
    untouched. Returns the written .ply path.
    """
    zmin, zmax = cfg["zmin"], cfg["zmax"]
    voxel, crop = cfg["voxel"], cfg["crop"]

    # object-scale registration knobs into the reused register_360 module (closer + cleaner
    # than the room-scan defaults) — the exact values the friend tuned in build_object_pi.
    R.ZMIN, R.ZMAX = zmin, zmax
    R.KP_MAX_STD, R.RANSAC_THRESH = PI.KP_MAX_STD, PI.RANSAC_THRESH
    R.MAX_RMSE, R.MIN_INLIERS = PI.MAX_RMSE, PI.MIN_INLIERS

    dirs = R.shot_dirs(session)                         # shot_*/ir_left.png, sorted
    if not dirs:
        sys.exit(f"no shot_*/ir_left.png in {session}")
    log(f"merge ({cfg['name']}): {len(dirs)} shots, band [{zmin}, {zmax}] m, "
        f"crop {crop*100:.0f}cm, voxel {voxel*1000:.1f}mm, win {window}, "
        f"{'SIFT' if force_sift else 'ORB->SIFT'}")

    # 1. per shot: IR + depth MASKED to the object (so features describe the OBJECT, not the floor)
    shots = []
    for d in dirs:
        s = PI.load_shot_masked(d, zmin, zmax, crop)
        if s is None:
            log(f"  {os.path.basename(d)}: object not found (skipped)")
            continue
        shots.append(s)
    if len(shots) < 2:
        sys.exit("fewer than 2 shots show the object — check the depth band / lighting / scan.")
    log(f"  {len(shots)} shots with a segmented object")

    # 2. windowed feature registration -> pose chain + loop closure (numpy/cv2)
    log("Registering ring pairs:")
    edges = PI.build_edges(shots, force_sift, window, log=log)
    if not edges:
        sys.exit("no pair registered — object too smooth/dark or too little overlap.")
    poses = PI.solve_poses(len(shots), edges, log=log)

    # 3. re-project placed shots into one frame, dedup + cull, save .ply
    log("Rendering:")
    pts, cols = PI.render(shots, poses, zmin, zmax, log=log)
    before = len(pts)
    pts, cols = PI.voxel_downsample(pts, cols, voxel)
    pts, cols = PI.remove_isolated(pts, cols, voxel, min_neighbors=3)
    log(f"  {before} -> {len(pts)} points after voxel/cull")

    ply = os.path.join(session, "object_pi.ply")
    G.save_ply(ply, pts, cols)
    try:
        PI.save_preview(os.path.join(session, "object_pi_preview.png"), pts, cols)
    except Exception as e:                                # noqa: BLE001
        log(f"  (preview skipped: {e})")
    return ply


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        v = cast(args[i + 1])
        del args[i:i + 2]
        return v
    return None


def main():
    args = sys.argv[1:]
    force_sift = "--sift" in args
    if force_sift:
        args.remove("--sift")
    obj = _pop(args, "--object", str)
    shots = _pop(args, "--shots", int)
    radius = _pop(args, "--radius", float)
    window = _pop(args, "--win", int) or 3
    build_only = _pop(args, "--build", str)              # skip capture, merge this session

    # pick the object FIRST: capture_orbit reads config.DEFAULT at import for its tracking gate.
    cfg = config.select(obj) if obj else config.DEFAULT

    if build_only:
        session = build_only
        if not os.path.isdir(session):
            sys.exit(f"--build: not a session folder: {session}")
        print(f"=== merge-only (skip capture): {session} ===")
    else:
        # capture on the Pi (robot orbit). Imported lazily so the merge-only path and the
        # laptop never need RasBot/smbus.
        import capture_orbit                              # noqa: E402  Pi only (pulls RasBot)
        s = shots if shots is not None else cfg["shots"]
        r = radius if radius is not None else cfg["radius"]
        print(f"=== capture: orbit {cfg['name']}  (shots={s}, R={r*100:.0f}cm) ===")
        session = capture_orbit.capture(shots=s, radius=r)

    print(f"=== merge on the Pi -> .ply ===")
    ply = merge_pi(session, cfg, force_sift=force_sift, window=window)
    print(f"\nDONE -> {ply}")
    print(f"  view on the Pi:  python3 "
          f"{os.path.join('..', '..', 'src', 'pointcloud', 'view3d.py')} {ply}")


if __name__ == "__main__":
    main()
