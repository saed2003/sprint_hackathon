import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config
import build_object_pi as PI
R = PI.R
G = PI.G

def merge_pi(session, cfg, force_sift=False, window=3, log=print):
    zmin, zmax = (cfg['zmin'], cfg['zmax'])
    voxel, crop = (cfg['voxel'], cfg['crop'])
    R.ZMIN, R.ZMAX = (zmin, zmax)
    R.KP_MAX_STD, R.RANSAC_THRESH = (PI.KP_MAX_STD, PI.RANSAC_THRESH)
    R.MAX_RMSE, R.MIN_INLIERS = (PI.MAX_RMSE, PI.MIN_INLIERS)
    dirs = R.shot_dirs(session)
    if not dirs:
        sys.exit(f'no shot_*/ir_left.png in {session}')
    log(f"merge ({cfg['name']}): {len(dirs)} shots, band [{zmin}, {zmax}] m, crop {crop * 100:.0f}cm, voxel {voxel * 1000:.1f}mm, win {window}, {('SIFT' if force_sift else 'ORB->SIFT')}")
    shots = []
    for d in dirs:
        s = PI.load_shot_masked(d, zmin, zmax, crop)
        if s is None:
            log(f'  {os.path.basename(d)}: object not found (skipped)')
            continue
        shots.append(s)
    if len(shots) < 2:
        sys.exit('fewer than 2 shots show the object — check the depth band / lighting / scan.')
    log(f'  {len(shots)} shots with a segmented object')
    log('Registering ring pairs:')
    edges = PI.build_edges(shots, force_sift, window, log=log)
    if not edges:
        sys.exit('no pair registered — object too smooth/dark or too little overlap.')
    poses = PI.solve_poses(len(shots), edges, log=log)
    log('Rendering:')
    pts, cols = PI.render(shots, poses, zmin, zmax, log=log)
    before = len(pts)
    pts, cols = PI.voxel_downsample(pts, cols, voxel)
    pts, cols = PI.remove_isolated(pts, cols, voxel, min_neighbors=3)
    log(f'  {before} -> {len(pts)} points after voxel/cull')
    ply = os.path.join(session, 'object_pi.ply')
    G.save_ply(ply, pts, cols)
    try:
        PI.save_preview(os.path.join(session, 'object_pi_preview.png'), pts, cols)
    except Exception as e:
        log(f'  (preview skipped: {e})')
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
    force_sift = '--sift' in args
    if force_sift:
        args.remove('--sift')
    obj = _pop(args, '--object', str)
    shots = _pop(args, '--shots', int)
    radius = _pop(args, '--radius', float)
    degrees = _pop(args, '--degrees', float) or 360.0
    window = _pop(args, '--win', int) or 3
    build_only = _pop(args, '--build', str)
    cfg = config.select(obj) if obj else config.DEFAULT
    if build_only:
        session = build_only
        if not os.path.isdir(session):
            sys.exit(f'--build: not a session folder: {session}')
        print(f'=== merge-only (skip capture): {session} ===')
    else:
        import capture_orbit
        s = shots if shots is not None else cfg['shots']
        r = radius if radius is not None else cfg['radius']
        print(f"=== capture: orbit {cfg['name']}  (shots={s}, R={r * 100:.0f}cm, {degrees:.0f}°) ===")
        session = capture_orbit.capture(shots=s, radius=r, degrees=degrees)
    print(f'=== merge on the Pi -> .ply ===')
    ply = merge_pi(session, cfg, force_sift=force_sift, window=window)
    print(f'\nDONE -> {ply}')
    print(f"  view on the Pi:  python3 {os.path.join('..', '..', 'src', 'pointcloud', 'view3d.py')} {ply}")
if __name__ == '__main__':
    main()
