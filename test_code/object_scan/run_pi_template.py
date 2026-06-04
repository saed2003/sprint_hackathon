import os
import sys
import math
import numpy as np
import cv2
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config
import build_object_pi as PI
import run_pi2 as M
G = PI.G
LEEWAY = 0.025
YAW_STEP = 30
FIT_MIN = 0.45
ICP_ITERS = 25

def load_template(path):
    if not os.path.exists(path):
        sys.exit(f'template not found: {path} (make it once on the laptop — see the header).')
    d = np.load(path)
    pts = d['pts'].astype(np.float64)
    return {'pts': pts, 'nrm': M._normals(pts), 'query': M._make_query(pts), 'center': np.median(pts, axis=0)}

def anchor_view0(reg0, tpl, leeway, iters, log=print):
    v0c = reg0['centroid']
    base = np.eye(4)
    base[:3, 3] = tpl['center'] - v0c
    best = (None, -1.0, 9.9)
    for yaw in range(0, 360, YAW_STEP):
        init = M._ry_about(tpl['center'], yaw) @ base
        T, fit, rmse = M.icp_p2pl(reg0['pts'], tpl['pts'], tpl['nrm'], tpl['query'], init, [leeway * 2.5, leeway * 1.5, leeway], iters)
        if fit > best[1]:
            best = (T, fit, rmse)
    log(f'  anchor view0: best fit {best[1]:.2f} (rmse {best[2] * 1000:.0f}mm)')
    return best[0]

def merge_template(session, cfg, tpl, force_sift=False, leeway=LEEWAY, reg_voxel=0.003, icp_iters=ICP_ITERS, fit_min=FIT_MIN, log=print):
    zmin, zmax, voxel, crop = (cfg['zmin'], cfg['zmax'], cfg['voxel'], cfg['crop'])
    obj_h = cfg.get('car_height') or cfg.get('fig_height') or 0.08
    R = PI.R
    R.ZMIN, R.ZMAX = (zmin, zmax)
    R.KP_MAX_STD, R.RANSAC_THRESH = (PI.KP_MAX_STD, PI.RANSAC_THRESH)
    R.MAX_RMSE, R.MIN_INLIERS = (PI.MAX_RMSE, PI.MIN_INLIERS)
    dirs = R.shot_dirs(session)
    if not dirs:
        sys.exit(f'no shot_*/ir_left.png in {session}')
    log(f"template merge ({cfg['name']}): {len(dirs)} shots | leeway {leeway * 1000:.0f}mm | NN {('kdtree' if M._HAVE_KD else 'numpy')}")
    shots, kept, regs = ([], [], [])
    for d in dirs:
        s = PI.load_shot_masked(d, zmin, zmax, crop)
        if s is None:
            continue
        s = M.crop_to_car(s, obj_h)
        rc = M.reg_cloud(s, zmin, zmax, reg_voxel)
        if rc is None:
            continue
        shots.append(s)
        kept.append(d)
        regs.append(rc)
    n = len(shots)
    if n < 2:
        sys.exit('fewer than 2 usable views.')
    angles = [M._read_angle(d) if M._read_angle(d) is not None else i * 360.0 / n for i, d in enumerate(kept)]
    center = np.median(np.array([rc['centroid'] for rc in regs]), axis=0)
    md = [leeway * 2.0, leeway * 1.3, leeway]
    T0 = anchor_view0(regs[0], tpl, leeway, icp_iters, log=log)
    poses, placed, dropped = ({}, [], [])
    for i in range(n):
        init = T0 @ M._ry_about(center, angles[i] - angles[0])
        T, fit, rmse = M.icp_p2pl(regs[i]['pts'], tpl['pts'], tpl['nrm'], tpl['query'], init, md, icp_iters)
        if fit >= fit_min:
            poses[i] = T
            placed.append(i)
        else:
            dropped.append(i)
    log(f'  placed {len(placed)}/{n} views onto the template' + (f'; dropped {dropped}' if dropped else ''))
    if len(placed) < 2:
        sys.exit('too few views matched the template — loosen --leeway or check the scan.')
    pts, cols = PI.render(shots, poses, zmin, zmax, log=log)
    before = len(pts)
    pts, cols = PI.voxel_downsample(pts, cols, voxel)
    pts, cols = PI.remove_isolated(pts, cols, voxel, min_neighbors=3)
    log(f'  {before} -> {len(pts)} points after voxel/cull')
    ply = os.path.join(session, 'object_pi_tpl.ply')
    G.save_ply(ply, pts, cols)
    try:
        PI.save_preview(os.path.join(session, 'object_pi_tpl_preview.png'), pts, cols)
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
    leeway = _pop(args, '--leeway', float) or LEEWAY
    fit_min = _pop(args, '--fit-min', float) or FIT_MIN
    template = _pop(args, '--template', str) or os.path.join(HERE, 'db5_template.npz')
    build_only = _pop(args, '--build', str)
    cfg = config.select(obj) if obj else config.DEFAULT
    tpl = load_template(template)
    if build_only:
        session = build_only
        if not os.path.isdir(session):
            sys.exit(f'--build: not a session folder: {session}')
        print(f'=== template merge-only: {session} ===')
    else:
        import capture_orbit
        s = shots if shots is not None else cfg['shots']
        r = radius if radius is not None else cfg['radius']
        print(f"=== capture: orbit {cfg['name']} (shots={s}, R={r * 100:.0f}cm) ===")
        session = capture_orbit.capture(shots=s, radius=r)
    ply = merge_template(session, cfg, tpl, force_sift=force_sift, leeway=leeway, fit_min=fit_min)
    print(f'\nDONE -> {ply}')
    print(f"  view on the Pi:  python3 {os.path.join('..', '..', 'src', 'pointcloud', 'view3d.py')} {ply}")
if __name__ == '__main__':
    main()
