import os
import sys
import math
import numpy as np
import cv2
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config
import build_object_pi as PI
R = PI.R
G = PI.G
try:
    from scipy.spatial import cKDTree
    _HAVE_KD = True
except Exception:
    _HAVE_KD = False
REG_VOXEL = 0.005
ICP_ITERS = 14
WINDOW = 3
RELAX_ITERS = 0
PRIOR_TOL = 35.0
FIT_MIN = 0.3
NORMAL_K = 12

def _ry_about(center, deg):
    a = math.radians(deg)
    c, s = (math.cos(a), math.sin(a))
    Rm = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = center - Rm @ center
    return T

def _rel_prior(center, angles, i, j):
    return np.linalg.inv(_ry_about(center, angles[j])) @ _ry_about(center, angles[i])

def _rot_deg(Ra, Rb):
    c = (np.trace(Ra @ Rb.T) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))

def _read_angle(folder):
    p = os.path.join(folder, 'angle.txt')
    try:
        with open(p) as f:
            return float(f.read().split()[0])
    except Exception:
        return None

def _voxel_down(pts, voxel):
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]

def _normals(pts, k=NORMAL_K):
    n = len(pts)
    out = np.zeros((n, 3))
    k = min(k, n)
    for s in range(0, n, 1024):
        blk = pts[s:s + 1024]
        d2 = ((blk[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
        idx = np.argpartition(d2, k - 1, axis=1)[:, :k]
        nb = pts[idx]
        nb = nb - nb.mean(1, keepdims=True)
        cov = np.einsum('bki,bkj->bij', nb, nb)
        _, v = np.linalg.eigh(cov)
        out[s:s + len(blk)] = v[:, :, 0]
    return out

def _make_query(tgt):
    if _HAVE_KD:
        tree = cKDTree(tgt)
        return lambda S: tree.query(S)

    def q(S):
        n = len(S)
        dist = np.empty(n)
        idx = np.empty(n, np.int64)
        for s in range(0, n, 2048):
            blk = S[s:s + 2048]
            d2 = ((blk[:, None, :] - tgt[None, :, :]) ** 2).sum(-1)
            j = np.argmin(d2, axis=1)
            idx[s:s + len(blk)] = j
            dist[s:s + len(blk)] = np.sqrt(d2[np.arange(len(blk)), j])
        return (dist, idx)
    return q

def crop_to_car(shot, obj_height, margin=0.03):
    depth = shot['depth_m']
    ys, xs = np.where(depth > 0)
    if len(ys) < 60:
        return shot
    intr = shot['intr']
    z = depth[ys, xs]
    y3d = (ys - intr['ppy']) * z / intr['fy']
    ytop = float(np.percentile(y3d, 2))
    band = y3d <= ytop + obj_height + margin
    if int(band.sum()) < 60:
        return shot
    m = np.zeros(depth.shape, bool)
    m[ys[band], xs[band]] = True
    out = dict(shot)
    out['gray'] = np.where(m, shot['gray'], 0).astype(shot['gray'].dtype)
    out['depth_m'] = np.where(m, depth, 0).astype(depth.dtype)
    out['mask'] = m
    return out

def reg_cloud(shot, zmin, zmax, voxel):
    depth = G.drop_depth_edges(shot['depth_m'])
    pts, _ = G.back_project_dense(depth, shot['gray'], shot['intr'], zmin, zmax)
    if len(pts) < 60:
        return None
    pts = _voxel_down(pts.astype(np.float64), voxel)
    if len(pts) < 60:
        return None
    return {'pts': pts, 'nrm': _normals(pts), 'query': _make_query(pts), 'centroid': np.median(pts, axis=0)}

def _fitness(src, T, query, md):
    S = src @ T[:3, :3].T + T[:3, 3]
    d, _ = query(S)
    return float((d < md).mean())

def icp_p2pl(src, tgt, tgt_n, query, init, max_dists, iters):
    T = init.astype(np.float64).copy()
    per = max(1, iters // len(max_dists))
    for md in max_dists:
        for _ in range(per):
            S = src @ T[:3, :3].T + T[:3, 3]
            dist, idx = query(S)
            m = dist < md
            if int(m.sum()) < 10:
                break
            p = S[m]
            q = tgt[idx[m]]
            nrm = tgt_n[idx[m]]
            A = np.concatenate([np.cross(p, nrm), nrm], axis=1)
            b = -np.einsum('ij,ij->i', p - q, nrm)
            try:
                x = np.linalg.solve(A.T @ A + 1e-09 * np.eye(6), A.T @ b)
            except np.linalg.LinAlgError:
                break
            w, t = (x[:3], x[3:])
            dR, _ = cv2.Rodrigues(w.reshape(3, 1))
            dT = np.eye(4)
            dT[:3, :3] = dR
            dT[:3, 3] = t
            T = dT @ T
            if np.linalg.norm(w) < 0.0001 and np.linalg.norm(t) < 0.0001:
                break
    S = src @ T[:3, :3].T + T[:3, 3]
    dist, _ = query(S)
    m = dist < max_dists[-1]
    fit = float(m.mean())
    rmse = float(np.sqrt((dist[m] ** 2).mean())) if m.any() else 9.9
    return (T, fit, rmse)

def build_edges_hq(regs, shots, center, angles, voxel, window, icp_iters, prior_tol, fit_min, force_sift, loop=True, log=print):
    n = len(regs)
    md = [voxel * 6, voxel * 3]
    pairs = sorted({(min(i, (i + dj) % n), max(i, (i + dj) % n)) for i in range(n) for dj in range(1, window + 1) if (i + dj) % n != i})
    edges = {}
    nf = nfold = 0
    for a, b in pairs:
        if not loop and b - a > window:
            continue
        if regs[a] is None or regs[b] is None:
            continue
        prior_T = _rel_prior(center, angles, a, b)
        res, det = R.register_pair_robust(shots[a], shots[b], force_sift)
        if res is not None and _rot_deg(res['T'][:3, :3], prior_T[:3, :3]) <= prior_tol:
            init, tag = (res['T'], det)
            nf += 1
        elif res is not None:
            init, tag = (prior_T, 'fold->prior')
            nfold += 1
        else:
            init, tag = (prior_T, 'prior')
        T, fit, rmse = icp_p2pl(regs[a]['pts'], regs[b]['pts'], regs[b]['nrm'], regs[b]['query'], init, md, icp_iters)
        fit0 = _fitness(regs[a]['pts'], init, regs[b]['query'], md[-1])
        if fit0 > fit:
            T, fit, rmse = (init, fit0, rmse)
        if fit < fit_min:
            log(f'  {a:2d}-{b:2d} [{tag}] reject (fit {fit:.2f})')
            continue
        edges[a, b] = {'T': T, 'n': int(fit * len(regs[a]['pts'])), 'rmse': rmse}
        log(f'  {a:2d}-{b:2d} [{tag}] fit {fit:.2f} rmse {rmse * 1000:.0f}mm')
    log(f'  {len(edges)} edges ({nf} feature, {nfold} folds vetoed)')
    return edges

def _avg_se3(mats):
    rs = np.array([cv2.Rodrigues(M[:3, :3])[0].ravel() for M in mats])
    ts = np.array([M[:3, 3] for M in mats])
    R0, _ = cv2.Rodrigues(np.median(rs, axis=0).reshape(3, 1))
    out = np.eye(4)
    out[:3, :3] = R0
    out[:3, 3] = np.median(ts, axis=0)
    return out

def relax_poses(poses, edges, iters, anchor, log=print):
    if iters <= 0:
        return poses
    adj = {i: [] for i in poses}
    for (a, b), e in edges.items():
        if a in poses and b in poses:
            T = e['T']
            adj[b].append((a, np.linalg.inv(T)))
            adj[a].append((b, T))
    for _ in range(iters):
        for i in poses:
            if i == anchor or not adj[i]:
                continue
            poses[i] = _avg_se3([poses[j] @ M for j, M in adj[i]])
    log(f'  relaxed poses over {len(edges)} edges ({iters} sweeps)')
    return poses

def merge_hq(session, cfg, force_sift=False, window=WINDOW, reg_voxel=REG_VOXEL, icp_iters=ICP_ITERS, relax_iters=RELAX_ITERS, prior_tol=PRIOR_TOL, fit_min=FIT_MIN, crop_stand=True, log=print):
    zmin, zmax, voxel, crop = (cfg['zmin'], cfg['zmax'], cfg['voxel'], cfg['crop'])
    obj_height = cfg.get('car_height') or cfg.get('fig_height') or 0.08
    R.ZMIN, R.ZMAX = (zmin, zmax)
    R.KP_MAX_STD, R.RANSAC_THRESH = (PI.KP_MAX_STD, PI.RANSAC_THRESH)
    R.MAX_RMSE, R.MIN_INLIERS = (PI.MAX_RMSE, PI.MIN_INLIERS)
    dirs = R.shot_dirs(session)
    if not dirs:
        sys.exit(f'no shot_*/ir_left.png in {session}')
    log(f"HQ merge ({cfg['name']}): {len(dirs)} shots | band [{zmin},{zmax}]m | reg-voxel {reg_voxel * 1000:.0f}mm | win {window} | icp {icp_iters} | relax {relax_iters} | {('CROP-STAND(top %.0fcm)' % ((obj_height + 0.03) * 100) if crop_stand else 'keep-stand')} | NN {('scipy-kdtree' if _HAVE_KD else 'numpy')}")
    shots, kept, regs = ([], [], [])
    for d in dirs:
        s = PI.load_shot_masked(d, zmin, zmax, crop)
        if s is None:
            log(f'  {os.path.basename(d)}: object not found (skipped)')
            continue
        if crop_stand:
            s = crop_to_car(s, obj_height)
        rc = reg_cloud(s, zmin, zmax, reg_voxel)
        if rc is None:
            log(f'  {os.path.basename(d)}: too few object points (skipped)')
            continue
        shots.append(s)
        kept.append(d)
        regs.append(rc)
    n = len(shots)
    if n < 2:
        sys.exit('fewer than 2 usable shots — check the depth band / lighting / scan.')
    log(f'  {n} usable views')
    angles = [_read_angle(d) if _read_angle(d) is not None else i * 360.0 / n for i, d in enumerate(kept)]
    center = np.median(np.array([rc['centroid'] for rc in regs]), axis=0)
    log('Registering + ICP-refining ring pairs:')
    edges = build_edges_hq(regs, shots, center, angles, reg_voxel, window, icp_iters, prior_tol, fit_min, force_sift, log=log)
    if not edges:
        sys.exit('no pair survived — object too smooth/dark or too little overlap (try --sift).')
    poses = PI.solve_poses(n, edges, log=log)
    poses = relax_poses(poses, edges, relax_iters, anchor=min(poses), log=log)
    log('Rendering:')
    pts, cols = PI.render(shots, poses, zmin, zmax, log=log)
    before = len(pts)
    pts, cols = PI.voxel_downsample(pts, cols, voxel)
    pts, cols = PI.remove_isolated(pts, cols, voxel, min_neighbors=3)
    log(f'  {before} -> {len(pts)} points after voxel/cull')
    ply = os.path.join(session, 'object_pi_hq.ply')
    G.save_ply(ply, pts, cols)
    try:
        PI.save_preview(os.path.join(session, 'object_pi_hq_preview.png'), pts, cols)
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
    crop_stand = '--keep-stand' not in args
    if '--keep-stand' in args:
        args.remove('--keep-stand')
    obj = _pop(args, '--object', str)
    shots = _pop(args, '--shots', int)
    radius = _pop(args, '--radius', float)
    window = _pop(args, '--win', int) or WINDOW
    reg_voxel = _pop(args, '--reg-voxel', float) or REG_VOXEL
    icp_iters = _pop(args, '--icp-iters', int) or ICP_ITERS
    relax_iters = _pop(args, '--relax-iters', int)
    if relax_iters is None:
        relax_iters = RELAX_ITERS
    prior_tol = _pop(args, '--prior-tol', float) or PRIOR_TOL
    build_only = _pop(args, '--build', str)
    cfg = config.select(obj) if obj else config.DEFAULT
    if build_only:
        session = build_only
        if not os.path.isdir(session):
            sys.exit(f'--build: not a session folder: {session}')
        print(f'=== HQ merge-only (skip capture): {session} ===')
    else:
        import capture_orbit
        s = shots if shots is not None else cfg['shots']
        r = radius if radius is not None else cfg['radius']
        print(f"=== capture: orbit {cfg['name']} (shots={s}, R={r * 100:.0f}cm) ===")
        session = capture_orbit.capture(shots=s, radius=r)
    ply = merge_hq(session, cfg, force_sift=force_sift, window=window, reg_voxel=reg_voxel, icp_iters=icp_iters, relax_iters=relax_iters, prior_tol=prior_tol, crop_stand=crop_stand)
    print(f'\nDONE -> {ply}')
    print(f"  view on the Pi:  python3 {os.path.join('..', '..', 'src', 'pointcloud', 'view3d.py')} {ply}")
if __name__ == '__main__':
    main()
