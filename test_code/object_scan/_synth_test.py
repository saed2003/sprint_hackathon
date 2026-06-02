"""
No-hardware self-test for the object-scan pipeline.

Renders a synthetic TEEMO-SCALE figure (body + head + hat + an offset mushroom buddy)
sitting on a table, seen from N turntable angles at the configured orbit radius, into
REAL capture folders (depth.npy + intrinsics.txt + color.png + angle.txt) — the exact
format the capturers produce. You then run build_object.py on it.

This lets you validate the whole laptop merge/mesh pipeline with NO camera and NO robot,
at the real object scale (~9 cm figure at ~40 cm). It's a test fixture, not the robot.

    python _synth_test.py
    python _synth_test.py --shots 24
"""
import os
import sys
import time

import numpy as np
import cv2

try:
    from config import DEFAULT as CFG
except Exception:
    CFG = dict(radius=0.40, shots=24, fig_height=0.09, fig_width=0.07, fig_depth=0.06, buddy=0.025)

# ── synthetic camera (D405-ish 848x480) ──────────────────────────────────────────
W, H = 848, 480
FX = FY = 422.0
PPX, PPY = 424.0, 240.0
DEPTH_SCALE = 0.001                       # depth.npy units = millimetres
CENTER = np.array([0.0, 0.0, CFG["radius"]])   # figure centre at the orbit radius


def _fib_dirs(n):
    """n roughly-uniform unit directions (fibonacci sphere)."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], axis=1)


def _ellipsoid(n, radii, center):
    """n points on an ellipsoid with outward normals."""
    d = _fib_dirs(n)
    pts = d * radii + center
    nrm = d / (np.array(radii) ** 2)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return pts, nrm


def _box(n, size, center):
    """~n points over the 6 faces of a box (size = (sx,sy,sz)), axis-aligned normals."""
    size = np.array(size, float)
    per = max(1, n // 6)
    pts, nrm = [], []
    for axis in range(3):
        for s in (-1, 1):
            uv = (np.random.rand(per, 2) - 0.5)
            p = np.zeros((per, 3)); p[:, axis] = s * size[axis] / 2
            other = [a for a in range(3) if a != axis]
            p[:, other[0]] = uv[:, 0] * size[other[0]]
            p[:, other[1]] = uv[:, 1] * size[other[1]]
            nv = np.zeros((per, 3)); nv[:, axis] = s
            pts.append(p + center); nrm.append(nv)
    return np.concatenate(pts), np.concatenate(nrm)


def build_figure():
    """A Teemo-ish asymmetric figure + mushroom buddy, centred at CENTER (camera frame,
    y is DOWN so 'up' is -y). Returns (points, normals, colors uint8 RGB)."""
    np.random.seed(0)
    fw, fh, fd = CFG["fig_width"], CFG["fig_height"], CFG["fig_depth"]
    parts = []

    # body (green), lower
    p, n = _ellipsoid(26000, (fw * 0.5, fh * 0.35, fd * 0.5), CENTER + [0, fh * 0.12, 0])
    parts.append((p, n, (70, 160, 80)))
    # head (tan), upper
    p, n = _ellipsoid(14000, (fw * 0.34, fh * 0.26, fd * 0.34), CENTER + [0, -fh * 0.30, 0])
    parts.append((p, n, (215, 185, 150)))
    # hat brim (red), asymmetric (pushed forward a touch)
    p, n = _box(9000, (fw * 0.85, fh * 0.06, fd * 0.85), CENTER + [0, -fh * 0.46, fd * 0.06])
    parts.append((p, n, (200, 60, 55)))
    # one ear/feather (red) — breaks symmetry so ICP can lock the rotation
    p, n = _box(5000, (fw * 0.16, fh * 0.34, fd * 0.16), CENTER + [fw * 0.30, -fh * 0.62, 0])
    parts.append((p, n, (205, 70, 60)))
    # mushroom buddy beside it: red cap + pale stem
    p, n = _ellipsoid(4000, (CFG["buddy"] * 0.6,) * 3, CENTER + [fw * 1.05, fh * 0.02, 0])
    parts.append((p, n, (210, 55, 50)))
    p, n = _box(2500, (CFG["buddy"] * 0.4, CFG["buddy"] * 0.7, CFG["buddy"] * 0.4),
                CENTER + [fw * 1.05, fh * 0.20, 0])
    parts.append((p, n, (235, 230, 215)))

    pts = np.concatenate([p for p, _, _ in parts])
    nrm = np.concatenate([n for _, n, _ in parts])
    cols = np.concatenate([np.tile(c, (len(p), 1)) for p, _, c in parts]).astype(np.uint8)
    return pts, nrm, cols


def build_car():
    """A DB5-ish asymmetric matte car (body + offset cabin + front detail + 4 wheels),
    centred at CENTER (camera frame, y is DOWN). Front != back + distinct sides make it
    easy for ICP to lock the rotation, and a solid body meshes cleanly. Light-grey body
    like the real 76911."""
    np.random.seed(1)
    L, W, Hc = CFG["car_length"], CFG["car_width"], CFG["car_height"]
    parts = []
    # lower body (silver), full length
    p, n = _box(20000, (L * 0.96, Hc * 0.5, W), CENTER + [0, Hc * 0.10, 0])
    parts.append((p, n, (180, 182, 188)))
    # cabin/greenhouse (darker windows), shorter and pushed toward the REAR (-x) => asymmetric
    p, n = _box(9000, (L * 0.42, Hc * 0.45, W * 0.9), CENTER + [-L * 0.06, -Hc * 0.32, 0])
    parts.append((p, n, (45, 50, 62)))
    # front detail (grille/bumper, chrome) at +x => front clearly differs from back
    p, n = _box(3000, (L * 0.06, Hc * 0.30, W * 0.8), CENTER + [L * 0.49, Hc * 0.06, 0])
    parts.append((p, n, (210, 212, 216)))
    # 4 wheels (near-black), stick out a touch in z
    r = Hc * 0.42
    for sx in (+1, -1):
        for sz in (+1, -1):
            p, n = _ellipsoid(2600, (r, r, r * 0.55),
                              CENTER + [sx * L * 0.34, Hc * 0.30, sz * W * 0.52])
            parts.append((p, n, (28, 28, 30)))
    pts = np.concatenate([p for p, _, _ in parts])
    nrm = np.concatenate([n for _, n, _ in parts])
    cols = np.concatenate([np.tile(c, (len(p), 1)) for p, _, c in parts]).astype(np.uint8)
    return pts, nrm, cols


def build_object_model():
    """The active object (per config 'shape'), WITHOUT the table."""
    return build_car() if CFG.get("shape") == "car" else build_figure()


def add_table(pts, nrm, cols):
    """A horizontal plane just under the object (y is DOWN) to test plane removal."""
    g = 28000
    tx = (np.random.rand(g) - 0.5) * 0.6
    tz = (np.random.rand(g) - 0.5) * 0.6 + CENTER[2]
    ty = np.full(g, pts[:, 1].max() + 0.003)              # just below the object's lowest point
    tp = np.stack([tx, ty, tz], axis=1)
    tn = np.tile([0, -1, 0], (g, 1)).astype(float)
    tc = np.tile([120, 120, 125], (g, 1)).astype(np.uint8)
    return np.concatenate([pts, tp]), np.concatenate([nrm, tn]), np.concatenate([cols, tc])


def ry_about(center, angle_deg):
    a = np.radians(angle_deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def render_view(pts, nrm, cols, angle_deg):
    """Rotate the scene about the figure's vertical axis, keep front-facing points,
    z-buffer rasterize to a depth image + colour image (camera at origin)."""
    R = ry_about(CENTER, angle_deg)
    p = (pts - CENTER) @ R.T + CENTER
    nv = nrm @ R.T
    visible = np.einsum('ij,ij->i', nv, p) < 0
    p, c = p[visible], cols[visible]
    Z = p[:, 2]
    u = np.round(FX * p[:, 0] / Z + PPX).astype(int)
    v = np.round(FY * p[:, 1] / Z + PPY).astype(int)
    inb = (u >= 0) & (u < W) & (v >= 0) & (v < H) & (Z > 0.05) & (Z < 0.7)
    u, v, Z, c = u[inb], v[inb], Z[inb], c[inb]

    depth = np.full((H, W), np.inf)
    color = np.zeros((H, W, 3), np.uint8)
    order = np.argsort(-Z)
    depth[v[order], u[order]] = Z[order]
    color[v[order], u[order]] = c[order][:, ::-1]          # store BGR for cv2
    depth[~np.isfinite(depth)] = 0
    depth += (depth > 0) * np.random.normal(0, 0.0007, depth.shape)
    return (depth / DEPTH_SCALE).astype(np.uint16), color


def write_intrinsics(path):
    with open(path, "w") as f:
        f.write(f"width {W}\nheight {H}\nfx {FX}\nfy {FY}\nppx {PPX}\nppy {PPY}\n")
        f.write(f"depth_scale {DEPTH_SCALE}\nbaseline_m 0.018\n")


def make_session(shots=None, out_root=None, log=print):
    shots = shots or CFG["shots"]
    step = 360.0 / shots
    obj = build_object_model()                            # object only (for an honest size log)
    pts, nrm, cols = add_table(*obj)
    out_root = out_root or os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")
    session = os.path.join(out_root, "synth_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    size = (obj[0].max(0) - obj[0].min(0)) * 100
    log(f"rendering {shots} synthetic views of '{CFG['name']}' "
        f"(~{size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f}cm) at R={CENTER[2]*100:.0f}cm -> {session}")
    for i in range(shots):
        depth_raw, color = render_view(pts, nrm, cols, -i * step)   # matches build's +angle prior
        folder = os.path.join(session, f"shot_{i:02d}")
        os.makedirs(folder, exist_ok=True)
        np.save(os.path.join(folder, "depth.npy"), depth_raw)
        cv2.imwrite(os.path.join(folder, "color.png"), color)
        cv2.imwrite(os.path.join(folder, "ir_left.png"), cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        write_intrinsics(os.path.join(folder, "intrinsics.txt"))
        with open(os.path.join(folder, "angle.txt"), "w") as f:
            f.write(f"{i*step:.3f}\n")
    return session


def main():
    args = sys.argv[1:]
    shots = int(args[args.index("--shots") + 1]) if "--shots" in args else None
    session = make_session(shots=shots)
    print(f"\nnow build it:\n  python build_object.py {session} --mesh")
    return session


if __name__ == "__main__":
    main()
