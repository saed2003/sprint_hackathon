"""
Interactive 3D point-cloud viewer that runs ON THE RASPBOT (Pi) — pure numpy + cv2,
no Open3D / matplotlib needed (neither has a Raspberry Pi wheel here).

Opens a .ply (the binary colored format written by scan360.write_ply) in an OpenCV
window and lets you orbit it in real 3D.

Controls
  Mouse drag        orbit (yaw / pitch)
  Mouse wheel       zoom in / out
  Arrow keys        orbit  (UP/DOWN = pitch, LEFT/RIGHT = yaw)
  + / -             zoom in / out
  [ / ]             smaller / bigger points
  R                 reset the view
  ESC / Q           close the window

Run standalone (newest scan by default):
  python3 pointcloud/view3d.py
  python3 pointcloud/view3d.py captures/scan_20260531_1700/merged_360.ply

Or it is launched for you from wasd/drive.py (press Y after building a cloud with T).
"""

import os
import sys
import glob

import numpy as np
import cv2

# captures/ lives at the project root (one level up from pointcloud/)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ── .ply reader (matches scan360.write_ply: binary_little_endian, xyz + rgb) ──────

def read_ply(path):
    """Read a binary-little-endian colored .ply -> (pts Nx3 float32, cols Nx3 uint8)."""
    with open(path, "rb") as f:
        # header is ascii, ends at 'end_header\n'
        header = b""
        while b"end_header\n" not in header:
            chunk = f.readline()
            if not chunk:
                raise ValueError(f"{path}: not a valid .ply (no end_header)")
            header += chunk
        text = header.decode("ascii", "replace")
        if "binary_little_endian" not in text:
            raise ValueError(f"{path}: only binary_little_endian .ply is supported")
        n = 0
        for line in text.splitlines():
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("r", "u1"), ("g", "u1"), ("b", "u1")])
        arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    cols = np.stack([arr["r"], arr["g"], arr["b"]], axis=1).astype(np.uint8)
    return pts, cols


# ── tiny orbit camera + numpy renderer ────────────────────────────────────────────

class Viewer:
    def __init__(self, pts, cols, w=960, h=720):
        self.w, self.h = w, h
        # center the cloud at its centroid so it orbits about itself
        self.center = pts.mean(axis=0)
        self.pts = pts - self.center
        # OpenCV uses BGR; our colors are RGB
        self.cols = cols[:, ::-1].copy()
        # a sensible starting distance from the cloud's spread
        self.span = float(np.percentile(np.linalg.norm(self.pts, axis=1), 95)) or 1.0
        self.reset()
        self.dragging = False
        self.last = (0, 0)

    def reset(self):
        self.yaw = 0.0
        self.pitch = 0.0
        self.dist = self.span * 2.5
        self.psize = 2

    def _rot(self):
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        cx, sx = np.cos(self.pitch), np.sin(self.pitch)
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], np.float32)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], np.float32)
        return Rx @ Ry

    def render(self):
        img = np.zeros((self.h, self.w, 3), np.uint8)
        cam = self.pts @ self._rot().T
        cam[:, 2] += self.dist
        z = cam[:, 2]
        valid = z > 1e-3
        f = 0.9 * self.h                      # focal length in pixels
        u = (f * cam[:, 0] / z + self.w / 2)
        v = (f * cam[:, 1] / z + self.h / 2)
        on = (valid & (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h))
        u, v, z, cols = u[on].astype(np.int32), v[on].astype(np.int32), z[on], self.cols[on]
        # paint far points first so near points land on top (cheap z-order)
        order = np.argsort(-z)
        u, v, cols = u[order], v[order], cols[order]
        ps = self.psize
        if ps <= 1:
            img[v, u] = cols
        else:
            for du in range(ps):
                for dv in range(ps):
                    vv, uu = v + dv, u + du
                    m = (vv < self.h) & (uu < self.w)
                    img[vv[m], uu[m]] = cols[m]
        self._hud(img, on.sum())
        return img

    def _hud(self, img, shown):
        lines = [
            f"points: {len(self.pts):,}  shown: {shown:,}",
            "drag=orbit  wheel/+-=zoom  [ ]=point size  R=reset  ESC/Q=quit",
        ]
        for i, t in enumerate(lines):
            cv2.putText(img, t, (10, 24 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (60, 220, 60), 1, cv2.LINE_AA)

    def on_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging, self.last = True, (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            dx, dy = x - self.last[0], y - self.last[1]
            self.last = (x, y)
            self.yaw += dx * 0.01
            self.pitch = float(np.clip(self.pitch + dy * 0.01, -1.5, 1.5))
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.dist *= 0.9 if flags > 0 else 1.1
            self.dist = float(np.clip(self.dist, self.span * 0.2, self.span * 20))


def view(ply, w=960, h=720):
    pts, cols = read_ply(ply)
    if len(pts) == 0:
        print(f"{ply}: empty cloud, nothing to show")
        return
    vw = Viewer(pts, cols, w, h)
    win = f"3D point cloud — {os.path.basename(ply)}"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, vw.on_mouse)
    print(f"showing {len(pts):,} points from {ply}")
    while True:
        cv2.imshow(win, vw.render())
        k = cv2.waitKey(16) & 0xFF
        if k in (27, ord('q')):                       # ESC / q
            break
        elif k in (ord('+'), ord('=')):
            vw.dist = max(vw.dist * 0.9, vw.span * 0.2)
        elif k == ord('-'):
            vw.dist = min(vw.dist * 1.1, vw.span * 20)
        elif k == ord('['):
            vw.psize = max(1, vw.psize - 1)
        elif k == ord(']'):
            vw.psize = min(6, vw.psize + 1)
        elif k == ord('r'):
            vw.reset()
        elif k == 81:   vw.yaw -= 0.08                 # left  arrow
        elif k == 83:   vw.yaw += 0.08                 # right arrow
        elif k == 82:   vw.pitch = float(np.clip(vw.pitch - 0.08, -1.5, 1.5))   # up
        elif k == 84:   vw.pitch = float(np.clip(vw.pitch + 0.08, -1.5, 1.5))   # down
        # window closed with the X button
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyWindow(win)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if args:
        ply = args[0]
    else:
        cands = (glob.glob(os.path.join(ROOT, "captures", "scan_*", "merged_360.ply"))
                 + glob.glob(os.path.join(ROOT, "captures", "*", "cloud.ply")))
        if not cands:
            sys.exit("No cloud found. Pass a .ply path, e.g.\n"
                     "  python3 pointcloud/view3d.py captures/scan_<ts>/merged_360.ply")
        ply = max(cands, key=os.path.getmtime)
    if not os.path.exists(ply):
        sys.exit(f"No such file: {ply}")
    view(ply)


if __name__ == "__main__":
    main()
