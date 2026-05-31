"""
Render an existing cloud.ply from sensible viewpoints, to show it IS coherent.
Usage: .venv/bin/python render_cloud.py captures/<timestamp>/cloud.ply
"""
import sys, glob, os
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ply = sys.argv[1] if len(sys.argv) > 1 else max(
    glob.glob("captures/*/cloud.ply"), key=os.path.getmtime)
pcd = o3d.io.read_point_cloud(ply)
p = np.asarray(pcd.points)

# drop a few far outliers so the color scale isn't dominated by stragglers
zlo, zhi = np.percentile(p[:, 2], [2, 98])
m = (p[:, 2] >= zlo) & (p[:, 2] <= zhi)
p = p[m]
# downsample for a fast plot
idx = np.random.choice(len(p), size=min(len(p), 40000), replace=False)
p = p[idx]
X, Y, Z = p[:, 0], p[:, 1], p[:, 2]

fig = plt.figure(figsize=(12, 5))

# LEFT: FRONT view — exactly how the camera looked. X right, -Y up, color = depth.
# This is the coherent one: it looks like the scene.
ax1 = fig.add_subplot(121)
ax1.scatter(X, -Y, c=Z, cmap="turbo", s=1, linewidths=0)
ax1.set_aspect("equal"); ax1.set_title("FRONT view (camera's eye)  color = distance")
ax1.set_xlabel("X (m)"); ax1.set_ylabel("-Y (m)")

# RIGHT: TOP-DOWN view — looking down from above. Shows the room has real depth/layout.
ax2 = fig.add_subplot(122)
ax2.scatter(X, Z, c=Z, cmap="turbo", s=1, linewidths=0)
ax2.set_aspect("equal"); ax2.set_title("TOP-DOWN view  (depth from above)")
ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z depth (m)")

out = os.path.join(os.path.dirname(ply), "cloud_views.png")
plt.tight_layout(); plt.savefig(out, dpi=110); plt.close()
print("saved ->", out)
