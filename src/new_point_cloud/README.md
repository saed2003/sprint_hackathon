# new_point_cloud — feature-based 360 reconstruction

Builds one 360° point cloud from a 10-shot spin by **measuring** the camera pose at
each stop from the images, instead of trusting the (uncalibrated, open-loop) turn.

Replaces `src/depth_map/combine_360.py`, which guessed the pose two ways and failed:

- *"trust 36°/step"* smears — the real step is ~40° and drifts (the robot spins ~400°).
- *ICP* locks onto noisy passive-stereo geometry and returns garbage.

## How it works

```
per shot:  CLAHE-enhance ir_left  ->  ORB features  ->  lift each to 3D via depth.npy
per pair :  match  ->  2D fundamental pre-filter  ->  Kabsch+RANSAC on 3D<->3D  ->  T
            (weak ORB pair retried with SIFT — robust to the big ~40° viewpoint jump)
graph    :  exhaustive over all 45 pairs  ->  pose graph
            (consecutive = trusted odometry, others = loop closures)
            ->  Open3D global optimization spreads drift around the ring
render   :  re-project every placed shot into one frame, voxel 5 mm, cull, save .ply
```

No angle seed anywhere — Kabsch solves pose from correspondences with no initial guess.
`angle.txt` is ignored.

## Run (laptop or Pi — needs `cv2`, `numpy`, `open3d`)

```bash
python register_360.py                       # newest scan under ../captures
python register_360.py <session_dir>
python register_360.py <session_dir> --sift  # SIFT every pair (slower, more robust)
python register_360.py <session_dir> --byshot# tint each shot a color (see seams/align)
python register_360.py <session_dir> --gray  # raw IR texture (default colors by depth)
python register_360.py <session_dir> --zmax 3.0 --zmin 0.2
```

Outputs `pointcloud_360.ply` + `pointcloud_360_preview.png` in this folder.
**Default color: RED = close, BLUE = far** (distance from camera). `--gray` for IR texture.

## Reading the output

The run prints a per-pair table, then the **connected components** of the
registration graph. A healthy scan is one component of all 10 shots. If shots are
split across components, the capture (not the code) is the problem — only the
**largest** component is placed; the rest are listed as dropped.

## Tunables (top of `register_360.py`)

| Const | Default | Meaning |
|---|---|---|
| `N_ORB` | 3000 | ORB features/image |
| `CLAHE_CLIP` | 3.0 | contrast boost before detection — the dim IR needs it (see below) |
| `KP_MAX_STD` | 0.20 | drop a keypoint if its 5×5 depth window varies > this (m) |
| `ZMIN, ZMAX` | 0.15, 2.0 | depth band kept (m); D405 is short range, far = noise/`65535` sentinel |
| `FUND_PX` | 2.0 | 2D fundamental RANSAC px (false-match pre-filter) |
| `RANSAC_THRESH` | 0.06 | 3D inlier distance (m); passive-stereo noise is ~5–12 cm |
| `MIN_INLIERS` | 6 | min 3D inliers to accept an edge |
| `MAX_RMSE` | 0.05 | max inlier RMSE to accept an edge (m) |
| `OUT_VOXEL` | 0.005 | final voxel size (m) |

## Why CLAHE matters (the bug that looked like a capture problem)

The D405 IR is **dim and low-contrast** (mean ~55/255). On the raw image ORB's corner
threshold finds almost nothing on the darker shots (~30 keypoints), so those shots fail
to register and the ring breaks — it *looks* like a blank wall but isn't. CLAHE
(adaptive contrast) before detection reveals the real texture (~30× more keypoints,
e.g. 53 → 992) and **every shot registers**. The original gray still colors the cloud;
CLAHE is detection-only.

Both example scans (`scan_20260602_174649`, `scan_20260602_174955`) go from 4/10 placed
to a full 10/10 single component with this on.

## Capture requirements (rare failures now)

CLAHE handles dim IR, so the remaining limits are physical:

- **Truly featureless surface** — a perfectly blank wall has no texture even after CLAHE.
- **Depth range** — the **D405 is short range** (good ~0.1–1 m, noisy by ~2 m). If two
  neighbors only share *far* geometry there is no usable 3D to fit. Keep textured
  surfaces within ~1–2 m.
- **Spin in place**, consistent steps, so every neighbor pair overlaps (FOV ≈ 90°).
- Under-rotating slightly so shot 9 overlaps shot 0 closes the loop (the exhaustive
  matching finds that 0–9 edge automatically).

The log's `components:` line is the health check: **one component with all 10 shots = full
360**. If it splits, check the per-pair table for the shot that won't match.
