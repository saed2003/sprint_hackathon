# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Street View Robot — a Raspbot V2 (Mecanum wheels, Raspberry Pi 5, Intel RealSense D405) that drives a room and builds a 360° point cloud at each stop. See `docs/` for theory, hardware, and API guides.

## Two Environments

Code runs in two distinct places with different available libraries:

| Environment | Libraries | What runs there |
|---|---|---|
| **Laptop** | `pyrealsense2`, `numpy`, `cv2`, `open3d`, `matplotlib` | `camera/`, `pointcloud/make_pointcloud.py`, `merge_clouds.py`, `render_cloud.py` |
| **Raspberry Pi (robot)** | `pyrealsense2`, `numpy`, `cv2`, `smbus` — **no Open3D** | `wasd/drive.py`, `pointcloud/scan360.py`, `view3d.py`, `setup_and_api/api/` |

**Critical:** `from rasbot.api import RasBot` imports `smbus` — it will crash on the laptop. All Pi-side perception is pure NumPy + OpenCV.

## Setup (Laptop)

```bash
uv venv --python 3.11 .venv          # pyrealsense2 has no wheels for Python 3.14
uv pip install -r requirements.txt
```

Run all scripts from the **project root** (not from inside subdirectories):

```bash
.venv/bin/python camera/capture.py
.venv/bin/python pointcloud/make_pointcloud.py
```

## Common Commands

**Capture frames (laptop, D405 plugged in via USB):**
```bash
.venv/bin/python camera/capture.py          # ENTER = save, q = quit
```

**Build point cloud from a capture (laptop):**
```bash
.venv/bin/python pointcloud/make_pointcloud.py               # newest capture
.venv/bin/python pointcloud/make_pointcloud.py --all         # all captures
.venv/bin/python pointcloud/make_pointcloud.py captures/<ts> # specific folder
```

**ICP merge of a full 360 scan (laptop):**
```bash
.venv/bin/python pointcloud/merge_clouds.py --angle 36 captures/scan_<ts>/shot_*/
```

**View a cloud interactively (laptop, Open3D):**
```bash
.venv/bin/python -c "import open3d as o3d, sys; o3d.visualization.draw_geometries([o3d.io.read_point_cloud(sys.argv[1])])" captures/<ts>/cloud.ply
```

**On the Pi — drive + scan:**
```bash
ssh sprint@sprint.local                    # WiFi SSID: Sprint9
python3 wasd/drive.py                      # WASD/QE drive, R=scan, T=build, Y=view, V=single capture
```

**On the Pi — rebuild a scan without re-driving:**
```bash
python3 pointcloud/scan360.py captures/scan_<ts>              # measured angle (default)
python3 pointcloud/scan360.py captures/scan_<ts> --known      # trust timed step
python3 pointcloud/scan360.py captures/scan_<ts> --angle 36   # force fixed angle
python3 pointcloud/scan360.py --calibrate --turned <degrees>  # calibrate rotation timing
```

**Clean up captures:**
```bash
python3 pointcloud/clean_captures.py        # delete all captures + output
python3 pointcloud/clean_captures.py --clouds  # keep raw camera data, delete generated clouds
```

**Copy scans from Pi to laptop:**
```bash
scp -r sprint@sprint.local:~/sprint_hackathon/captures ~/sprint_hackathon/
```

## Architecture

### Import convention

Every runnable script adds the project root to `sys.path`. Import by package name from root:
```python
from camera.rs_capture import StereoCapture
from pointcloud import scan360
from rasbot.api import RasBot   # Pi only
```

`rasbot/` is a shim that points to `setup_and_api/api/`. On the Pi, `rasbot/api/` must be on the Python path.

### The shared contract: capture folders

The single interface between all producers and consumers:
```
captures/<timestamp>/
├── depth.npy         uint16 raw depth (× depth_scale → meters)
├── depth_color.png   colorized depth
├── ir_left.png       left IR image
├── ir_right.png      right IR image
└── intrinsics.txt    key-value: width height fx fy ppx ppy depth_scale baseline_m
```

A 360 scan is `captures/scan_<ts>/shot_NN/` (each a capture folder) + `merged_360.ply`.

**Rule:** never pass camera data as ad-hoc variables. Write/read capture folders so Pi and laptop tools stay compatible.

### Key files

| File | Runs on | Role |
|---|---|---|
| `camera/rs_capture.py` | Pi/laptop | `StereoCapture` — shared D405 pipeline, produces capture folders |
| `camera/capture.py` | Pi/laptop | ENTER-to-capture REPL |
| `pointcloud/scan360.py` | Pi | **The heart** — 360 timed sweep + angle-aware merge (pure NumPy/cv2) |
| `pointcloud/view3d.py` | Pi | Software 3D orbit viewer (numpy+cv2, no Open3D) |
| `wasd/drive.py` | Pi | **The conductor** — WASD teleop + R/T/Y/V scan workflow |
| `pointcloud/make_pointcloud.py` | laptop | One capture → `cloud.ply` (Open3D) |
| `pointcloud/merge_clouds.py` | laptop | Many captures → `merged.ply` via ICP (Open3D) |
| `setup_and_api/api/robot.py` | Pi | `RasBot` class — all hardware (movement, servos, sensors via I²C) |
| `line_following/line_follow.py` | Pi | **Scaffold only** — Mode 2 IR line-following, not yet tuned |

### Perception pipeline (data flow)

```
D405 camera
  │  rs_capture.py (StereoCapture)
  ▼
captures/<ts>/ { depth.npy, ir_left/right.png, intrinsics.txt }
  │
  ├── make_pointcloud.py  (laptop, Open3D)  → cloud.ply
  │     back-project: Z=depth, X=(u-ppx)Z/fx, Y=(v-ppy)Z/fy
  │
  └── scan360.py  (Pi, numpy+cv2)
        ├── estimate_yaw(): ORB → homography → yaw (merge-only, when no angle.txt)
        ├── back_project(): depth.npy → 3D points
        ├── ry(angle): rotate each view about Y (vertical) into view-0 frame
        ├── voxel_downsample(): 1 cm voxel grid
        └── write_ply() → merged_360.ply
```

### Open-loop rotation (important)

The RasBot has **no IMU or wheel encoders**, so rotation is **purely timed** — the camera is **not** used to steer. `scan360.py` pulses the motors for `SCAN_SEC_PER_DEG * step` between shots (default 10 shots → 36° each). Calibrate `SCAN_SEC_PER_DEG` once per robot/floor so a full turn lands on start: `python3 pointcloud/scan360.py --calibrate --turned <deg>` (battery level and floor grip shift it).

After the last shot it returns to the start heading (`SCAN_RETURN_HOME`); `SCAN_RETURN_MODE` selects how:
- `"forward"` — one more step to finish the 360° circle (short; only lands on start if `SCAN_SEC_PER_DEG` is dialed in).
- `"rewind"` — spin back the exact `shots-1` steps just made (lands on start regardless of calibration, but turns ~324° backward).

Image-based angle recovery (`estimate_yaw`, ORB → homography → yaw) still exists but **only in the merge** (`build_from_session`), to reconstruct per-step angles when a scan folder has no `angle.txt`.

### RasBot API key methods

```python
with RasBot() as bot:               # auto-stops on exit
    bot.forward(speed)              # 0–255
    bot.rotate_left/right(speed)    # in-place spin
    bot.stop()
    bot.read_line_sensors()         # → (left_outer, left_inner, right_inner, right_outer) bool
    bot.read_distance()             # ultrasonic, cm
    frames = bot.capture_all()      # color + depth(mm) + ir_left + ir_right (synced)
    intr   = bot.get_stereo_intrinsics()
```

Pi API: **640×480**, depth in **millimeters**. Laptop `capture.py`: 848×480, raw units. Same back-projection math — mind the units.

## Still to Build

1. **Custom stereo depth** from `ir_left`/`ir_right` with `cv2.StereoSGBM` (the brief's explicit requirement). Write result into `depth.npy` in the capture folder — the rest of the pipeline picks it up automatically.
2. **IR line-following** (`line_following/line_follow.py`) — scaffold exists, needs tuning of thresholds, speeds, and stop-marker debounce on the real robot.
