# Street View Robot

**Mentor:** Mr. Rajaei Khatib
**Team:** 5 members

---

## Goal

Design a navigation robot that traverses a room and builds **360° panoramic + point cloud** representations of its surroundings at each sampled location.

---

## Overview

A mobile robot that drives around a room and, at each stop, constructs a **3D point cloud** of what it sees. By visiting many locations, it incrementally builds a full scan of the room.

---

## Hardware

Core components for this project:

| Component | Details |
|---|---|
| **Robot platform** | Raspbot V2 educational robot — chassis with 4× Mecanum wheels → omnidirectional movement (forward, back, strafe, diagonal, in-place rotation), I2C motor driver |
| **Compute** | Raspberry Pi 5 — 8 GB |
| **Depth camera** | Intel RealSense D405 — factory-calibrated stereo IR pair |

Onboard Raspbot V2 sensors also used:

- **Ultrasonic rangefinder** — obstacle distance.
- **4× infrared line-tracking sensors** — ground reference / line following.

---

## Setup (do first)

Each group must:

1. Assemble chassis — mount motors, wheels, camera, sensors.
2. Initialize the Raspberry Pi 5:
   - Install OS.
   - Configure I2C + camera interfaces.
   - Set up Python env with **OpenCV**, **NumPy**, **pyrealsense2**.

---

## Core Task

At each location the robot rotates and captures stereo frames. Students design and implement an algorithm to:

1. **Compute depth** from stereo pairs.
2. **Generate 3D point clouds.**
3. **Merge** views from different rotation positions → single **360° point cloud**.

---

## Python API

All hardware abstracted into simple method calls, e.g.:

```python
forward(speed)
capture_stereo()
set_tilt(angle)
```

The system supports **two navigation modes**.

### Mode 1 — Manual Control

- Keyboard-driven terminal interface.
- Drive in real time with **WASD** (omnidirectional + in-place rotation).
- One keypress triggers the capture routine: rotate → capture → merge, runs autonomously, then returns control to the user.

### Mode 2 — Autonomous Line Following

- Dark tape path laid on the floor connects capture locations.
- Robot follows the line using its 4 IR line-tracking sensors.
- **Stop markers** = perpendicular cross-marks on the tape that trigger all 4 sensors at once → mark a capture location.
- On detecting a stop marker: halt → run capture routine → resume line following to next marker.

Both modes let the user incrementally build a collection of point clouds across the room.

---

## Software

### Two places the code runs

```
  LAPTOP  (develop the perception)            RASPBERRY PI  (drive the robot)
  D405 over USB, pyrealsense2                  D405 + motors + sensors
  camera/ + pointcloud/ scripts                the RasBot API (setup_and_api/api)
  (capture, make/merge/render cloud)           bot.forward(), bot.capture_all(), ...
```

The RasBot API talks to the motors over **I2C (`smbus`)**, which only exists on the Pi — so you
**cannot** `import RasBot` on a laptop. You develop the depth / point-cloud algorithms on a laptop
with the D405 plugged in by USB, then run them on the Pi by swapping the frame source
(`pyrealsense2` → `bot.capture_all()`). The math is identical.

### Project layout

```
sprint_hackathon/
├── camera/          D405 capture code (camera drivers + capture)
│   ├── rs_capture.py       StereoCapture: the shared D405 pipeline
│   └── capture.py          standalone "ENTER = save a capture"
├── wasd/            Mode 1 — manual control
│   └── drive.py            WASD/QE teleop + R/T/Y/V scan keys (the conductor)
├── line_following/  Mode 2 — autonomous line following (placeholder, to build)
│   └── line_follow.py      scaffold using read_line_sensors() + the 360 scan
├── pointcloud/      perception: build & view 3D point clouds
│   ├── scan360.py          360 sweep + measured-angle merge (on the Pi)
│   ├── view3d.py           orbit a .ply (numpy+cv2 viewer, on the Pi)
│   ├── make_pointcloud.py  one capture → cloud.ply (laptop, Open3D)
│   ├── merge_clouds.py     many captures → merged.ply via ICP (laptop)
│   ├── render_cloud.py     static preview PNG (laptop)
│   └── clean_captures.py   delete capture data to start fresh
├── setup_and_api/   the official RasBot hardware API (+ Pi SETUP.md)
│   └── api/                rasbot.api: movement, servos, sensors, camera over I2C
├── rasbot/          import shim so `from rasbot.api import RasBot` works (api → ../setup_and_api/api)
├── docs/            all guides, the brief, the work plan, sprint info
└── captures/        camera output, created at runtime (git-ignored)
```

> **How imports work:** each runnable script puts the **project root** on `sys.path` and
> imports across folders by name — `from camera.rs_capture import StereoCapture`,
> `from pointcloud import scan360`, `from rasbot.api import RasBot`. Run scripts from the
> project root, e.g. `python3 wasd/drive.py` or `.venv/bin/python pointcloud/make_pointcloud.py`.

### Docs (in [`docs/`](docs/))

| Doc | What it is |
|---|---|
| [RUN_GUIDE.md](docs/RUN_GUIDE.md) | How to run everything (laptop + Pi). |
| [CHANGE_GUIDE.md](docs/CHANGE_GUIDE.md) | On-Pi 360 scan workflow (R/T/Y keys) + rotation calibration. |
| [POINTCLOUD_GUIDE.md](docs/POINTCLOUD_GUIDE.md) | Code/architecture guide for the perception pipeline. |
| [D405_Depth_Point_Clouds.md](docs/D405_Depth_Point_Clouds.md) | Theory: how the D405 & stereo depth work. |
| [setup_and_api/SETUP.md](setup_and_api/SETUP.md) | Prepare the Raspberry Pi from a blank SD card. |
| [setup_and_api/api/README.md](setup_and_api/api/README.md) | Full RasBot API reference. |

### How the pipeline works

```
  D405 (two IR cameras, 18 mm apart, factory-calibrated, NO projector)
    │  camera/capture.py
    ▼
  depth image + intrinsics (fx, fy, ppx, ppy, baseline, depth_scale)
    │  pointcloud/make_pointcloud.py   —  X=(u-ppx)Z/fx,  Y=(v-ppy)Z/fy,  Z=depth
    ▼
  one point cloud per viewpoint (cloud.ply)
    │  pointcloud/merge_clouds.py  —  align overlapping views with ICP (seed the known rotation)
    ▼
  one 360° point cloud per room location (merged.ply)   ← project goal
```

> ⚠️ The D405 is **passive stereo (no IR projector)** — aim it at **textured, well-lit** scenes;
> blank walls give empty depth. For a good **merge**, consecutive shots must **overlap ~70–80%**
> (rotate only ~10–15° between them); on the robot, seed the known rotation with `--angle`.

### Quick start (laptop)

```bash
uv venv --python 3.11 .venv          # pyrealsense2 has no wheels for Python 3.14
uv pip install -r requirements.txt
.venv/bin/python camera/capture.py              # ENTER = save a capture, q = quit
.venv/bin/python pointcloud/make_pointcloud.py  # newest capture -> cloud.ply + preview
```

See [`docs/POINTCLOUD_GUIDE.md`](docs/POINTCLOUD_GUIDE.md) for the full capture → merge workflow.

---

## Connecting to the Robot (Raspberry Pi)

The Pi is headless — you reach it over WiFi by SSH. Credentials are in
[`docs/sprint info.txt`](docs/sprint%20info.txt).

1. **Join the robot's WiFi** (SSID `Sprint9`) on your laptop.
2. **SSH into the Pi** (hostname `sprint`, user `sprint`):
   ```bash
   ssh sprint@sprint.local
   # if sprint.local doesn't resolve, find the IP from the router and use ssh sprint@<ip>
   ```
   Tip: develop comfortably with the **VS Code "Remote - SSH"** extension — edit files on the Pi
   from your laptop, with a terminal that runs on the Pi.
3. **Set up the Pi** following [`setup_and_api/SETUP.md`](setup_and_api/SETUP.md) (flash 64-bit OS,
   enable I2C, build `librealsense`). Verify the robot board with `i2cdetect -y 1` (shows `0x2b`).
4. **Make the API importable:** the package imports as `rasbot.api`, so place the `api/` folder
   inside a folder named `rasbot/` on the Pi, and run Python from the folder that contains
   `rasbot/`. Then:
   ```python
   import time
   from rasbot.api import RasBot, Color

   with RasBot() as bot:                 # auto-stops & cleans up on exit
       bot.forward(120); time.sleep(1); bot.stop()
       frames = bot.capture_all()        # color + depth(mm) + ir_left + ir_right (synced)
       intr   = bot.get_stereo_intrinsics()
       # feed frames.depth + intr into the SAME back-projection as pointcloud/make_pointcloud.py
   ```

The brief's required API maps directly: `forward(speed)` → `bot.forward(speed)`,
`capture_stereo()` → `bot.capture_stereo()`, `set_tilt(angle)` → `bot.set_tilt(angle)`. Full method
list in [`setup_and_api/api/README.md`](setup_and_api/api/README.md).

> Note: the Pi API uses **640×480** and returns **depth in millimeters** (the laptop
> `camera/capture.py` uses 848×480 and raw units). Same math — just mind the units when reusing code.

### Still to build
1. Our own stereo depth from the IR pair (OpenCV `StereoSGBM`) — the brief's explicit requirement.
2. The per-location rotate → capture → merge routine (seeded ICP) for the full 360° cloud.
3. The two modes on the Pi: WASD manual control ([`wasd/`](wasd/), working) and IR line-following
   with stop-markers ([`line_following/`](line_following/), scaffolded — see its README).
