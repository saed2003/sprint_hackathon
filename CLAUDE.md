# CLAUDE.md

Guidance for Claude Code when working in this repo. **This is the real, current
project.** (The loose files one level up in `…/sprint/` are an older flat
prototype — see [the parent CLAUDE.md](../CLAUDE.md).)

## Project

**Street View Robot** — a Yahboom **Raspbot V2** (4× Mecanum wheels, **Raspberry
Pi 5**, **Intel RealSense D405** stereo IR camera) that drives a room and builds a
**360° point cloud** at each stop. University sprint/hackathon; mentor Mr. Rajaei
Khatib; 5-person team. The user does the **software/perception**; a teammate built
the chassis. See `docs/` for the brief, theory, hardware, and API.

The brief asks for **two navigation modes**: (1) **manual** WASD teleop, and (2)
**autonomous line-following** on dark tape with cross "stop markers" that trigger a
scan. Both feed the same rotate→capture→merge routine.

## Where everything lives (orient here first)

```
sprint/                         ← working-dir root: OLD flat prototype + the venv
└── sprint_hackathon/           ← THE PROJECT (everything below is here)
    ├── main.py                 single launcher: web | drive | status
    ├── CLAUDE.md  README.md  DASHBOARD_ARCHITECTURE.md  CAMERA_STREAM_SETUP.md
    ├── requirements.txt  .gitignore
    ├── top/                    web dashboard (robot_control_dashboard.html, index.html→it)
    ├── docs/                   brief, work plan, guides, sprint info.txt (Pi creds)
    ├── test_code/              scratch / experiments (isolated; never imported by the robot)
    │   ├── capture.py  make_pointcloud.py   earliest flat copies
    │   └── object_scan/        ★ experimental OBJECT 3D-scan (turntable + robot orbit → mesh)
    ├── captures/              runtime scan output (git-ignored; also exists at src/captures)
    └── src/                    ← ALL the code (run scripts from here as the root)
        ├── control_server.py     port 9000 control API (web side brain)
        ├── controller.py         SSH/terminal WASD teleop + F line-follow (termios)
        ├── oled_message.py       animated OLED team-names splash (demo)
        ├── camera/               D405 + USB-cam capture & streaming
        │   ├── rs_capture.py       StereoCapture — the shared D405 pipeline
        │   ├── capture.py          ENTER-to-save REPL
        │   ├── live_view.py        quick camera preview/test
        │   └── stream_server.py    port 8000 MJPEG from the USB webcam
        ├── camera_move/          pan/tilt servo controllers + hw test scripts
        │   ├── camera_move.py      arrow-key tilt/pan over bare SSH (termios)
        │   ├── pygame_servo.py     same but pygame/VNC (hold to move)
        │   └── motor_test.py  servo_test.py  servo_probe.py   (hardware probes)
        ├── pointcloud/           build & view 3D clouds
        │   ├── scan360.py          ★ 360 timed sweep + angle-aware merge (Pi, numpy/cv2)
        │   ├── view3d.py           orbit a .ply (numpy+cv2, no Open3D — Pi)
        │   ├── make_pointcloud.py  one capture → cloud.ply (laptop, Open3D)
        │   ├── merge_clouds.py     many captures → merged.ply via ICP (laptop, Open3D)
        │   ├── build_from_depth_pngs.py   build a cloud from depth PNGs
        │   ├── render_cloud.py     static preview PNG (laptop)
        │   └── clean_captures.py   wipe capture data to start fresh
        ├── depth_map/            ★ OUR OWN stereo depth (brief's requirement)
        │   ├── depth_map.py        cv2.StereoSGBM disparity → metric depth
        │   ├── ALGORITHM.md        how SGBM stereo works
        │   ├── capture_depth.py    grab an IR pair to feed it
        │   ├── pointcloud.py       4-step laptop 360 ❶ one our-depth map → cloud.ply
        │   ├── combine_360.py      ❷ stack 10 shots, nominal-36°/ICP align  (ICP FAILS on real arc data)
        │   ├── pointcloud_360.py   ❸ dedup overlap + save merged 360 .ply + preview
        │   └── test_images/  result_ref/  depth_map_result.png
        ├── new_point_cloud/      ★★ the 360 THAT WORKS — feature-based, supersedes depth_map/combine_360
        │   ├── register_360.py     CLAHE+ORB/SIFT 3D↔3D registration + pose-graph loop closure
        │   ├── geometry.py         pure-numpy math: back-project, Kabsch/RANSAC, .ply IO
        │   ├── view_ply.py         interactive Open3D viewer (drag/zoom, +/- point size)
        │   └── README.md  explain.html   why nominal/ICP failed & how CLAHE rescues it
        ├── wasd/                 Mode 1 — manual control
        │   └── drive.py            pygame WASD teleop + R/T/Y/V scan keys (the conductor)
        ├── tape_following/       ★ Mode 2 — autonomous line following (the REAL one)
        │   ├── line_follow.py      state-machine + per-state PID line follower (U-turns)
        │   └── drive.py            WASD teleop + F to toggle line-follow
        ├── radar/                ultrasonic PPI radar viz (sweep servo + distance)
        │   ├── radar.py            --web (MJPEG) | --window (VNC) | --demo
        │   └── radar_vnc.py        fancy "military" pygame radar
        ├── game/                 Room Explorer split-screen VNC game (coverage map)
        │   └── game_test.py
        ├── scan_continuous_concept/   alt 360 idea: spin continuously, vision-merge
        ├── setup_and_api/        ★ the WORKING RasBot API copy (+ Pi SETUP.md)
        │   └── api/                robot.py, constants.py, __init__.py, README.md
        └── rasbot/               legacy import shim (see "Import conventions")
```

★ = the files you'll touch most / that carry the project's core logic.
★★ = `new_point_cloud/` is the **current working 360** — start here for cloud work.

## Two environments

Code runs in two places with different available libraries:

| Environment | Libraries | What runs there |
|---|---|---|
| **Laptop** | `pyrealsense2`, `numpy`, `cv2`, `open3d`, `matplotlib` | `camera/`, `depth_map/`, `new_point_cloud/` (needs Open3D), `pointcloud/make_pointcloud.py`, `merge_clouds.py`, `render_cloud.py` |
| **Raspberry Pi (robot)** | `pyrealsense2`, `numpy`, `cv2`, `pygame`, `smbus`, `PIL` — **no Open3D** | everything that imports `RasBot`: `control_server.py`, `wasd/`, `tape_following/`, `camera_move/`, `radar/`, `game/`, `oled_message.py`, `pointcloud/scan360.py`, `view3d.py` |

**Critical:** importing the RasBot API pulls `smbus`, which only exists on the Pi —
it crashes on the laptop. All Pi-side **perception** is pure NumPy + OpenCV (no
Open3D on the Pi). Develop depth/cloud algorithms on the laptop with the D405 over
USB, then run them on the Pi by swapping the frame source — the math is identical.

## Import conventions & the API (read this — it's the #1 gotcha)

Every runnable script puts **`src/` on `sys.path`** and imports cross-folder by
package name. Run scripts from `src/` (or via `main.py`, which adds `src/` for you):

```python
from setup_and_api.api import RasBot, Color        # the hardware API (Pi only)
from camera.rs_capture import StereoCapture
from pointcloud import scan360
```

**There are two copies of the RasBot API** — don't confuse them:

| Copy | Import path | Notes |
|---|---|---|
| `src/setup_and_api/api/` | `from setup_and_api.api import RasBot` | **the live one.** `robot.py` uses relative imports; `__init__.py` exports `RasBot, RealSenseFrames, Color`. All `src/` code uses this. |
| `…/sprint/setup_and_api/api/` (parent dir) | `from rasbot.api...` | the **originally-distributed** vendor zip (has a `__MACOSX/` junk folder). `robot.py` imports `rasbot.api.constants`. Reference only — not used by `src/`. |

`src/rasbot/` is a **legacy shim**: empty `__init__.py` + a file `api` whose
contents are the text `../setup_and_api/api` (a git symlink checked out as a plain
file — **broken on this non-git laptop checkout**). `from rasbot.api import RasBot`
will NOT work here; use `setup_and_api.api`. The docs/README still mention
`rasbot.api` and a `line_following/` folder — **both are stale**.

## The shared contract: capture folders

The single interface between every producer (camera/robot) and consumer (cloud
tools), so Pi and laptop tools stay compatible:

```
captures/<timestamp>/
├── depth.npy         uint16 raw depth (× depth_scale → meters)
├── depth_color.png   colorized depth (for looking at)
├── ir_left.png       left IR  → input to our own stereo depth
├── ir_right.png      right IR
└── intrinsics.txt    key/value: width height fx fy ppx ppy depth_scale baseline_m
```

A 360 scan is `captures/scan_<ts>/shot_NN/` (each a capture folder) + a
`merged_360.ply` (+ `merged_360_preview.png`). Continuous-scan concept uses
`cscan_<ts>/`. **Rule:** never pass camera data as ad-hoc variables — write/read
capture folders. `StereoCapture` writes them; `scan360.back_project()` reads them.

## Setup (laptop)

```bash
uv venv --python 3.11 .venv          # pyrealsense2 has no wheels for 3.14
uv pip install -r requirements.txt   # pyrealsense2, numpy, opencv-python, open3d, matplotlib
```

Pinned versions in `requirements.txt`: pyrealsense2 2.58, numpy 2.4, opencv 4.13,
open3d 0.19, matplotlib 3.10. On the Pi (ARM) pyrealsense2/open3d may need building
from source — see `src/setup_and_api/SETUP.md`.

## Two ways to run / drive the robot

### A) Web dashboard (the main interface) — `main.py`

One launcher, started by a systemd service (`rasbot.service` → `main.py web`):

```bash
python3 main.py web      # dashboard (:80, falls back :8080) + control API (:9000)
python3 main.py drive    # pygame WASD teleop (needs Pi desktop/VNC)
python3 main.py status   # which servers are up
```

Three HTTP servers, three ports (full detail in **DASHBOARD_ARCHITECTURE.md**):

| Port | Server | Role |
|---|---|---|
| 80/8080 | `main.py` static handler | serves `top/robot_control_dashboard.html` |
| 9000 | `src/control_server.py` | control API: run on/off, `/api/drive`, `/api/servo`, captures (`scan360`/`single`/`build`), `/api/cloud/download`, spawns the stream |
| 8000 | `src/camera/stream_server.py` | live **MJPEG** from the **USB webcam** (not the D405) |

Key idea: `control_server.py` **reuses `wasd/drive.py`'s** `desired_command` /
`apply_command` (translating web key strings → pygame keycodes) — no duplicate
motion logic. A browser **heartbeat** (~150 ms) + a server **watchdog** (0.6 s)
halt the robot if the tab/Wi-Fi drops. Live stream = USB cam; captures = D405, so
streaming and capturing don't fight over a device.

### B) Desktop / terminal teleops (one driver at a time!)

```bash
python3 wasd/drive.py              # pygame WASD; R=360 scan, T=build, Y=view, V=single  (VNC)
python3 tape_following/drive.py    # WASD + F=toggle line-follow                          (VNC)
python3 controller.py              # bare-SSH WASD (W/S fwd/back, A/D rotate) + F follow
python3 tape_following/line_follow.py [--calibrate]   # standalone line follower
```

⚠️ The web stack and the desktop teleops share the **same I2C bus** — only one may
*actively drive* at a time. Stop the web run (or `sudo systemctl stop
rasbot.service`) before using a teleop.

## Common perception commands

```bash
# Capture frames (laptop, D405 over USB):
.venv/bin/python camera/capture.py            # ENTER = save, q = quit

# Build a cloud from a capture (laptop, Open3D):
.venv/bin/python pointcloud/make_pointcloud.py            # newest
.venv/bin/python pointcloud/make_pointcloud.py --all      # all
.venv/bin/python pointcloud/make_pointcloud.py captures/<ts>

# Our own stereo depth from an IR pair (laptop):
python3 depth_map/depth_map.py                # demos on depth_map/test_images/

# Build the WORKING 360 from a scan (laptop, Open3D) — feature-based, no angle guess:
python3 new_point_cloud/register_360.py                       # newest captures/scan_*
python3 new_point_cloud/register_360.py captures/scan_<ts>
python3 new_point_cloud/register_360.py captures/scan_<ts> --sift    # SIFT every pair (robust)
python3 new_point_cloud/register_360.py captures/scan_<ts> --byshot  # tint shots to see seams
python3 new_point_cloud/register_360.py captures/scan_<ts> --gray --zmax 1.5
#   → writes new_point_cloud/pointcloud_360.ply (+ _preview.png); watch the `components:` line

# On the Pi — rebuild a 360 scan without re-driving:
python3 pointcloud/scan360.py captures/scan_<ts>            # measured angle (default)
python3 pointcloud/scan360.py captures/scan_<ts> --known    # trust the timed step
python3 pointcloud/scan360.py captures/scan_<ts> --angle 36 # force fixed angle
python3 pointcloud/scan360.py --calibrate --turned <deg>    # calibrate rotation timing

# View a cloud:
python3 new_point_cloud/view_ply.py [cloud.ply]                 # laptop, interactive Open3D
python3 pointcloud/view3d.py captures/scan_<ts>/merged_360.ply  # Pi (numpy+cv2)

# Copy scans Pi → laptop:
scp -r sprint@sprint.local:~/sprint_hackathon/captures ~/sprint_hackathon/
```

## Perception pipeline (data flow)

```
D405 (two IR cams 18 mm apart, factory-calibrated, NO IR projector — passive stereo)
  │  camera/rs_capture.py (StereoCapture)            848×480 laptop / 640×480 Pi
  ▼
captures/<ts>/ { depth.npy, ir_left/right.png, depth_color.png, intrinsics.txt }
  │
  ├── depth_map/depth_map.py   our own SGBM disparity → depth   (brief's requirement)
  │
  ├── pointcloud/make_pointcloud.py  (laptop, Open3D)  → cloud.ply
  │     back-project:  Z=depth,  X=(u-ppx)Z/fx,  Y=(v-ppy)Z/fy
  │
  ├── pointcloud/scan360.py  (Pi, numpy+cv2)  → merged_360.ply
  │     ├── back_project(): depth.npy → 3D points (+ IR as gray color)
  │     ├── cumulative_angles(): recorded angle.txt, else ORB→homography yaw, else nominal
  │     ├── ry(angle): rotate each view about Y (vertical) into view-0's frame
  │     ├── voxel_downsample(VOXEL=1cm) + remove_isolated() (drop passive-stereo flyers)
  │     └── write_ply() + view3d.save_view() preview
  │
  └── new_point_cloud/register_360.py  (laptop, Open3D)  → pointcloud_360.ply   ★ the 360 that works
        ├── per shot:  CLAHE→ORB/SIFT features, lift each to 3D via depth.npy
        ├── per pair:  match → 2D fundamental filter → Kabsch+RANSAC → rigid T  (NO angle guess)
        ├── pose graph: consecutive=odometry, others=loop closures → Open3D global opt spreads drift
        └── re-project all shots into one frame, voxel 5 mm, cull flyers → .ply + preview
```

> ⚠️ Passive stereo (no projector): aim at **textured, well-lit** scenes — blank
> walls give empty depth. Keep depth in **0.1–1.5 m** (`ZMIN/ZMAX`); past ~1.5 m is
> noise. For a good merge, consecutive shots need heavy overlap.

## The three 360 pipelines (which one to use)

Three different mergers exist because the first two fought the same enemy — the
robot has **no IMU/encoder** and the camera sits **off the spin axis**, so each step
is rotation **+ a little translation (an arc)**, and the real step is ~40° (the spin
over-rotates to ~400°), not the nominal 36°. They differ only in how they recover
each shot's pose:

| Pipeline | Pose source | Depth source | Runs on | Verdict |
|---|---|---|---|---|
| `pointcloud/scan360.py` | **timed** `angle.txt`, else image-yaw | hardware `depth.npy` | **Pi** (numpy+cv2) | on-robot quick merge; smears with arc drift |
| `depth_map/{pointcloud,combine_360,pointcloud_360}.py` | nominal 36° **or ICP** | **our SGBM** depth | laptop | ❌ nominal smears, **ICP locks onto passive-stereo noise → garbage** |
| **`new_point_cloud/register_360.py`** | **measured** (feature 3D↔3D + pose graph) | hardware `depth.npy` | laptop (Open3D) | ✅ **the one that works — use this** |

**Why `register_360` wins:** it *measures* each pose from image correspondences
instead of guessing an angle. Per shot it CLAHE-enhances the IR, finds ORB (SIFT
fallback) features, and lifts each to 3D through the depth map; for **every** pair it
matches, pre-filters with a 2D fundamental matrix, and solves a rigid transform with
**Kabsch inside RANSAC** — no initial angle. The accepted pairs form a pose graph
(consecutive = trusted odometry, the rest = loop closures) that Open3D globally
optimizes to spread drift around the ring.

**CLAHE is the whole game** (see `new_point_cloud/README.md`): the
D405 IR is dim (~mean 55/255), so raw ORB finds ~30 keypoints on dark shots and the
ring breaks — looks like a blank wall but isn't. CLAHE before detection yields ~30×
more keypoints and **every shot registers** (4/10 → 10/10). It is detection-only;
the original gray still colors the cloud. The run's `components:` line is the health
check — **one component of all 10 shots = a full 360**; a split means the *capture*
(not the code) is bad. `depth_map/combine_360.py` is kept only as the cautionary
prototype it documents in its own docstring.

## scan360 open-loop rotation (important)

The Raspbot has **no IMU or wheel encoders**, so rotation between shots is **purely
timed** — the camera is **not** used to steer. `run_scan()` pulses the motors for
`SCAN_SEC_PER_DEG * step` between `SCAN_SHOTS` (default 10 → 36° each), writing each
shot's cumulative angle to `shot_NN/angle.txt`. The merge rotates each view by that
recorded angle. If `angle.txt` is missing (e.g. continuous-scan concept), the merge
falls back to **image-measured yaw** (ORB → `H = K R K⁻¹` → yaw) per step.

Calibrate `SCAN_SEC_PER_DEG` once per robot/floor (battery + grip shift it):
`python3 pointcloud/scan360.py --calibrate --turned <deg>`. `SCAN_RETURN_HOME` /
`SCAN_RETURN_MODE` control the optional return-to-start turn after the last shot.

## RasBot API key methods (`src/setup_and_api/api/`)

```python
with RasBot() as bot:                 # auto-stops/cleans up on exit (__exit__→cleanup)
    bot.forward(speed)                # 0–255; also backward/left/right (strafe)
    bot.rotate_left/right(speed)      # in-place spin
    bot.move(speed, angle_deg)        # omnidirectional: 0=right 90=fwd 180=left 270=back
    bot.drift(speed, angle, rot_rate) # translate + rotate (mecanum)
    bot.stop()
    bot.set_pan(0-180) / set_tilt(0-100) / look_center()
    bot.set_all_leds_color(Color.GREEN) / leds_off() / beep(sec)
    bot.read_line_sensors()           # (left_outer, left_inner, right_inner, right_outer) bool
    bot.read_distance()               # ultrasonic, cm
    frames = bot.capture_all()        # RealSenseFrames(color, depth_mm, ir_left, ir_right)
    bot.capture_stereo()              # (ir_left, ir_right)
    intr = bot.get_stereo_intrinsics()
    bot.display_text(str, line)       # 128×32 OLED
```

Hardware constants (`constants.py`): I2C addr **0x2B**, bus **1**; 14 LEDs;
pan default 90 (0–180), tilt default 25 (0–100); D405 defaults **640×480@30**,
usable depth **70–500 mm**. Pi API returns **depth in millimeters**; the laptop
`StereoCapture` uses 848×480 raw units. Same back-projection math — **mind the units.**

## Connecting to the Pi

Pi is headless; reach it over Wi-Fi by SSH (creds in `docs/sprint info.txt`):

- Wi-Fi SSID **Sprint9** / pass `sprintgroup9`
- `ssh sprint@sprint.local` (user `sprint`, pass `group9`); last known IP `192.168.137.74`
- Tip: VS Code **Remote-SSH** to edit on the Pi. Verify the board: `i2cdetect -y 1` → `0x2b`.
- After editing `main.py`/`control_server.py`: `sudo systemctl restart rasbot.service`;
  follow logs with `journalctl -u rasbot.service -f`. Editing the HTML needs no restart.

## Status / still to build

- **360 reconstruction** — `new_point_cloud/register_360.py` is the **working**
  merger (feature 3D↔3D + pose graph + CLAHE); both example scans reach a 10/10
  single component. It reads the **hardware** `depth.npy`. The `depth_map/` 4-step
  pipeline (nominal-36°/ICP on **our** SGBM depth) is the superseded prototype — its
  ICP path fails on real arc-motion captures. See "The three 360 pipelines" above.
- **Custom stereo depth** (`depth_map/depth_map.py`) exists and works on test images
  (SGBM + optional WLS hole-fill). Not yet wired to overwrite `depth.npy` in live
  capture folders — doing so makes the whole pipeline use our depth automatically.
  (Note: the working 360 currently uses the **hardware** depth, not ours.)
- **Line following** (`tape_following/line_follow.py`) is a full state-machine PID
  follower; tune thresholds/speeds on the real tape (`--calibrate`).
- The web dashboard supports manual mode + captures; **autonomous mode** in
  `control_server.start_run` is `not_implemented` (only `manual` is armed).
</content>
