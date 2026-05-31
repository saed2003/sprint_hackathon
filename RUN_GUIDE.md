# How to Run Everything

This explains the `setup_and_api` folder, and exactly how to run the camera, the capture,
and the point cloud — both **now on your laptop** and **later on the robot (Pi)**.

---

## The big picture: TWO environments

There are two completely separate places this code runs, and they use different tools:

```
  ┌─────────────────────────────┐        ┌──────────────────────────────────┐
  │   LAPTOP  (now)             │         │   RASPBERRY PI on the robot       │
  │   D405 plugged in by USB     │        │   D405 + motors + sensors         │
  │                             │         │                                    │
  │   capture.py                │         │   the RasBot API (setup_and_api)   │
  │   make_pointcloud.py        │         │   bot.forward(), bot.capture_all() │
  │   uses pyrealsense2 directly│         │   uses smbus (I2C) + pyrealsense2  │
  └─────────────────────────────┘        └──────────────────────────────────┘
        develop the perception                drive the robot for real
        algorithms here                       here
```

**Why two?** The RasBot API talks to the motors over **I2C** using the `smbus` library, which
only exists on the Raspberry Pi. So you **cannot** `import RasBot` on your laptop. On the laptop
you talk to the camera directly with `pyrealsense2` (that is what `capture.py` does). The depth /
point-cloud algorithms you write on the laptop move over to the Pi unchanged — only the way you
*get the frames* changes (direct pyrealsense2 → `bot.capture_all()`).

---

## Part 1 — What is in `setup_and_api/`

```
setup_and_api/
├── SETUP.md          how to prepare the Raspberry Pi from a blank SD card
└── api/              the "rasbot.api" Python package = the official hardware API
    ├── constants.py  I2C address, register map, motor IDs, servo ranges, camera defaults
    ├── robot.py      the RasBot class: every hardware function lives here
    ├── __init__.py   lets you do `from rasbot.api import RasBot, Color`
    └── README.md     full documentation of every RasBot method
```

### `SETUP.md` — preparing the Pi (your teammate / robot side)
Step-by-step to set up the Pi: flash **64-bit** Raspberry Pi OS, enable **I2C**, install
`numpy`/`opencv`/`smbus`, and — the hard part — **build `librealsense` from source** (there is no
prebuilt package for ARM, so it must be compiled; takes ~1 hr). It verifies success with
`i2cdetect -y 1` (the robot board should appear at address **0x2B**) and a Python import test.

### `api/` — the RasBot API (the "Python API" from the project brief)
One class, `RasBot`, wraps **all** the hardware into simple calls. This is the
`forward(speed)` / `capture_stereo()` / `set_tilt(angle)` abstraction the brief asks for.

| Brief asks for | RasBot method |
|---|---|
| `forward(speed)` | `bot.forward(speed)` (+ `backward`, `left`, `right`, `rotate_left/right`, `move(speed, angle)`) |
| `capture_stereo()` | `bot.capture_stereo()` → `(ir_left, ir_right)` |
| `set_tilt(angle)` | `bot.set_tilt(angle)` (+ `set_pan`) |

Plus: `read_distance()` (ultrasonic, cm), `read_line_sensors()` (the 4 IR sensors for line
following), `capture_depth()` (mm), `capture_all()` (color+depth+IR in one synced shot),
`get_stereo_intrinsics()` / `get_stereo_baseline()` (the numbers for point clouds), LEDs, buzzer,
OLED, audio. Full list in `setup_and_api/api/README.md`.

> ⚠️ **Two things to know about the API package**
> 1. It imports `smbus` at the top → it only runs **on the Pi**, not the laptop.
> 2. Its imports are `from rasbot.api...`, so the `api` folder must sit inside a folder named
>    **`rasbot/`** that is on the Python path. On the Pi, arrange it as `rasbot/api/...` and run
>    Python from the folder that *contains* `rasbot/`.
> 3. The API uses **640×480** and returns **depth in millimeters** (our laptop `capture.py` uses
>    848×480 and raw units). Same math, just mind the units when you reuse code.

---

## Part 2 — Run the camera + capture (LAPTOP, do this now)

Everything below uses the project's `.venv` (Python 3.11). Always prefix with `.venv/bin/python`.

### Capture frames from the D405
```bash
cd ~/sprint_hackathon   # wherever you cloned this repo
.venv/bin/python capture.py
```
- Point the camera at a **textured, well-lit** scene (remember: D405 has no projector, so blank
  walls give empty depth).
- Press **ENTER** to save a capture, repeat for several scenes, then **q** + ENTER to quit.
- Each capture lands in `captures/<timestamp>/` with: `depth.npy`, `depth_color.png`,
  `ir_left.png`, `ir_right.png`, `intrinsics.txt`.

### Build a point cloud from a capture
```bash
.venv/bin/python make_pointcloud.py                 # newest capture
.venv/bin/python make_pointcloud.py captures/2026... # a specific one
```
This back-projects depth → 3D points and writes `cloud.ply` + `cloud_preview.png` into the
capture folder.

### View the point cloud interactively (rotate it with the mouse)
```bash
.venv/bin/python -c "import open3d as o3d, sys; o3d.visualization.draw_geometries([o3d.io.read_point_cloud(sys.argv[1])])" captures/<timestamp>/cloud.ply
```
(Opens a 3D window on your desktop — drag to orbit, scroll to zoom.)

---

## Part 3 — Run it on the robot (PI, later)

Once `SETUP.md` is done on the Pi and the `api` folder is placed as `rasbot/api`:

```python
import time
from rasbot.api import RasBot, Color

with RasBot() as bot:                 # auto-stops & cleans up on exit
    bot.set_all_leds_color(Color.GREEN)
    bot.forward(120); time.sleep(1); bot.stop()

    frames = bot.capture_all()        # color + depth(mm) + ir_left + ir_right, synced
    intr   = bot.get_stereo_intrinsics()   # fx, fy, ppx, ppy
    # -> feed frames.depth + intr into the SAME back-projection as make_pointcloud.py
```

The capture routine for each room location will be: rotate a step (`bot.rotate_right`), capture a
cloud, repeat around 360°, then merge the clouds (ICP) into one — the final goal.

---

## Quick map: which file does what

| File | Where it runs | Purpose |
|---|---|---|
| `capture.py` | laptop | grab depth + IR frames from the D405, save them |
| `make_pointcloud.py` | laptop | turn a capture into a 3D `.ply` + preview |
| `D405_Depth_Point_Clouds.md` | — | the theory: how depth & point clouds work |
| `setup_and_api/SETUP.md` | Pi | prepare the Raspberry Pi |
| `setup_and_api/api/` | Pi | the RasBot hardware API (motors, sensors, camera) |

## Still to build (next milestones)
1. **Our own stereo depth** from `ir_left`/`ir_right` with OpenCV `StereoSGBM` (project requirement).
2. **Merge** several rotated clouds into one 360° cloud with ICP (Open3D).
3. **Modes** on the Pi: WASD manual control, and IR line-following with stop-markers.
