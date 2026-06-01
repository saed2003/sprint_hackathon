# Point-Cloud Pipeline — Code & Architecture Guide

**Who this is for:** teammates working on different parts of the project who need to
understand how the perception code fits together — what each file does, the theory behind
it, a little of the actual code, and how the pieces hand off to each other.

This is the *architecture* doc. The others are:
- **[D405_Depth_Point_Clouds.md](D405_Depth_Point_Clouds.md)** — the camera + stereo-depth **theory** (how depth is made).
- **[CHANGE_GUIDE.md](CHANGE_GUIDE.md)** — the on-Pi **scan workflow** (R/T/Y keys) and rotation **calibration**.
- **[RUN_GUIDE.md](RUN_GUIDE.md)** / **[README.md](../README.md)** — how to set up and **run** everything.

> One-line summary of the whole pipeline: **the camera makes a depth image → we turn each
> depth image into 3D points → we rotate the views by the angle the robot turned and stack
> them into one 360° cloud → we view/save it as a `.ply`.**

---

## 0. Where the code runs (two worlds)

The code splits cleanly into two environments. Knowing which world a file lives in tells you
which libraries it may use.

| World | Libraries available | Files | Job |
|---|---|---|---|
| **On the robot (Raspberry Pi)** | `pyrealsense2`, `numpy`, `cv2` — **no Open3D** | [rs_capture.py](../camera/rs_capture.py), [capture.py](../camera/capture.py), [scan360.py](../pointcloud/scan360.py), [view3d.py](../pointcloud/view3d.py), [drive.py](../wasd/drive.py), [rasbot/](../setup_and_api/api/robot.py) | capture + a self-contained 360 scan/merge/view |
| **On the laptop** | `+ open3d`, `matplotlib` | [make_pointcloud.py](../pointcloud/make_pointcloud.py), [merge_clouds.py](../pointcloud/merge_clouds.py), [render_cloud.py](../pointcloud/render_cloud.py) | high-quality per-shot clouds + ICP merge |
| **Either** | stdlib only | [clean_captures.py](../pointcloud/clean_captures.py) | delete captures to start fresh |

**Why the split?** Open3D has no easy Raspberry Pi build, so everything that must run *on the
robot* is written in pure NumPy + OpenCV. The laptop tools are allowed to use Open3D for the
nicer ICP merge. Both worlds read and write the **same capture-folder format** (Section 2), so
they are interchangeable.

---

## 1. The big picture (data flow)

```
  D405 camera (2 IR imagers, factory-calibrated, NO projector)
        │  pyrealsense2
        ▼
  ┌──────────────────┐
  │  rs_capture.py   │   StereoCapture: grabs depth + IR, reads calibration
  │  (the producer)  │
  └────────┬─────────┘
           │  writes a "capture folder" (THE shared contract — Section 2)
           ▼
   captures/<ts>/ { depth.npy, ir_left.png, ir_right.png, intrinsics.txt, depth_color.png }
           │
   ┌───────┴────────────────────────────────────────────────┐
   │                                                          │
   ▼  ONE folder                                              ▼  a whole scan_<ts>/ of folders
 make_pointcloud.py  (laptop, Open3D)                      scan360.py  (Pi, numpy+cv2)
   one view -> cloud.ply                                    back-project every shot,
                                                            rotate each by the MEASURED angle,
   merge_clouds.py   (laptop, Open3D, ICP)                  stack -> merged_360.ply
   many views -> merged.ply
           │                                                          │
           └───────────────────────────┬──────────────────────────────┘
                                        ▼
                                   a .ply file
                                        │
                 ┌──────────────────────┼───────────────────────┐
                 ▼                       ▼                        ▼
            view3d.py (Pi)        3dviewer.net / f3d         render_cloud.py
            orbit live            (external viewers)         static preview PNG
```

The robot driver [drive.py](../wasd/drive.py) is the **conductor**: it owns the keyboard, calls
`scan360` to do a scan, and launches `view3d` to show the result.

---

## 2. The shared contract: a "capture folder"

This is the single most important thing to understand, because it is the **interface** that
lets everyone work independently. Every producer writes it; every consumer reads it. If you
respect this layout, your code plugs into the pipeline.

```
captures/<timestamp>/           # one capture = one camera viewpoint
├── depth.npy        uint16 array, raw depth units   (× depth_scale → meters)
├── depth_color.png  colorized depth, just for looking at
├── ir_left.png      left  infrared image  ── the stereo pair (and what scan360
├── ir_right.png     right infrared image  ──  uses to MEASURE the rotation angle)
└── intrinsics.txt   width height fx fy ppx ppy depth_scale baseline_m
```

A **360 scan** is just a folder of these:

```
captures/scan_<timestamp>/
├── shot_00/  (a capture folder)
├── shot_01/  (a capture folder)
│   ...
├── shot_08/
└── merged_360.ply   ← written by the merge step
```

`intrinsics.txt` is plain `key value` lines, parsed everywhere by the same tiny reader
([scan360.py:63](../pointcloud/scan360.py#L63)):

```python
def load_intrinsics(path):
    vals = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                vals[p[0]] = float(p[1])
    return vals
```

> **Rule of thumb:** never pass camera data around as ad-hoc variables — write/read a capture
> folder. That is how the Pi tools and the laptop tools stay compatible.

---

## 3. Module-by-module

For each module: **what it does · where it runs · the theory · a bit of code · what it
connects to.**

### 3.1 `rs_capture.py` — the camera producer

- **What:** `StereoCapture` owns one D405 pipeline and saves capture folders. Open it once,
  call `save()` / `save_to()` as many times as you like.
- **Runs on:** Pi (needs `pyrealsense2`). Used by both `capture.py` and `drive.py`/`scan360`.
- **Theory:** the D405 is *passive* stereo (two IR imagers, factory-calibrated, **no laser
  projector**) — see [D405_Depth_Point_Clouds.md](D405_Depth_Point_Clouds.md). On `start()` it
  reads the calibration once (`fx, fy, ppx, ppy, depth_scale, baseline`) and warms up
  auto-exposure so the first frame isn't dark.
- **Key code** ([rs_capture.py:96](../camera/rs_capture.py#L96)) — `save_to()` grabs one synced frameset
  and writes the contract from Section 2:
  ```python
  frames = self.pipeline.wait_for_frames()
  depth = frames.get_depth_frame()
  irl   = frames.get_infrared_frame(1)   # left
  irr   = frames.get_infrared_frame(2)   # right
  np.save(.../"depth.npy", np.asanyarray(depth.get_data()))   # uint16
  cv2.imwrite(.../"ir_left.png",  np.asanyarray(irl.get_data()))
  # ... + intrinsics.txt
  ```
- **Connects to:** produces the folders that *everything downstream* consumes.

### 3.2 `capture.py` — standalone single capture

- **What:** a tiny REPL — press ENTER to save a capture, `q` to quit. Just a thin wrapper
  around `StereoCapture`.
- **Runs on:** Pi. **Use it** to collect test data without driving.
- **Connects to:** writes capture folders → `make_pointcloud.py` / `merge_clouds.py`.

### 3.3 `make_pointcloud.py` — one view → one cloud (laptop)

- **What:** turns **one** capture folder into a `cloud.ply` + a quick `cloud_preview.png`.
- **Runs on:** laptop (uses Open3D to write the ply, matplotlib for the preview).
- **Theory — back-projection.** A depth image gives only distance `Z` per pixel `(u,v)`. To get
  full 3D `(X,Y,Z)` we invert the pinhole-camera projection using the intrinsics
  ([make_pointcloud.py:83](../pointcloud/make_pointcloud.py#L83)):
  ```python
  Z = depth_raw * depth_scale                 # raw units -> meters
  X = (uu - ppx) * Z / fx                      # ← the core equation
  Y = (vv - ppy) * Z / fy
  valid = (Z > 0.05) & (Z < 3.0)               # keep the D405's useful range
  ```
- **Connects to:** per-shot clouds for inspection; the *real* combined result comes from
  `scan360` (on Pi) or `merge_clouds` (laptop).

### 3.4 `scan360.py` — the heart: sweep + on-Pi 360 merge

This is the most important file and where most recent work happened. It does **two** things:
drives the robot through a 360 sweep, and merges the shots into one cloud — all in pure
NumPy + OpenCV so it runs on the Pi. It also runs **standalone** to rebuild a cloud from
already-saved shots (great for developing on the laptop with no robot).

**(a) The sweep — `run_scan()`** ([scan360.py:269](../pointcloud/scan360.py#L269)). Stop, settle, shoot,
rotate a step, repeat. For N shots it rotates **N−1** times (shot 0…N−1 cover 0°…(N−1)·step;
position N would equal shot 0):
```python
for i in range(shots):
    bot.stop(); time.sleep(settle_pause)   # let the chassis stop shaking -> less blur
    cam.save_to(folder)                    # shot i  -> scan_<ts>/shot_0i/
    if i < shots - 1:
        _rotate_step(bot, rotate_speed, step_time, direction, brake_tap)
```

**The open-loop problem (read this).** The RasBot has **no IMU or wheel encoders**, so the
rotation is *timed*: `step_time = SCAN_SEC_PER_DEG × (360/shots)`. A short motor pulse
overshoots (it coasts after `stop()`, and motors are nonlinear at low speed), so the real
angle drifts from the nominal 40°. We handle this **two ways**:

- **Track A — calibrate the *pulsed* motion.** `--calibrate` does N scan-like pulses (not one
  long spin) so you can measure the true time→angle ratio and set `SCAN_SEC_PER_DEG`. See
  [CHANGE_GUIDE.md](CHANGE_GUIDE.md) Step 0. Optional `SCAN_BRAKE_TAP` adds a tiny reverse
  pulse to kill coast.
- **Track B — don't trust the timer; *measure* the angle from the images** (below). This is
  what actually makes the cloud correct.

**(b) Back-projection — `back_project()`** ([scan360.py:117](../pointcloud/scan360.py#L117)): same equation
as §3.3, returning points + grayscale colors from `ir_left.png`.

**(c) Measuring the real rotation — `estimate_yaw()`** ([scan360.py:81](../pointcloud/scan360.py#L81)).
*Theory:* two photos taken `~40°` apart overlap a lot (the D405 has a wide FOV). From matched
features in the two IR images we recover the camera motion with the **essential matrix**, which
needs the known intrinsics `K`. The recovered rotation `R` is almost pure yaw (the robot spins
about the vertical axis), so we read the yaw straight out of `R`:
```python
E, mask     = cv2.findEssentialMat(pa, pb, K, cv2.RANSAC, 0.999, 1.0)
n_in, R, *_ = cv2.recoverPose(E, pa, pb, K, mask=mask)
yaw = math.degrees(math.atan2(R[0, 2], R[0, 0]))   # the Y-axis rotation inside R
```
*Why it works on the robot:* the camera sits **off** the chassis turn-center, so an in-place
spin moves the camera (parallax) → the essential matrix is well-conditioned. (With *zero*
parallax it degenerates; the inlier guard below catches that.)

**(d) Deciding which angle to trust — `cumulative_angles()`** ([scan360.py:182](../pointcloud/scan360.py#L182)).
We trust the camera for the step **magnitude** but keep the **known turn direction**, and fall
back to the nominal 40° if a pair has too few inliers or an out-of-range value:
```python
if inliers >= MEASURE_MIN_INLIERS and lo <= cand <= hi:   # sane, well-supported
    step = math.copysign(cand, nominal_step)              # camera magnitude, known sign
else:
    step = nominal_step                                   # safe fallback (timed 40°)
```
Then the per-step angles are summed into a running total (view k's absolute angle).

**(e) Rotating views into one frame — `ry()` + the merge.** *Theory:* to put every shot in a
common frame we undo the robot's turn by rotating shot k's points about the **vertical (camera
Y) axis** by its cumulative angle. The rotation matrix ([scan360.py:138](../pointcloud/scan360.py#L138)):
```python
def ry(a):  # rotate about Y (vertical)
    c, s = cos(a), sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
```
`build_from_session()` ([scan360.py:217](../pointcloud/scan360.py#L217)) ties it together:
```python
angles = cumulative_angles(dirs, nominal_step, measure=...)   # Track B (or known-angle)
for d, ang in zip(dirs, angles):
    pts, cols = back_project(d)
    pts = pts @ ry(ang).T            # rotate this view into view-0's frame
    all_pts.append(pts)
pts, cols = voxel_downsample(np.concatenate(all_pts), ..., VOXEL)
write_ply(out, pts, cols)
```

**(f) Downsample — `voxel_downsample()`** ([scan360.py:145](../pointcloud/scan360.py#L145)). After stacking,
many points pile up; we keep **one point per 1 cm cube** so the cloud is light and even:
```python
keys = np.floor(pts / voxel).astype(np.int64)        # which cell each point is in
_, idx = np.unique(keys, axis=0, return_index=True)  # keep one per cell
```

**(g) Saving — `write_ply()`** ([scan360.py:154](../pointcloud/scan360.py#L154)) writes a **binary
little-endian colored PLY** (xyz + rgb). This exact format is what `view3d.py` reads back.

**CLI / standalone use** (no robot needed):
```bash
python3 pointcloud/scan360.py captures/scan_<ts>            # rebuild, MEASURED angle (default)
python3 pointcloud/scan360.py captures/scan_<ts> --known    # trust the timed step instead
python3 pointcloud/scan360.py captures/scan_<ts> --angle 40 # force a fixed step angle
python3 pointcloud/scan360.py --calibrate --turned 470      # pulsed calibration helper
```

### 3.5 `view3d.py` — the on-Pi 3D viewer

- **What:** opens a `.ply` in an OpenCV window and lets you orbit it (mouse / arrows). Pure
  numpy + cv2, so it works on the Pi screen with no Open3D.
- **Theory:** a tiny software renderer — it rotates points by the view orientation, does a
  perspective divide (`u = f·X/Z + w/2`), and paints far points first so near ones land on top.
- **Reads** the same PLY format `scan360.write_ply` produces ([view3d.py:34](../pointcloud/view3d.py#L34)).
- **Connects to:** launched by `drive.py` on the **Y** key.

### 3.6 `merge_clouds.py` — the laptop ICP merge (highest quality)

- **What:** merges many capture folders into one `merged.ply` using **Open3D registration**.
- **Runs on:** laptop (Open3D). This is the "nice" merge when you want the best result.
- **Theory — ICP (Iterative Closest Point).** Instead of trusting a single rotation angle, ICP
  *geometrically* snaps overlapping clouds together: repeatedly match nearest points and solve
  for the rigid transform that minimizes their distance. We seed it with the known yaw so it
  starts close, then refine ([merge_clouds.py:90](../pointcloud/merge_clouds.py#L90), [:99](../pointcloud/merge_clouds.py#L99)):
  ```python
  init = yaw_seed(angle)                      # start from "rotated about vertical by angle"
  res  = registration_icp(src, dst, ..., init, PointToPlane())  # snap together
  cumulative = cumulative @ res.transformation # chain into cloud-0's frame
  ```
  It prints a **`fitness`** per pair = fraction of points that overlapped (higher = better).
- **Connects to:** consume the raw `scan_<ts>/shot_*/` folders the robot saved:
  `merge_clouds.py --angle 40 captures/scan_<ts>/shot_*/`.

### 3.7 `render_cloud.py` — static preview (laptop)

- **What:** saves a front + top-down PNG of a `cloud.ply` (no interactive window). Handy for
  sanity-checking a cloud over SSH. ([render_cloud.py](../pointcloud/render_cloud.py))

### 3.8 `drive.py` — the conductor (teleop + scan)

- **What:** raw-keyboard WASD/QE teleop + camera servos, and the scan workflow:
  - **R** → `scan360.run_scan()` (capture 9 shots) · **T** → `scan360.build_from_session()`
    (build, measured angle) · **Y** → launch `view3d.py` · **V** → single capture.
- **Runs on:** Pi. It is the only file that talks to the keyboard and orchestrates the others.
- **Theory note:** it uses a *hold-to-move* model — `select()` with a timeout, and if no key
  arrives the motors stop. Keeps the robot safe if you let go.
- **Connects to:** `RasBot` (movement), `StereoCapture` (camera), `scan360`, `view3d`.

### 3.9 `rasbot/` — the hardware API

- **What:** [setup_and_api/api/robot.py](../setup_and_api/api/robot.py) is the official `RasBot`
  class (imported as `rasbot.api`). Movement, servos, LEDs, sensors over I²C. `constants.py`
  holds the register map and chassis geometry.
- **Theory — mecanum wheels.** Four independently-driven wheels can move/strafe/rotate by mixing
  per-wheel speeds. For an **in-place spin** the two sides turn opposite directions
  ([robot.py:167](../setup_and_api/api/robot.py#L167)):
  ```python
  def rotate_left(self, speed=100):   # in-place CCW
      self._apply_motors(-speed, -speed, speed, speed)
  ```
  General motion comes from `_compute_mecanum_speeds()` ([robot.py:120](../setup_and_api/api/robot.py#L120)),
  which decomposes a speed+direction(+rotation) into the four wheel speeds.
- **Connects to:** `scan360.run_scan()` and `drive.py` call `rotate_left/right`, `stop`, etc.

### 3.10 `clean_captures.py` — reset

Deletes captures so the next run starts fresh. `--clouds` keeps the raw camera data and deletes
only generated clouds (recursively, including `scan_*/shot_*/` and `merged_360.ply`). Stdlib
only, runs anywhere.

---

## 4. The math, collected in one place

| Step | Equation / idea | Where |
|---|---|---|
| Stereo → depth | `depth = fx · baseline / disparity` | (camera does it; theory in [D405 doc](D405_Depth_Point_Clouds.md)) |
| Depth → 3D (back-projection) | `X=(u−ppx)Z/fx,  Y=(v−ppy)Z/fy,  Z=depth` | `back_project` |
| Measure rotation between two shots | essential matrix `E` from matched features → `R` → yaw | `estimate_yaw` |
| Put views in one frame | rotate shot k by its cumulative yaw about **Y** (vertical) | `ry` + `build_from_session` |
| Combine many overlapping views (laptop) | ICP minimizes nearest-point distances | `merge_clouds` |
| Keep the cloud light | one point per voxel (1 cm cube) | `voxel_downsample` |

**Coordinate frame** (camera's own): **X** = right, **Y** = down, **Z** = forward (out of the
lens). "Vertical axis" = Y, which is why every rotation here is about Y (`ry`).

---

## 5. Two ways to build the 360 cloud — when to use which

| | On-Pi (`scan360`) | Laptop (`merge_clouds`) |
|---|---|---|
| Library | numpy + cv2 | Open3D |
| Alignment | back-project + rotate by **measured** angle | RANSAC/FGR + **ICP** (geometric) |
| Speed | seconds, on the robot | slower, on the laptop |
| Quality | good, robust to bad timing | best (snaps overlaps) |
| Use when | live demo / no laptop | you copied the shots off and want the cleanest map |

Both read the **same** `scan_<ts>/shot_*/` folders, so you can do the quick Pi merge for a demo
and the ICP merge later for the deliverable — no re-capture needed.

---

## 6. Call map for a full scan (what happens on R then T)

```
drive.py  [R]  ── scan360.run_scan() ─┬─ bot.stop / bot.rotate_left   (rasbot API, I2C)
                                      └─ cam.save_to()                (rs_capture -> shot_NN/)
drive.py  [T]  ── scan360.build_from_session()
                     ├─ cumulative_angles()
                     │     └─ estimate_yaw()  per pair   (cv2: ORB → essential → pose)
                     ├─ back_project()  per shot         (depth.npy + intrinsics → 3D)
                     ├─ ry() rotate each view
                     ├─ voxel_downsample()
                     └─ write_ply()  → scan_<ts>/merged_360.ply
drive.py  [Y]  ── view3d.py  (read_ply → orbit window)
```

---

## 7. Where to plug in (extension points for teammates)

- **Your own stereo depth.** The project asks us to compute depth from `ir_left/right`
  ourselves (StereoSGBM → `depth = fx·baseline/disparity`). See
  [D405_Depth_Point_Clouds.md §5](D405_Depth_Point_Clouds.md). Write your depth into a capture
  folder's `depth.npy` and the whole pipeline downstream just works.
- **Better registration.** Improve `estimate_yaw` (e.g., add a homography fallback for
  low-parallax scenes) or the ICP parameters in `merge_clouds`.
- **Color.** Right now clouds are grayscale from IR. The `rasbot` API and `capture_all()` can
  give a color frame — add an `rgb.png` to the contract and color the points in `back_project`.
- **Navigation / autonomy.** Lives in `rasbot` + `drive.py` (line sensors, ultrasonic, the
  WASD/line-following modes from the brief). It only needs the movement API, not the cloud code.
- **New viewer / export.** Anything that reads the PLY (`view3d.read_ply`) or a standard `.ply`
  works — e.g. [3dviewer.net](https://3dviewer.net), `f3d`, Blender.

**Golden rule:** keep the **capture-folder contract** (Section 2) intact and your part stays
decoupled from everyone else's.

---

## 8. File index

| File | Runs on | One-line role |
|---|---|---|
| [rs_capture.py](../camera/rs_capture.py) | Pi | D405 capture → capture folder (the producer) |
| [capture.py](../camera/capture.py) | Pi | standalone ENTER-to-capture |
| [scan360.py](../pointcloud/scan360.py) | Pi | 360 sweep + measured-angle merge (the heart) |
| [view3d.py](../pointcloud/view3d.py) | Pi/laptop | orbit a `.ply` (numpy+cv2 viewer) |
| [drive.py](../wasd/drive.py) | Pi | WASD teleop + R/T/Y/V scan workflow (the conductor) |
| [make_pointcloud.py](../pointcloud/make_pointcloud.py) | laptop | one capture → `cloud.ply` (Open3D) |
| [merge_clouds.py](../pointcloud/merge_clouds.py) | laptop | many captures → `merged.ply` via ICP |
| [render_cloud.py](../pointcloud/render_cloud.py) | laptop | static front/top preview PNG |
| [clean_captures.py](../pointcloud/clean_captures.py) | either | delete captures / generated clouds |
| [setup_and_api/api/robot.py](../setup_and_api/api/robot.py) | Pi | `RasBot` hardware API (movement, servos, sensors) |

For **how to run** these, see [RUN_GUIDE.md](RUN_GUIDE.md) and [CHANGE_GUIDE.md](CHANGE_GUIDE.md).
For the **camera/depth theory**, see [D405_Depth_Point_Clouds.md](D405_Depth_Point_Clouds.md).
