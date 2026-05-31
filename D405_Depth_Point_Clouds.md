# Street View Robot — How It Works (D405 + Depth + Point Clouds)

This file explains, for our project:

1. How the Intel RealSense **D405** camera actually works
2. The **stereo depth theory** (the math from the videos)
3. What [`capture.py`](capture.py) does, line by line
4. What **we** need to build for this project
5. How to compute depth **ourselves** (the project requirement)
6. Which **libraries** to use and why

All numbers below (`fx`, baseline, etc.) are the **real values measured from our own camera**.

---

## 1. How the D405 works

The D405 is a **stereo depth camera**. Inside it there are **two infrared (IR) cameras**
sitting side by side, separated by a small fixed distance:

```
        baseline = 18.03 mm
   ┌──────────────┐   ┌──────────────┐
   │  LEFT imager │   │ RIGHT imager │      <-- two IR cameras
   └──────────────┘   └──────────────┘
            \                /
             \              /
              \            /
               \          /
                 (scene)
```

It works exactly like **your two eyes**:

- Each imager sees the same scene from a slightly different position.
- An object **close** to the camera appears in a **very different spot** in the left vs right image.
- An object **far away** appears in **almost the same spot** in both images.
- That left↔right shift is called **disparity**. Big disparity = close. Small disparity = far.

The camera (or our code) matches points between the two images, measures the disparity for
every pixel, and converts it into a **distance** for every pixel. That grid of distances is
the **depth image**.

### Key facts about *our* D405 (measured)

| Property | Value | Meaning |
|---|---|---|
| Resolution used | 848 × 480 | size of each image |
| `fx`, `fy` (focal length) | **422.06 px** | how "zoomed in" the lens is, in pixels |
| `ppx`, `ppy` (principal point) | 419.1, 242.65 | the optical center pixel |
| **baseline** | **18.03 mm** | distance between the two IR cameras |
| `depth_scale` | 0.0001 | raw depth value × this = **meters** |
| IR projector | **NONE** | see warning below |

### ⚠️ Very important: the D405 has NO laser projector

Many RealSense cameras (D435, etc.) shine an invisible **IR dot pattern** onto the scene so
that even blank walls get "texture" to match. **Our D405 does not have this.** We confirmed it:
`Emitter Enabled` and `Laser Power` are *not supported*.

This means the D405 is **passive stereo** — it can only measure depth where the scene already
has visible detail/texture. Practical consequences for us:

- **Blank white walls, glossy floors → bad or no depth** (nothing to match left↔right).
- **Good, even lighting matters.** Too dark = no detail = no depth.
- Textured surfaces (posters, books, patterned objects, edges) → great depth.
- The D405 is **short-range** (best roughly 0.1 m – 1 m, usable to a few meters). It is designed
  for close work, not large rooms — keep this in mind when placing capture spots.

### Factory calibration

The two imagers were precisely aligned and calibrated **at the factory**. That is why we can
trust `fx`, `baseline`, etc. directly, and why the built-in depth "just works". When we compute
depth ourselves, we will reuse these same calibrated numbers.

---

## 2. The stereo depth theory (the math from the videos)

This is the single most important equation in the whole project:

```
              fx * baseline
   depth  =  ───────────────
                disparity
```

Where:

- `depth`   = distance to the point, in **meters**
- `fx`      = focal length in pixels (**422.06** for us)
- `baseline`= distance between the two cameras in **meters** (**0.01803 m** for us)
- `disparity` = how many **pixels** a point shifted between the left and right image

**Intuition:** if a point barely moves between the two images (small disparity), the bottom of
the fraction is small, so depth is **large** (far away). If a point moves a lot (large disparity),
depth is **small** (close). This matches the "your two eyes" idea above.

### From depth to a 3D point (back-projection)

A depth image only gives distance `Z` for each pixel. To make a **point cloud** we need full
3D coordinates `(X, Y, Z)` for each pixel `(u, v)`. Using the intrinsics:

```
   Z = depth(u, v)
   X = (u - ppx) * Z / fx
   Y = (v - ppy) * Z / fy
```

Do this for every pixel → you get a cloud of 3D points = a **point cloud**. That is the core of
what the project asks for.

### The two steps, summarized

```
  IR left + IR right ──(stereo matching)──► disparity ──(eqn above)──► depth
                                                                          │
                                                          (back-projection)
                                                                          ▼
                                                                   3D point cloud
```

---

## 3. What `capture.py` does

[`capture.py`](capture.py) connects to the D405, grabs frames, and saves everything we need.
Section by section:

**Setup the streams** — we ask the camera for three things at 848×480, 30 fps:
```python
config.enable_stream(rs.stream.depth,      W, H, rs.format.z16, FPS)  # built-in depth
config.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8,  FPS)  # LEFT  IR image
config.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8,  FPS)  # RIGHT IR image
```
We grab the IR pair because **we** need them to compute depth ourselves later. We also grab the
camera's own depth so we can **check our work against it**.

**Read the calibration** — once at startup we read the numbers from Section 1:
```python
depth_scale = depth_sensor.get_depth_scale()   # raw value -> meters
intr = depth_profile.get_intrinsics()           # fx, fy, ppx, ppy
baseline_m = ...extrinsics between the two IR cameras...
```
These get saved into `intrinsics.txt` with every capture so the depth/point-cloud scripts can use them.

**Capture loop** — each time you press ENTER:
- it throws away a few frames first so **auto-exposure settles** (otherwise the first frame is dark),
- grabs one synced set of `depth`, `ir_left`, `ir_right`,
- converts them to NumPy arrays,
- saves them into `captures/<timestamp>/`.

**What gets saved per capture:**

| File | Type | Purpose |
|---|---|---|
| `depth.npy` | uint16 array | the camera's depth, in raw units (× `depth_scale` → meters). Our **"ground truth"** to compare against. |
| `depth_color.png` | image | colorized depth, just to look at |
| `ir_left.png`, `ir_right.png` | images | the stereo pair → **input to our own depth algorithm** |
| `intrinsics.txt` | text | `fx, fy, ppx, ppy, depth_scale, baseline` → needed for point clouds |

---

## 4. What we need to build for this project

The project = drive a robot around a room, and at each spot build a **360° point cloud**. Software
pieces, in build order:

1. **`capture.py`** ✅ done — get real frames from the camera.
2. **Depth → point cloud** — turn one capture's `depth.npy` + `intrinsics.txt` into a 3D `.ply`
   file we can view (uses the back-projection math in Section 2).
3. **Our own stereo depth** — compute depth from `ir_left`/`ir_right` ourselves (Section 5). This
   is the **explicit project requirement**: *"design and implement an algorithm to compute depth
   from the stereo pairs."*
4. **Merge rotated views into 360°** — at one spot the robot rotates in steps, captures a cloud at
   each angle, rotates each cloud by its known angle, and fine-aligns them with **ICP** into one
   single 360° cloud.
5. **Robot control** — `forward()`, `rotate()` etc. over I2C (the hardware abstraction), plus the
   two modes: WASD manual control and IR line-following.

Steps 2, 3 and 4 can be developed **entirely on saved captures on the laptop** — no robot needed.
That is why we start there while the chassis is being built.

---

## 5. How to compute depth OURSELVES (the requirement)

The camera gives depth for free, but the project wants us to *implement the algorithm*. The
standard way, fully supported by OpenCV, is **block matching / Semi-Global Block Matching (SGBM)**:

**Step A — the images are already rectified.** Because the D405 is factory-calibrated, the left
and right IR images are aligned so that a point in the left image lies on the **same horizontal
row** in the right image. So matching only has to search left↔right along a row (not up/down).
This is a huge simplification and it is done for us.

**Step B — block matching.** For each small patch in the left image, slide along the row in the
right image to find the best-matching patch. The horizontal shift of the best match = the
**disparity** for that pixel. OpenCV's `StereoSGBM` does this for the whole image:

```python
import cv2
left  = cv2.imread("ir_left.png",  cv2.IMREAD_GRAYSCALE)
right = cv2.imread("ir_right.png", cv2.IMREAD_GRAYSCALE)

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=128,   # search range (must be multiple of 16)
    blockSize=5,
)
disparity = stereo.compute(left, right).astype("float32") / 16.0  # SGBM returns disp*16
```

**Step C — disparity → depth** using our equation from Section 2:

```python
fx, baseline = 422.06, 0.01803
depth_meters = (fx * baseline) / disparity      # where disparity > 0
```

**Step D — check our work.** Compare `depth_meters` against the camera's own `depth.npy`
(× `depth_scale`). They should roughly agree where both are valid. This validates our algorithm.

> Reminder: because the D405 has no projector (Section 1), **our** stereo will also fail on blank
> walls — that's expected, it's a property of passive stereo, not a bug in our code. Point it at
> textured scenes for the best results.

---

## 6. Libraries we use (and why)

| Library | Install | Why we need it |
|---|---|---|
| **pyrealsense2** | `uv pip install pyrealsense2` | talk to the D405: stream depth + IR, read calibration. ✅ installed |
| **NumPy** | `uv pip install numpy` | all the array math (depth, back-projection). ✅ installed |
| **OpenCV** (`opencv-python`) | `uv pip install opencv-python` | image I/O, **StereoSGBM** for our own depth. ✅ installed |
| **Open3D** | `uv pip install open3d` | build, **view**, and **merge** point clouds; provides **ICP** for the 360° merge. (next step) |

We do **not** need to write stereo matching or ICP from scratch — OpenCV and Open3D provide them.
"Design and implement the algorithm" = wire these together correctly with the right calibration,
understand what each step does, and validate the result.

### Environment note

Everything runs in the project's `.venv/` (Python **3.11**), created with `uv`, because the
system Python (3.14) is too new to have prebuilt `pyrealsense2`. Always run scripts as:

```bash
.venv/bin/python capture.py
```

---

## Quick reference — the whole pipeline

```
  D405 (two IR cameras, 18mm apart, no projector, factory-calibrated)
        │
        ├── ir_left.png, ir_right.png ──► StereoSGBM ──► disparity
        │                                                   │
        │                          depth = fx*baseline / disparity
        │                                                   ▼
        └── depth.npy (camera's own, for checking) ◄──► our depth
                                                          │
                               back-project: X=(u-ppx)Z/fx, Y=(v-ppy)Z/fy, Z=depth
                                                          ▼
                                                   point cloud (.ply)
                                                          │
                              rotate each view by its angle + ICP align
                                                          ▼
                                            single 360° point cloud  ✅ goal
```
