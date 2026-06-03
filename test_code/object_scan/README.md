# Object Scan (experimental — isolated in `test_code/`)

3D-scan a **single object** with the D405 — the camera's real sweet spot (7–50 cm,
sub-mm). This is the "extra credit" idea: instead of spinning in place to map a room,
go *around an object* and reconstruct it as a clean point cloud + a textured mesh.

Everything here lives under `test_code/` and **only reads** the standard capture
folders (and read-only-imports the robot API for the orbit). **Nothing here changes
the running robot** — the room-scan pipeline works exactly as before.

**Two demo objects** are defined in **`config.py`** (pick with `--object`):

| `--object` | What | Why | Best output |
|---|---|---|---|
| **`db5`** (default, **recommended**) | LEGO Speed Champions *007 Aston Martin DB5* #76911 (~17×7×5 cm) | matte LEGO (clean depth) + **solid, chunky, asymmetric** (front≠back) → easy ICP, clean mesh | cloud **and** mesh |
| `teemo` | Funko Pop *Teemo with Mushroom* #1138 (~9 cm) | solid + colourful + asymmetric, but glossy vinyl → some depth holes | cloud + mesh (a bit holey) |

Change one number in `config.py` to retune either (radius, depth gate, crop, shots…).

## ONE command (this is the "main" — capture → merge → mesh → preview)

```bash
cd sprint_hackathon/test_code/object_scan
PY=../../../.venv/bin/python

$PY run.py                  # SELF-TEST: synthetic DB5 car, NO hardware — start here
$PY run.py turntable        # laptop + D405: you spin the OBJECT        (most reliable)
$PY run.py orbit            # Pi: the ROBOT drives around the object     (the ambitious one)
$PY run.py build <session>  # just rebuild from already-captured shots

$PY run.py --object teemo            # switch object (default is db5)
$PY run.py turntable --object teemo  # ...works with any mode

$PY run.py clean                     # delete capture sessions (asks first)
$PY run.py clean --yes               # ...without asking
$PY run.py clean --outputs           # keep raw shots, delete only built .ply / preview .png
$PY run.py clean orbit_<ts>          # delete just one session
```
`clean` is stdlib-only, so it also runs on the Pi (`python3 run.py clean`) to wipe
`captures/` before committing — handy so you don't push old scans.
Each prints the output paths: `merged_object.ply`, `merged_object_mesh.ply`, and 4-view
preview PNGs. Flags after the mode: `--object NAME`, `--shots N`, `--radius M`,
`--dir -1` (if mirrored), `--no-loop` (partial scan), `--no-mesh`.

### Which machine / which python?

- **`synth` and `turntable`**: run on the **laptop** with the **venv** interpreter
  (`.venv/bin/python` = Python 3.11). The system `python` on Arch is 3.14 and can't load
  pyrealsense2/Open3D — always use `.venv/bin/python` (that's why the examples set `PY=`).
- **`orbit`**: runs on the **Pi** with **`python3`** (the robot has pyrealsense2 + smbus but
  **no Open3D**). So `orbit` is **two steps**: the Pi captures, then it prints a line telling
  you to copy the session to the laptop and finish with `run.py build <session> --object …`
  (the merge/mesh needs Open3D). `turntable` does everything in one go because it's all on
  the laptop.

> `synth` is just the no-hardware self-test (default when you give no mode) — it doesn't move
> the robot or use the camera; it proves the merge/mesh pipeline on a fake object.

> **Either the figure moves or the robot moves — same result.** `turntable` keeps the
> camera still and you rotate the figure (rock-solid). `orbit` drives the robot around
> the figure (uses the camera to aim + hold the radius). Both write identical capture
> folders, so the merge/mesh is the same. For a pre-recorded demo, do a couple of takes
> and keep the best.

### The two capture paths

| Path | `run.py` mode | Reliability | What moves |
|---|---|---|---|
| Turntable / hand-stepped | `turntable` | ★★★ rock solid | the **figure** (camera still) |
| Robot orbit (stop-and-shoot) | `orbit` | ★★ needs tuning | the **robot** (figure still) |

### Robot orbit — one-time calibration on the Pi (real floor/battery)
```bash
python3 capture_orbit.py --calibrate-turn 3.0   # spin 3s, measure deg -> set SEC_PER_DEG
python3 capture_orbit.py --calibrate-fwd  3.0   # drive 3s, measure m  -> set SEC_PER_M
python3 run.py orbit                            # then scan; copy captures/ to laptop to build
```
The orbit uses the camera to **aim** (pan servo centres the figure) and to **hold the
radius** (drives in/out to keep ~40 cm), so open-loop drift matters less. If the model
comes out mirrored, rebuild with `--dir -1`; flip `PAN_SIGN` in `capture_orbit.py` if the
camera turns away from the figure.

## Files

| File | Role |
|---|---|
| `run.py` | **the one entry point** — capture (synth/turntable/orbit) → build → mesh → preview |
| `config.py` | object + scan settings (Teemo dims, radius, depth gate, crop, shots) — tune here |
| `segment.py` | capture folder → object-only point cloud (depth gate + RANSAC plane removal + crop) |
| `build_object.py` | **the merge (laptop, sharpest)** — per-view masked features → Open3D point-to-plane ICP refine, windowed redundant ring + edge gating + symmetry-fold veto + largest-component pose-graph global optimisation, fuse → `merged_object.ply` |
| `trim_car.py` | post-process: cut the stand off a `merged_object.ply` → `merged_car.ply` (+ mesh) |
| `mesh.py` | point cloud → watertight textured **mesh** (Poisson; `--bpa` ball-pivoting fallback) |
| `preview.py` | headless 4-view PNG of any cloud/mesh (works over SSH) |
| `capture_session.py` | turntable/hand-stepped capturer (own RealSense pipeline, **aligned colour** → coloured model) |
| `capture_orbit.py` | robot stop-and-shoot orbit (Pi): pan-servo aim + radius-hold + turn-drive-turn stepping |
| `build_object_pi.py` | the friend's **pure-numpy/cv2** feature merge — runs on the **Pi** (no Open3D) |
| **`run_pi.py` / `run_pi2.py` / `run_pi_template.py`** | **all-on-the-Pi** capture+merge → `.ply` (no Open3D, no mesh) — see below |
| `db5_template.npz` | canonical DB5 cloud for `run_pi_template.py` (made once on the laptop) |
| `_synth_test.py` | renders a synthetic Teemo-scale scan so you can test the whole build with no camera |

## Run it ALL on the Pi (no Open3D) — `run_pi*.py`

The normal merge needs Open3D (laptop only). These three scripts do **capture + merge → `.ply`
entirely on the Pi** in pure numpy/cv2 (reusing `build_object_pi.py`). **No mesh.** Each is one
command; each also takes `--build captures/orbit_<ts>` to skip capture and merge an existing scan.
They are new, isolated files — they change nothing else and never touch the running robot.

| Script | What it adds | Output |
|---|---|---|
| `run_pi.py` | the friend's **feature merge** (fast, coarse) | `object_pi.ply` |
| `run_pi2.py` | pure-numpy **point-to-plane ICP** refine + **symmetry-fold veto** + **known-model stand-crop** (drops the rotationally-symmetric stand using the car's known height, so ICP/features lock on the distinctive car). `--relax-iters` pose-graph relax is **off by default** (it spread the cloud). | `object_pi_hq.ply` (car-only) |
| `run_pi_template.py` | **model-to-frame**: snaps each view onto a saved canonical **template** with **leeway** (trimmed ICP + orientation search + drops non-conforming views — because captures vary) | `object_pi_tpl.ply` |

```bash
# on the Pi (RasBot + pyrealsense2 + cv2 + numpy; scipy optional; NO Open3D):
python3 run_pi.py                       # --object teemo | --shots 36 | --radius 0.35 | --sift
python3 run_pi2.py                      # --reg-voxel | --icp-iters | --win | --relax-iters | --keep-stand
python3 run_pi_template.py              # --leeway 0.03 | --template db5_template.npz | --fit-min 0.45
python3 run_pi2.py --build captures/orbit_<ts>     # any of them: merge an existing scan, no robot
```

The template is made **once on the laptop** (needs Open3D) and committed so the Pi just loads it:
```bash
../../../.venv/bin/python -c "import numpy as np,open3d as o3d; p=o3d.io.read_point_cloud('captures/<good_scan>/merged_car.ply').voxel_down_sample(0.003); np.savez_compressed('db5_template.npz', pts=np.asarray(p.points,np.float32), cols=(np.asarray(p.colors)*255).astype(np.uint8))"
```

> ⚠️ **Honest quality note.** All three Pi merges come out **noticeably fuzzier than the laptop
> `build_object.py`**. The laptop wins because of Open3D's *joint* robust pose-graph optimisation
> (all views constrained against each other at once); the numpy stand-ins (BFS+loop, median relax,
> per-view template ICP) can't match it, and the template's leeway trades sharpness for robustness
> to capture variation. **For the sharpest `.ply`: capture on the Pi, build on the laptop** with
> `build_object.py` (+ `trim_car.py`) — or run `build_object.py` on the Pi *if* it has Open3D. The
> `run_pi*` scripts are for getting a usable cloud with **no laptop and no Open3D**.

## How the merge works (and why it reuses the room-scan idea)

`scan360.py` merges room views by rotating each about a vertical axis through the
**camera**. An object orbit is the same operation about a vertical axis through the
**object centre** (`build_object.ry_about`). So:

1. **Segment** each view to the object (it's the closest thing → easy to isolate).
2. **Angle prior**: rotate each view about the object's vertical axis by its
   `angle.txt` (else a uniform 360/N step) into view-0's frame — a good ICP start.
   The axis (x,z) is found by `estimate_rotation_axis` (a single view only sees the
   front surface, so we iteratively push the axis back to the true centre).
3. **Refine + close the loop**: Open3D multiway registration — point-to-plane ICP on
   neighbours + a last→first loop-closure edge → pose-graph global optimization. This
   is what absorbs the robot's open-loop motion error.
4. **Fuse** → voxel downsample → outlier removal → `merged_object.ply` → Poisson mesh.

## Picking the object (this decides whether the demo works)

- **Good:** textured, matte, asymmetric, rigid, ~10–25 cm — a sneaker, stuffed toy,
  potted plant, cereal box, Rubik's cube, figurine.
- **Bad:** shiny/metallic, transparent, plain single-colour smooth, thin, or
  **symmetric** (a ball/cylinder — ICP can't lock the rotation). The D405 is passive
  stereo (no projector), so it needs **texture + good light**.

**DB5 (LEGO 76911) note — the recommended pick:** matte ABS gives the D405 clean depth,
and the car is solid + chunky + clearly asymmetric (front≠back, distinct sides), so ICP
locks easily and Poisson makes a clean mesh. One caveat: it's mostly **silver/light-grey**,
so *colour* is muted — but the **geometric** texture (panel lines, wheels, gaps) carries
the stereo match. **Light it diffusely** (soft, even light; no point-lights/glare) so the
flat-silver parts don't blow out to specular holes. Wheels/underside won't be captured from
a turntable (you get top + sides — normal for a car scan).

**Teemo (Funko vinyl) note:** Pop figures are smooth and partly glossy, which is the
hard case for passive stereo — expect some **depth holes** on plain/shiny patches.
Mitigations: bright, soft, even lighting (no glare); a matte backdrop; and the figure's
colour variation (green/brown/hat + the asymmetric weapon/ears) gives ICP enough to lock.
The merge's outlier removal + Poisson meshing fill small holes, and the **colour** makes
it read as Teemo even where geometry is rough. If depth is very sparse, scan closer
(`--radius 0.30`) and/or take more shots (`--shots 30`). A light dusting of matte/dry-
shampoo spray on a spare figure is the classic trick if you're allowed.

## Tuning knobs

- `build_object.py --zmax 0.45` — tighten to just past your object to cut background.
- `--voxel 0.002` finer / `0.005` coarser+faster. `--crop 0.15` if clutter remains.
- `--dir -1` if mirrored. `--no-loop` for a partial (non-360) scan.
- `mesh.py --depth 10` finer Poisson; `--bpa` if Poisson over-inflates an open scan.
- Orbit aim: in `capture_orbit.py`, flip `PAN_SIGN` if the camera turns *away* from the
  object; raise `ORBIT_SHOTS` (smaller steps = more overlap = more robust ICP).

## Status

- ✅ **One-command pipeline (`run.py`): validated** end-to-end with no hardware for BOTH
  objects at real scale — the synthetic **DB5 car** (~17 cm at 35 cm) and the **Teemo
  figure** (~9 cm at 40 cm) both reconstruct cleanly, full 360, with colour, and mesh.
- ✅ **Turntable capturer**: ready to run with a D405 over USB (`run.py turntable`).
- ⚠️ **Robot orbit** (`run.py orbit`): complete and imports cleanly, with camera aim +
  radius-hold, but the open-loop motion (`SEC_PER_DEG`, `SEC_PER_M`, `PAN_SIGN`) **must be
  tuned on the real robot** — expect to iterate. Since the demo is pre-recorded, do a few
  takes. If it fights you, fall back to `turntable` — same merge, same mesh.

## Worked example: orbit-scan the DB5 car (copy/paste)

Full round-trip — no scp/FileZilla, everything moves over git. The Pi captures; the
laptop builds (Open3D is laptop-only). Replace repo paths if yours differ.

### 1. Laptop — push the code (only the first time, or after code changes)
```bash
cd ~/Desktop/UNI/4_2/sprint/sprint_hackathon
git add test_code/object_scan && git commit -m "object scan pipeline"
git pull --rebase && git push
```

### 2. Pi — over SSH, on the robot's network (uses python3; no Open3D needed here)
```bash
cd ~/sprint_hackathon                 # the repo on the Pi
git pull
cd test_code/object_scan

# --- one time only: calibrate the open-loop motion, then edit the two values it prints
#     into capture_orbit.py (SEC_PER_DEG, SEC_PER_M). Skip on later runs. ---
python3 capture_orbit.py --calibrate-turn 3.0    # measure degrees turned -> SEC_PER_DEG
python3 capture_orbit.py --calibrate-fwd  3.0    # measure metres driven  -> SEC_PER_M

# --- the scan: put the DB5 ~35 cm in front of the camera, well & evenly lit ---
python3 run.py clean --yes            # wipe any old scans so you don't push stale data
python3 run.py orbit --object db5     # robot orbits + captures -> captures/orbit_<ts>/

# --- push the capture back ---
cd ~/sprint_hackathon
git add test_code/object_scan/captures
git commit -m "DB5 orbit capture"
git pull --rebase && git push
```

### 3. Laptop — pull, then build the model (this is the Open3D merge + mesh)
```bash
cd ~/Desktop/UNI/4_2/sprint/sprint_hackathon
git pull
cd test_code/object_scan

# build the newest orbit capture (auto-picks the latest orbit_<ts>):
../../../.venv/bin/python run.py build "$(ls -dt captures/orbit_* | head -1)" --object db5
```
Outputs land in that session folder: `merged_object.ply`, `merged_object_mesh.ply`, and
`*_preview.png` (open the PNGs to eyeball it). View the mesh interactively:
```bash
../../../.venv/bin/python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_triangle_mesh('$(ls -dt captures/orbit_*/merged_object_mesh.ply | head -1)')])"
```
If the model looks **mirrored/smeared**, rebuild with `--dir -1`. If background junk
remains, add `--crop 0.12`. If depth is sparse on the silver, light it more diffusely
(or scan closer: `--radius 0.30` on the Pi next time).
