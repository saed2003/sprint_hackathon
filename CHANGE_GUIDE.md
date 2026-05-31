# Change Guide — 360 scan on the Pi, capture while driving, clean up

This explains the recent changes and **exactly** how to use them end to end.

## What changed

1. **`drive.py` — press `R` to capture a full 360 on the Pi, `T` to build the cloud.**
   The robot rotates in place and takes **9 shots** (40° apart) into
   `captures/scan_<ts>/` (R), then merges them into one 3D point cloud
   `captures/scan_<ts>/merged_360.ply` (T) — no laptop needed. Press **Y** to view it,
   or **V** for a single capture in place (handy for testing).
2. **New `scan360.py`** — the 360 sweep + a pure-numpy merge (no Open3D, which has no
   Raspberry Pi wheel). The merge **measures the real per-step angle from the camera
   images** (ORB → essential matrix), so it's robust to the open-loop rotation
   overshooting. Also runs standalone to rebuild a cloud, and to **calibrate** the
   rotation timing (pulsed — matches how a scan actually moves).
3. **`make_pointcloud.py` can now do many captures at once** (`--all` or a list of
   folders). Before it only did one per run — that was the confusing part.
4. **New `clean_captures.py`** — one command to delete captures so you can start fresh.
5. New shared module **`rs_capture.py`** — the camera code that `capture.py`, `drive.py`
   and `scan360.py` all share, so every capture folder is identical.

> **Open-loop note.** The RasBot has no IMU/odometry, so the 360 *rotation* is timed and
> tends to overshoot. We no longer trust that timing for the cloud: the merge now
> **measures** the true angle each step turned from the overlapping camera images
> (falling back to the nominal step only when it can't). Calibration below still matters
> so the steps land near 40° (keeps enough image overlap to measure). For a still-cleaner
> result, copy the raw shots to the laptop and run the **ICP** merge (`merge_clouds.py`).

---

## Where each script runs

- **On the robot (Raspberry Pi):** `drive.py`, `capture.py` — these talk to the D405.
  They only need `pyrealsense2 + numpy + opencv` (already installed on the Pi).
- **On the laptop (the `.venv` with Open3D):** `make_pointcloud.py`, `merge_clouds.py`,
  `render_cloud.py` — these build/merge/view point clouds.
- **`clean_captures.py`** runs on either (standard library only).

So the flow is: **capture on the Pi → copy the `captures/` folder to the laptop →
make + merge clouds on the laptop.**

---

## Step 0 — Calibrate the rotation (do this ONCE, then re-check if the floor/battery changes)

The merge measures the real angle, but the steps still need to land near 40° so consecutive
shots overlap enough to be measured. Calibrate the way the scan actually moves — short
start/stop pulses, **not** one long continuous spin:

```bash
cd ~/sprint_hackathon
python3 scan360.py --calibrate          # does 8 scan-like pulses; mark heading, watch it
```

Measure the **total degrees it turned**, then let it do the math for you:

```bash
python3 scan360.py --calibrate --turned 470   # prints the exact SCAN_SEC_PER_DEG to set
```

or set it yourself in `scan360.py`:

```
SCAN_SEC_PER_DEG = (total pulse seconds) / (degrees turned)
# e.g. 8 pulses x 0.67 s = 5.33 s of driving that turned 470°  ->  5.33 / 470 = 0.01134
```

Edit `SCAN_SEC_PER_DEG` near the top of [scan360.py](scan360.py), then re-run `--calibrate`
to confirm it now turns ~360° total. If the cloud later looks "unwound"/mirrored, flip
`SCAN_DIR` (1 ↔ -1). If it coasts/overshoots a lot, set a small `SCAN_BRAKE_TAP` (e.g. 0.05).

---

## Step 1 — Drive and scan (everything on the Pi)

```bash
python3 drive.py
```

Drive with **W A S D / Q E**, aim with the **arrow keys**. At a spot you want to map:

- Press **R** → the LEDs turn **blue** and the robot runs the 360 capture by itself:
  stop → shoot → rotate 40° → stop → shoot … ×9. **Don't touch it** during the sweep.
- Press **T** → it builds the cloud on the Pi (measuring the real per-step angle from
  the images) and prints `--- cloud ready -> captures/scan_<ts>/merged_360.ply ---`.
- Press **Y** → orbit the cloud on the Pi screen.
- Press **V** → a single capture in place → `captures/<timestamp>/` (for testing).
- The first capture is a little slow (camera warm-up ~1 s); after that it's fast.

Quit with **ESC** (also closes the camera safely).

The 3D map for each spot is `captures/scan_<ts>/merged_360.ply`. Copy it off the Pi and
open it in any PLY viewer (MeshLab, CloudCompare, or Open3D). That's the deliverable —
**no laptop processing required.**

### Rebuild a scan without re-driving
Re-merge an existing scan (e.g. to compare measured vs timed, or after flipping the sign):

```bash
python3 scan360.py captures/scan_20260531_1700              # measured angle (default)
python3 scan360.py captures/scan_20260531_1700 --known      # trust the timed step instead
python3 scan360.py captures/scan_20260531_1700 --angle 40   # force a fixed step angle
python3 scan360.py captures/scan_20260531_1700 --dir -1     # flip rotation sign
```

---

## Step 2 (OPTIONAL) — Clean ICP merge on the laptop

The on-Pi cloud is open-loop and approximate. For a polished result, copy the raw shots
to the laptop (which has Open3D) and run ICP. Copy over (run on the **laptop**):

```bash
scp -r pi@<robot-ip>:~/sprint_hackathon/captures ~/sprint_hackathon/
```

Then merge one scan's shots, in order, seeding the known step angle:

```bash
.venv/bin/python merge_clouds.py --angle 40 captures/scan_20260531_1700/shot_*/
```

This writes `merged.ply` (ICP-refined). The steps below (`make_pointcloud.py`, etc.)
are the lower-level laptop tools if you want per-shot clouds.

---

## Per-shot point clouds (on the laptop, optional)

**This was your question:** `make_pointcloud.py` makes **one `cloud.ply` per capture
folder** — it does **not** automatically do all of them unless you tell it to. Three ways:

```bash
cd ~/sprint_hackathon

# A) just the newest capture (default, no arguments)
.venv/bin/python make_pointcloud.py

# B) EVERY capture folder, in one command   <-- "do all of them"
.venv/bin/python make_pointcloud.py --all

# C) only the ones you name
.venv/bin/python make_pointcloud.py captures/20260531_120101 captures/20260531_120130
```

Each run writes `cloud.ply` + `cloud_preview.png` **inside that capture's own folder**.
So after `--all` you have one `cloud.ply` per shot you captured.

> `cloud.ply` is **one capture's** 3D points (one viewpoint). It is NOT the combined
> result — combining happens automatically in the 360 scan (Step 1), or with
> `merge_clouds.py` below.

---

## Merge into one cloud (on the laptop)

Give the captures **in the order you took them** (oldest → newest). If you rotated in
roughly equal steps, seed the angle so alignment is fast and reliable:

```bash
# merge ALL captures (oldest -> newest)
.venv/bin/python merge_clouds.py --angle 20

# or merge specific ones, in order
.venv/bin/python merge_clouds.py --angle 20 captures/A captures/B captures/C
```

This writes **`merged.ply`** + **`merged_views.png`** in the project folder and prints a
`fitness` score per pair (higher = better overlap; see `POINTCLOUD_GUIDE.md` §3).

View it (mouse to orbit):

```bash
.venv/bin/python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('merged.ply')])"
```

---

## Clean up captured points

`clean_captures.py` deletes captured data so the next scan starts fresh. It **asks
first** unless you pass `--yes`.

```bash
# delete EVERYTHING: all captures/ folders + merged.ply + merged_views.png
python3 clean_captures.py

# same, without the confirmation prompt
python3 clean_captures.py --yes

# keep the raw captures, delete only the GENERATED files
# (cloud.ply, cloud_preview.png, cloud_views.png, merged.ply, merged_views.png)
python3 clean_captures.py --clouds
```

Use `--clouds` when you want to re-run `make_pointcloud.py` / `merge_clouds.py` from
scratch but keep the original camera data. Use the plain command to wipe everything.

---

## One-look cheat sheet

```bash
# ── on the Pi: calibrate once, then scan ──
python3 scan360.py --calibrate --turned <deg>  # pulsed calibrate -> set SCAN_SEC_PER_DEG
python3 drive.py                               # drive; R = 360 capture, T = build, Y = view
                                               #         V = single capture
# result per spot: captures/scan_<ts>/merged_360.ply   (view with: python3 view3d.py <ply>)

# ── OPTIONAL: cleaner ICP merge on the laptop ──
scp -r pi@<robot-ip>:~/sprint_hackathon/captures ~/sprint_hackathon/
.venv/bin/python merge_clouds.py --angle 40 captures/scan_<ts>/shot_*/

# ── start over ──
python3 clean_captures.py                  # delete captures + merged output
```
