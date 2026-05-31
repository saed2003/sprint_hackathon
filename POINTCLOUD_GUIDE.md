# Point Cloud Guide — copy & paste

Make a point cloud for each photo, then merge them into one. Every command is meant to be
copy-pasted from the project folder:

```bash
cd ~/sprint_hackathon   # wherever you cloned this repo
```

All scripts run with `.venv/bin/python` (the Python 3.11 env that has the libraries).

---

## 0. (Optional) Take new photos

```bash
.venv/bin/python capture.py
```
Press **ENTER** to save each shot, **q** + ENTER to quit. For a good MERGE later:
rotate the camera only **~15–20° between shots** so they overlap, and aim at a
**textured, well-lit** scene (the D405 has no projector — blank walls give empty depth).

Find your newest captures:
```bash
ls -1dt captures/*/ | head
```

---

## 1. Make a point cloud for EACH photo

Pick the captures you want. To do the **whole newest batch** (example: the 12:18 group):

```bash
for d in captures/20260531_1218*/; do .venv/bin/python make_pointcloud.py "$d"; done
```

Or do them **one at a time** (replace the timestamp with your own):

```bash
.venv/bin/python make_pointcloud.py captures/20260531_121821
.venv/bin/python make_pointcloud.py captures/20260531_121823
.venv/bin/python make_pointcloud.py captures/20260531_121826
.venv/bin/python make_pointcloud.py captures/20260531_121830
.venv/bin/python make_pointcloud.py captures/20260531_121833
```

Each writes `cloud.ply` + `cloud_preview.png` inside that capture's folder.

### Look at one cloud (clean front + top-down render)
```bash
.venv/bin/python render_cloud.py captures/20260531_121821/cloud.ply
```
Opens nothing — it saves `cloud_views.png` in that folder. To **orbit it live with the mouse**:
```bash
.venv/bin/python -c "import open3d as o3d,sys; o3d.visualization.draw_geometries([o3d.io.read_point_cloud(sys.argv[1])])" captures/20260531_121821/cloud.ply
```

---

## 2. Merge the photos into ONE cloud

Give the captures **in the order you took them**. Whole newest batch:

```bash
.venv/bin/python merge_clouds.py captures/20260531_1218*/
```

Or list them explicitly:

```bash
.venv/bin/python merge_clouds.py \
  captures/20260531_121821 \
  captures/20260531_121823 \
  captures/20260531_121826 \
  captures/20260531_121830 \
  captures/20260531_121833
```

This writes **`merged.ply`** + **`merged_views.png`** in the project folder, and prints a
`fitness` score for each pair.

### If you know the rotation between shots (recommended — what the robot will do)
Seed the known angle (degrees) so alignment is fast and reliable:
```bash
.venv/bin/python merge_clouds.py --angle 20 captures/20260531_1218*/
```

### View the merged result
```bash
.venv/bin/python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('merged.ply')])"
```

---

## 3. Reading the `fitness` score (is the merge good?)

`fitness` = fraction of points that overlapped after alignment. **Higher = better.**

| fitness | meaning | fix |
|---|---|---|
| **> 0.6** | good — clean merge | 👍 |
| 0.3 – 0.6 | weak — clouds smear together | rotate **less** between shots (more overlap) |
| **< 0.3** | bad — almost no overlap | re-shoot: small rotations, textured scene, or use `--angle` |

The previous merge looked bad because fitness was ~0.13–0.39 (too little overlap between the
handheld burst frames). The cure is **overlap**, not more processing.

---

## 4. Cleaning the view

`merge_clouds.py` already cleans automatically: it voxel-downsamples (1 cm) and removes
statistical outliers (floating specks). If the merged cloud still looks noisy, the usual causes
and fixes:

- **Smeared / doubled** → bad alignment (low fitness). Re-shoot with more overlap, or use `--angle`.
- **Stray floating dots** → already filtered; for more aggressive cleaning, lower `std_ratio` in
  `merge_clouds.py` (e.g. `std_ratio=1.0`).
- **Far-away noise** → the clouds keep only 0.05–3.0 m; tighten the `< 3.0` in `make_pointcloud.py`
  / `merge_clouds.py` to e.g. `< 1.5` for close scenes (D405 is short-range).

---

## Cheat sheet

```bash
cd ~/sprint_hackathon   # wherever you cloned this repo

# 1) cloud for each photo (newest batch)
for d in captures/20260531_1218*/; do .venv/bin/python make_pointcloud.py "$d"; done

# 2) merge them
.venv/bin/python merge_clouds.py captures/20260531_1218*/

# 3) view the result (mouse to orbit)
.venv/bin/python -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('merged.ply')])"
```
