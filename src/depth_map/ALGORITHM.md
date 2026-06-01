# Depth Map Algorithm Explained

## The core idea: stereo disparity

You have two cameras side by side (left and right), like your two eyes.
Because they sit at slightly different positions, every object lands at a
slightly different **horizontal** spot in each image.

- A **near** object jumps a lot between the left and right image.
- A **far** object barely moves.

That horizontal shift, measured in pixels, is called **disparity**.

```
depth  ∝  1 / disparity
```

So if we can measure disparity for every pixel, we get a depth map for free.
In the sample images, the backpack (close) shifts a lot → big disparity →
shows up bright. The room behind it barely shifts → small disparity → dark.

## Step by step (what the code does)

### 1. Load both images as grayscale
Matching only needs brightness patterns, not color. IR images are already
gray.

### 2. Match every pixel between left and right — StereoSGBM
This is the heart of it. We use OpenCV's **Semi-Global Block Matching (SGBM)**.

For each pixel in the left image:
1. Take a small square window around it (`block_size`).
2. Slide that window across the right image, only horizontally, up to
   `num_disparities` pixels.
3. At each shift, score how well the windows match (sum of absolute
   differences of pixel intensities).
4. The shift with the best score = that pixel's disparity.

**Why "block"?** A single pixel is ambiguous (many pixels have the same gray
value). A window of pixels has a more unique texture, so the match is reliable.

**Why "semi-global"?** Plain block matching decides each pixel alone, which
gives noisy, holey results. SGBM adds a **smoothness penalty** (`P1`, `P2`):
neighboring pixels are encouraged to have similar disparities, *unless* there's
a real edge. It enforces this along multiple directions across the image
("semi-global"), giving much cleaner maps. This is the classic Hirschmüller
2008 method.

Other parameters clean up the result:
- `uniquenessRatio` — throw away matches that are not clearly the best (avoids
  guessing in textureless areas).
- `disp12MaxDiff` — left-right consistency check: match L→R must agree with
  R→L, else reject (kills occlusion errors).
- `speckleWindowSize` / `speckleRange` — remove small noisy blobs.

### 3. Convert disparity → real depth (optional)
If you know the camera **focal length** (in pixels) and the **baseline**
(distance between the two cameras, in meters):

```
depth = (focal_length × baseline) / disparity
```

Without those numbers you still get a perfectly usable *relative* depth map —
bright = near, dark = far.

### 4. Colorize for viewing
Normalize disparity to 0–255 and apply a JET colormap so it's easy to see.

## Important assumption: rectification

The matching only searches **horizontally**. That works only if the two images
are **rectified** — same object lies on the same image *row* in both. Real
stereo rigs do this with a one-time calibration (`cv2.stereoRectify`). The
sample IR pair already lines up well enough to demo. If your real cameras are
not calibrated, the depth map will look broken — calibrate first.

## Why SGBM and not alternatives

| Method | Note |
|--------|------|
| `StereoBM` (plain block match) | Faster, but noisy and holey. Good only for high-texture scenes. |
| **`StereoSGBM`** (used here) | Best accuracy/speed tradeoff in OpenCV. Default choice. |
| Deep learning (RAFT-Stereo, etc.) | Higher quality, but needs GPU + a trained model. Overkill for a classic pipeline. |

## How to run

```bash
python src/depth_map/depth_map.py
```

Outputs `depth_map_result.png`. Or import and call:

```python
from depth_map import compute_disparity, disparity_to_depth, colorize

disp  = compute_disparity("left.png", "right.png")
depth = disparity_to_depth(disp, focal_length_px=700, baseline_m=0.06)
```

## Knobs to tune if results look bad

- Disparity too small / object too close to measure → raise `num_disparities`
  (keep multiple of 16).
- Too noisy → raise `block_size` (smoother) or `uniquenessRatio`.
- Losing fine detail → lower `block_size`.
- Big black holes → scene has flat textureless surfaces; SGBM can't match
  those. Add texture or accept the holes.
```
