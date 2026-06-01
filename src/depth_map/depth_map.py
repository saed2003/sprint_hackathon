"""
Stereo depth map from a left/right image pair.

Idea in one line:
    The same object appears at slightly different horizontal positions in the
    left and right camera. That shift is called DISPARITY. Near objects shift a
    lot, far objects shift little. So disparity is an inverse measure of depth.

This file builds a disparity map with OpenCV's Semi-Global Block Matching
(StereoSGBM), then (optionally) converts it to real-world depth in meters.
"""

import cv2
import numpy as np

# --- Intel RealSense D405 IR camera, measured from the device via SDK ---
# Values depend on IMAGE RESOLUTION. The sample images are 848x480, so use
# the 848x480 set. If you capture at a different resolution, re-read the
# intrinsics for that resolution (focal length scales with width).
D405_848x480 = {
    "focal_length_px": 422.06,   # fx
    "baseline_m": 0.018026,      # 18 mm between the two IR imagers
}
D405_1280x720 = {
    "focal_length_px": 637.08,
    "baseline_m": 0.018026,
}

# Default camera params used by the demo below. Test images are 1280x720.
FOCAL_LENGTH_PX = D405_1280x720["focal_length_px"]
BASELINE_M = D405_1280x720["baseline_m"]


def compute_disparity(
    left_img,
    right_img,
    num_disparities=192,
    block_size=11,
    use_wls=False,
    wls_lambda=8000.0,
    wls_sigma=1.5,
    median_size=0,
):
    """
    Compute a disparity map from a rectified stereo pair.

    Parameters
    ----------
    left_img, right_img : np.ndarray or str
        The two stereo images. Can be file paths or already-loaded arrays.
        Assumed to be RECTIFIED (rows line up between left and right).
    num_disparities : int
        How many pixels of shift to search for. Must be divisible by 16.
        Bigger = can measure closer objects, but slower.
    block_size : int
        Size of the matching window (odd number, 3..11). Bigger = smoother
        but less detail.
    use_wls : bool
        If True, use a WLS (Weighted Least Squares) filter to FILL the black
        holes (invalid pixels). Important: we keep the sharp RAW disparity
        wherever the match was valid, and only borrow the smoothed WLS value
        inside the holes. That fills gaps without softening real detail.
        Needs opencv-contrib (cv2.ximgproc).
    wls_lambda : float
        How strongly the hole-fill smooths. Higher = fills bigger holes.
    wls_sigma : float
        How tightly the fill respects image edges. ~0.8..2.0 typical.
    median_size : int
        Size of a final median filter (odd, e.g. 3 or 5) that removes
        salt-and-pepper speckle while keeping edges sharp. 0 disables it.

    Returns
    -------
    disparity : np.ndarray (float32)
        Per-pixel disparity in pixels. Invalid/unknown pixels are 0.
    """
    # --- 1. Load images as grayscale (matching only needs intensity) ---
    left = _load_gray(left_img)
    right = _load_gray(right_img)

    # --- 2. Build the Semi-Global Block Matcher (the LEFT matcher) ---
    # It slides a small window from each left pixel across the right image,
    # looking for the best-matching window. The horizontal offset of the best
    # match is the disparity.
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        # P1/P2 penalize disparity changes between neighbor pixels.
        # This is the "global smoothness" part that makes SGBM better than
        # plain block matching. Formula is OpenCV's recommended default.
        P1=8 * block_size * block_size,
        P2=32 * block_size * block_size,
        disp12MaxDiff=1,        # left-right consistency check tolerance
        uniquenessRatio=10,     # reject ambiguous matches
        speckleWindowSize=100,  # remove small noisy blobs
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )

    # NOTE: StereoSGBM returns disparity * 16 as int16. We keep that RAW scale
    # for the WLS filter (it expects it), and divide by 16 only at the end.
    left_disp = left_matcher.compute(left, right)
    raw = left_disp.astype(np.float32) / 16.0

    if not use_wls:
        return _median(raw, median_size)

    # --- 3. WLS hole-filling filter ---
    # We also match the OTHER direction (right matcher). Comparing the two
    # tells the filter which pixels are trustworthy. It fills holes and
    # smooths, guided by the left image so borders stay sharp.
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    right_disp = right_matcher.compute(right, left)

    wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls.setLambda(wls_lambda)
    wls.setSigmaColor(wls_sigma)

    filtered = wls.filter(left_disp, left, disparity_map_right=right_disp)
    filtered = filtered.astype(np.float32) / 16.0

    # --- 4. Keep the sharp RAW disparity, fill ONLY the holes from WLS ---
    # This is the key to detail: WLS alone would blur everything. By using it
    # only inside the invalid pixels, real surfaces keep their crisp values.
    result = raw.copy()
    holes = raw <= 0
    result[holes] = filtered[holes]

    return _median(result, median_size)


def _median(disp, size):
    """Light median filter to kill speckle without blurring edges."""
    if size and size >= 3:
        return cv2.medianBlur(disp.astype(np.float32), size)
    return disp


def disparity_to_depth(disparity, focal_length_px, baseline_m):
    """
    Convert disparity (pixels) to depth (meters).

    Geometry: depth = (focal_length * baseline) / disparity

    Parameters
    ----------
    disparity : np.ndarray
        Output of compute_disparity.
    focal_length_px : float
        Camera focal length in PIXELS.
    baseline_m : float
        Distance between the two cameras in METERS.

    Returns
    -------
    depth : np.ndarray (float32), depth in meters. 0 where disparity invalid.
    """
    depth = np.zeros_like(disparity, dtype=np.float32)
    valid = disparity > 0  # avoid divide-by-zero
    depth[valid] = (focal_length_px * baseline_m) / disparity[valid]
    return depth


def colorize(disparity):
    """Turn a disparity map into a color image for viewing.

    Uses the JET colormap: near objects (high disparity) = RED,
    far objects (low disparity) = BLUE. Pixels with no valid match
    are painted BLACK.
    """
    # Only the pixels that got a real match are valid.
    valid = disparity > 0

    # Robust range: clip to 5th/95th percentile of valid pixels. We use 95
    # (not 98/100) on purpose: a few % of pixels are false high-disparity
    # noise (reflective table, bad matches). Letting them set the red end
    # squashes the real nearest object (the bag) down to cyan. Clipping at
    # the 95th percentile makes the true near surface reach RED, like the
    # reference image. The handful of noise pixels just saturate to red.
    if np.any(valid):
        lo, hi = np.percentile(disparity[valid], [5, 95])
    else:
        lo, hi = 0.0, 1.0

    # Scale disparity into 0..255 for the colormap.
    scaled = np.clip((disparity - lo) / max(hi - lo, 1e-6), 0, 1)
    vis = (scaled * 255).astype(np.uint8)

    color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)

    # Paint invalid pixels black instead of leaving them dark blue.
    color[~valid] = (0, 0, 0)
    return color


def _load_gray(img):
    """Accept a path or array, return a grayscale uint8 image."""
    if isinstance(img, str):
        img = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {img}")
        return img
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


# --- Quick demo: run this file directly to test on the sample IR images ---
if __name__ == "__main__":
    import os

    here = os.path.dirname(__file__)
    left_path = os.path.join(here, "test_images", "ir_left.png")
    right_path = os.path.join(here, "test_images", "ir_right.png")

    disp = compute_disparity(left_path, right_path)
    color = colorize(disp)

    out_path = os.path.join(here, "depth_map_result.png")
    cv2.imwrite(out_path, color)
    print(f"Saved disparity map -> {out_path}")
    print(f"Disparity range: {disp.min():.1f} .. {disp.max():.1f} px")

    # Convert to real depth in meters using the D405 camera parameters.
    depth = disparity_to_depth(disp, FOCAL_LENGTH_PX, BASELINE_M)
    valid = depth > 0
    print(f"\nMetric depth (D405, fx={FOCAL_LENGTH_PX}, baseline={BASELINE_M} m):")
    print(f"  valid pixels: {100*valid.mean():.1f}%")
    print(f"  range: {depth[valid].min()*100:.1f} .. {depth[valid].max()*100:.1f} cm")
    print(f"  median: {np.median(depth[valid])*100:.1f} cm")

    # Center-pixel depth — quick spot check (the backpack region).
    h, w = depth.shape
    c = depth[h // 2, w // 2]
    print(f"  center pixel: {c*100:.1f} cm" if c > 0 else "  center pixel: (no match)")
