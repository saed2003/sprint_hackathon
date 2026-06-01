"""
Capture a stereo IR pair from the RealSense D405 for depth mapping.

Key difference from calibrate.py:
    Here we keep the IR PROJECTOR ON. The projector throws an invisible dot
    pattern onto the scene. That gives texture to blank surfaces (white bag,
    shiny table) so the stereo matcher can actually find matches there. This
    is the single biggest quality win for textureless scenes.

    (calibrate.py turns the projector OFF, because the dots would ruin
    chessboard detection. Opposite needs.)

Run:
    python capture_depth.py            # save one pair (overwrites test_images)
    python capture_depth.py --avg 10   # average 10 frames -> less noise

Then run the depth map:
    python depth_map.py
"""

import argparse
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "test_images")
WIDTH, HEIGHT = 1280, 720


def capture(num_avg=1, preview=True):
    import cv2
    import pyrealsense2 as rs

    os.makedirs(OUT_DIR, exist_ok=True)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, 1, WIDTH, HEIGHT, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.infrared, 2, WIDTH, HEIGHT, rs.format.y8, 30)
    profile = pipe.start(cfg)

    # --- Turn the IR projector ON (texture for matching) + max power ---
    depth_sensor = profile.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 1)
    if depth_sensor.supports(rs.option.laser_power):
        rng = depth_sensor.get_option_range(rs.option.laser_power)
        depth_sensor.set_option(rs.option.laser_power, rng.max)

    print("Warming up sensor...")
    for _ in range(15):           # let auto-exposure settle
        pipe.wait_for_frames()

    if preview:
        print("Live preview. SPACE = capture, Q = quit without saving.")
        captured = False
        try:
            while True:
                f = pipe.wait_for_frames()
                left = np.asanyarray(f.get_infrared_frame(1).get_data())
                right = np.asanyarray(f.get_infrared_frame(2).get_data())
                view = np.hstack([left, right])
                cv2.imshow("D405 IR (L | R) - SPACE to capture", view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    captured = True
                    break
        finally:
            cv2.destroyAllWindows()
        if not captured:
            pipe.stop()
            print("Quit, nothing saved.")
            return

    # --- Grab and (optionally) average several frames to cut noise ---
    accL = np.zeros((HEIGHT, WIDTH), np.float32)
    accR = np.zeros((HEIGHT, WIDTH), np.float32)
    for _ in range(num_avg):
        f = pipe.wait_for_frames()
        accL += np.asanyarray(f.get_infrared_frame(1).get_data())
        accR += np.asanyarray(f.get_infrared_frame(2).get_data())
    left = (accL / num_avg).astype(np.uint8)
    right = (accR / num_avg).astype(np.uint8)
    pipe.stop()

    lp = os.path.join(OUT_DIR, "ir_left.png")
    rp = os.path.join(OUT_DIR, "ir_right.png")
    cv2.imwrite(lp, left)
    cv2.imwrite(rp, right)
    print(f"Saved {WIDTH}x{HEIGHT} pair (avg of {num_avg}):")
    print(f"  {lp}")
    print(f"  {rp}")
    print("Now run:  python depth_map.py")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Capture D405 IR pair (emitter ON)")
    p.add_argument("--avg", type=int, default=1,
                   help="average N frames to reduce noise (e.g. 10)")
    p.add_argument("--no-preview", action="store_true",
                   help="capture immediately, no live window")
    args = p.parse_args()
    capture(num_avg=args.avg, preview=not args.no_preview)
