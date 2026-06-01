"""
Capture depth + left/right infrared frames from the Intel RealSense D405.

Run it, point the camera at a scene, and press ENTER to save a capture.
Press 'q' then ENTER to quit.

Each capture writes into captures/<timestamp>/ :
    depth.npy        raw depth, uint16, in depth units (multiply by depth_scale -> meters)
    depth_color.png  colorized depth, just for looking at
    ir_left.png      left infrared image   (use these two for your own stereo matching)
    ir_right.png     right infrared image
    intrinsics.txt   fx, fy, ppx, ppy, depth_scale, baseline  (needed to make point clouds)
"""

import os
import time
import numpy as np
import cv2
import pyrealsense2 as rs

# --- resolution / fps (848x480@30 is a safe, fast default for the D405) ---
W, H, FPS = 848, 480, 30

OUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


def main():
    os.makedirs(OUT_ROOT, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    config.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)  # left
    config.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8, FPS)  # right

    profile = pipeline.start(config)

    # depth_scale converts raw uint16 depth values into meters
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # intrinsics of the depth stream (and the stereo baseline)
    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = depth_profile.get_intrinsics()
    ir1 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
    ir2 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
    baseline_m = abs(ir2.get_extrinsics_to(ir1).translation[0])

    print(f"D405 streaming {W}x{H}@{FPS}")
    print(f"  fx={intr.fx:.2f} fy={intr.fy:.2f} ppx={intr.ppx:.2f} ppy={intr.ppy:.2f}")
    print(f"  depth_scale={depth_scale} m/unit   baseline={baseline_m*1000:.2f} mm")
    print("Press ENTER to save a capture, or type q + ENTER to quit.\n")

    colorizer = rs.colorizer()
    try:
        while True:
            # let auto-exposure settle a bit before each save
            for _ in range(5):
                frames = pipeline.wait_for_frames()

            cmd = input("[ENTER]=capture  q=quit > ").strip().lower()
            if cmd == "q":
                break

            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            irl = frames.get_infrared_frame(1)
            irr = frames.get_infrared_frame(2)
            if not depth or not irl or not irr:
                print("  dropped a frame, try again")
                continue

            depth_np = np.asanyarray(depth.get_data())          # uint16
            depth_vis = np.asanyarray(colorizer.colorize(depth).get_data())
            irl_np = np.asanyarray(irl.get_data())              # uint8
            irr_np = np.asanyarray(irr.get_data())

            stamp = time.strftime("%Y%m%d_%H%M%S")
            d = os.path.join(OUT_ROOT, stamp)
            os.makedirs(d, exist_ok=True)
            np.save(os.path.join(d, "depth.npy"), depth_np)
            cv2.imwrite(os.path.join(d, "depth_color.png"), depth_vis)
            cv2.imwrite(os.path.join(d, "ir_left.png"), irl_np)
            cv2.imwrite(os.path.join(d, "ir_right.png"), irr_np)
            with open(os.path.join(d, "intrinsics.txt"), "w") as f:
                f.write(f"width {intr.width}\nheight {intr.height}\n")
                f.write(f"fx {intr.fx}\nfy {intr.fy}\nppx {intr.ppx}\nppy {intr.ppy}\n")
                f.write(f"depth_scale {depth_scale}\nbaseline_m {baseline_m}\n")

            print(f"  saved -> {d}")
    finally:
        pipeline.stop()
        print("stopped.")


if __name__ == "__main__":
    main()
