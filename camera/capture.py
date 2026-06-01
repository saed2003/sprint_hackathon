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

The actual camera work lives in rs_capture.StereoCapture, which drive.py reuses
for its 'c' (capture-in-place) key so both produce identical capture folders.
"""

import os
import sys

# allow running as `python3 camera/capture.py` from anywhere: put the project
# root (the folder that contains camera/, pointcloud/, rasbot/, ...) on sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera.rs_capture import StereoCapture


def main():
    cam = StereoCapture()
    cam.start()
    print(cam.info())
    print("Press ENTER to save a capture, or type q + ENTER to quit.\n")

    try:
        while True:
            cmd = input("[ENTER]=capture  q=quit > ").strip().lower()
            if cmd == "q":
                break

            folder = cam.save()
            if folder is None:
                print("  dropped a frame, try again")
                continue
            print(f"  saved -> {folder}")
    finally:
        cam.close()
        print("stopped.")


if __name__ == "__main__":
    main()
