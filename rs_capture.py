"""
Shared Intel RealSense D405 stereo capture helper.

Used by both capture.py (standalone, ENTER to save) and drive.py (press 'c'
while driving). Both save the SAME folder layout the point-cloud scripts expect,
so make_pointcloud.py / merge_clouds.py work on robot captures without changes:

    captures/<timestamp>/
        depth.npy        raw uint16 depth   (× depth_scale -> meters)
        depth_color.png  colorized depth     (just for looking at)
        ir_left.png      left infrared       -> input to our own stereo depth
        ir_right.png     right infrared
        intrinsics.txt   width height fx fy ppx ppy depth_scale baseline_m

Only needs pyrealsense2 + numpy + cv2 (NOT open3d) — so it runs on the Pi.
"""

import os
import time

import numpy as np
import cv2
import pyrealsense2 as rs

# 848x480@30 is a safe, fast default for the D405
W, H, FPS = 848, 480, 30


def default_out_root():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


class StereoCapture:
    """Owns a D405 pipeline (depth + left/right IR) and saves captures.

    Open once, then call save() as many times as you like. Calling save()
    before start() starts the camera lazily (with an auto-exposure warm-up).
    """

    def __init__(self, width=W, height=H, fps=FPS):
        self.width, self.height, self.fps = width, height, fps
        self.pipeline = None
        self.colorizer = rs.colorizer()
        self.depth_scale = None
        self.intr = None
        self.baseline_m = None

    def start(self, warmup=5):
        """Start streaming and read the factory calibration once."""
        if self.pipeline is not None:
            return
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)  # left
        config.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)  # right

        profile = pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.intr = depth_profile.get_intrinsics()
        ir1 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        ir2 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        self.baseline_m = abs(ir2.get_extrinsics_to(ir1).translation[0])

        self.pipeline = pipeline

        # let auto-exposure settle before the first real capture
        for _ in range(warmup):
            pipeline.wait_for_frames()

    def info(self):
        """One-line summary of the calibration (call after start())."""
        return (
            f"D405 streaming {self.width}x{self.height}@{self.fps}  "
            f"fx={self.intr.fx:.2f} fy={self.intr.fy:.2f} "
            f"ppx={self.intr.ppx:.2f} ppy={self.intr.ppy:.2f}  "
            f"depth_scale={self.depth_scale} m/unit  "
            f"baseline={self.baseline_m * 1000:.2f} mm"
        )

    def save(self, out_root=None, settle=5):
        """Grab one frameset into a fresh captures/<timestamp>/ folder.

        Returns the capture folder path, or None if a frame was dropped.
        """
        if self.pipeline is None:
            self.start()
        out_root = out_root or default_out_root()
        os.makedirs(out_root, exist_ok=True)
        return self.save_to(self._new_folder(out_root), settle=settle)

    def save_to(self, folder, settle=5):
        """Grab one synced frameset and write it into an explicit folder.

        Used by the 360 scan to lay shots out as scan_<ts>/shot_00, shot_01, ...
        Returns the folder, or None if a frame was dropped.
        `settle` re-settles auto-exposure for the new view before grabbing.
        """
        if self.pipeline is None:
            self.start()
        os.makedirs(folder, exist_ok=True)

        # re-settle exposure (the scene/angle may have changed since last save)
        for _ in range(settle):
            self.pipeline.wait_for_frames()

        frames = self.pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        irl = frames.get_infrared_frame(1)
        irr = frames.get_infrared_frame(2)
        if not depth or not irl or not irr:
            return None

        depth_np = np.asanyarray(depth.get_data())                       # uint16
        depth_vis = np.asanyarray(self.colorizer.colorize(depth).get_data())
        irl_np = np.asanyarray(irl.get_data())                           # uint8
        irr_np = np.asanyarray(irr.get_data())

        d = folder
        np.save(os.path.join(d, "depth.npy"), depth_np)
        cv2.imwrite(os.path.join(d, "depth_color.png"), depth_vis)
        cv2.imwrite(os.path.join(d, "ir_left.png"), irl_np)
        cv2.imwrite(os.path.join(d, "ir_right.png"), irr_np)
        with open(os.path.join(d, "intrinsics.txt"), "w") as f:
            f.write(f"width {self.intr.width}\nheight {self.intr.height}\n")
            f.write(f"fx {self.intr.fx}\nfy {self.intr.fy}\nppx {self.intr.ppx}\nppy {self.intr.ppy}\n")
            f.write(f"depth_scale {self.depth_scale}\nbaseline_m {self.baseline_m}\n")
        return d

    @staticmethod
    def _new_folder(out_root):
        """captures/<timestamp>/ — add a suffix if two saves land in one second."""
        stamp = time.strftime("%Y%m%d_%H%M%S")
        d = os.path.join(out_root, stamp)
        n = 1
        while os.path.exists(d):
            n += 1
            d = os.path.join(out_root, f"{stamp}_{n}")
        os.makedirs(d)
        return d

    def close(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
