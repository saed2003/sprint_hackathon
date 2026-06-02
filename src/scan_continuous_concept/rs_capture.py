"""
COPY (concept sandbox) of camera/rs_capture.py with two additions for the continuous
360 scan. This is a self-contained copy on purpose: the experiment lives entirely in
scan_continuous_concept/ so the real camera/rs_capture.py is never touched. Delete this
whole folder to go back to the shipped pipeline.

Added vs the original:
  * save_arrays()        — write an ALREADY-grabbed frameset (so the continuous scan can
                           stream frames itself and still produce the standard folder).
  * set_manual_exposure()— lock a short exposure to cut motion blur while spinning.

Same on-disk layout as the original, so pointcloud/scan360.py's merge works unchanged:

    captures/<...>/
        depth.npy        raw uint16 depth   (x depth_scale -> meters)
        depth_color.png  colorized depth     (only written by save_to())
        ir_left.png      left infrared
        ir_right.png     right infrared
        intrinsics.txt   width height fx fy ppx ppy depth_scale baseline_m
"""

import os
import time

import numpy as np
import cv2
import pyrealsense2 as rs

# 848x480@30 is a safe, fast default for the D405
W, H, FPS = 848, 480, 30


def default_out_root():
    # captures/ lives at the project root (two levels up from this concept folder:
    # src/scan_continuous_concept/ -> src/ -> <root>)
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(root, "captures")


class StereoCapture:
    """Owns a D405 pipeline (depth + left/right IR) and saves captures."""

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
        """Grab one frameset into a fresh captures/<timestamp>/ folder."""
        if self.pipeline is None:
            self.start()
        out_root = out_root or default_out_root()
        os.makedirs(out_root, exist_ok=True)
        return self.save_to(self._new_folder(out_root), settle=settle)

    def save_to(self, folder, settle=5):
        """Grab one synced frameset and write it into an explicit folder."""
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

        return self.save_arrays(folder, depth_np, irl_np, irr_np, depth_vis=depth_vis)

    def save_arrays(self, folder, depth_np, irl_np, irr_np, depth_vis=None):
        """Write an ALREADY-grabbed frameset into `folder`, in the standard layout.

        Lets the continuous scan stream frames itself (while the robot spins) and still
        produce the exact folder pointcloud/scan360.py expects. `depth_color.png` is only
        written when a `depth_vis` is supplied — skipping the colorize keeps the per-frame
        write fast during a spin (the preview isn't needed to build a cloud). Returns `folder`.
        """
        os.makedirs(folder, exist_ok=True)
        np.save(os.path.join(folder, "depth.npy"), depth_np)
        if depth_vis is not None:
            cv2.imwrite(os.path.join(folder, "depth_color.png"), depth_vis)
        cv2.imwrite(os.path.join(folder, "ir_left.png"), irl_np)
        cv2.imwrite(os.path.join(folder, "ir_right.png"), irr_np)
        with open(os.path.join(folder, "intrinsics.txt"), "w") as f:
            f.write(f"width {self.intr.width}\nheight {self.intr.height}\n")
            f.write(f"fx {self.intr.fx}\nfy {self.intr.fy}\nppx {self.intr.ppx}\nppy {self.intr.ppy}\n")
            f.write(f"depth_scale {self.depth_scale}\nbaseline_m {self.baseline_m}\n")
        return folder

    def set_manual_exposure(self, microseconds):
        """Lock the D405 depth/IR exposure (microseconds) to cut MOTION BLUR while the
        robot spins, or pass None to restore auto-exposure. A short exposure (e.g. 6000 us)
        freezes the IR images so passive-stereo matching stays sharp during the sweep.
        Starts the camera lazily; no-ops gracefully if the option is unsupported."""
        if self.pipeline is None:
            self.start()
        try:
            sensor = self.pipeline.get_active_profile().get_device().first_depth_sensor()
            if microseconds is None:
                sensor.set_option(rs.option.enable_auto_exposure, 1)
            else:
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, float(microseconds))
        except Exception:
            pass  # ignore if this firmware doesn't expose the option

    def grab_ir(self, settle=2):
        """Grab one left-IR frame as a uint8 numpy array, WITHOUT saving anything."""
        if self.pipeline is None:
            self.start()
        for _ in range(settle):
            self.pipeline.wait_for_frames()
        frames = self.pipeline.wait_for_frames()
        irl = frames.get_infrared_frame(1)
        if not irl:
            return None
        return np.asanyarray(irl.get_data())

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
