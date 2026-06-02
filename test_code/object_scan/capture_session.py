"""
Capture a TURNTABLE / hand-stepped object scan into shot_NN folders (laptop, D405 USB).

This is the RELIABLE capture path: the camera stays still and the OBJECT rotates a
fixed step between shots (a lazy-susan, a hand-turned plate, or a marked turntable).
Because the camera never moves, there's no open-loop drift — the cleanest result and
the safest thing to demo.

It writes the SAME capture folder the whole robot uses, PLUS a real-colour color.png
(D405 colour stream, aligned to depth) so the merged model comes out coloured:

    captures/obj_<ts>/shot_NN/
        depth.npy  depth_color.png  ir_left.png  ir_right.png  color.png  intrinsics.txt
        angle.txt        # this shot's cumulative turn angle (i * step) -> merge prior

Standalone: it opens its own RealSense pipeline, so it does NOT import or modify any
robot code. Open3D is NOT needed here (capture only).

    python capture_session.py                     # 12 shots, 30 deg each (full 360)
    python capture_session.py --shots 24          # finer: 24 shots, 15 deg each
    python capture_session.py --step 20           # 20 deg/shot (set to your turntable click)

Workflow: press ENTER, the shot saves, then rotate the turntable by `step` degrees,
press ENTER again ... 'q' then ENTER to stop early. Keep the object TEXTURED + LIT.
"""
import os
import sys
import time

import numpy as np
import cv2
import pyrealsense2 as rs

W, H, FPS = 848, 480, 30


def default_out_root():
    """test_code/object_scan/captures/  (kept out of the robot's own captures/)."""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


class ColorStereoCapture:
    """D405 pipeline that streams depth + left/right IR + colour, colour aligned to depth."""

    def __init__(self, width=W, height=H, fps=FPS):
        self.width, self.height, self.fps = width, height, fps
        self.pipeline = None
        self.colorizer = rs.colorizer()
        self.align = rs.align(rs.stream.depth)
        self.has_color = False
        self.depth_scale = None
        self.intr = None
        self.baseline_m = None

    def start(self, warmup=10):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
        config.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)
        try:
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self.has_color = True
        except Exception:
            self.has_color = False

        try:
            profile = pipeline.start(config)
        except Exception:
            # some D405 firmwares won't do colour at this resolution -> retry without it
            self.has_color = False
            config = rs.config()
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
            config.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)
            profile = pipeline.start(config)

        ds = profile.get_device().first_depth_sensor()
        self.depth_scale = ds.get_depth_scale()
        dp = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.intr = dp.get_intrinsics()
        ir1 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        ir2 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        self.baseline_m = abs(ir2.get_extrinsics_to(ir1).translation[0])
        self.pipeline = pipeline
        for _ in range(warmup):
            pipeline.wait_for_frames()

    def info(self):
        return (f"D405 {self.width}x{self.height}@{self.fps}  color={'yes' if self.has_color else 'no'}  "
                f"fx={self.intr.fx:.1f} baseline={self.baseline_m*1000:.1f}mm "
                f"depth_scale={self.depth_scale}")

    def save_to(self, folder, settle=5):
        """Grab one synced frameset and write the standard folder (+ color.png)."""
        os.makedirs(folder, exist_ok=True)
        for _ in range(settle):
            self.pipeline.wait_for_frames()
        frames = self.pipeline.wait_for_frames()
        if self.has_color:
            frames = self.align.process(frames)        # colour -> depth grid
        depth = frames.get_depth_frame()
        irl = frames.get_infrared_frame(1)
        irr = frames.get_infrared_frame(2)
        if not depth or not irl or not irr:
            return None

        np.save(os.path.join(folder, "depth.npy"), np.asanyarray(depth.get_data()))
        cv2.imwrite(os.path.join(folder, "depth_color.png"),
                    np.asanyarray(self.colorizer.colorize(depth).get_data()))
        cv2.imwrite(os.path.join(folder, "ir_left.png"), np.asanyarray(irl.get_data()))
        cv2.imwrite(os.path.join(folder, "ir_right.png"), np.asanyarray(irr.get_data()))
        if self.has_color:
            cframe = frames.get_color_frame()
            if cframe:
                cv2.imwrite(os.path.join(folder, "color.png"), np.asanyarray(cframe.get_data()))
        with open(os.path.join(folder, "intrinsics.txt"), "w") as f:
            f.write(f"width {self.intr.width}\nheight {self.intr.height}\n")
            f.write(f"fx {self.intr.fx}\nfy {self.intr.fy}\nppx {self.intr.ppx}\nppy {self.intr.ppy}\n")
            f.write(f"depth_scale {self.depth_scale}\nbaseline_m {self.baseline_m}\n")
        return folder

    def grab_depth(self, settle=2):
        """Grab one depth frame in METRES (no save) — for the orbit's vision aiming."""
        for _ in range(settle):
            self.pipeline.wait_for_frames()
        frames = self.pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            return None
        return np.asanyarray(depth.get_data()).astype(np.float32) * self.depth_scale

    def close(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


def run_session(shots=None, step=None, out_root=None, cam=None, log=print):
    """Interactive turntable capture -> a session of shot_NN folders. Returns the path.

    ENTER saves the next shot; rotate the object `step` deg; repeat. 'q'+ENTER stops.
    Defaults (shots) come from config so the whole demo is tuned in one place.
    """
    try:
        from config import DEFAULT
        shots = shots or DEFAULT["shots"]
    except Exception:
        shots = shots or 12
    if step is None:
        step = 360.0 / shots                     # spread `shots` evenly over a full turn

    own_cam = cam is None
    cam = cam or ColorStereoCapture()
    if cam.pipeline is None:
        log("Opening D405 (warming up auto-exposure)...")
        cam.start()
        log(cam.info())

    session = os.path.join(out_root or default_out_root(), "obj_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    log(f"\nTurntable scan -> {session}")
    log(f"Plan: {shots} shots, rotate the object ~{step:.0f} deg between each.")
    log("ENTER = save the next shot, then rotate.   'q' + ENTER = stop.\n")

    i = 0
    try:
        while i < shots:
            ans = input(f"[{i+1}/{shots}] angle ~{i*step:.0f} deg — ENTER to capture (q=quit): ")
            if ans.strip().lower() == "q":
                break
            folder = cam.save_to(os.path.join(session, f"shot_{i:02d}"))
            if folder is None:
                log("  frame dropped, retrying same shot")
                continue
            with open(os.path.join(folder, "angle.txt"), "w") as f:
                f.write(f"{i*step:.3f}\n")
            log(f"  saved shot_{i:02d}  (rotate the object ~{step:.0f} deg now)")
            i += 1
    finally:
        if own_cam:
            cam.close()
    log(f"\ndone: {i} shots in {session}")
    return session


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        v = cast(args[i + 1])
        del args[i:i + 2]
        return v
    return None


def main():
    args = sys.argv[1:]
    shots = _pop(args, "--shots", int)
    step = _pop(args, "--step", float)
    session = run_session(shots=shots, step=step)
    print("Build the model on the laptop:")
    print(f"  .venv/bin/python build_object.py {session} --mesh")


if __name__ == "__main__":
    main()
