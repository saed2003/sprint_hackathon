"""Quick headless camera test — opens the D405, captures one frameset, saves images."""
import os
import numpy as np
import cv2
import pyrealsense2 as rs

W, H, FPS = 848, 480, 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
config.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
config.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8, FPS)

print("Starting D405 pipeline...")
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
intr = depth_profile.get_intrinsics()
ir1 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
ir2 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
baseline_m = abs(ir2.get_extrinsics_to(ir1).translation[0])

print(f"  Resolution : {W}x{H} @ {FPS} fps")
print(f"  fx={intr.fx:.2f}  fy={intr.fy:.2f}  ppx={intr.ppx:.2f}  ppy={intr.ppy:.2f}")
print(f"  depth_scale={depth_scale} m/unit   baseline={baseline_m*1000:.2f} mm")

# warm up auto-exposure
print("Warming up (5 frames)...")
for _ in range(5):
    pipeline.wait_for_frames()

frames = pipeline.wait_for_frames()
depth  = frames.get_depth_frame()
irl    = frames.get_infrared_frame(1)
irr    = frames.get_infrared_frame(2)

if not depth or not irl or not irr:
    print("ERROR: missing frames")
    pipeline.stop()
    raise SystemExit(1)

depth_np = np.asanyarray(depth.get_data())
irl_np   = np.asanyarray(irl.get_data())
irr_np   = np.asanyarray(irr.get_data())

colorizer = rs.colorizer()
depth_vis = np.asanyarray(colorizer.colorize(depth).get_data())

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_capture")
os.makedirs(out, exist_ok=True)
cv2.imwrite(os.path.join(out, "depth_color.png"), depth_vis)
cv2.imwrite(os.path.join(out, "ir_left.png"),     irl_np)
cv2.imwrite(os.path.join(out, "ir_right.png"),    irr_np)
np.save(os.path.join(out, "depth.npy"),           depth_np)

centre = depth_np[H//2, W//2]
print(f"\nCenter pixel depth: {centre} units = {centre * depth_scale * 100:.1f} cm")
print(f"Saved to: {out}/")
print("  depth_color.png  ir_left.png  ir_right.png  depth.npy")
print("\nCAMERA TEST PASSED")

pipeline.stop()
