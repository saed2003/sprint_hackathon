"""Point-cloud / perception code.

Builds and views 3D point clouds from captures/<timestamp>/ folders.

On-the-Pi (pure numpy + cv2, no Open3D):
  scan360         — 360 sweep + measured-angle merge -> merged_360.ply
  view3d          — orbit a .ply in an OpenCV window

On the laptop (Open3D + matplotlib):
  make_pointcloud — one capture  -> cloud.ply (+ preview)
  merge_clouds    — many captures -> merged.ply via ICP
  render_cloud    — static front/top-down preview PNG of a .ply

Either:
  clean_captures  — delete capture data to start fresh (stdlib only)
"""
