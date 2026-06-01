"""Camera drivers and capture code for the Intel RealSense D405.

Modules:
  rs_capture  — StereoCapture: the shared D405 pipeline (depth + L/R IR) that
                writes the standard captures/<timestamp>/ folder format.
  capture     — standalone "press ENTER to save a capture" tool.
"""
