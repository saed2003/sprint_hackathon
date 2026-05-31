# Street View Robot

**Mentor:** Mr. Rajaei Khatib
**Team:** 5 members

---

## Goal

Design a navigation robot that traverses a room and builds **360° panoramic + point cloud** representations of its surroundings at each sampled location.

---

## Overview

A mobile robot that drives around a room and, at each stop, constructs a **3D point cloud** of what it sees. By visiting many locations, it incrementally builds a full scan of the room.

---

## Hardware

Core components for this project:

| Component | Details |
|---|---|
| **Robot platform** | Raspbot V2 educational robot — chassis with 4× Mecanum wheels → omnidirectional movement (forward, back, strafe, diagonal, in-place rotation), I2C motor driver |
| **Compute** | Raspberry Pi 5 — 8 GB |
| **Depth camera** | Intel RealSense D405 — factory-calibrated stereo IR pair |

Onboard Raspbot V2 sensors also used:

- **Ultrasonic rangefinder** — obstacle distance.
- **4× infrared line-tracking sensors** — ground reference / line following.

---

## Setup (do first)

Each group must:

1. Assemble chassis — mount motors, wheels, camera, sensors.
2. Initialize the Raspberry Pi 5:
   - Install OS.
   - Configure I2C + camera interfaces.
   - Set up Python env with **OpenCV**, **NumPy**, **pyrealsense2**.

---

## Core Task

At each location the robot rotates and captures stereo frames. Students design and implement an algorithm to:

1. **Compute depth** from stereo pairs.
2. **Generate 3D point clouds.**
3. **Merge** views from different rotation positions → single **360° point cloud**.

---

## Python API

All hardware abstracted into simple method calls, e.g.:

```python
forward(speed)
capture_stereo()
set_tilt(angle)
```

The system supports **two navigation modes**.

### Mode 1 — Manual Control

- Keyboard-driven terminal interface.
- Drive in real time with **WASD** (omnidirectional + in-place rotation).
- One keypress triggers the capture routine: rotate → capture → merge, runs autonomously, then returns control to the user.

### Mode 2 — Autonomous Line Following

- Dark tape path laid on the floor connects capture locations.
- Robot follows the line using its 4 IR line-tracking sensors.
- **Stop markers** = perpendicular cross-marks on the tape that trigger all 4 sensors at once → mark a capture location.
- On detecting a stop marker: halt → run capture routine → resume line following to next marker.

Both modes let the user incrementally build a collection of point clouds across the room.
