# Street View Robot — System Specification

**Project:** Street View Robot (Yahboom Raspbot V2)
**Goal of this document:** define the single integrated system that combines every
feature the team has built — two drive modes, live view, a one-button 360° depth
scan, and the ultrasonic radar — into one web dashboard.

This is the **target spec**. Where the current code already does something, it says
so; where a feature exists but is **not yet wired into the unified dashboard**, it is
called out as a **gap to build**. Sections end with acceptance criteria.

---

## 1. Product summary

A driveable robot that builds a map of a room. The operator opens one web
dashboard and can:

1. **Drive the robot** two ways — switchable at runtime:
   - **Manual (WASD)** teleop, or
   - **Autonomous line-following** on dark tape.
2. **Watch a live camera feed** (USB webcam) the whole time.
3. **Press one button → 360° scan**: the robot spins in place, takes **10**
   stereo shots of its surroundings, builds a **360° point cloud (`.ply`)**, and
   **shows it** in the dashboard.
4. **See an ultrasonic radar** (PPI sweep) of nearby obstacles.

One operator, one browser tab, one robot.

---

## 2. Hardware

| Part | Detail | Used for |
|---|---|---|
| Chassis | Raspbot V2, 4× Mecanum wheels | omnidirectional drive |
| Compute | Raspberry Pi 5 | runs everything on-robot |
| Board | Yahboom expansion, I2C addr `0x2B`, bus 1 | motors, servos, LEDs, sensors |
| Depth camera | Intel RealSense **D405** (stereo IR, ~18 mm baseline, **no IR projector** → passive stereo) | depth map + 360 point cloud |
| Live camera | USB webcam | live MJPEG view (kept separate so streaming and depth capture don't fight over one device) |
| Line sensor | 4-channel IR array (`L_out, L_in, R_in, R_out`) | line following |
| Distance | Ultrasonic HC-SR04 on the pan/tilt head | radar |
| Output | 128×32 OLED, 14 RGB LEDs, buzzer | status / feedback |
| Servos | pan (0–180, default 90), tilt (0–100, default 25) | aim camera + radar sweep |

**No IMU, no wheel encoders.** Rotation between scan shots is **open-loop timed**
(see §6.3). This drives several design decisions below.

---

## 3. Software architecture

### 3.1 Two environments (a hard constraint)

| Environment | Has | Runs |
|---|---|---|
| **Raspberry Pi (robot)** | `pyrealsense2`, `numpy`, `cv2`, `pygame`, `smbus`, `PIL` — **no Open3D** | all robot control + on-device perception |
| **Laptop** | adds **`open3d`** | high-quality offline 360 rebuild, viewers |

Importing the robot API pulls `smbus`, which **only exists on the Pi**. All Pi-side
perception is therefore **pure NumPy + OpenCV**. The high-quality 360 merger uses
Open3D and runs on the **laptop**. This split shapes the 360 flow in §6.

### 3.2 Processes & ports (the live system)

Launched by `main.py web` (systemd `rasbot.service`):

```
                      ┌─────────────────── Raspberry Pi ───────────────────┐
   browser  ──HTTP──► :80/8080  static dashboard  (top/robot_control_dashboard.html)
            ──HTTP──► :9000     control API        (src/control_server.py)  ◄─ the brain
            ──MJPEG─► :8000     live webcam stream  (src/camera/stream_server.py)
            ──MJPEG─► :8001     ultrasonic radar    (src/radar/radar.py)     ◄─ GAP: integrate
                      └─────────────────────────────────────────────────────┘
```

- **:80/8080** — serves the dashboard HTML (static).
- **:9000 control API** — owns the robot. Drive, servo, run on/off, captures,
  point-cloud download. Reuses `wasd/drive.py`'s motion functions so there is **one**
  definition of each motion.
- **:8000 live stream** — MJPEG from the **USB webcam** (not the D405).
- **:8001 radar** — MJPEG PPI radar. **Currently a standalone server**; §6.4 covers
  folding it into the dashboard.

**Safety:** the browser sends a drive **heartbeat ~150 ms**; the server runs a
**watchdog (0.6 s)** that halts the robot if commands stop (tab closed, Wi-Fi drop).
Only **one driver may own the I2C bus at a time** — the web stack and any desktop
teleop are mutually exclusive.

### 3.3 The capture-folder contract

Every producer (camera/robot) and consumer (cloud tools) talk through folders on
disk, never ad-hoc variables:

```
captures/<timestamp>/
├── depth.npy         uint16 raw depth (× depth_scale → metres)
├── depth_color.png   colorized depth (for looking at)
├── ir_left.png       left IR   → input to our own stereo depth
├── ir_right.png      right IR
└── intrinsics.txt    width height fx fy ppx ppy depth_scale baseline_m

captures/scan_<ts>/
├── shot_00/ … shot_09/   (each a capture folder above, + angle.txt)
├── merged_360.ply
└── merged_360_preview.png
```

This contract is what lets the **same scan** be rebuilt on the Pi (quick) or the
laptop (high quality) without changing the capture step.

---

## 4. Feature: Movement (two modes, toggleable)

The operator picks the mode, then arms the run. Mode can be switched when a run is
stopped.

### 4.1 Mode 1 — Manual (WASD) — **working**

| Control | Action |
|---|---|
| `W / S` | forward / back |
| `A / D` | strafe left / right (Mecanum) |
| `Q / E` | rotate in place left / right |
| `+ / −` | speed down/up (band **40–255**, default 120) |
| arrows | pan/tilt the camera head |

Web path: dashboard collects held keys → `POST /api/drive {keys, speed}` → server
translates to `wasd/drive.py`'s `desired_command` / `apply_command`. The heartbeat
re-sends the held command; releasing all keys (or losing the tab) stops the robot.

### 4.2 Mode 2 — Autonomous line-following — **built, NOT wired to web (gap)**

`src/tape_following/line_follow.py` is a complete follower:

- Reads the 4-channel IR array; error weights `−3, −1, +1, +3`.
- **State machine:** `STRAIGHT → CURVE → SHARP → UTURN → RECOVERY → LOST`, with
  per-state PID gains, predictive speed zones, and smoothed motor output.
- Handles **junctions** (all-4-on) and **U-turns** using last-known turn direction.
- Optional **red "stop marker"** detection (camera) → triggers a 360 scan, then
  resumes. (`SKIP_SCAN` currently true → marker = a 1 s pause.)
- Entry point is web-ready: `line_follow.run(bot, cam, stop_event=…)` runs in a
  thread and stops cleanly when `stop_event` is set.

**Gap to build:** `control_server.start_run('autonomous')` currently returns
`not_implemented`; the dashboard refuses to start a non-manual run. To finish:

1. In `control_server.py`, on `start_run('autonomous')`: spawn a background thread
   running `line_follow.run(bot, cam, stop_event)`; on `stop_run()` set the event and
   join. Guard with the same `robot_lock` so manual drive and the follower never both
   touch the bus.
2. While autonomous is active, **ignore `/api/drive`** (the follower owns motion).
3. Dashboard already toggles `MANUAL ⇄ AUTONOMOUS` (key `M`) and shows the mode;
   remove the "only MANUAL works" guard in `toggleRun()` once the server supports it.

**Acceptance:** with tape on the floor, selecting AUTONOMOUS + START makes the robot
follow the line hands-off; STOP halts it within the watchdog window; a red stop
marker (when enabled) fires a 360 scan and the robot then continues.

---

## 5. Feature: Live view — **working**

- `src/camera/stream_server.py` serves **MJPEG from the USB webcam** on **:8000**.
- The control API spawns it on demand (`POST /api/stream/start` / `…/stop`); the
  dashboard's **Connect** button shows it in the right-hand panel.
- Deliberately the **USB cam, not the D405**, so the live feed keeps running while
  the D405 is busy with a 360 scan or single capture.

**Acceptance:** Connect shows live video < 3 s; it keeps streaming during a 360 scan;
Disconnect stops the server.

---

## 6. Feature: One-button 360° depth scan — **core demo**

> The headline feature: **press one button → spin, shoot 10 frames, build a 360°
> point cloud, show it.**

### 6.1 Operator flow (target)

```
[360° SCAN]  ──►  confirm "robot will spin in place, clear the area"
     │
     ▼
 robot rotates in place, stopping 10× to capture a D405 stereo shot
     │   (LEDs blue; live webcam keeps streaming; drive disabled meanwhile)
     ▼
 build merged_360.ply  on-device (numpy)         ◄─ immediate, for "show it now"
     │
     ▼
 SHOW the cloud in the dashboard (interactive 3D)  ◄─ GAP: in-browser viewer
     │
     └──►  optional: download .ply / copy scan to laptop for the high-quality rebuild
```

### 6.2 What exists today

| Step | Web hook | Implementation | Status |
|---|---|---|---|
| Spin + 10 captures | `POST /api/capture/scan360` | `scan360.run_scan()` (Pi) | ✅ works |
| Single capture | `POST /api/capture/single` | `StereoCapture.save()` | ✅ works |
| Build cloud | `POST /api/capture/build` | `scan360.build_from_session()` (Pi, numpy) | ✅ works |
| Download `.ply` | `GET /api/cloud/download` | streams last build | ✅ works |
| **Show in browser** | — | — | ❌ **gap** (today: download, or view on Pi desktop / laptop) |

Capture, build and download are non-blocking with status polling
(`GET /api/capture/status`, `…/cloud/status`). A 360 scan **rotates the robot**, so
the dashboard confirms first and disables driving while it runs.

### 6.3 Why 360 is the hard part (and why there are two builders)

The robot has **no IMU/encoder** and the camera sits **off the spin axis**, so each
step is a rotation **plus a small arc translation**, and the real per-step angle
(~40°) differs from the nominal 36°. Pose between shots must be **recovered**, not
assumed.

| Builder | Pose source | Depth | Runs on | Use |
|---|---|---|---|---|
| `pointcloud/scan360.py` | timed `angle.txt`, else image yaw | hardware `depth.npy` | **Pi** (numpy) | **on-device quick build** for instant "show it" |
| `new_point_cloud/register_360.py` | **measured** (CLAHE+ORB/SIFT 3D↔3D + pose-graph) | hardware `depth.npy` | **laptop** (Open3D) | **high-quality** rebuild from the same scan |
| `depth_map/*` (nominal-36° / ICP) | guessed / ICP | our SGBM depth | laptop | ❌ superseded prototype (ICP fails on arc data) |

**`register_360.py` is the quality reference** — it measures each pose from image
correspondences and globally optimizes a pose graph; **CLAHE on the dim IR is the key**
that makes every one of the 10 shots register. Its `components:` line is the health
check (**one component of all 10 shots = a full 360**). Because it needs Open3D, it is
**laptop-only**; the on-device `scan360` build is what makes the dashboard show a cloud
immediately. Both read the **same `captures/scan_<ts>/`**, so the demo loop and the
quality loop never diverge.

> Passive-stereo capture rule: aim at **textured, well-lit** scenes, keep depth
> **0.1–1.5 m**, and give consecutive shots heavy overlap, or the merge breaks.

### 6.4 Gap to build — "show it" in the dashboard

Today the dashboard can only **download** the `.ply`; viewing happens on the Pi
desktop (`pointcloud/view3d.py`) or the laptop (`new_point_cloud/view_ply.py`). For
the one-button flow to feel complete, add an **in-browser 3D view**:

1. **Serve the cloud** from the control API as a static file (already downloadable;
   add a stable `GET /api/cloud/latest.ply`).
2. **Render it** in the dashboard with a WebGL viewer (e.g. three.js `PLYLoader` +
   `OrbitControls`) in a panel/modal — drag to orbit, scroll to zoom.
3. **Auto-open** the viewer when `capture/status` reports the build is done.
4. Fallback for low-end clients: show `merged_360_preview.png` (already produced by
   the build) and keep the Download button.

**Acceptance:** pressing **360° SCAN** with the area clear results, with no further
clicks, in: robot spins → 10 shots → build → an **interactive 3D cloud appears in the
browser**; the same scan, copied to the laptop, rebuilds to a **single-component**
high-quality cloud via `register_360.py`.

---

## 7. Feature: Ultrasonic radar — **built, standalone (gap: integrate)**

`src/radar/radar.py` sweeps the pan servo, reads the ultrasonic sensor at each step,
and renders a classic green **PPI** radar: range rings, bearing spokes, noise-filtered
blips, **object clustering** (range/bearing/width), and **collision alerts** (banner +
throttled beep + LED tint). Backends: `--web` (MJPEG, **:8001**), `--window` (VNC),
`--demo` (fake data, no hardware).

**Gap to build:** it runs as its **own** server and **shares the pan servo + I2C bus**
with driving and scanning, so it can't sweep while the robot drives. Integration
options for the unified dashboard:

- **A — Embedded panel (passive):** add a radar `<img>` tile fed by **:8001**;
  surface it as a togglable panel. Simplest; still mutually exclusive with drive.
- **B — Arbitrated (recommended):** move the radar sweep under the control API so it
  yields the servo/bus when a drive command or a 360 scan arrives, and resumes when
  idle. One owner of the bus, no conflicts.

**Acceptance:** the dashboard can display the radar; radar activity never fights
manual drive or a 360 scan for the servo/bus (mode B arbitrates automatically; mode A
documents the exclusivity).

---

## 8. Feature: Our own stereo depth map — **built, not in the live path**

`src/depth_map/depth_map.py` computes depth ourselves with `cv2.StereoSGBM`
(disparity → metric depth, optional WLS hole-fill) from a D405 IR pair — this is the
brief's "build your own depth map" requirement and works on the test images.

**Status / optional gap:** the live pipeline and the working 360 currently use the
**hardware** `depth.npy`, not ours. Wiring `depth_map.py` to overwrite `depth.npy` in
each capture folder would make the whole pipeline (point clouds included) run on our
own depth automatically. Out of scope for the core demo; tracked here for completeness.

---

## 9. Unified dashboard — the one screen

A single page ties it together. Existing layout: left = mode + controls + hotkeys,
right = live stream. Target additions marked **(new)**.

```
┌───────────────────────────┬──────────────────────────────────────┐
│ MODE:  [ MANUAL | AUTO ]   │                                      │
│  ▶ START / ■ STOP RUN       │            LIVE WEBCAM                │
│  status line                │            (MJPEG :8000)             │
│                            │                                      │
│  W A S D   Q E   speed ±    │   [ Connect ]  [ Disconnect ]  [⚙]   │
│  pan/tilt arrows            ├──────────────────────────────────────┤
│                            │  RADAR panel (MJPEG :8001)      (new) │
│  [360° SCAN] [Single]       │                                      │
│  [Build] [⬇ Download]       │  3D CLOUD viewer (WebGL)        (new) │
│                            │   ← opens when a build finishes      │
│  hotkeys legend             │                                      │
└───────────────────────────┴──────────────────────────────────────┘
```

**Hotkeys (current):** `Enter` start/stop run · `M` toggle mode · `W A S D / Q E`
drive · arrows pan/tilt · `+/−` speed · `R` 360 scan · `V` single · `T` build ·
`Y`/`⬇` download.

**Dashboard work to reach the spec:**
- Allow **AUTONOMOUS** start once the server supports it (§4.2) — drop the
  "only MANUAL works" guard.
- Add the **radar panel** (§7) and the **in-browser 3D viewer** (§6.4).

---

## 10. Control API reference (current)

| Method & path | Body | Purpose |
|---|---|---|
| `GET /health` | — | server alive |
| `POST /api/run/start` | `{mode}` | arm a run (`manual` ✅, `autonomous` ❌ → §4.2) |
| `POST /api/run/stop` | — | halt + disarm |
| `GET /api/run/status` | — | `{run_active, mode, last_command}` |
| `POST /api/drive` | `{keys, speed}` | manual drive (ignored unless manual run active) |
| `POST /api/servo` | `{axis, delta}` | nudge pan/tilt |
| `POST /api/capture/scan360` | — | spin + 10-shot 360 capture |
| `POST /api/capture/single` | — | one D405 capture |
| `POST /api/capture/build` | — | merge last scan → `.ply` (on-device) |
| `GET /api/capture/status` | — | `{busy, status}` |
| `GET /api/cloud/status` | — | `{has_cloud, name}` |
| `GET /api/cloud/download` | — | download last `.ply` |
| `POST /api/stream/start` · `/stop` · `GET /api/stream/status` | — | live webcam server |
| **(new)** `GET /api/cloud/latest.ply` | — | serve cloud for the in-browser viewer (§6.4) |
| **(new)** radar embed / arbitration | — | §7 |

---

## 11. Build checklist (to fully combine the features)

In priority order for the demo:

1. **Wire autonomous mode into the web stack** (§4.2) — biggest functional gap; the
   follower already exists.
2. **In-browser 3D cloud viewer** (§6.4) — completes "press button → see the 360".
3. **Radar in the dashboard** (§7) — panel (A) for the demo, arbitration (B) to do it
   right.
4. *(optional)* **Swap in our own depth** (§8) for the brief's depth-map requirement
   end-to-end.

Already done: manual drive, live view, 360 capture/build/download, on-device + laptop
360 builders, line-follower logic, radar renderer.

---

## 12. Non-functional requirements

- **Safety first:** any loss of operator input (tab/Wi-Fi) stops the robot within the
  0.6 s watchdog; a 360 scan disables driving; mode switches stop the active run.
- **Single bus owner:** exactly one of {manual drive, line-follower, radar sweep, 360
  scan} drives the servos/motors at any instant; the control API arbitrates.
- **Demo resilience:** the live webcam must survive a 360 scan; a bad capture must
  surface as a status message, not a hang (build is backgrounded + polled).
- **Reproducibility:** every scan persists as a `captures/scan_<ts>/` folder so it can
  be rebuilt later (Pi quick build or laptop high-quality build) without re-driving.
- **Two-environment discipline:** nothing Open3D runs on the Pi; nothing `smbus` runs
  on the laptop.
```
