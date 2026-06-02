# Web Dashboard — Architecture

How the browser dashboard, the Python servers, and the systemd services fit
together to **drive the robot** and **watch the camera** from any device on the
same network.

> Companion to [CAMERA_STREAM_SETUP.md](CAMERA_STREAM_SETUP.md) (original
> stream/control setup). This doc covers the full picture *after* the web
> driving + capture features were added.

---

## 1. Big picture

```
┌──────────────────────────────────────────────────────────────────────┐
│  Any device on the LAN  (phone / laptop / the Pi itself)               │
│                                                                        │
│   Browser ──► http://sprint.local/        (the dashboard HTML)         │
│      │                                                                  │
│      │  fetch() JSON over HTTP                                          │
│      ├──► http://sprint.local:9000/...     control API   (drive/run)   │
│      └──► http://sprint.local:8000/stream.mjpg   live video  (<img>)   │
└──────────────────────────────────────────────────────────────────────┘
                       │                     │
                       ▼                     ▼
          ┌────────────────────┐   ┌────────────────────┐
          │  control_server.py │   │  stream_server.py  │
          │     port 9000      │   │     port 8000      │
          │  • run on/off      │   │  • grabs USB cam   │
          │  • /api/drive      │   │    (/dev/video0)   │
          │  • /api/capture/*  │   │  • re-serves MJPEG │
          │  • spawns stream ──┼──►│                    │
          └─────────┬──────────┘   └────────────────────┘
                    │ I2C
                    ▼
              ┌───────────┐        RealSense D405 (USB) ── used only by
              │  RasBot   │        captures (scan360 / StereoCapture),
              │ (motors,  │        NOT by the live MJPEG stream.
              │  servos)  │
              └───────────┘
```

Three independent HTTP servers, three ports:

| Port | Served by | Purpose |
|------|-----------|---------|
| **80**   | `python -m http.server` | Serves the static dashboard HTML |
| **9000** | `control_server.py`     | Control API: run on/off, drive, captures, stream start/stop |
| **8000** | `stream_server.py`      | Live MJPEG video from the USB webcam |

There is **no RTSP** anywhere — the browser shows MJPEG directly in an `<img>`,
which needs no plugin or transcoding.

---

## 2. The HTML — [top/robot_control_dashboard.html](top/robot_control_dashboard.html)

A single self-contained file (HTML + CSS + JS, no build step, no dependencies).
`index.html` in the same folder is a symlink to it, so `http://sprint.local/`
loads it at the root.

### Layout (two panels)

```
┌─ left-panel (420px) ────────┐ ┌─ right-panel (fills rest) ─────┐
│  Current Mode               │ │                                │
│   ├ MANUAL/AUTONOMOUS       │ │        Camera stream           │
│   └ ▶ START RUN (Enter)     │ │        (MJPEG <img>)           │
│  Control Layout             │ │                                │
│   ├ W / A S D   (move)      │ ├────────────────────────────────┤
│   ├ Q   E       (rotate)    │ │ Status • Connect • Disconnect ⚙│
│   ├ − SPEED +   (speed)     │ └────────────────────────────────┘
│   ├ R   V       (captures)  │
│   └ M           (mode)      │   Only the "Active Hotkeys" box
│  Active Hotkeys  (scrolls)  │   scrolls if the screen is short;
└─────────────────────────────┘   the panel itself never scrolls.
```

### Key JavaScript pieces

State lives in two objects near the top of the `<script>`:

- `config` — `controlServer` and `streamServer` host:port. **Defaults to the
  host the page was loaded from** (`window.location.hostname`), so opening from
  another device "just works". Saved in `localStorage`; the `⚙` panel edits it.
- `state` — `currentMode`, `keysDown` (held keys), `runActive`, `speed`.

| Function | What it does |
|----------|--------------|
| `connectStream()` | POST `…:9000/api/stream/start`, then loads `…:8000/stream.mjpg` into the `<img>` |
| `toggleRun()` | The **ON/OFF** button (or **Enter**). Starts/stops the run via `/api/run/start` \| `/stop` |
| `setRunActive(on)` | Flips button colour and starts/stops the drive **heartbeat** |
| `sendDrive()` | POSTs the currently-held movement keys + speed to `/api/drive` |
| `changeSpeed(±20)` | Adjusts speed (40–255) and updates the on-screen value |
| `nudgeServo(axis,δ)` | POST `/api/servo` — pan/tilt the camera (arrow keys / buttons) |
| `captureScan360()` / `captureSingle()` | POST `/api/capture/scan360` \| `/single`, then poll status |
| `captureBuild()` | **T** — POST `/api/capture/build` (merge last scan into a `.ply`) |
| `downloadCloud()` | **Y** — checks `/api/cloud/status`, then downloads `/api/cloud/download` |

**Why a heartbeat?** While you hold a key, the browser fires `keydown` only once.
`setRunActive(true)` starts an interval that re-sends the held command every
~150 ms. This keeps the server-side **watchdog** fed; if the tab closes or Wi-Fi
drops, the commands stop and the robot halts on its own.

### Controls

| Key / Button | Action | Sent to |
|---|---|---|
| `W A S D` | Move (blended into diagonals) | `/api/drive` |
| `Q` / `E` | Rotate left / right | `/api/drive` |
| `+` / `−` | Speed up / down | (client-side, included in next drive) |
| `R` | 360° scan (**rotates the robot**) | `/api/capture/scan360` |
| `V` | Single capture | `/api/capture/single` |
| `M` | Toggle mode (stops the run for safety) | (client-side) |
| `Enter` / **START RUN** button | Arm / disarm the run | `/api/run/start` \| `/stop` |

Safety in the page: keystrokes are ignored while typing in the config box, and
the robot is told to stop on window **blur** and on **page close**.

---

## 3. The Python

### [src/control_server.py](src/control_server.py) — port 9000

The brain of the web side. A `ThreadingHTTPServer` (multi-threaded so frequent
drive commands aren't blocked) that:

- Talks to the robot through **`RasBot`** (the shared hardware API in
  [src/setup_and_api/api/robot.py](src/setup_and_api/api/robot.py)) over I2C.
  `RasBot` is created lazily on first use.
- Holds the **run state** (`run_active`, `run_mode`) — driving is only accepted
  while a manual run is armed.
- Runs a **watchdog thread** that halts the robot if drive commands stop
  arriving mid-move.
- **Spawns `stream_server.py`** as a subprocess when the stream is started.
- Kicks off **captures** in a background thread (reusing `scan360.run_scan` and
  `StereoCapture.save`, saved to the project-root `captures/`).

It **reuses `drive.py`'s own movement code**: it imports
[src/wasd/drive.py](src/wasd/drive.py) and calls its `desired_command` /
`apply_command`, translating the web's key strings into the pygame keycodes
those functions expect. No duplicate mapping. The capture actions call the same
underlying modules `drive.py` uses (`scan360`, `StereoCapture`).

**Endpoints**

| Method | Path | Purpose |
|---|---|---|
| GET  | `/health` | Liveness check |
| GET  | `/api/run/status` | Is a run armed? last command? |
| POST | `/api/run/start` `{mode}` | Arm a run (only `manual` works; beeps + green LEDs) |
| POST | `/api/run/stop` | Disarm + halt the robot |
| POST | `/api/drive` `{keys, speed}` | `drive.py`'s `desired_command`/`apply_command` (ignored unless armed) |
| POST | `/api/servo` `{axis, delta}` | Nudge camera pan/tilt (`bot.set_pan`/`set_tilt`) |
| POST | `/api/capture/scan360` | **R** — 360° scan (rotates the robot) — background |
| POST | `/api/capture/single` | **V** — one RealSense frame → `captures/` — background |
| POST | `/api/capture/build` | **T** — merge last scan into a `.ply` (`scan360.build_from_session`) |
| GET  | `/api/capture/status` | Capture/build busy? last result? |
| GET  | `/api/cloud/status` | Is a built `.ply` ready to download? |
| GET  | `/api/cloud/download` | **Y** — download the built `.ply` to the browser |
| POST | `/api/stream/start` `/stop` | Launch / kill `stream_server.py` |
| GET  | `/api/stream/status` | Is the stream subprocess alive? |

All responses send `Access-Control-Allow-Origin: *` and there's an `OPTIONS`
handler, so the browser can call it cross-origin.

### [src/camera/stream_server.py](src/camera/stream_server.py) — port 8000

A tiny `ThreadingHTTPServer` (multi-threaded so **several viewers** can watch at
once — single-threaded would give the 2nd viewer a black screen). One background
thread grabs frames from the **USB webcam** (`/dev/video0`) into a shared
`latest_frame`; each `/stream.mjpg` request encodes that frame to JPEG and writes
it as an MJPEG multipart stream.

- `GET /stream.mjpg` — the live MJPEG stream (what the dashboard `<img>` loads)
- `GET /` — a minimal standalone viewer page

> The **live stream uses the USB webcam**; the **captures use the RealSense
> D405** (`StereoCapture`). They're different cameras, so streaming and capturing
> don't fight over a device.

---

## 4. main.py — the single entry point

[main.py](main.py) is the one launcher for the whole system:

```bash
python3 main.py web      # serve the dashboard (:80) + control API (:9000)
python3 main.py drive    # run the pygame WASD teleop (needs the Pi desktop/VNC)
python3 main.py status   # show which parts are currently up
```

`main.py web` runs the static dashboard server **and** `control_server` in **one
process** (control spawns the stream on demand). It falls back to port 8080 if it
can't bind 80, so it's also runnable by hand in dev.

## 5. The systemd service

One unit runs everything: `rasbot.service` executes `main.py web`. It's `enabled`
(starts on boot) and restarts on failure.

```ini
WorkingDirectory=/home/sprint/sprint_hackathon
ExecStart=/home/sprint/sprint_hackathon/.venv/bin/python \
          /home/sprint/sprint_hackathon/main.py web
AmbientCapabilities=CAP_NET_BIND_SERVICE   # bind port 80 without full root
```

Uses the project venv (so `smbus`, `cv2`, `pyrealsense2`, `pygame` are
available). `stream_server.py` is **not** its own service — `control_server.py`
(inside `main.py web`) launches it on demand.

> This replaces the earlier split `robot-dashboard.service` +
> `rasbot-control.service`. Editing the HTML needs **no restart** (just refresh);
> after editing `main.py` / `control_server.py`, run:

```bash
sudo systemctl restart rasbot.service
journalctl -u rasbot.service -f      # follow logs
```

---

## 6. End-to-end: clicking "Connect" then "START RUN"

1. Browser loads `http://sprint.local/` (served by `main.py web`'s dashboard server).
2. **Connect** → `POST :9000/api/stream/start` → `control_server` spawns
   `stream_server.py` → the `<img>` loads `:8000/stream.mjpg` → video appears.
3. **START RUN** (or Enter) → `POST :9000/api/run/start {manual}` → `RasBot`
   beeps, LEDs go green, run is armed.
4. Hold **W** → `keydown` + a 150 ms heartbeat POST `:9000/api/drive {keys:["w"]}`
   → `control_server` calls **`drive.py`'s** `desired_command`/`apply_command`
   → `bot.move(...)` over I2C → robot moves.
5. Release **W** → drive POST with no keys → `bot.stop()`.
6. **STOP RUN** / closing the tab / losing focus → robot halts (and the watchdog
   is the backstop).

---

## 7. One driver at a time

Both this web stack and the desktop [src/wasd/drive.py](src/wasd/drive.py) talk
to the **same robot over the same I2C bus**. Don't run both *actively driving* at
once. Use `python3 main.py drive` **or** the web run — not both. To use the
teleop, click **STOP RUN** first (or `sudo systemctl stop rasbot.service`).
