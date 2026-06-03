"""Control server for RasotV2 robot.

Manages camera streaming and robot control via HTTP API.
Run this on your control computer or Raspberry Pi:
    python control_server.py

API Endpoints:
    GET  /health               - Server status
    POST /api/stream/start     - Start camera stream
    POST /api/stream/stop      - Stop camera stream
    GET  /api/stream/status    - Check if stream is running
"""
import os
import sys
import json
import math
import shutil
import subprocess
import threading
import time
from pathlib import Path
from http.server import HTTPServer, ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import socket

# Make sibling packages (setup_and_api, camera, ...) importable regardless of CWD.
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Configuration
STREAM_SERVER_PORT = 8000
CONTROL_SERVER_PORT = 9000
CAMERA_SCRIPT = Path(__file__).parent / "camera" / "stream_server.py"

# Global state
stream_process = None
stream_running = False
start_time = None

# ── Robot driving state ────────────────────────────────────────
# Fallback speed only; the safe [min, max] band lives in drive.py (clamp_speed).
DRIVE_SPEED_DEFAULT = 120
# Safety: auto-stop if no drive command arrives within this window while moving
# (covers a browser tab closing or the network dropping mid-drive).
DRIVE_WATCHDOG_S = 0.6

# We REUSE the movement logic from wasd/drive.py rather than duplicating it.
# drive.py is pygame-based, so it gets imported lazily (pulls pygame) and we
# translate web key strings into the pygame keycodes its functions expect.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
_drive_mod = None          # the imported wasd.drive module
_keycode = None            # {'w': pygame.K_w, ...}

bot = None                 # lazily-created RasBot instance
robot_lock = threading.Lock()
run_active = False
run_mode = 'manual'
last_cmd = None            # last motion command tuple sent to the bot
last_cmd_time = 0.0        # wall-clock of the last drive command (for the watchdog)

# Autonomous line-follow run (spec §4.2 — the web hook for Mode 2). A background
# thread runs tape_following.line_follow on the shared bot; setting this event
# stops it cleanly so manual drive and the follower never both own the bus.
line_stop_event = None
line_thread = None

# Camera pan/tilt servo state. The step size + clamp live in drive.py
# (nudge_servo); we just hold the current angle so nudges accumulate.
pan_angle = 90             # 0-180, default centered
tilt_angle = 25            # 0-100, default


def get_bot():
    """Lazily create the RasBot. Returns (bot, None) or (None, error_message)."""
    global bot
    if bot is not None:
        return bot, None
    try:
        from setup_and_api.api import RasBot
        bot = RasBot()
        return bot, None
    except Exception as e:
        return None, str(e)


def get_drive():
    """Lazily import wasd/drive.py (pulls pygame). Returns (module, None) or (None, err)."""
    global _drive_mod, _keycode
    if _drive_mod is not None:
        return _drive_mod, None
    try:
        import pygame
        from wasd import drive as drive_mod
        _keycode = {'w': pygame.K_w, 'a': pygame.K_a, 's': pygame.K_s,
                    'd': pygame.K_d, 'q': pygame.K_q, 'e': pygame.K_e}
        _drive_mod = drive_mod
        return _drive_mod, None
    except Exception as e:
        return None, str(e)


def _keys_to_pressed(keys):
    """Translate web key strings into the pygame-keycode->bool map drive.py reads."""
    held = {str(k).lower() for k in keys}
    return {code: (ch in held) for ch, code in _keycode.items()}


def drive(keys, speed):
    """Apply a drive command, but only while a manual run is active."""
    global last_cmd, last_cmd_time
    if not run_active or run_mode != 'manual':
        return {"status": "ignored", "message": "Run not active (press START in manual mode)"}
    if capture_busy:
        return {"status": "ignored", "message": "Capture in progress"}
    b, err = get_bot()
    if b is None:
        return {"status": "error", "message": f"Robot unavailable: {err}"}
    dmod, derr = get_drive()
    if dmod is None:
        return {"status": "error", "message": f"drive.py unavailable: {derr}"}
    try:
        speed = dmod.clamp_speed(speed)     # drive.py owns the speed band
    except (TypeError, ValueError):
        speed = DRIVE_SPEED_DEFAULT
    # drive.py's OWN functions decide and apply the motion (no duplicate logic here).
    pressed = _keys_to_pressed(keys)
    cmd = dmod.desired_command(pressed, speed)
    with robot_lock:
        last_cmd_time = time.time()
        if cmd != last_cmd:          # only hit I2C when the command actually changes
            dmod.apply_command(b, cmd)
            last_cmd = cmd
    return {"status": "ok", "command": cmd[0]}


def servo(axis, delta):
    """Nudge a camera servo (pan/tilt) by `delta` degrees — like drive.py's arrows."""
    global pan_angle, tilt_angle
    b, err = get_bot()
    if b is None:
        return {"status": "error", "message": f"Robot unavailable: {err}"}
    dmod, derr = get_drive()
    if dmod is None:
        return {"status": "error", "message": f"drive.py unavailable: {derr}"}
    try:
        delta = int(delta)
    except (TypeError, ValueError):
        return {"status": "error", "message": "delta must be a number"}
    with robot_lock:
        try:
            # drive.py owns the clamp + set_pan/set_tilt; we only track the value.
            if axis == "pan":
                pan_angle = dmod.nudge_servo(b, "pan", pan_angle, delta)
                val = pan_angle
            elif axis == "tilt":
                tilt_angle = dmod.nudge_servo(b, "tilt", tilt_angle, delta)
                val = tilt_angle
            else:
                return {"status": "error", "message": f"unknown axis '{axis}'"}
        except Exception as e:
            return {"status": "error", "message": str(e)}
    return {"status": "ok", "axis": axis, "value": val}


def _run_line_follow(b, stop_event):
    """Background worker: run the tape follower until stop_event is set."""
    try:
        from tape_following import line_follow
        # cam=None: the stop-marker scan is disabled (SKIP_SCAN), so the follower
        # never needs the D405 — leaving it free for live capture.
        line_follow.run(b, cam=None, stop_event=stop_event)
    except Exception as e:
        print("line-follow error:", e)
    finally:
        with robot_lock:
            try:
                b.stop()
            except Exception:
                pass


def start_run(mode):
    """Arm a run. 'manual' = WASD teleop; 'autonomous' = tape line-following."""
    global run_active, run_mode, last_cmd, line_stop_event, line_thread
    if mode not in ('manual', 'autonomous'):
        return {"status": "error", "message": f"Unknown mode '{mode}'"}
    b, err = get_bot()
    if b is None:
        return {"status": "error", "message": f"Robot unavailable: {err}"}
    from setup_and_api.api import Color
    with robot_lock:
        b.stop()
        last_cmd = ('stop',)
        run_mode = mode
        run_active = True
        try:
            b.set_all_leds_color(Color.GREEN)
            b.beep(0.1)
        except Exception:
            pass
    if mode == 'autonomous':
        # The follower drives the bot directly in its own thread (it doesn't go
        # through /api/drive, so the drive watchdog stays out of its way).
        line_stop_event = threading.Event()
        line_thread = threading.Thread(
            target=_run_line_follow, args=(b, line_stop_event), daemon=True)
        line_thread.start()
        return {"status": "running", "mode": mode,
                "message": "Autonomous run started — place the robot on the tape"}
    return {"status": "running", "mode": mode,
            "message": "Manual run started — drive with W/A/S/D and Q/E"}


def stop_run():
    """Disarm: halt the robot, stop the follower (if any), stop accepting drive."""
    global run_active, last_cmd, line_stop_event, line_thread
    if line_stop_event is not None:        # ask the follower to exit first
        line_stop_event.set()
    b, _ = get_bot()
    with robot_lock:
        run_active = False
        last_cmd = ('stop',)
        if b is not None:
            try:
                b.stop()
                b.leds_off()
            except Exception:
                pass
    # Join the follower OUTSIDE the lock — it grabs robot_lock on its way out.
    t = line_thread
    if t is not None:
        t.join(timeout=2.0)
    line_thread = None
    line_stop_event = None
    return {"status": "stopped", "message": "Run stopped — robot halted"}


def _watchdog():
    """Stop the robot if a run is moving but no command has arrived recently."""
    global last_cmd
    while True:
        time.sleep(0.15)
        if run_active and last_cmd is not None and last_cmd[0] != 'stop':
            if time.time() - last_cmd_time > DRIVE_WATCHDOG_S:
                b, _ = get_bot()
                if b is not None:
                    with robot_lock:
                        try:
                            b.stop()
                        except Exception:
                            pass
                        last_cmd = ('stop',)


# ── Captures (RealSense D405) ───────────────────────────────────
# These mirror drive.py's R (360 scan) and V (single capture). They use the
# RealSense via StereoCapture — separate from the MJPEG webcam, so the live
# stream keeps running. A 360 scan ROTATES the robot in place.
capture_lock = threading.Lock()
capture_busy = False
capture_status = "idle"
cam = None                 # lazily-created StereoCapture
last_session = None        # folder from the last 360 scan (R) -> input for build (T)
last_ply = None            # .ply built by build (T) -> served by the download (Y)
# Canonical project-root captures/ (what the laptop build tools and scp expect).
# StereoCapture's own default resolves to src/captures, so we pass this explicitly.
CAPTURES_ROOT = str(Path(__file__).resolve().parent.parent / "captures")

# The 360 merge + the 3D viewer both need Open3D, which the control server's own
# venv lacks. So they run in a SEPARATE interpreter discovered here: the feature
# merge (register_360) needs open3d + cv2; the viewer (view_ply) needs just open3d.
SRC_DIR      = Path(__file__).resolve().parent
VIEW_PLY     = SRC_DIR / "new_point_cloud" / "view_ply.py"
VIEW3D       = SRC_DIR / "pointcloud" / "view3d.py"
REGISTER_360 = SRC_DIR / "new_point_cloud" / "register_360.py"
# register_360 always writes here (fixed name, overwritten each run); we copy the
# result into the scan folder so each scan keeps its own cloud (capture contract).
REGISTER_OUT_PLY = SRC_DIR / "new_point_cloud" / "pointcloud_360.ply"
REGISTER_OUT_PNG = SRC_DIR / "new_point_cloud" / "pointcloud_360_preview.png"

# Interpreters that may carry Open3D, best first. (o3d-venv has open3d+cv2.)
_O3D_CANDIDATES = ["/home/sprint/o3d-venv/bin/python",
                   "/home/sprint/open3d-venv/bin/python",
                   "/home/sprint/open3d313-venv/bin/python"]
_register_py = "?"        # cached: interpreter with open3d+cv2 (None if none found)
_viewer_py = "?"          # cached: interpreter with open3d


def _python_with(mods):
    """First candidate interpreter that can import every module in `mods`."""
    for py in _O3D_CANDIDATES:
        if not os.path.exists(py):
            continue
        try:
            r = subprocess.run(
                [py, "-c", "import importlib.util as u, sys; "
                 "sys.exit(0 if all(u.find_spec(m) for m in sys.argv[1:]) else 1)",
                 *mods], timeout=25)
            if r.returncode == 0:
                return py
        except Exception:
            continue
    return None


def _register_python():
    global _register_py
    if _register_py == "?":
        _register_py = _python_with(["open3d", "cv2", "numpy"])
    return _register_py


def _viewer_python():
    global _viewer_py
    if _viewer_py == "?":
        _viewer_py = _python_with(["open3d"])
    return _viewer_py


def _build_360_register(session):
    """Build the 360 cloud with new_point_cloud/register_360.py (the merge that
    works: CLAHE+ORB/SIFT 3D↔3D + pose-graph). Returns the .ply path or raises.

    Runs in the Open3D interpreter; copies the fixed output into the scan folder
    as merged_360.ply so each scan keeps its own cloud and the download is stable.
    """
    py = _register_python()
    if py is None:
        raise RuntimeError("no Open3D+cv2 interpreter found (expected ~/o3d-venv)")
    try:
        REGISTER_OUT_PLY.unlink()         # clear stale output so we don't reuse it
    except FileNotFoundError:
        pass
    proc = subprocess.run([py, str(REGISTER_360), session],
                          capture_output=True, text=True, timeout=420)
    if proc.returncode != 0 or not REGISTER_OUT_PLY.exists():
        out = ((proc.stdout or "") + (proc.stderr or "")).strip().splitlines()
        raise RuntimeError(out[-1] if out else f"register_360 exit {proc.returncode}")
    dst = os.path.join(session, "merged_360.ply")
    shutil.copy2(str(REGISTER_OUT_PLY), dst)
    if REGISTER_OUT_PNG.exists():
        try:
            shutil.copy2(str(REGISTER_OUT_PNG),
                         os.path.join(session, "merged_360_preview.png"))
        except Exception:
            pass
    return dst


def _build_360_cloud(session, dmod):
    """Build the 360 .ply, preferring register_360 (Open3D feature merge); on any
    failure fall back to the numpy scan360 build so a cloud still appears.
    Returns (ply_path, note). Raises only if BOTH builders fail."""
    try:
        return _build_360_register(session), "register_360"
    except Exception as e:
        ply = dmod.build_cloud(session, log=lambda *a, **k: None)
        return ply, f"numpy fallback — register_360 failed: {e}"


def _launch_ply_viewer(ply):
    """Pop up an interactive 3D window for a freshly-built cloud (best effort).

    Uses the Open3D view_ply.py (the one the user asked for); falls back to the
    numpy view3d.py if no Open3D interpreter is around. Never raises — a missing
    display just means no window, while the browser download still works.
    """
    if not ply or not os.path.isfile(ply):
        return False
    try:
        env = dict(os.environ)
        env.setdefault("DISPLAY", ":0")          # the Pi desktop session
        py = _viewer_python()
        if py and VIEW_PLY.exists():
            cmd = [py, str(VIEW_PLY), ply]
        else:
            cmd = [sys.executable, str(VIEW3D), ply]   # numpy viewer, no Open3D
        subprocess.Popen(cmd, env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception:
        return False


def get_cam():
    """Lazily create the StereoCapture (RealSense D405)."""
    global cam
    if cam is None:
        from camera.rs_capture import StereoCapture
        cam = StereoCapture()
    return cam


def _run_capture(kind):
    """Background worker for a capture/build; updates capture_status as it goes."""
    global capture_busy, capture_status, last_cmd, last_session, last_ply
    try:
        # All three actions run through drive.py's capture helpers (one workflow).
        dmod, derr = get_drive()
        if dmod is None:
            capture_status = f"error: drive.py unavailable ({derr})"
            return

        # Build (T) is pure compute — no robot, no camera.
        if kind == "build":
            if not last_session or not os.path.isdir(last_session):
                capture_status = "error: no scan yet — run a 360 scan (R) first"
                return
            capture_status = "building 360 cloud (feature registration, can take a minute)..."
            ply, note = _build_360_cloud(last_session, dmod)
            last_ply = ply
            _launch_ply_viewer(ply)       # pop the interactive window on the Pi
            capture_status = f"done: built {os.path.basename(ply)} ({note}) — ready to download"
            return

        # scan360 (R legacy), scan (R one-button), single (V) need robot + RealSense.
        b, err = get_bot()
        if b is None:
            capture_status = f"error: robot unavailable ({err})"
            return
        from setup_and_api.api import Color
        with robot_lock:                      # make sure we're not driving
            b.stop()
            last_cmd = ('stop',)
        c = get_cam()
        try:
            b.set_all_leds_color(Color.BLUE)
        except Exception:
            pass
        if kind in ("scan360", "scan"):
            capture_status = "360 scan: rotating + capturing..."
            session = dmod.scan360_capture(b, c, out_root=CAPTURES_ROOT, log=lambda *a, **k: None)
            last_session = session
            last_ply = None               # a new scan invalidates the old build
            if kind == "scan":
                # One-button flow: spin → 10 shots → build .ply → pop the viewer.
                capture_status = "building 360 cloud (feature registration, can take a minute)..."
                ply, note = _build_360_cloud(session, dmod)
                last_ply = ply
                _launch_ply_viewer(ply)
                capture_status = (f"done: {os.path.basename(ply)} ({note}) — "
                                  "opening viewer, download ready")
            else:
                capture_status = f"done: {os.path.basename(session)} (scan) — press Build (T)"
        else:  # single
            capture_status = "capturing single frame..."
            folder = dmod.single_capture(c, out_root=CAPTURES_ROOT)
            capture_status = ("done: " + os.path.basename(folder)) if folder else "frame dropped"
        try:
            b.set_all_leds_color(Color.GREEN)
            b.beep(0.1)
        except Exception:
            pass
    except Exception as e:
        capture_status = f"error: {e}"
    finally:
        if kind != "build":               # only touch the robot if we used it
            b2, _ = get_bot()
            if b2 is not None:
                with robot_lock:
                    try:
                        b2.stop()
                    except Exception:
                        pass
                    last_cmd = ('stop',)
                    if not run_active:
                        try:
                            b2.leds_off()
                        except Exception:
                            pass
        capture_busy = False


def start_capture(kind):
    """Kick off a capture in the background if one isn't already running."""
    global capture_busy, capture_status
    if kind not in ("scan360", "scan", "single", "build"):
        return {"status": "error", "message": f"Unknown capture '{kind}'"}
    # While the line-follower owns the bus, a scan/single would fight it for the
    # motors (build is pure compute, so it's fine). Stop the run first.
    if run_active and run_mode == "autonomous" and kind != "build":
        return {"status": "busy",
                "message": "Stop the autonomous run before capturing"}
    with capture_lock:
        if capture_busy:
            return {"status": "busy", "message": "A capture is already running",
                    "capture_status": capture_status}
        capture_busy = True
        capture_status = "starting..."
    threading.Thread(target=_run_capture, args=(kind,), daemon=True).start()
    return {"status": "started", "kind": kind}


def get_local_ip():
    """Get local IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"


def start_stream_server():
    """Start the camera streaming server."""
    global stream_process, stream_running, start_time

    # Only report "already running" if the process is genuinely still alive.
    # A stale flag with a dead/crashed process would otherwise make the
    # dashboard think the stream is up while serving nothing.
    if stream_running and stream_process and stream_process.poll() is None:
        return {"status": "already_running", "message": "Stream already running"}
    # Stale state (process died or was killed) -> reset and start fresh.
    stream_running = False

    if not CAMERA_SCRIPT.exists():
        return {"status": "error", "message": f"Camera script not found: {CAMERA_SCRIPT}"}

    try:
        stream_process = subprocess.Popen(
            [sys.executable, str(CAMERA_SCRIPT)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        stream_running = True
        start_time = time.time()
        time.sleep(2)  # Give server time to start
        return {"status": "started", "message": "Stream server started successfully"}
    except Exception as e:
        return {"status": "error", "message": f"Failed to start stream: {str(e)}"}


def stop_stream_server():
    """Stop the camera streaming server."""
    global stream_process, stream_running

    if not stream_running:
        return {"status": "not_running", "message": "Stream not running"}

    try:
        if stream_process:
            stream_process.terminate()
            stream_process.wait(timeout=5)
        stream_running = False
        return {"status": "stopped", "message": "Stream server stopped"}
    except Exception as e:
        # Force kill if terminate didn't work
        if stream_process:
            stream_process.kill()
        stream_running = False
        return {"status": "stopped", "message": f"Stream forced stopped: {str(e)}"}


def get_stream_status():
    """Check if stream is running."""
    global stream_process, stream_running

    if stream_running and stream_process:
        if stream_process.poll() is None:  # Process still alive
            uptime = time.time() - start_time
            local_ip = get_local_ip()
            return {
                "status": "running",
                "stream_url": f"http://{local_ip}:{STREAM_SERVER_PORT}/stream.mjpg",
                "uptime_seconds": int(uptime),
                "message": "Stream server is running"
            }
        else:
            stream_running = False

    return {"status": "stopped", "message": "Stream server not running"}


class ControlHandler(BaseHTTPRequestHandler):
    """HTTP request handler for control API."""

    def do_GET(self):
        """Handle GET requests."""
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/health":
            self.send_json(200, {"status": "ok", "service": "RasotV2 Control Server"})

        elif path == "/api/stream/status":
            status = get_stream_status()
            self.send_json(200, status)

        elif path == "/api/run/status":
            self.send_json(200, {
                "run_active": run_active,
                "mode": run_mode,
                "last_command": last_cmd[0] if last_cmd else None,
            })

        elif path == "/api/capture/status":
            self.send_json(200, {"busy": capture_busy, "status": capture_status})

        elif path == "/api/cloud/status":
            ready = bool(last_ply and os.path.isfile(last_ply))
            self.send_json(200, {
                "has_cloud": ready,
                "name": os.path.basename(last_ply) if ready else None,
            })

        elif path == "/api/cloud/download":
            self.send_cloud()

        elif path == "/api/cloud/latest.ply":
            # Same bytes as download, but served INLINE so the in-browser
            # WebGL viewer can fetch it instead of triggering a save dialog.
            self.send_cloud(as_attachment=False)

        elif path == "/":
            self.send_html(200, self.get_landing_page())

        else:
            self.send_json(404, {"error": "Not found"})

    def do_POST(self):
        """Handle POST requests."""
        path = self.path

        if path == "/api/stream/start":
            result = start_stream_server()
            status_code = 200 if result["status"] in ["started", "already_running"] else 400
            self.send_json(status_code, result)

        elif path == "/api/stream/stop":
            result = stop_stream_server()
            self.send_json(200, result)

        elif path == "/api/run/start":
            body = self.read_json()
            result = start_run(body.get("mode", "manual"))
            code = 200 if result["status"] == "running" else 500
            self.send_json(code, result)

        elif path == "/api/run/stop":
            self.send_json(200, stop_run())

        elif path == "/api/drive":
            body = self.read_json()
            result = drive(body.get("keys", []), body.get("speed", DRIVE_SPEED_DEFAULT))
            self.send_json(200, result)

        elif path == "/api/capture/scan":
            # One-button R: spin → 10 shots → build .ply → pop the 3D viewer.
            self.send_json(200, start_capture("scan"))

        elif path == "/api/capture/scan360":
            self.send_json(200, start_capture("scan360"))

        elif path == "/api/capture/single":
            self.send_json(200, start_capture("single"))

        elif path == "/api/capture/build":
            self.send_json(200, start_capture("build"))

        elif path == "/api/servo":
            body = self.read_json()
            self.send_json(200, servo(body.get("axis"), body.get("delta", 0)))

        else:
            self.send_json(404, {"error": "Not found"})

    def read_json(self):
        """Parse a JSON request body; returns {} if absent/invalid."""
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length <= 0:
                return {}
            return json.loads(self.rfile.read(length).decode() or "{}")
        except Exception:
            return {}

    def send_cloud(self, as_attachment=True):
        """Stream the last built .ply — as a download (attachment) or inline (viewer)."""
        if not (last_ply and os.path.isfile(last_ply)):
            self.send_json(404, {"status": "error",
                                 "message": "No cloud built yet — run a 360 scan (R) then Build (T)"})
            return
        try:
            size = os.path.getsize(last_ply)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            if as_attachment:
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{os.path.basename(last_ply)}"')
            self.send_header("Content-Length", str(size))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with open(last_ply, "rb") as f:
                while True:
                    chunk = f.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except Exception as e:
            # headers may already be sent; best effort
            try:
                self.send_json(500, {"status": "error", "message": str(e)})
            except Exception:
                pass

    def do_OPTIONS(self):
        """Handle CORS preflight requests."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def send_json(self, code, data):
        """Send JSON response."""
        self.send_response(code)
        self.send_header("Content-type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def send_html(self, code, html):
        """Send HTML response."""
        self.send_response(code)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(html.encode())

    def log_message(self, format, *args):
        """Suppress noisy logs."""
        pass

    @staticmethod
    def get_landing_page():
        """Return landing page HTML."""
        local_ip = get_local_ip()
        return f"""
        <html>
        <head>
            <title>RasotV2 Control Server</title>
            <style>
                body {{ font-family: Arial; background: #1a1a1a; color: #fff; padding: 40px; text-align: center; }}
                .container {{ max-width: 600px; margin: 0 auto; background: #2a2a2a; padding: 30px; border-radius: 8px; }}
                h1 {{ color: #64b5f6; }}
                .status {{ padding: 15px; background: #3a3a3a; border-radius: 4px; margin: 20px 0; }}
                .button {{ padding: 10px 20px; margin: 10px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }}
                .btn-start {{ background: #4caf50; color: white; }}
                .btn-stop {{ background: #f44336; color: white; }}
                .btn-dashboard {{ background: #2196f3; color: white; }}
                .link {{ margin: 10px 0; }}
                a {{ color: #64b5f6; text-decoration: none; }}
                a:hover {{ text-decoration: underline; }}
            </style>
        </head>
        <body>
            <div class="container">
                <h1>🤖 RasotV2 Control Server</h1>
                <div class="status">
                    <p><strong>Control Server:</strong> <span style="color: #4caf50;">✓ Running on port {CONTROL_SERVER_PORT}</span></p>
                    <p><strong>Stream Server Port:</strong> {STREAM_SERVER_PORT}</p>
                    <p><strong>Your IP:</strong> {local_ip}</p>
                </div>

                <h2>Quick Links</h2>
                <div class="link">
                    <a href="/api/stream/status" target="_blank">📊 Check Stream Status</a>
                </div>
                <div class="link">
                    <a href="http://{local_ip}:8000/" target="_blank">📹 Stream Viewer</a>
                </div>

                <h2>Access Control Dashboard</h2>
                <p style="color: #aaa; font-size: 12px;">Open the HTML file and enter this IP for the stream:</p>
                <code style="background: #1a1a1a; padding: 10px; display: block; margin: 10px 0; border-radius: 4px;">
                    http://{local_ip}:8000/stream.mjpg
                </code>

                <h2>API Endpoints</h2>
                <ul style="text-align: left; display: inline-block;">
                    <li><code>POST /api/stream/start</code> - Start streaming</li>
                    <li><code>POST /api/stream/stop</code> - Stop streaming</li>
                    <li><code>GET /api/stream/status</code> - Check status</li>
                    <li><code>GET /health</code> - Server health</li>
                </ul>
            </div>
        </body>
        </html>
        """


def main():
    """Start the control server."""
    print(f"\n{'='*60}")
    print("RasotV2 Control Server")
    print(f"{'='*60}")

    local_ip = get_local_ip()
    print(f"Control Server: http://{local_ip}:{CONTROL_SERVER_PORT}/")
    print(f"Stream Server:  http://{local_ip}:{STREAM_SERVER_PORT}/stream.mjpg")
    print(f"{'='*60}\n")

    # ThreadingHTTPServer so frequent drive commands aren't blocked behind the
    # 2s sleep in stream start, and so the watchdog can run alongside requests.
    server = ThreadingHTTPServer(("0.0.0.0", CONTROL_SERVER_PORT), ControlHandler)
    server.daemon_threads = True

    # Safety watchdog: halts the robot if drive commands stop arriving mid-move.
    threading.Thread(target=_watchdog, daemon=True).start()

    try:
        print("Press Ctrl+C to stop\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_run()
        stop_stream_server()
        if bot is not None:
            try:
                bot.cleanup()
            except Exception:
                pass
        server.shutdown()


if __name__ == "__main__":
    main()
