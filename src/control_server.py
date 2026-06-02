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
DRIVE_SPEED_DEFAULT = 120
DRIVE_SPEED_MIN = 40
DRIVE_SPEED_MAX = 255
# Safety: auto-stop if no drive command arrives within this window while moving
# (covers a browser tab closing or the network dropping mid-drive).
DRIVE_WATCHDOG_S = 0.6

# held key -> translation contribution (vx: right +, vy: forward +).
# Mirrors src/wasd/drive.py so the HTML drives exactly like the pygame version.
_TRANS = {'w': (0, 1), 's': (0, -1), 'd': (1, 0), 'a': (-1, 0)}
_ROT = {'q': +1, 'e': -1}            # +1 = rotate_left (CCW), -1 = rotate_right (CW)

bot = None                 # lazily-created RasBot instance
robot_lock = threading.Lock()
run_active = False
run_mode = 'manual'
last_cmd = None            # last motion command tuple sent to the bot
last_cmd_time = 0.0        # wall-clock of the last drive command (for the watchdog)


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


def compute_command(keys, speed):
    """Map a set of held keys to a motion command (mirrors wasd/drive.py)."""
    keys = {str(k).lower() for k in keys}
    vx = sum(d[0] for k, d in _TRANS.items() if k in keys)
    vy = sum(d[1] for k, d in _TRANS.items() if k in keys)
    rot = sum(v for k, v in _ROT.items() if k in keys)
    if vx or vy:
        # angle convention matches bot.move(): 0=right, 90=forward, 180=left, 270=back
        angle = round(math.degrees(math.atan2(vy, vx)))
        return ('move', angle, speed)
    if rot > 0:
        return ('rotate_left', speed)
    if rot < 0:
        return ('rotate_right', speed)
    return ('stop',)


def apply_command(b, cmd):
    """Send a command tuple from compute_command() to the robot."""
    kind = cmd[0]
    if kind == 'move':
        b.move(cmd[2], cmd[1])
    elif kind == 'rotate_left':
        b.rotate_left(cmd[1])
    elif kind == 'rotate_right':
        b.rotate_right(cmd[1])
    else:
        b.stop()


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
    try:
        speed = int(speed)
    except (TypeError, ValueError):
        speed = DRIVE_SPEED_DEFAULT
    speed = max(DRIVE_SPEED_MIN, min(DRIVE_SPEED_MAX, speed))
    cmd = compute_command(keys, speed)
    with robot_lock:
        last_cmd_time = time.time()
        if cmd != last_cmd:          # only hit I2C when the command actually changes
            apply_command(b, cmd)
            last_cmd = cmd
    return {"status": "ok", "command": cmd[0]}


def start_run(mode):
    """Arm a run in the given mode. Only 'manual' is implemented for now."""
    global run_active, run_mode, last_cmd
    if mode != 'manual':
        return {"status": "not_implemented",
                "message": f"'{mode}' mode is not implemented yet — only manual mode works."}
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
    return {"status": "running", "mode": mode,
            "message": "Manual run started — drive with W/A/S/D and Q/E"}


def stop_run():
    """Disarm: halt the robot and stop accepting drive commands."""
    global run_active, last_cmd
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
# Canonical project-root captures/ (what the laptop build tools and scp expect).
# StereoCapture's own default resolves to src/captures, so we pass this explicitly.
CAPTURES_ROOT = str(Path(__file__).resolve().parent.parent / "captures")


def get_cam():
    """Lazily create the StereoCapture (RealSense D405)."""
    global cam
    if cam is None:
        from camera.rs_capture import StereoCapture
        cam = StereoCapture()
    return cam


def _run_capture(kind):
    """Background worker for a capture; updates capture_status as it goes."""
    global capture_busy, capture_status, last_cmd
    b, err = get_bot()
    if b is None:
        capture_status = f"error: robot unavailable ({err})"
        capture_busy = False
        return
    try:
        from setup_and_api.api import Color
        with robot_lock:                      # make sure we're not driving
            b.stop()
            last_cmd = ('stop',)
        c = get_cam()
        try:
            b.set_all_leds_color(Color.BLUE)
        except Exception:
            pass
        if kind == "scan360":
            from pointcloud import scan360
            capture_status = "360 scan: rotating + capturing..."
            session = scan360.run_scan(b, c, out_root=CAPTURES_ROOT, log=lambda *a, **k: None)
            capture_status = f"done: {os.path.basename(session)} (scan)"
        else:  # single
            capture_status = "capturing single frame..."
            folder = c.save(out_root=CAPTURES_ROOT)
            capture_status = ("done: " + os.path.basename(folder)) if folder else "frame dropped"
        try:
            b.set_all_leds_color(Color.GREEN)
            b.beep(0.1)
        except Exception:
            pass
    except Exception as e:
        capture_status = f"error: {e}"
    finally:
        with robot_lock:
            try:
                b.stop()
            except Exception:
                pass
            last_cmd = ('stop',)
            if not run_active:
                try:
                    b.leds_off()
                except Exception:
                    pass
        capture_busy = False


def start_capture(kind):
    """Kick off a capture in the background if one isn't already running."""
    global capture_busy, capture_status
    if kind not in ("scan360", "single"):
        return {"status": "error", "message": f"Unknown capture '{kind}'"}
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
            code = 200 if result["status"] in ("running", "not_implemented") else 500
            self.send_json(code, result)

        elif path == "/api/run/stop":
            self.send_json(200, stop_run())

        elif path == "/api/drive":
            body = self.read_json()
            result = drive(body.get("keys", []), body.get("speed", DRIVE_SPEED_DEFAULT))
            self.send_json(200, result)

        elif path == "/api/capture/scan360":
            self.send_json(200, start_capture("scan360"))

        elif path == "/api/capture/single":
            self.send_json(200, start_capture("single"))

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
