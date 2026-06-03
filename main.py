#!/usr/bin/env python3
"""Single entry point for the Street View Robot.

One launcher for the whole system so you don't memorize separate commands.

    python3 main.py web      # serve the dashboard (:80) + control API (:9000)
    python3 main.py drive    # run the pygame WASD teleop (needs the Pi desktop/VNC)
    python3 main.py status   # show which parts are currently up

The systemd service runs `main.py web`. Only ONE driver may own the robot at a
time, so don't run `drive` while the web run is armed (and vice-versa) — they
share the same I2C bus.
"""
import sys
import socket
import functools
import threading
from pathlib import Path
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
TOP = ROOT / "top"
sys.path.insert(0, str(SRC))      # make control_server, wasd, camera, ... importable

DASHBOARD_PORT = 80               # falls back to 8080 if not allowed to bind 80
CONTROL_PORT = 9000
STREAM_PORT = 8000


class _QuietStatic(SimpleHTTPRequestHandler):
    """Static file handler that doesn't spam the log with every request."""
    def log_message(self, *a):
        pass


def _serve_dashboard():
    """Start the static dashboard server (top/) in a background thread.

    Returns the port it actually bound (80, or 8080 if 80 was denied).
    """
    handler = functools.partial(_QuietStatic, directory=str(TOP))
    try:
        httpd = ThreadingHTTPServer(("0.0.0.0", DASHBOARD_PORT), handler)
        port = DASHBOARD_PORT
    except PermissionError:
        httpd = ThreadingHTTPServer(("0.0.0.0", 8080), handler)
        port = 8080
        print("  (no permission to bind port 80 — dashboard on 8080 instead)")
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return port


def cmd_web():
    """Serve the dashboard + control API, and auto-start the live webcam stream.

    The stream comes up at launch (not on Connect) so the dashboard shows video
    the moment it's opened — the control API still stops it cleanly on Ctrl+C.
    """
    import control_server

    dash_port = _serve_dashboard()
    # Live view starts with the system, per spec — the dashboard auto-connects to it.
    try:
        res = control_server.start_stream_server()
        if res.get("status") not in ("started", "already_running"):
            print(f"  (live stream did not start: {res.get('message')})")
    except Exception as e:
        print(f"  (live stream auto-start failed: {e})")
    ip = _local_ip()
    print("=" * 60)
    print("Street View Robot — WEB")
    print("=" * 60)
    print(f"Dashboard:   http://{ip}:{dash_port}/   (open this in a browser)")
    print(f"Control API: http://{ip}:{CONTROL_PORT}/")
    print(f"Stream:      http://{ip}:{STREAM_PORT}/stream.mjpg  (live, auto-started)")
    print("=" * 60)
    control_server.main()         # binds :9000 and blocks until Ctrl+C


def cmd_drive():
    """Run the pygame WASD teleop (needs a display — run inside the Pi desktop/VNC)."""
    print("Starting pygame WASD teleop (drive.py)...")
    print("NOTE: stop the web run first — only one driver may own the robot.")
    from wasd import drive
    drive.main()


def cmd_status():
    """Show which servers are currently reachable."""
    print("Street View Robot — status")
    for name, port in [("dashboard", DASHBOARD_PORT), ("dashboard(alt)", 8080),
                       ("control", CONTROL_PORT), ("stream", STREAM_PORT)]:
        print(f"  {name:<15} :{port:<5} {'UP' if _port_open(port) else 'down'}")
    print(f"  pi address     {_local_ip()}")


def _port_open(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.3)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def _local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "localhost"
    finally:
        s.close()


COMMANDS = {"web": cmd_web, "drive": cmd_drive, "status": cmd_status}


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "web"
    fn = COMMANDS.get(cmd)
    if fn is None:
        print(f"Unknown command '{cmd}'. Use one of: {', '.join(COMMANDS)}")
        sys.exit(2)
    fn()


if __name__ == "__main__":
    main()
