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
import subprocess
import threading
import time
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import socket

# Configuration
STREAM_SERVER_PORT = 8000
CONTROL_SERVER_PORT = 9000
CAMERA_SCRIPT = Path(__file__).parent / "camera" / "stream_server.py"

# Global state
stream_process = None
stream_running = False
start_time = None


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

    if stream_running:
        return {"status": "already_running", "message": "Stream already running"}

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

        else:
            self.send_json(404, {"error": "Not found"})

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

    server = HTTPServer(("0.0.0.0", CONTROL_SERVER_PORT), ControlHandler)

    try:
        print("Press Ctrl+C to stop\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_stream_server()
        server.shutdown()


if __name__ == "__main__":
    main()
