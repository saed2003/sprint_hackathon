"""MJPEG streaming server for the Raspbot V2 camera.

Streams live camera feed over HTTP as MJPEG.
Access the stream at: http://localhost:8000/stream.mjpg

Run on the Raspberry Pi (or any machine with the camera connected):
    python stream_server.py
"""
import os
import sys
import time
import cv2
import io
import socket
from http.server import HTTPServer, ThreadingHTTPServer, BaseHTTPRequestHandler
import threading

# Default 640x480: a cheap USB webcam on the Pi's USB power drops off the bus
# (errno 19 "No such device") at 720p30 MJPG, freezing the feed. 480p is far
# lighter and reliable. Bump back up via env if your cam/power can take it
# (e.g. STREAM_W=1280 STREAM_H=720), or point at another camera with STREAM_CAM.
W   = int(os.environ.get("STREAM_W", "640"))
H   = int(os.environ.get("STREAM_H", "480"))
FPS = int(os.environ.get("STREAM_FPS", "30"))
CAM_INDEX = int(os.environ.get("STREAM_CAM", "0"))

# Global camera and frame state
camera_lock = threading.Lock()
latest_frame = None
camera_ready = False


def open_cam(index):
    """Open camera with fallback to default backend."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    return cap


def configure(cap, use_mjpg):
    """Set format/res/fps. Returns True if frames look valid (not black)."""
    if use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Warm up + sanity-check brightness
    ok = False
    for _ in range(5):
        ok, frame = cap.read()
    return ok and frame is not None and frame.mean() > 5.0


def camera_thread():
    """Capture frames into latest_frame, reopening the device when it stalls.

    Hardened against the freeze bug: a transient cap.read() failure used to
    busy-loop forever and freeze the stream on a single frame. Now we sleep on
    failure, reopen the device after a sustained stall, and rate-limit logging
    so a never-drained stdout pipe can't block (and freeze) the capture thread.
    """
    global latest_frame, camera_ready

    while True:                                    # (re)open loop — never give up
        cap = open_cam(CAM_INDEX)
        if not cap or not cap.isOpened():
            print(f"camera: cannot open index {CAM_INDEX}; retrying in 2s", flush=True)
            time.sleep(2)
            continue

        # Try MJPG first; fall back to native format if frames are black.
        if configure(cap, use_mjpg=True):
            mode = "MJPG"
        else:
            cap.release()
            cap = open_cam(CAM_INDEX)
            configure(cap, use_mjpg=False)
            mode = "native"

        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        afp = cap.get(cv2.CAP_PROP_FPS)
        print(f"✓ Camera open: {aw}x{ah} @ {afp:.0f}fps mode={mode}", flush=True)

        camera_ready = True
        prev = time.time()
        fps = 0.0
        fails = 0
        last_err = 0.0

        try:
            while True:
                try:
                    ok, frame = cap.read()
                except Exception:
                    ok, frame = False, None

                if not ok or frame is None:
                    fails += 1
                    now = time.time()
                    if now - last_err > 1.0:                  # rate-limit the log
                        print(f"camera: frame grab failed (x{fails})", flush=True)
                        last_err = now
                    if fails >= 30:                           # ~1s stalled -> reopen
                        print("camera: stalled; reopening device", flush=True)
                        break
                    time.sleep(0.03)                          # never busy-spin
                    continue

                fails = 0
                now = time.time()
                dt = now - prev
                prev = now
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)

                cv2.putText(frame, f"{fps:4.1f} FPS", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                with camera_lock:
                    latest_frame = frame.copy()

        except KeyboardInterrupt:
            print("Shutting down camera...", flush=True)
            try:
                cap.release()
            except Exception:
                pass
            return
        finally:
            try:
                cap.release()
            except Exception:
                pass
        time.sleep(0.5)                            # brief pause before reopening


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler for streaming MJPEG frames."""

    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--boundary')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()

            try:
                while True:
                    # Grab a reference to the latest frame under the lock, then
                    # release it BEFORE the (relatively slow) JPEG encode so the
                    # capture thread and other viewers aren't blocked.
                    with camera_lock:
                        frame = latest_frame
                    if frame is None:
                        time.sleep(0.01)
                        continue

                    ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

                    if ret:
                        frame_data = jpeg.tobytes()
                        self.wfile.write('--boundary\r\n'.encode())
                        self.wfile.write('Content-type: image/jpeg\r\n'.encode())
                        self.wfile.write(f'Content-Length: {len(frame_data)}\r\n\r\n'.encode())
                        self.wfile.write(frame_data)
                        self.wfile.write('\r\n'.encode())

                    time.sleep(0.03)  # ~30fps
            except:
                pass

        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            html = b"""
            <html>
            <head>
                <title>Raspbot V2 Camera Stream</title>
                <style>
                    body { background: #1a1a1a; color: #fff; font-family: Arial; text-align: center; padding: 20px; }
                    img { max-width: 90%; height: auto; border: 2px solid #444; border-radius: 8px; }
                    h1 { color: #64b5f6; }
                </style>
            </head>
            <body>
                <h1>Raspbot V2 - Live Camera Feed</h1>
                <img src="/stream.mjpg" style="width: 100%; max-width: 1280px;">
            </body>
            </html>
            """
            self.wfile.write(html)
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        """Handle CORS preflight requests."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        """Suppress noisy HTTP logs."""
        pass


def get_local_ip():
    """Get local IP address for display."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"


if __name__ == '__main__':
    # Start camera thread
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()

    # Wait for camera to be ready
    time.sleep(2)

    # Start HTTP server
    PORT = 8000
    # ThreadingHTTPServer so multiple viewers can watch the stream at once.
    # Plain HTTPServer is single-threaded: the first client's /stream.mjpg loop
    # never returns, blocking every other viewer (they get a black screen).
    server = ThreadingHTTPServer(('0.0.0.0', PORT), MJPEGHandler)
    server.daemon_threads = True  # don't let viewer threads block shutdown
    local_ip = get_local_ip()

    print(f"\n{'='*60}")
    print(f"MJPEG Stream Server Running")
    print(f"{'='*60}")
    print(f"Stream URL: http://{local_ip}:{PORT}/stream.mjpg")
    print(f"Dashboard:  http://{local_ip}:{PORT}/")
    print(f"{'='*60}")
    print(f"Press Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()
