"""MJPEG streaming server for the Raspbot V2 camera.

Streams live camera feed over HTTP as MJPEG.
Access the stream at: http://localhost:8000/stream.mjpg

Run on the Raspberry Pi (or any machine with the camera connected):
    python stream_server.py
"""
import sys
import time
import cv2
import io
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

W, H, FPS = 1280, 720, 30
CAM_INDEX = 0

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
    """Capture frames from camera and store in global variable."""
    global latest_frame, camera_ready

    cap = open_cam(CAM_INDEX)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera index {CAM_INDEX}")
        return

    # Try MJPG first; fallback to native format if frames are black
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
    print(f"✓ Camera open: {aw}x{ah} @ {afp:.0f}fps mode={mode}")

    camera_ready = True
    prev = time.time()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame grab failed; retrying...")
                continue

            # Calculate FPS
            now = time.time()
            dt = now - prev
            prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            # Add FPS text overlay
            cv2.putText(frame, f"{fps:4.1f} FPS", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Store latest frame
            with camera_lock:
                latest_frame = frame.copy()

    except KeyboardInterrupt:
        print("Shutting down camera...")
    finally:
        cap.release()


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler for streaming MJPEG frames."""

    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--boundary')
            self.end_headers()

            try:
                while True:
                    with camera_lock:
                        if latest_frame is None:
                            continue
                        # Encode frame as JPEG
                        ret, jpeg = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

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
    server = HTTPServer(('0.0.0.0', PORT), MJPEGHandler)
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
