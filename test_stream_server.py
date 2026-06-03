#!/usr/bin/env python3
"""Simple test MJPEG server for local testing without a camera."""
import os
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler
import time

class TestMJPEGHandler(BaseHTTPRequestHandler):
    """Serve a test MJPEG stream (solid color frames)."""

    def do_GET(self):
        if self.path == '/stream.mjpg':
            print(f"[{time.strftime('%H:%M:%S')}] Client connected: {self.client_address[0]}")

            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--boundary')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()

            # Send a few simple JPEG frames
            frame_count = 0
            try:
                while True:
                    # Very simple JPEG (a colored square)
                    # This is a minimal valid JPEG (red square)
                    jpeg_data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x11\x00\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xc4\x00\xb5\x10\x00\x02\x01\x03\x03\x02\x04\x03\x05\x05\x04\x04\x00\x00\x01}\x01\x02\x03\x00\x04\x11\x05\x12!1A\x06\x13Qa\x07"q\x142\x81\x91\xa1\x08#B\xb1\xc1\x15R\xd1\xf0$3br\x82\t\n\x16\x17\x18\x19\x1a%&\'()*456789:CDEFGHIJSTUVWXYZcdefghijstuvwxyz\x83\x84\x85\x86\x87\x88\x89\x8a\x92\x93\x94\x95\x96\x97\x98\x99\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xfc\x00\xff\xd9'

                    frame_count += 1

                    # Send boundary and frame
                    self.wfile.write(b'--boundary\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(jpeg_data)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg_data)
                    self.wfile.write(b'\r\n')

                    if frame_count % 10 == 0:
                        print(f"  Sent {frame_count} frames")

                    time.sleep(0.033)  # ~30 fps
            except Exception as e:
                print(f"  Stream ended: {e}")
                return

        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body><h1>Test MJPEG Server Running</h1><img src="/stream.mjpg" style="max-width:100%;"></body></html>')

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        """Suppress verbose logs."""
        pass

    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.end_headers()


if __name__ == '__main__':
    PORT = 8000
    server = HTTPServer(('0.0.0.0', PORT), TestMJPEGHandler)
    print(f"\n{'='*60}")
    print(f"Test MJPEG Server")
    print(f"{'='*60}")
    print(f"Stream URL: http://localhost:{PORT}/stream.mjpg")
    print(f"HTML Test:  file://<path>/stream_test.html")
    print(f"{'='*60}")
    print("Press Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()
