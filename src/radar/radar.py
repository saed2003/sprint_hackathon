"""Ultrasonic radar for the Raspbot V2.

Sweeps the pan servo back and forth across an arc, reads the ultrasonic
distance sensor at each step (the sensor is mounted on the pan/tilt head, so it
points wherever the servo points), and renders a classic green PPI ("plan
position indicator") radar sweep.

Three display backends:

  --web      MJPEG over HTTP — view from any browser, works headless over SSH.
             This is the default and the right choice when you're SSH'd into
             the Pi (no desktop needed).
  --window   OpenCV window — needs the Pi desktop over VNC (like pygame_servo).
  --demo     No hardware: synthesizes distances so you can see the display on
             the laptop. Does NOT import the RasBot API (no smbus needed).

Run on the Pi (data collection needs smbus), view from the laptop:
    python3 radar/radar.py                       # web, http://sprint.local:8001/
    python3 radar/radar.py --window              # VNC desktop window
    python3 radar/radar.py --arc 60 --max 150    # narrow 60deg arc, 1.5m range

Preview the display on the laptop with fake data:
    python radar/radar.py --demo

Keys (window mode): q / ESC = quit.
"""

import os
import sys
import math
import time
import socket
import argparse
import threading
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── tunables ─────────────────────────────────────────────────────────────────
PAN_CENTER   = 90        # servo angle that points straight ahead
STEP_DEG     = 3         # servo move per radar step (smaller = finer, slower)
SETTLE_S     = 0.05      # wait after moving the servo before reading (settle)
MAX_RANGE_CM = 200       # display radius + valid-echo cutoff (HC-SR04 ~400 max)
MIN_RANGE_CM = 2         # readings below this are treated as no-echo
PERSIST_S    = 4.0       # how long a blip lingers (phosphor afterglow)
TRAIL_S      = 0.7       # length of the sweep-line motion trail
SIZE_PX      = 720       # output image is SIZE_PX x SIZE_PX
WEB_PORT     = 8001      # stream_server uses 8000; stay off it

# Colors are BGR (OpenCV order).
C_BG     = (8, 18, 8)
C_RING   = (35, 80, 35)
C_RING2  = (55, 120, 55)
C_GRID   = (40, 95, 40)
C_SWEEP  = (90, 255, 90)
C_BLIP   = (80, 240, 80)
C_NEAR   = (60, 60, 255)     # closest object highlight (red)
C_TEXT   = (90, 220, 90)
C_TEXTHI = (200, 255, 200)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Radar:
    """Shared scan state + renderer. One scan thread writes, renderer reads."""

    def __init__(self, arc=180, max_range=MAX_RANGE_CM, step=STEP_DEG,
                 settle=SETTLE_S, size=SIZE_PX, leds=True):
        self.max_range = max_range
        self.step = step
        self.settle = settle
        self.size = size
        self.leds = leds

        # Pan sweep limits, centered on straight-ahead, clamped to servo range.
        half = arc / 2.0
        self.pan_lo = int(_clamp(PAN_CENTER - half, 0, 180))
        self.pan_hi = int(_clamp(PAN_CENTER + half, 0, 180))

        self._lock = threading.Lock()
        self._hits = {}              # pan_angle -> (distance_cm, timestamp)
        self._trail = deque(maxlen=64)   # (timestamp, pan_angle) sweep history
        self._cur_pan = PAN_CENTER
        self._running = True

    # ── geometry ─────────────────────────────────────────────────────────────

    def _bearing(self, pan):
        """Pan servo angle -> bearing in deg, 0 = ahead, + = right."""
        return pan - PAN_CENTER

    def _to_xy(self, cx, cy, ppc, pan, dist_cm):
        """Map (pan, distance) to a screen pixel. Up = forward."""
        b = math.radians(self._bearing(pan))
        r = dist_cm * ppc
        return int(cx + r * math.sin(b)), int(cy - r * math.cos(b))

    # ── scanning ─────────────────────────────────────────────────────────────

    def _record(self, pan, dist_cm):
        now = time.time()
        valid = MIN_RANGE_CM <= dist_cm <= self.max_range
        with self._lock:
            self._cur_pan = pan
            self._trail.append((now, pan))
            if valid:
                self._hits[pan] = (dist_cm, now)
            else:
                self._hits.pop(pan, None)   # clear stale blip at this angle

    def scan_hardware(self):
        """Sweep the real servo and read the ultrasonic sensor (Pi only)."""
        from setup_and_api.api import RasBot, Color

        with RasBot() as bot:
            bot.set_pan(self.pan_lo)
            time.sleep(0.3)
            direction = 1
            pan = self.pan_lo
            while self._running:
                bot.set_pan(pan)
                time.sleep(self.settle)
                dist = bot.read_distance()
                self._record(pan, dist)

                if self.leds:
                    self._update_leds(bot, Color, dist)

                pan += direction * self.step
                if pan >= self.pan_hi:
                    pan, direction = self.pan_hi, -1
                elif pan <= self.pan_lo:
                    pan, direction = self.pan_lo, 1
            bot.look_center()

    def scan_demo(self):
        """Fake distances so the display works on the laptop (no hardware)."""
        direction = 1
        pan = self.pan_lo
        while self._running:
            b = self._bearing(pan)
            # A flat wall ahead at 150 cm + an angled wall on the right.
            wall = 150.0 / max(math.cos(math.radians(b)), 0.4)
            if b > 20:
                wall = min(wall, 90.0 / max(math.cos(math.radians(b - 45)), 0.3))
            dist = wall + np.random.randn() * 3.0
            if abs(b) > 75:                 # open space at the edges
                dist = 0.0
            self._record(pan, float(dist))

            pan += direction * self.step
            if pan >= self.pan_hi:
                pan, direction = self.pan_hi, -1
            elif pan <= self.pan_lo:
                pan, direction = self.pan_lo, 1
            time.sleep(self.settle)

    def _update_leds(self, bot, Color, dist):
        """Tint the eye LEDs by proximity: red close, yellow mid, green far."""
        try:
            if not (MIN_RANGE_CM <= dist <= self.max_range):
                bot.set_all_leds_color(Color.GREEN)
            elif dist < self.max_range * 0.25:
                bot.set_all_leds_color(Color.RED)
            elif dist < self.max_range * 0.5:
                bot.set_all_leds_color(Color.YELLOW)
            else:
                bot.set_all_leds_color(Color.GREEN)
        except Exception:
            pass   # never let a cosmetic LED write kill the scan

    def stop(self):
        self._running = False

    # ── rendering ────────────────────────────────────────────────────────────

    def render(self):
        """Build the current radar image as a BGR uint8 array."""
        s = self.size
        cx, cy = s // 2, s - int(s * 0.06)     # origin near the bottom center
        radius = int(s * 0.84)
        ppc = radius / self.max_range
        now = time.time()

        img = np.full((s, s, 3), C_BG, dtype=np.uint8)

        # snapshot shared state under the lock
        with self._lock:
            hits = dict(self._hits)
            trail = list(self._trail)
            cur_pan = self._cur_pan

        self._draw_grid(img, cx, cy, radius, ppc)
        self._draw_trail(img, cx, cy, radius, trail, now)
        self._draw_sweep(img, cx, cy, radius, cur_pan)
        nearest = self._draw_blips(img, cx, cy, ppc, hits, now)
        self._draw_hud(img, cur_pan, hits, nearest)
        return img

    def _draw_grid(self, img, cx, cy, radius, ppc):
        # range rings every 50 cm
        ring_cm = 50
        n = int(self.max_range // ring_cm)
        for i in range(1, n + 1):
            r = int(i * ring_cm * ppc)
            col = C_RING2 if i == n else C_RING
            cv2.circle(img, (cx, cy), r, col, 1, cv2.LINE_AA)
            cv2.putText(img, f"{i*ring_cm}", (cx + 4, cy - r + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_RING2, 1, cv2.LINE_AA)
        # bearing spokes every 30 deg across the swept arc
        b = -90
        while b <= 90:
            pan = PAN_CENTER + b
            if self.pan_lo - 1 <= pan <= self.pan_hi + 1:
                x = int(cx + radius * math.sin(math.radians(b)))
                y = int(cy - radius * math.cos(math.radians(b)))
                cv2.line(img, (cx, cy), (x, y), C_GRID, 1, cv2.LINE_AA)
                cv2.putText(img, f"{b:+d}", (x - 10, y - 6 if b > -90 else y + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_GRID, 1, cv2.LINE_AA)
            b += 30

    def _draw_trail(self, img, cx, cy, radius, trail, now):
        for t, pan in trail:
            age = now - t
            if age > TRAIL_S:
                continue
            a = 1.0 - age / TRAIL_S
            b = math.radians(self._bearing(pan))
            x = int(cx + radius * math.sin(b))
            y = int(cy - radius * math.cos(b))
            col = tuple(int(c * a * 0.6) for c in C_SWEEP)
            cv2.line(img, (cx, cy), (x, y), col, 1, cv2.LINE_AA)

    def _draw_sweep(self, img, cx, cy, radius, pan):
        b = math.radians(self._bearing(pan))
        x = int(cx + radius * math.sin(b))
        y = int(cy - radius * math.cos(b))
        cv2.line(img, (cx, cy), (x, y), C_SWEEP, 2, cv2.LINE_AA)

    def _draw_blips(self, img, cx, cy, ppc, hits, now):
        nearest = None     # (dist, pan, x, y)
        for pan, (dist, t) in hits.items():
            age = now - t
            if age > PERSIST_S:
                continue
            a = 1.0 - age / PERSIST_S
            x, y = self._to_xy(cx, cy, ppc, pan, dist)
            col = tuple(int(c * a) for c in C_BLIP)
            cv2.circle(img, (x, y), 4, col, -1, cv2.LINE_AA)
            if nearest is None or dist < nearest[0]:
                nearest = (dist, pan, x, y)

        if nearest is not None:
            _, _, x, y = nearest
            cv2.circle(img, (x, y), 8, C_NEAR, 2, cv2.LINE_AA)
            cv2.line(img, (x - 11, y), (x - 5, y), C_NEAR, 1, cv2.LINE_AA)
            cv2.line(img, (x + 5, y), (x + 11, y), C_NEAR, 1, cv2.LINE_AA)
            cv2.line(img, (x, y - 11), (x, y - 5), C_NEAR, 1, cv2.LINE_AA)
            cv2.line(img, (x, y + 5), (x, y + 11), C_NEAR, 1, cv2.LINE_AA)
        return nearest

    def _draw_hud(self, img, cur_pan, hits, nearest):
        cv2.putText(img, "ULTRASONIC RADAR", (16, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_TEXTHI, 2, cv2.LINE_AA)
        lines = [
            f"bearing : {self._bearing(cur_pan):+d} deg",
            f"range   : 0-{self.max_range} cm",
            f"blips   : {len(hits)}",
        ]
        if nearest is not None:
            dist, pan, _, _ = nearest
            lines.append(f"nearest : {dist:5.1f} cm @ {self._bearing(pan):+d} deg")
        else:
            lines.append("nearest : --")
        y = 54
        for ln in lines:
            cv2.putText(img, ln, (16, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, C_TEXT, 1, cv2.LINE_AA)
            y += 22

    # ── display backends ─────────────────────────────────────────────────────

    def run_window(self):
        cv2.namedWindow("radar", cv2.WINDOW_AUTOSIZE)
        while self._running:
            cv2.imshow("radar", self.render())
            if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
                break
        cv2.destroyAllWindows()
        self.stop()

    def run_web(self, port=WEB_PORT):
        radar = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/radar.mjpg":
                    self.send_response(200)
                    self.send_header(
                        "Content-type",
                        "multipart/x-mixed-replace; boundary=--b")
                    self.end_headers()
                    try:
                        while radar._running:
                            ok, jpg = cv2.imencode(
                                ".jpg", radar.render(),
                                [cv2.IMWRITE_JPEG_QUALITY, 80])
                            if ok:
                                data = jpg.tobytes()
                                self.wfile.write(b"--b\r\n")
                                self.wfile.write(b"Content-type: image/jpeg\r\n")
                                self.wfile.write(
                                    f"Content-Length: {len(data)}\r\n\r\n"
                                    .encode())
                                self.wfile.write(data)
                                self.wfile.write(b"\r\n")
                            time.sleep(0.05)   # ~20 fps
                    except Exception:
                        pass
                elif self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-type", "text/html")
                    self.end_headers()
                    self.wfile.write(_PAGE)
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, *_):
                pass

        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        server.daemon_threads = True
        ip = _local_ip()
        print(f"\n{'='*52}\n  Ultrasonic Radar")
        print(f"  view:  http://{ip}:{port}/")
        print(f"{'='*52}\n  Ctrl+C to stop\n")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nstopping...")
        finally:
            self.stop()
            server.shutdown()


_PAGE = b"""<!doctype html><html><head><title>Ultrasonic Radar</title>
<style>body{background:#050a05;margin:0;height:100vh;display:flex;
align-items:center;justify-content:center}
img{max-width:96vmin;max-height:96vmin;image-rendering:pixelated}</style>
</head><body><img src="/radar.mjpg"></body></html>"""


def _local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def main():
    ap = argparse.ArgumentParser(description="Ultrasonic sweep radar.")
    ap.add_argument("--window", action="store_true",
                    help="show an OpenCV window (needs a desktop / VNC)")
    ap.add_argument("--demo", action="store_true",
                    help="fake data, no hardware (preview on the laptop)")
    ap.add_argument("--arc", type=float, default=180,
                    help="total sweep angle in degrees (default 180)")
    ap.add_argument("--max", type=int, default=MAX_RANGE_CM,
                    help="max display/valid range in cm (default 200)")
    ap.add_argument("--step", type=float, default=STEP_DEG,
                    help="servo step per reading in degrees (default 3)")
    ap.add_argument("--settle", type=float, default=SETTLE_S,
                    help="servo settle time per step in seconds (default 0.05)")
    ap.add_argument("--size", type=int, default=SIZE_PX,
                    help="output image size in pixels (default 720)")
    ap.add_argument("--port", type=int, default=WEB_PORT,
                    help="web server port (default 8001)")
    ap.add_argument("--no-leds", action="store_true",
                    help="don't tint the eye LEDs by proximity")
    args = ap.parse_args()

    radar = Radar(arc=args.arc, max_range=args.max, step=args.step,
                  settle=args.settle, size=args.size, leds=not args.no_leds)

    scan = radar.scan_demo if args.demo else radar.scan_hardware
    threading.Thread(target=scan, daemon=True).start()

    if args.window:
        radar.run_window()
    else:
        radar.run_web(args.port)


if __name__ == "__main__":
    main()
