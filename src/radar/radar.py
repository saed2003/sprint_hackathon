"""Ultrasonic radar for the Raspbot V2.

Sweeps the pan servo back and forth across an arc, reads the ultrasonic
distance sensor at each step (the sensor is mounted on the pan/tilt head, so it
points wherever the servo points), and renders a classic green PPI ("plan
position indicator") radar sweep.

Enhanced features:
  • Noise filtering  — per-angle median filter + outlier rejection, so a single
                       spurious HC-SR04 spike no longer paints a phantom blip.
  • Object detection — connected returns are clustered into discrete objects,
                       each labeled with range, bearing and width; nearest tracked.
  • Distance colors  — returns are tinted red (near) → yellow → green (far).
  • Collision alerts — on-screen warning banner + throttled beep + LED tint when
                       an object enters the danger zone.

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
    python3 radar/radar.py --danger 40           # collision warning under 40 cm

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
import statistics
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
PERSIST_S    = 4.0       # how long a return lingers (phosphor afterglow)
TRAIL_S      = 0.7       # length of the sweep-line motion trail
SURF_JUMP_CM = 25        # depth jump that breaks the surface (object edge)
SURF_GAP_MUL = 1.8       # max angular gap (x step) before the surface breaks
SIZE_PX      = 720       # output image is SIZE_PX x SIZE_PX
WEB_PORT     = 8001      # stream_server uses 8000; stay off it

# ── noise filtering ────────────────────────────────────────────────────────
MEDIAN_WINDOW = 5        # per-angle ring buffer length for the median filter
OUTLIER_CM    = 40       # a read this far from the running median is rejected once
                         # (a genuine change confirms on the next pass)

# ── collision alerts ────────────────────────────────────────────────────────
DANGER_CM    = 30        # nearest return under this → red alert + beep
WARN_CM      = 60        # nearest return under this → amber caution
BEEP_THROTTLE_S = 1.0    # minimum seconds between collision beeps

# ── object detection ─────────────────────────────────────────────────────────
MIN_OBJECT_PTS = 1       # runs with fewer points than this are ignored (0 = keep all)

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
C_WARN   = (60, 200, 255)    # amber caution (BGR)
C_DANGER = (60, 60, 255)     # red alert (BGR)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _dist_color(dist_cm, max_range):
    """Distance-graded color: red (near) → yellow → green (far), BGR."""
    frac = _clamp(dist_cm / max_range, 0.0, 1.0)
    if frac < 0.5:                       # red → yellow over the near half
        t = frac / 0.5
        b, g, r = 40, int(60 + 195 * t), 255
    else:                                # yellow → green over the far half
        t = (frac - 0.5) / 0.5
        b, g, r = 40, 255, int(255 - 195 * t)
    return (b, g, r)


class Radar:
    """Shared scan state + renderer. One scan thread writes, renderer reads."""

    def __init__(self, arc=180, max_range=MAX_RANGE_CM, step=STEP_DEG,
                 settle=SETTLE_S, size=SIZE_PX, leds=True, danger=DANGER_CM):
        self.max_range = max_range
        self.step = step
        self.settle = settle
        self.size = size
        self.leds = leds
        self.danger = danger

        # Pan sweep limits, centered on straight-ahead, clamped to servo range.
        half = arc / 2.0
        self.pan_lo = int(_clamp(PAN_CENTER - half, 0, 180))
        self.pan_hi = int(_clamp(PAN_CENTER + half, 0, 180))

        self._lock = threading.Lock()
        self._hits = {}              # pan_angle -> (distance_cm, timestamp)  [filtered]
        self._raw = {}               # pan_angle -> deque of recent raw distances (median filter)
        self._trail = deque(maxlen=64)   # (timestamp, pan_angle) sweep history
        self._cur_pan = PAN_CENTER
        self._running = True

        # telemetry
        self._read_times = deque(maxlen=60)   # timestamps of recent reads (scan rate)
        self._read_rate = 0.0
        self._last_beep = 0.0
        self._last_render_t = None
        self._fps = 0.0

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

    def _filter(self, pan, dist_cm):
        """Median-filter a raw reading at this angle and reject lone outliers.

        Returns the filtered distance, or None if the reading is an outlier
        that hasn't been confirmed yet (so a single HC-SR04 spike is dropped).
        """
        buf = self._raw.setdefault(pan, deque(maxlen=MEDIAN_WINDOW))

        # Outlier rejection: if we have history and this read jumps far from the
        # running median, drop it this once but remember it so a real change
        # (which repeats) confirms on the next pass.
        if buf:
            med = statistics.median(buf)
            if abs(dist_cm - med) > OUTLIER_CM:
                # tentatively record so a sustained change still gets through
                buf.append(dist_cm)
                # if the buffer now agrees on the new value, accept the median
                return statistics.median(buf)
        buf.append(dist_cm)
        return statistics.median(buf)

    def _record(self, pan, dist_cm):
        now = time.time()
        valid = MIN_RANGE_CM <= dist_cm <= self.max_range

        with self._lock:
            self._cur_pan = pan
            self._trail.append((now, pan))
            self._read_times.append(now)
            if len(self._read_times) >= 2:
                span = self._read_times[-1] - self._read_times[0]
                self._read_rate = (len(self._read_times) - 1) / span if span > 0 else 0.0

            if valid:
                filtered = self._filter(pan, dist_cm)
                if filtered is not None:
                    self._hits[pan] = (filtered, now)
            else:
                self._raw.pop(pan, None)
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
                self._maybe_beep(bot, dist)

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
            # inject the occasional spike so the noise filter has work to do
            if np.random.random() < 0.05:
                dist = np.random.uniform(MIN_RANGE_CM, self.max_range)
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
            elif dist < self.danger:
                bot.set_all_leds_color(Color.RED)
            elif dist < WARN_CM:
                bot.set_all_leds_color(Color.YELLOW)
            else:
                bot.set_all_leds_color(Color.GREEN)
        except Exception:
            pass   # never let a cosmetic LED write kill the scan

    def _maybe_beep(self, bot, dist):
        """Throttled collision beep when an object is inside the danger zone."""
        if not (MIN_RANGE_CM <= dist <= self.danger):
            return
        now = time.time()
        if now - self._last_beep < BEEP_THROTTLE_S:
            return
        self._last_beep = now
        try:
            bot.beep(0.05)
        except Exception:
            pass

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

        # render FPS (exponential moving average)
        if self._last_render_t is not None:
            dt = now - self._last_render_t
            if dt > 0:
                inst = 1.0 / dt
                self._fps = inst if self._fps == 0 else 0.9 * self._fps + 0.1 * inst
        self._last_render_t = now

        img = np.full((s, s, 3), C_BG, dtype=np.uint8)

        # snapshot shared state under the lock
        with self._lock:
            hits = dict(self._hits)
            trail = list(self._trail)
            cur_pan = self._cur_pan
            read_rate = self._read_rate

        objects = self._objects(hits, now)
        nearest = min(objects, key=lambda o: o["range"]) if objects else None

        self._draw_grid(img, cx, cy, radius, ppc)
        self._draw_trail(img, cx, cy, radius, trail, now)
        self._draw_sweep(img, cx, cy, radius, cur_pan)
        self._draw_objects(img, cx, cy, ppc, objects, now)
        self._draw_nearest_marker(img, nearest)
        self._draw_hud(img, cur_pan, objects, nearest, read_rate)
        self._draw_alert(img, nearest)
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

    def _segment(self, hits, now):
        """Group fresh returns into runs of a connected surface.

        Adjacent pan angles join the same run unless the depth jumps (an object
        edge) or an angle is missing (no echo) — those start a new run.
        """
        fresh = [(pan, d, t) for pan, (d, t) in hits.items()
                 if now - t <= PERSIST_S]
        fresh.sort(key=lambda h: h[0])
        gap = self.step * SURF_GAP_MUL
        runs, cur = [], []
        for h in fresh:
            if cur:
                p0, d0, _ = cur[-1]
                if h[0] - p0 > gap or abs(h[1] - d0) > SURF_JUMP_CM:
                    runs.append(cur)
                    cur = []
            cur.append(h)
        if cur:
            runs.append(cur)
        return runs

    def _objects(self, hits, now):
        """Cluster returns into labeled objects with range / bearing / width.

        Each object dict holds:
          run      — the raw [(pan, dist, t), ...] points
          range    — closest distance in the cluster (cm)
          bearing  — distance-weighted mean bearing (deg, + = right)
          span_deg — angular width (deg)
          width_cm — approximate physical width (chord length, cm)
          age      — seconds since the freshest point (for fade)
        """
        objects = []
        for run in self._segment(hits, now):
            if len(run) < MIN_OBJECT_PTS:
                continue
            dists = [d for _, d, _ in run]
            pans  = [p for p, _, _ in run]
            rng      = min(dists)
            bearing  = sum(self._bearing(p) for p in pans) / len(pans)
            span_deg = (max(pans) - min(pans)) if len(pans) > 1 else 0
            # chord width: 2 * R * sin(span/2), using the cluster's mean range
            mean_r   = sum(dists) / len(dists)
            width_cm = 2.0 * mean_r * math.sin(math.radians(span_deg / 2.0)) \
                if span_deg > 0 else 0.0
            age = now - max(t for _, _, t in run)
            objects.append({
                "run": run, "range": rng, "bearing": bearing,
                "span_deg": span_deg, "width_cm": width_cm, "age": age,
            })
        # nearest object first → stable O1 = closest labeling
        objects.sort(key=lambda o: o["range"])
        return objects

    def _draw_objects(self, img, cx, cy, ppc, objects, now):
        """Draw connected object outlines (distance-graded) with labels."""
        overlay = img.copy()

        for idx, obj in enumerate(objects, start=1):
            run = obj["run"]
            pts = [self._to_xy(cx, cy, ppc, p, d) for p, d, _ in run]
            a = max(0.0, 1.0 - obj["age"] / PERSIST_S)
            col = _dist_color(obj["range"], self.max_range)

            if len(pts) >= 2:
                poly = np.array([(cx, cy)] + pts, dtype=np.int32)
                cv2.fillPoly(overlay, [poly], tuple(int(c * 0.30) for c in col))
                arr = np.array(pts, dtype=np.int32)
                cv2.polylines(img, [arr], False,
                              tuple(int(c * a * 0.5) for c in col), 5, cv2.LINE_AA)
                cv2.polylines(img, [arr], False,
                              tuple(int(c * a) for c in col), 2, cv2.LINE_AA)
            else:
                x, y = pts[0]
                cv2.circle(img, (x, y), 3, tuple(int(c * a) for c in col),
                           -1, cv2.LINE_AA)

            # label at the cluster's mid point
            mx, my = pts[len(pts) // 2]
            label = f"O{idx} {obj['range']:.0f}cm"
            if obj["width_cm"] >= 1:
                label += f" ~{obj['width_cm']:.0f}w"
            cv2.putText(img, label, (mx + 6, my - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        tuple(int(c * a) for c in C_TEXTHI), 1, cv2.LINE_AA)

        cv2.addWeighted(overlay, 0.35, img, 0.65, 0, img)

    def _draw_nearest_marker(self, img, nearest):
        """Crosshair on the closest point of the nearest object."""
        if nearest is None:
            return
        run = nearest["run"]
        # closest point in that cluster
        p, d, _ = min(run, key=lambda h: h[1])
        s = self.size
        cx, cy = s // 2, s - int(s * 0.06)
        radius = int(s * 0.84)
        ppc = radius / self.max_range
        x, y = self._to_xy(cx, cy, ppc, p, d)
        col = C_DANGER if nearest["range"] < self.danger else C_NEAR
        cv2.line(img, (x - 13, y), (x - 5, y), col, 2, cv2.LINE_AA)
        cv2.line(img, (x + 5, y), (x + 13, y), col, 2, cv2.LINE_AA)
        cv2.line(img, (x, y - 13), (x, y - 5), col, 2, cv2.LINE_AA)
        cv2.line(img, (x, y + 5), (x, y + 13), col, 2, cv2.LINE_AA)
        cv2.circle(img, (x, y), 16, col, 1, cv2.LINE_AA)

    def _draw_hud(self, img, cur_pan, objects, nearest, read_rate):
        cv2.putText(img, "ULTRASONIC RADAR", (16, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_TEXTHI, 2, cv2.LINE_AA)
        lines = [
            f"bearing : {self._bearing(cur_pan):+d} deg",
            f"range   : 0-{self.max_range} cm",
            f"objects : {len(objects)}",
            f"scan    : {read_rate:4.1f} reads/s   {self._fps:4.1f} fps",
        ]
        if nearest is not None:
            lines.append(
                f"nearest : {nearest['range']:5.1f} cm @ "
                f"{nearest['bearing']:+.0f} deg")
        else:
            lines.append("nearest : --")
        y = 54
        for ln in lines:
            cv2.putText(img, ln, (16, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, C_TEXT, 1, cv2.LINE_AA)
            y += 22

    def _draw_alert(self, img, nearest):
        """Bold warning banner when the nearest object is close."""
        if nearest is None:
            return
        rng = nearest["range"]
        if rng >= WARN_CM:
            return
        danger = rng < self.danger
        col   = C_DANGER if danger else C_WARN
        text  = "!! COLLISION !!" if danger else "! CAUTION !"
        s = self.size
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
        x = (s - tw) // 2
        y = int(s * 0.12)
        # blink on danger
        if danger and int(time.time() * 3) % 2 == 0:
            return
        cv2.rectangle(img, (x - 16, y - th - 12), (x + tw + 16, y + 12),
                      (0, 0, 0), -1)
        cv2.rectangle(img, (x - 16, y - th - 12), (x + tw + 16, y + 12),
                      col, 2)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 3,
                    cv2.LINE_AA)

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
    ap.add_argument("--danger", type=int, default=DANGER_CM,
                    help="collision-warning distance in cm (default 30)")
    ap.add_argument("--no-leds", action="store_true",
                    help="don't tint the eye LEDs by proximity")
    args = ap.parse_args()

    radar = Radar(arc=args.arc, max_range=args.max, step=args.step,
                  settle=args.settle, size=args.size, leds=not args.no_leds,
                  danger=args.danger)

    scan = radar.scan_demo if args.demo else radar.scan_hardware
    threading.Thread(target=scan, daemon=True).start()

    if args.window:
        radar.run_window()
    else:
        radar.run_web(args.port)


if __name__ == "__main__":
    main()
