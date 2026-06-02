"""
=============================================================================
  RASPBOT RADAR v1.0 — military-style ultrasonic PPI radar (pygame / VNC)
  Yahboom RASPBOT V2 · Raspberry Pi 5
=============================================================================

A real-time submarine/military style radar:
  • 180° green sweep with fading phosphor trail
  • Range rings (20 / 50 / 100 / 200 cm) + 30° bearing spokes
  • Danger/warning/safe background tint zones
  • Red object blips (size scales with proximity, pulsing when very close)
  • Object detection + width-based size classification (small/medium/large)
  • Movement detection across sweeps → orange blip + direction arrow
  • Distance-based beep system (rate + pitch) with continuous close alarm
  • Side info panel + DANGER border flash
  • Everything threaded: sensor, sound and UI never block each other

──────────────────────────────────────────────────────────────────────────
HOW TO RUN OVER VNC
──────────────────────────────────────────────────────────────────────────
  1. Connect to the Pi with VNC Viewer (host: sprint.local).
  2. Open a terminal INSIDE the Pi desktop (not bare SSH — pygame needs the
     desktop's display server).
  3. Install pygame once if needed:
         sudo apt install -y python3-pygame
  4. Run:
         cd ~/sprint_hackathon
         python3 src/radar/radar_vnc.py
  5. Click the window so it has keyboard focus.

  Preview on the laptop with fake data (no robot, no smbus):
         python radar/radar_vnc.py --demo

──────────────────────────────────────────────────────────────────────────
CALIBRATING THE ULTRASONIC SENSOR
──────────────────────────────────────────────────────────────────────────
  • Place a flat object (a book) at a known distance, e.g. 50 cm.
  • Run with --demo off; read the "Distance" panel value.
  • If it reads consistently high/low, set SENSOR_OFFSET_CM (added to every
    reading). If it reads noisy, raise SENSOR_SAMPLES (median of N reads).
  • HC-SR04 is unreliable past ~300 cm and on soft/angled surfaces — keep
    MAX_DISTANCE realistic for your room.

──────────────────────────────────────────────────────────────────────────
TUNING DETECTION SENSITIVITY
──────────────────────────────────────────────────────────────────────────
  • OBJECT_BREAK_CM  — depth jump that splits two objects. Lower = splits more
                       eagerly (more, smaller objects). Raise to merge.
  • OBJECT_GAP_DEG   — max angular gap before a surface breaks. Tied to STEP.
  • MOVE_THRESHOLD_CM— range change between sweeps to call an object MOVING.
                       Lower = more sensitive (noise may trigger it).
  • SENSOR_SAMPLES   — reads per angle (median). More = cleaner but slower.
  • DANGER_ZONE / WARNING_ZONE — alarm thresholds.
=============================================================================
"""

import os
import sys
import math
import time
import argparse
import threading
from collections import deque

# ── TUNABLES ──────────────────────────────────────────────────────────────────
MAX_DISTANCE   = 200          # cm — display radius + valid-echo cutoff
MIN_DISTANCE   = 2            # cm — below this = no echo
SWEEP_SPEED    = 2            # degrees of servo travel per scan step
SWEEP_MIN      = 1            # min/max for the +/- keys
SWEEP_MAX      = 8
DANGER_ZONE    = 20           # cm — red alert
WARNING_ZONE   = 80           # cm — amber caution
FPS            = 30
WINDOW_SIZE    = (900, 700)

PAN_CENTER     = 90           # servo angle pointing straight ahead
ARC_DEG        = 180          # total swept arc (centered on PAN_CENTER)
SETTLE_S       = 0.04         # servo settle before reading

# sensor
SENSOR_SAMPLES   = 3          # reads per angle (median filter)
SENSOR_OFFSET_CM = 0.0        # calibration offset added to every reading

# object detection
OBJECT_BREAK_CM  = 25         # depth jump that splits two objects
OBJECT_GAP_DEG   = SWEEP_SPEED * 2.2   # angular gap that breaks a surface
MOVE_THRESHOLD_CM = 5         # range change between sweeps → MOVING
BLIP_PERSIST_S    = 3.5       # how long a blip lingers + fades

# sound
SND_SAMPLE_RATE = 44100
PITCH_FAR_HZ    = 420         # beep pitch at far range
PITCH_NEAR_HZ   = 1400        # beep pitch up close

# ── colors (R, G, B for pygame) ───────────────────────────────────────────────
C_BLACK   = (0, 0, 0)
C_BG      = (2, 8, 2)
C_RING    = (0, 70, 0)
C_RING_HI = (0, 120, 0)
C_GRID    = (0, 45, 0)
C_SWEEP   = (80, 255, 120)
C_SWEEPHI = (200, 255, 210)
C_RED     = (255, 50, 50)
C_RED_DIM = (150, 30, 30)
C_ORANGE  = (255, 150, 30)
C_TEXT    = (90, 220, 110)
C_TEXTHI  = (200, 255, 210)
C_PANEL   = (4, 14, 4)
C_PANELLN = (0, 80, 0)
C_DANGER  = (255, 40, 40)
C_WARNCOL = (255, 200, 40)

# very dark background tint zones
Z_DANGER  = (28, 0, 0)
Z_WARNING = (28, 24, 0)
Z_SAFE    = (0, 20, 0)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ═══════════════════════════════════════════════════════════════════════════════
#  ULTRASONIC SENSOR
# ═══════════════════════════════════════════════════════════════════════════════

class UltrasonicSensor:
    """Reads distance via the RasBot ultrasonic sensor (median of N samples).

    In --demo mode it synthesizes a room so the UI works with no hardware.
    """

    def __init__(self, demo=False):
        self.demo = demo
        self._bot = None

    def attach(self, bot):
        """Share the already-open RasBot instance from the servo controller."""
        self._bot = bot

    def read(self, bearing_deg=0.0):
        """Return a median-filtered distance in cm (0 = no echo)."""
        if self.demo:
            return self._demo_distance(bearing_deg)

        samples = []
        for _ in range(SENSOR_SAMPLES):
            try:
                d = self._bot.read_distance()
            except Exception:
                d = 0.0
            if MIN_DISTANCE <= d <= MAX_DISTANCE:
                samples.append(d + SENSOR_OFFSET_CM)
        if not samples:
            return 0.0
        samples.sort()
        return samples[len(samples) // 2]

    def _demo_distance(self, b):
        """Fake room: flat wall ahead + angled wall on the right + a 'person'."""
        import random
        wall = 150.0 / max(math.cos(math.radians(b)), 0.4)
        if b > 20:
            wall = min(wall, 95.0 / max(math.cos(math.radians(b - 45)), 0.3))
        # a slow-moving "person" blip around a wandering bearing
        person_b = 30 * math.sin(time.time() * 0.4)
        if abs(b - person_b) < 4:
            wall = min(wall, 60 + 20 * math.sin(time.time() * 0.4))
        d = wall + random.gauss(0, 2.5)
        if random.random() < 0.04:                 # occasional spike
            d = random.uniform(MIN_DISTANCE, MAX_DISTANCE)
        if abs(b) > 78:                             # open edges
            return 0.0
        return _clamp(d, 0, MAX_DISTANCE)


# ═══════════════════════════════════════════════════════════════════════════════
#  SERVO CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class ServoController:
    """Sweeps the pan servo smoothly back and forth across the arc.

    Owns the RasBot instance (the sensor borrows it). In demo mode no
    hardware is touched.
    """

    def __init__(self, demo=False):
        self.demo = demo
        self.bot = None
        self._Color = None
        half = ARC_DEG / 2.0
        self.pan_lo = int(_clamp(PAN_CENTER - half, 0, 180))
        self.pan_hi = int(_clamp(PAN_CENTER + half, 0, 180))
        self.pan = PAN_CENTER
        self.direction = 1

    def open(self):
        if self.demo:
            self.pan = self.pan_lo
            return
        from setup_and_api.api import RasBot, Color
        self.bot = RasBot()
        self.bot.__enter__()
        self._Color = Color
        self.bot.set_pan(self.pan_lo)
        self.pan = self.pan_lo
        time.sleep(0.3)

    def close(self):
        if self.demo or self.bot is None:
            return
        try:
            self.bot.look_center()
        except Exception:
            pass
        try:
            self.bot.__exit__(None, None, None)
        except Exception:
            pass

    def step(self, speed):
        """Advance one sweep step. Returns (pan, bearing, reversed?)."""
        reversed_dir = False
        self.pan += self.direction * speed
        if self.pan >= self.pan_hi:
            self.pan, self.direction = self.pan_hi, -1
            reversed_dir = True
        elif self.pan <= self.pan_lo:
            self.pan, self.direction = self.pan_lo, 1
            reversed_dir = True
        if not self.demo:
            try:
                self.bot.set_pan(int(self.pan))
            except Exception:
                pass
        return self.pan, self.pan - PAN_CENTER, reversed_dir

    def set_leds(self, distance):
        if self.demo or self.bot is None:
            return
        try:
            C = self._Color
            if not (MIN_DISTANCE <= distance <= MAX_DISTANCE):
                self.bot.set_all_leds_color(C.GREEN)
            elif distance < DANGER_ZONE:
                self.bot.set_all_leds_color(C.RED)
            elif distance < WARNING_ZONE:
                self.bot.set_all_leds_color(C.YELLOW)
            else:
                self.bot.set_all_leds_color(C.GREEN)
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
#  OBJECT TRACKER
# ═══════════════════════════════════════════════════════════════════════════════

class ObjectTracker:
    """Records returns, clusters them into objects, classifies size + movement.

    Thread-safe: the scan thread calls record()/finish_sweep(); the UI thread
    calls snapshot().
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._blips = {}                  # bearing -> (dist, ts)  (live, fading)
        self._sweeps = deque(maxlen=3)    # last 3 completed sweeps' object lists
        self._cur = {}                    # bearing -> dist for the in-progress sweep
        self._objects = []                # most recent classified objects

    def record(self, bearing, dist):
        now = time.time()
        with self._lock:
            if MIN_DISTANCE <= dist <= MAX_DISTANCE:
                self._blips[bearing] = (dist, now)
                self._cur[bearing] = dist
            else:
                self._blips.pop(bearing, None)
                self._cur.pop(bearing, None)

    def finish_sweep(self):
        """End of one pass: cluster, classify size, compare to last sweep."""
        with self._lock:
            cur = dict(self._cur)
            self._cur.clear()
            prev_objs = self._sweeps[-1] if self._sweeps else []

        objs = self._cluster(cur)
        for o in objs:
            o["moving"], o["from"] = self._match_movement(o, prev_objs)

        with self._lock:
            self._objects = objs
            self._sweeps.append(objs)

    def _cluster(self, readings):
        """Group consecutive bearings into objects with size classification."""
        items = sorted(readings.items())          # [(bearing, dist), ...]
        objs, cur = [], []
        for bearing, dist in items:
            if cur:
                pb, pd = cur[-1]
                if (bearing - pb) > OBJECT_GAP_DEG or abs(dist - pd) > OBJECT_BREAK_CM:
                    objs.append(self._make_object(cur)); cur = []
            cur.append((bearing, dist))
        if cur:
            objs.append(self._make_object(cur))
        objs.sort(key=lambda o: o["range"])
        return objs

    def _make_object(self, pts):
        bearings = [b for b, _ in pts]
        dists    = [d for _, d in pts]
        rng      = min(dists)
        mean_r   = sum(dists) / len(dists)
        span     = (max(bearings) - min(bearings)) if len(bearings) > 1 else 0
        bearing  = sum(bearings) / len(bearings)
        width_cm = 2.0 * mean_r * math.sin(math.radians(span / 2.0)) if span else 0.0
        if span <= 5:
            size = "SMALL"
        elif span <= 15:
            size = "MEDIUM"
        else:
            size = "LARGE"
        return {"bearing": bearing, "range": rng, "span": span,
                "width_cm": width_cm, "size": size, "moving": False, "from": None}

    def _match_movement(self, obj, prev_objs):
        """Find the same object in the previous sweep; flag movement."""
        best, best_db = None, 12.0       # match within 12° of bearing
        for p in prev_objs:
            db = abs(p["bearing"] - obj["bearing"])
            if db < best_db:
                best, best_db = p, db
        if best is None:
            return False, None
        if abs(best["range"] - obj["range"]) > MOVE_THRESHOLD_CM:
            return True, (best["bearing"], best["range"])
        return False, None

    def snapshot(self):
        now = time.time()
        with self._lock:
            blips = {b: (d, t) for b, (d, t) in self._blips.items()
                     if now - t <= BLIP_PERSIST_S}
            self._blips = blips
            return dict(blips), list(self._objects)

    def clear(self):
        with self._lock:
            self._blips.clear()
            self._cur.clear()
            self._sweeps.clear()
            self._objects = []


# ═══════════════════════════════════════════════════════════════════════════════
#  SOUND ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

class SoundEngine:
    """Distance-driven beeps generated programmatically (no audio files).

    Beep rate and pitch scale with proximity; <10 cm = continuous alarm;
    moving objects get a distinctive double-beep. Runs in its own thread.
    """

    def __init__(self):
        self.muted = False
        self.enabled = True
        self._distance = None
        self._moving = False
        self._lock = threading.Lock()
        self._running = True
        self._cache = {}
        self._ok = False
        self._np = None

    def start(self):
        try:
            import numpy as np
            import pygame
            self._np = np
            pygame.mixer.pre_init(SND_SAMPLE_RATE, -16, 1, 512)
            pygame.mixer.init()
            self._ok = True
        except Exception as e:
            print(f"[sound] disabled ({e})")
            self._ok = False
        threading.Thread(target=self._loop, daemon=True).start()

    def update(self, distance, moving):
        with self._lock:
            self._distance = distance
            self._moving = moving

    def toggle_mute(self):
        self.muted = not self.muted
        return self.muted

    def stop(self):
        self._running = False

    def _tone(self, freq, ms, volume=0.5):
        import pygame
        key = (int(freq), int(ms))
        if key in self._cache:
            return self._cache[key]
        np = self._np
        n = int(SND_SAMPLE_RATE * ms / 1000.0)
        t = np.linspace(0, ms / 1000.0, n, False)
        wave = np.sin(2 * np.pi * freq * t)
        env = np.ones(n)
        fade = max(1, int(SND_SAMPLE_RATE * 0.005))
        env[:fade] = np.linspace(0, 1, fade)
        env[-fade:] = np.linspace(1, 0, fade)
        audio = (wave * env * volume * 32767).astype(np.int16)
        snd = pygame.sndarray.make_sound(audio)
        self._cache[key] = snd
        return snd

    @staticmethod
    def _pitch(d):
        d = _clamp(d, 5, MAX_DISTANCE)
        frac = (MAX_DISTANCE - d) / (MAX_DISTANCE - 5)
        return PITCH_FAR_HZ + frac * (PITCH_NEAR_HZ - PITCH_FAR_HZ)

    @staticmethod
    def _interval(d):
        """Seconds between beeps. 0 = continuous, None = silent."""
        if d < 10:   return 0.0
        if d < DANGER_ZONE:  return 0.2     # ~5/s
        if d < 50:   return 0.5             # 2/s
        if d < 100:  return 2.0             # 1 per 2 s
        return None

    def _loop(self):
        while self._running:
            if not self._ok:
                time.sleep(0.2); continue
            with self._lock:
                d = self._distance
                moving = self._moving
            if self.muted or not self.enabled or d is None or d <= 0 or d >= 100:
                time.sleep(0.08); continue

            freq = self._pitch(d)
            dur  = 0.4 if d < 10 else 0.16
            try:
                self._tone(freq, int(dur * 1000)).play()
                if moving and d >= 10:                  # distinctive double-beep
                    time.sleep(0.10)
                    self._tone(freq * 1.35, 90).play()
            except Exception:
                pass

            iv = self._interval(d)
            if iv is None:
                time.sleep(0.1)
            elif iv == 0.0:
                time.sleep(dur)            # continuous: replay immediately
            else:
                time.sleep(iv)


# ═══════════════════════════════════════════════════════════════════════════════
#  RADAR UI
# ═══════════════════════════════════════════════════════════════════════════════

class RadarUI:
    """Draws the entire radar scene with pygame (double-buffered, 30 FPS)."""

    def __init__(self, tracker):
        import pygame
        self.pg = pygame
        self.tracker = tracker
        self.w, self.h = WINDOW_SIZE
        self.panel_w = 230
        self.radar_w = self.w - self.panel_w
        self.cx = self.radar_w // 2
        self.cy = self.h - 40
        self.radius = min(self.radar_w // 2 - 10, self.cy - 20)
        self.ppc = self.radius / MAX_DISTANCE

        self.screen = pygame.display.set_mode(WINDOW_SIZE, pygame.DOUBLEBUF)
        pygame.display.set_caption("RASPBOT RADAR v1.0")
        self.font_s = pygame.font.SysFont("consolas", 15)
        self.font_m = pygame.font.SysFont("consolas", 19, bold=True)
        self.font_l = pygame.font.SysFont("consolas", 26, bold=True)
        self.font_xl = pygame.font.SysFont("consolas", 52, bold=True)

        # static background (zones + grid) pre-rendered once
        self._bg = self._build_background()

    # ── geometry ──────────────────────────────────────────────────────────────

    def polar(self, bearing, dist_cm):
        b = math.radians(bearing)
        r = dist_cm * self.ppc
        return int(self.cx + r * math.sin(b)), int(self.cy - r * math.cos(b))

    # ── pre-rendered background ─────────────────────────────────────────────────

    def _build_background(self):
        pg = self.pg
        surf = pg.Surface((self.radar_w, self.h))
        surf.fill(C_BG)

        # zone tints (drawn large→small so inner zones overwrite outer)
        for radius_cm, col in ((MAX_DISTANCE, Z_SAFE),
                               (WARNING_ZONE, Z_WARNING),
                               (DANGER_ZONE, Z_DANGER)):
            r = int(radius_cm * self.ppc)
            pg.draw.circle(surf, col, (self.cx, self.cy), r)
        # repaint bottom (below origin) black so it's a clean semicircle
        pg.draw.rect(surf, C_BLACK, (0, self.cy, self.radar_w, self.h - self.cy))

        # range rings + labels
        for cm in (20, 50, 100, 200):
            if cm > MAX_DISTANCE:
                continue
            r = int(cm * self.ppc)
            col = C_RING_HI if cm == 200 else C_RING
            pg.draw.circle(surf, col, (self.cx, self.cy), r, 1)
            lbl = self.font_s.render(f"{cm}cm", True, C_RING_HI)
            surf.blit(lbl, (self.cx + 4, self.cy - r - 16))

        # bearing spokes every 30°
        b = -90
        while b <= 90:
            x = int(self.cx + self.radius * math.sin(math.radians(b)))
            y = int(self.cy - self.radius * math.cos(math.radians(b)))
            pg.draw.line(surf, C_GRID, (self.cx, self.cy), (x, y), 1)
            lbl = self.font_s.render(f"{b:+d}", True, C_GRID)
            surf.blit(lbl, (x - 12, y - 6 if b > -90 else y + 6))
            b += 30
        return surf

    # ── per-frame draw ──────────────────────────────────────────────────────────

    def draw(self, state, sweep_bearing, trail, distance, angle,
             blips, objects, nearest, muted, speed, sweep_on):
        pg = self.pg
        self.screen.fill(C_BLACK)
        self.screen.blit(self._bg, (0, 0))

        self._draw_trail(trail)
        self._draw_sweep(sweep_bearing)
        self._draw_blips(blips)
        self._draw_objects(objects)
        self._draw_nearest(nearest)
        self._draw_panel(state, distance, angle, objects, nearest, muted,
                         speed, sweep_on)
        self._draw_border(state)
        pg.display.flip()

    def _draw_trail(self, trail):
        pg = self.pg
        now = time.time()
        for t, b in trail:
            age = now - t
            if age > 1.2:
                continue
            a = 1.0 - age / 1.2
            x = int(self.cx + self.radius * math.sin(math.radians(b)))
            y = int(self.cy - self.radius * math.cos(math.radians(b)))
            col = (int(C_SWEEP[0] * a * 0.5),
                   int(C_SWEEP[1] * a * 0.5),
                   int(C_SWEEP[2] * a * 0.5))
            pg.draw.line(self.screen, col, (self.cx, self.cy), (x, y), 1)

    def _draw_sweep(self, bearing):
        pg = self.pg
        x = int(self.cx + self.radius * math.sin(math.radians(bearing)))
        y = int(self.cy - self.radius * math.cos(math.radians(bearing)))
        pg.draw.line(self.screen, (0, 90, 30), (self.cx, self.cy), (x, y), 5)
        pg.draw.line(self.screen, C_SWEEP, (self.cx, self.cy), (x, y), 2)
        pg.draw.circle(self.screen, C_SWEEPHI, (x, y), 4)

    def _blip_color_size(self, dist):
        if dist < DANGER_ZONE:
            pulse = 4 + int(3 * abs(math.sin(time.time() * 6)))
            return C_RED, 8 + pulse
        if dist < WARNING_ZONE:
            return C_RED, 6
        return C_RED_DIM, 4

    def _draw_blips(self, blips):
        pg = self.pg
        now = time.time()
        for bearing, (dist, ts) in blips.items():
            a = max(0.2, 1.0 - (now - ts) / BLIP_PERSIST_S)
            x, y = self.polar(bearing, dist)
            col, size = self._blip_color_size(dist)
            col = (int(col[0] * a), int(col[1] * a), int(col[2] * a))
            # glow
            pg.draw.circle(self.screen, (int(col[0] * 0.4),
                                         int(col[1] * 0.4),
                                         int(col[2] * 0.4)), (x, y), size + 3)
            pg.draw.circle(self.screen, col, (x, y), size)

    def _draw_objects(self, objects):
        pg = self.pg
        for o in objects:
            x, y = self.polar(o["bearing"], o["range"])
            moving = o["moving"]
            col = C_ORANGE if moving else C_RED
            # size label
            tag = f"{o['size'][0]}"            # S / M / L
            if moving:
                tag += " MOV"
            lbl = self.font_s.render(tag, True, col)
            self.screen.blit(lbl, (x + 8, y - 8))
            # movement arrow from previous position
            if moving and o["from"] is not None:
                fb, fr = o["from"]
                fx, fy = self.polar(fb, fr)
                pg.draw.line(self.screen, C_ORANGE, (fx, fy), (x, y), 2)
                self._arrow_head(fx, fy, x, y)

    def _arrow_head(self, x0, y0, x1, y1):
        pg = self.pg
        ang = math.atan2(y1 - y0, x1 - x0)
        for da in (math.radians(150), math.radians(-150)):
            hx = x1 + 9 * math.cos(ang + da)
            hy = y1 + 9 * math.sin(ang + da)
            pg.draw.line(self.screen, C_ORANGE, (x1, y1), (int(hx), int(hy)), 2)

    def _draw_nearest(self, nearest):
        if nearest is None:
            return
        pg = self.pg
        x, y = self.polar(nearest["bearing"], nearest["range"])
        col = C_DANGER if nearest["range"] < DANGER_ZONE else (255, 120, 120)
        pg.draw.circle(self.screen, col, (x, y), 16, 1)
        pg.draw.line(self.screen, col, (x - 20, y), (x - 8, y), 1)
        pg.draw.line(self.screen, col, (x + 8, y), (x + 20, y), 1)
        pg.draw.line(self.screen, col, (x, y - 20), (x, y - 8), 1)
        pg.draw.line(self.screen, col, (x, y + 8), (x, y + 20), 1)

    # ── side panel ──────────────────────────────────────────────────────────────

    def _draw_panel(self, state, distance, angle, objects, nearest, muted,
                    speed, sweep_on):
        pg = self.pg
        px = self.radar_w
        pg.draw.rect(self.screen, C_PANEL, (px, 0, self.panel_w, self.h))
        pg.draw.line(self.screen, C_PANELLN, (px, 0), (px, self.h), 2)

        x = px + 14
        y = 18
        self.screen.blit(self.font_m.render("RASPBOT RADAR v1.0", True, C_TEXTHI),
                         (x, y)); y += 30
        pg.draw.line(self.screen, C_PANELLN, (x, y), (px + self.panel_w - 14, y), 1)
        y += 16

        dist_txt = f"{distance:.0f} cm" if distance and distance > 0 else "-- cm"
        rows = [
            ("Distance:", dist_txt),
            ("Angle:", f"{angle:+.0f}°"),
            ("Object:", "YES" if nearest else "NO"),
            ("Status:", state),
            ("Objects:", str(len(objects))),
        ]
        for label, val in rows:
            self.screen.blit(self.font_s.render(label, True, C_TEXT), (x, y))
            vc = C_TEXTHI
            if label == "Status:":
                vc = (C_DANGER if state == "DANGER"
                      else C_ORANGE if state == "MOVING"
                      else C_WARNCOL if state == "PAUSED" else C_TEXTHI)
            self.screen.blit(self.font_s.render(str(val), True, vc), (x + 95, y))
            y += 24

        y += 8
        pg.draw.line(self.screen, C_PANELLN, (x, y), (px + self.panel_w - 14, y), 1)
        y += 14

        # large distance readout when in danger
        if state == "DANGER" and distance:
            self.screen.blit(self.font_xl.render(f"{distance:.0f}", True, C_DANGER),
                             (x, y))
            self.screen.blit(self.font_s.render("cm", True, C_DANGER), (x + 96, y + 30))
        y += 64

        # sweep progress bar
        frac = (angle + 90) / 180.0
        frac = _clamp(frac, 0, 1)
        bar_w = self.panel_w - 28
        pg.draw.rect(self.screen, C_PANELLN, (x, y, bar_w, 16), 1)
        pg.draw.rect(self.screen, C_SWEEP, (x + 1, y + 1,
                                            int((bar_w - 2) * frac), 14))
        self.screen.blit(self.font_s.render(f"SWEEP {int(frac*100):3d}%", True, C_TEXT),
                         (x, y + 20))
        y += 50

        # controls + state
        pg.draw.line(self.screen, C_PANELLN, (x, y), (px + self.panel_w - 14, y), 1)
        y += 12
        ctrl = [
            f"[S] Sweep  : {'ON' if sweep_on else 'OFF'}",
            f"[M] Sound  : {'MUTED' if muted else 'ON'}",
            f"[+/-] Speed: {speed}",
            "[R] Reset dots",
            "[Q] Quit",
        ]
        for c in ctrl:
            self.screen.blit(self.font_s.render(c, True, C_TEXT), (x, y))
            y += 22

    def _draw_border(self, state):
        pg = self.pg
        if state == "DANGER":
            if int(time.time() * 4) % 2 == 0:
                pg.draw.rect(self.screen, C_DANGER, (0, 0, self.w, self.h), 6)
        elif state == "MOVING":
            pg.draw.rect(self.screen, C_ORANGE, (0, 0, self.w, self.h), 3)


# ═══════════════════════════════════════════════════════════════════════════════
#  RADAR SYSTEM  (main controller)
# ═══════════════════════════════════════════════════════════════════════════════

class RadarSystem:
    def __init__(self, demo=False):
        self.demo    = demo
        self.servo   = ServoController(demo=demo)
        self.sensor  = UltrasonicSensor(demo=demo)
        self.tracker = ObjectTracker()
        self.sound   = SoundEngine()

        self.speed    = SWEEP_SPEED
        self.sweep_on = True
        self.running  = True

        # shared scan state (written by scan thread, read by UI)
        self._lock = threading.Lock()
        self._bearing = 0.0
        self._distance = 0.0
        self._trail = deque(maxlen=90)

    # ── scan thread ─────────────────────────────────────────────────────────────

    def _scan_loop(self):
        self.servo.open()
        self.sensor.attach(self.servo.bot)
        while self.running:
            if not self.sweep_on:
                time.sleep(0.05)
                continue
            pan, bearing, reversed_dir = self.servo.step(self.speed)
            time.sleep(SETTLE_S)
            dist = self.sensor.read(bearing)

            self.tracker.record(bearing, dist)
            self.servo.set_leds(dist)
            if reversed_dir:
                self.tracker.finish_sweep()

            with self._lock:
                self._bearing = bearing
                self._distance = dist
                self._trail.append((time.time(), bearing))
        self.servo.close()

    # ── state machine ───────────────────────────────────────────────────────────

    def _state(self, distance, objects, nearest):
        if not self.sweep_on:
            return "PAUSED"
        if nearest and nearest["range"] < DANGER_ZONE:
            return "DANGER"
        if any(o["moving"] for o in objects):
            return "MOVING"
        if not objects:
            return "CLEAR"
        return "SCANNING"

    # ── main loop (UI) ──────────────────────────────────────────────────────────

    def run(self):
        import pygame
        pygame.init()
        ui = RadarUI(self.tracker)
        self.sound.start()

        threading.Thread(target=self._scan_loop, daemon=True).start()
        clock = pygame.time.Clock()

        while self.running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False
                elif ev.type == pygame.KEYDOWN:
                    self._on_key(ev.key, pygame)

            with self._lock:
                bearing = self._bearing
                distance = self._distance
                trail = list(self._trail)

            blips, objects = self.tracker.snapshot()
            nearest = min(objects, key=lambda o: o["range"]) if objects else None
            state = self._state(distance, objects, nearest)

            # feed the sound engine
            snd_d = nearest["range"] if nearest else (distance if distance > 0 else None)
            snd_moving = any(o["moving"] for o in objects)
            self.sound.update(snd_d, snd_moving)

            ui.draw(state, bearing, trail, distance, bearing, blips, objects,
                    nearest, self.sound.muted, self.speed, self.sweep_on)
            clock.tick(FPS)

        self.sound.stop()
        time.sleep(0.1)
        pygame.quit()

    def _on_key(self, key, pygame):
        if key in (pygame.K_q, pygame.K_ESCAPE):
            self.running = False
        elif key == pygame.K_s:
            self.sweep_on = not self.sweep_on
        elif key == pygame.K_m:
            self.sound.toggle_mute()
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.speed = min(SWEEP_MAX, self.speed + 1)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.speed = max(SWEEP_MIN, self.speed - 1)
        elif key == pygame.K_r:
            self.tracker.clear()
            with self._lock:
                self._trail.clear()


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="RASPBOT pygame radar (VNC).")
    ap.add_argument("--demo", action="store_true",
                    help="fake data, no hardware (preview on the laptop)")
    args = ap.parse_args()

    try:
        RadarSystem(demo=args.demo).run()
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
