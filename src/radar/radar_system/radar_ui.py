"""
radar_ui.py — STEP 4: full military radar display with glow + fonts.

Reads a shared data_bus (filled by main.py's scan thread) and renders a
neon PPI radar: glowing range rings, 5-layer glow sweep arm with quadratic
fade trail, glowing object dots (pulsing in danger, orange + arrow when
moving, ghost trail), a side info panel with bloom text, DANGER / MOVEMENT
overlays, and a CRT scanline overlay. Quality auto-scales via VNCOptimizer.

    ui = RadarUI(data_bus, data_lock, optimizer, fonts, sweep_sync, on_key)
    ui.run()
"""

import os
import sys
import math
import time
from collections import deque

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from constants import (
    WINDOW_SIZE, FPS_TARGET, RADAR_RADIUS, MAX_DISTANCE, DANGER_ZONE,
    WARNING_ZONE, NEON_GREEN, DANGER_RED, WARNING_YEL, SAFE_GREEN, ORANGE,
    BLACK, DIM_GREEN, TEXT_GREEN, TEXT_HI, PANEL_BG, PANEL_LINE, FONT_SIZES,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── compact glow helper (self-contained for this package) ─────────────────────

class _Glow:
    def __init__(self, pygame, w, h, full_w):
        self.pg = pygame
        self.layer = pygame.Surface((w, h), pygame.SRCALPHA)
        self.w, self.h = w, h
        self.scan = self._scanlines(pygame, full_w, h)
        self._tcache = {}

    def begin(self):
        self.layer.fill((0, 0, 0, 0))

    def commit(self, screen):
        screen.blit(self.layer, (0, 0))

    def clear_below(self, y):
        self.layer.fill((0, 0, 0, 0), rect=(0, y, self.w, self.h - y))

    def line(self, a, b, color, layers, fade=1.0):
        for width, alpha in layers:
            al = int(alpha * fade)
            if al > 0:
                self.pg.draw.line(self.layer, (*color, min(255, al)), a, b, max(1, width))

    def circle(self, c, base_r, color, layers, fade=1.0):
        for extra, alpha in layers:
            al = int(alpha * fade)
            r = int(base_r + extra)
            if al > 0 and r > 0:
                self.pg.draw.circle(self.layer, (*color, min(255, al)), c, r)

    def ring(self, c, r, color, layers, target):
        for width, alpha in layers:
            if alpha > 0:
                self.pg.draw.circle(target, (*color, min(255, alpha)), c, r, max(1, width))

    def zone(self, c, r, rgba):
        self.pg.draw.circle(self.layer, rgba, c, int(r))

    def _scanlines(self, pygame, w, h, gap=3, alpha=25):
        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        try:
            import numpy as np
            a = np.zeros((w, h), dtype=np.uint8)
            a[:, ::gap] = alpha
            px = pygame.surfarray.pixels_alpha(surf)
            px[:, :] = a
            del px
        except Exception:
            for y in range(0, h, gap):
                pygame.draw.line(surf, (0, 0, 0, alpha), (0, y), (w, y))
        return surf

    def text(self, screen, font, text, color, pos, bloom=True, glow_color=None):
        key = (id(font), text, color, bloom, glow_color)
        ent = self._tcache.get(key)
        if ent is None:
            sharp = font.render(text, True, color)
            glow = None
            if bloom:
                gc = glow_color or NEON_GREEN
                base = font.render(text, True, gc)
                gw = max(1, int(base.get_width() * 1.06) + 2)
                gh = max(1, int(base.get_height() * 1.06) + 2)
                glow = self.pg.transform.smoothscale(base, (gw, gh)).convert_alpha()
                glow.fill((255, 255, 255, 60), special_flags=self.pg.BLEND_RGBA_MULT)
            if len(self._tcache) > 500:
                self._tcache.clear()
            ent = (sharp, glow)
            self._tcache[key] = ent
        sharp, glow = ent
        if glow is not None:
            dx = (glow.get_width() - sharp.get_width()) // 2
            dy = (glow.get_height() - sharp.get_height()) // 2
            screen.blit(glow, (pos[0] - dx, pos[1] - dy))
        screen.blit(sharp, pos)


class RadarUI:
    """Full radar display. Reads data_bus; calls on_key(name) for controls."""

    def __init__(self, data_bus, data_lock, optimizer, fonts, sweep_sync,
                 on_key, sound=None):
        import pygame
        self.pg = pygame
        self.bus = data_bus
        self.lock = data_lock
        self.opt = optimizer
        self.fonts = fonts
        self.sync = sweep_sync
        self.on_key = on_key
        self.sound = sound

        self.w, self.h = optimizer.window_size
        self.panel_w = 320
        self.radar_w = self.w - self.panel_w
        self.cx = self.radar_w // 2
        self.cy = self.h - 60
        self.radius = min(RADAR_RADIUS, self.radar_w // 2 - 20, self.cy - 30)
        self.scale = self.radius / MAX_DISTANCE

        self.screen = optimizer.set_mode()
        pygame.display.set_caption("RASPBOT RADAR v1.0")

        # fonts
        self.f_title = fonts.title(FONT_SIZES["panel_title"])
        self.f_data  = fonts.data(FONT_SIZES["data"])
        self.f_small = fonts.small(FONT_SIZES["small"])
        self.f_alert = fonts.alert(FONT_SIZES["alert"])
        self.f_big   = fonts.alert(FONT_SIZES["big"])
        self.f_med   = fonts.reg(FONT_SIZES["med"])
        self.f_ring  = fonts.small(12)

        self.glow = _Glow(pygame, self.radar_w, self.h, self.w)
        self._border_buf = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        self._vignette = self._build_vignette()
        self._bg = self._build_background()

        self._trail = deque(maxlen=90)
        self.running = True

    # ── geometry ──────────────────────────────────────────────────────────────

    def polar(self, bearing, dist_cm):
        b = math.radians(bearing)
        r = dist_cm * self.scale
        return int(self.cx + r * math.sin(b)), int(self.cy - r * math.cos(b))

    # ── static layers ────────────────────────────────────────────────────────

    def _build_vignette(self):
        pg = self.pg
        v = pg.Surface((self.radar_w, self.h), pg.SRCALPHA)
        try:
            import numpy as np
            yy, xx = np.mgrid[0:self.h, 0:self.radar_w]
            d = np.sqrt((xx - self.cx) ** 2 + (yy - self.cy) ** 2)
            d = np.clip(d / d.max(), 0, 1)
            a = (d ** 2 * 120).astype(np.uint8).T          # darker at edges
            px = pg.surfarray.pixels_alpha(v)
            px[:, :] = a
            del px
        except Exception:
            pass
        return v

    def _build_background(self):
        pg = self.pg
        surf = pg.Surface((self.radar_w, self.h))
        surf.fill(BLACK)

        # filled colour zones (semicircle), large→small
        for cm, col in ((MAX_DISTANCE, (0, 30, 8)),
                        (WARNING_ZONE, (35, 30, 0)),
                        (DANGER_ZONE, (40, 0, 0))):
            pg.draw.circle(surf, col, (self.cx, self.cy), int(cm * self.scale))
        pg.draw.rect(surf, BLACK, (0, self.cy, self.radar_w, self.h - self.cy))

        # glowing range rings on a temp alpha layer
        gl = pg.Surface((self.radar_w, self.h), pg.SRCALPHA)
        for cm in (20, 50, 100, 200):
            if cm > MAX_DISTANCE:
                continue
            self.glow.ring((self.cx, self.cy), int(cm * self.scale),
                           NEON_GREEN, [(3, 20), (2, 60), (1, 150)], target=gl)
        gl.fill((0, 0, 0, 0), rect=(0, self.cy, self.radar_w, self.h - self.cy))
        surf.blit(gl, (0, 0))

        # ring labels
        for cm in (20, 50, 100, 200):
            if cm > MAX_DISTANCE:
                continue
            lbl = self.f_ring.render(f"{cm}cm", True, SAFE_GREEN)
            surf.blit(lbl, (self.cx + 4, self.cy - int(cm * self.scale) - 15))

        # angle spokes every 15° + labels
        b = -90
        while b <= 90:
            x = int(self.cx + self.radius * math.sin(math.radians(b)))
            y = int(self.cy - self.radius * math.cos(math.radians(b)))
            pg.draw.line(surf, DIM_GREEN, (self.cx, self.cy), (x, y), 1)
            if b % 30 == 0:
                lbl = self.f_ring.render(f"{b:+d}", True, DIM_GREEN)
                surf.blit(lbl, (x - 12, y - 4 if b > -90 else y + 6))
            b += 15

        surf.blit(self._vignette, (0, 0))
        return surf

    # ── main loop ──────────────────────────────────────────────────────────────

    def run(self):
        pg = self.pg
        clock = pg.time.Clock()
        while self.running:
            for ev in pg.event.get():
                if ev.type == pg.QUIT:
                    self._quit()
                elif ev.type == pg.KEYDOWN:
                    self._key(ev)

            snap = self._snapshot()
            self._frame(snap)
            pg.display.flip()

            fps = clock.get_fps()
            self.opt.tick(fps if fps > 0 else FPS_TARGET)
            self.opt.bandwidth_estimate(fps if fps > 0 else FPS_TARGET)
            clock.tick(FPS_TARGET)

    def _snapshot(self):
        with self.lock:
            return dict(self.bus)

    def _key(self, ev):
        pg = self.pg
        mapping = {
            pg.K_s: "S", pg.K_m: "M", pg.K_r: "R",
            pg.K_q: "Q", pg.K_ESCAPE: "Q",
            pg.K_PLUS: "+", pg.K_EQUALS: "+", pg.K_KP_PLUS: "+",
            pg.K_MINUS: "-", pg.K_KP_MINUS: "-",
        }
        name = mapping.get(ev.key)
        if name == "Q":
            self._quit()
        elif name:
            self.on_key(name)

    def _quit(self):
        self.running = False

    # ── per-frame compositing ───────────────────────────────────────────────────

    def _frame(self, s):
        screen = self.screen
        screen.fill(BLACK)
        screen.blit(self._bg, (0, 0))

        bearing = self.sync.get_display_angle() if self.sync else s.get("current_angle", 0.0)
        self._trail.append((time.time(), bearing))

        glow_on = self.opt.glow_enabled()
        full = self.opt.full_glow()

        self.glow.begin()
        self._danger_zone(s)
        if glow_on:
            self._trail_glow(full)
        self._sweep(bearing, full)
        self._dots(s, full)
        self._arrows(s)
        self.glow.clear_below(self.cy)
        self.glow.commit(screen)

        self._dot_labels(s)
        self._panel(s)
        self._alerts(s)

        if self.opt.scanlines_enabled():
            screen.blit(self.glow.scan, (0, 0))
        self._border(s)

    def _danger_zone(self, s):
        if s.get("zone") != "DANGER":
            return
        pulse = abs(math.sin(time.time() * 8))
        alpha = int(12 + pulse * 48)
        self.glow.zone((self.cx, self.cy), DANGER_ZONE * self.scale,
                       (255, 0, 0, alpha))

    def _trail_glow(self, full):
        now = time.time()
        n = len(self._trail)
        for i, (t, b) in enumerate(self._trail):
            age = now - t
            if age > 1.5:
                continue
            frac = max(0.0, 1.0 - age / 1.5) ** 2          # quadratic fade
            x = int(self.cx + self.radius * math.sin(math.radians(b)))
            y = int(self.cy - self.radius * math.cos(math.radians(b)))
            layers = [(3, 30), (1, 110)] if full else [(1, 90)]
            self.glow.line((self.cx, self.cy), (x, y), NEON_GREEN, layers, fade=frac)

    def _sweep(self, bearing, full):
        x = int(self.cx + self.radius * math.sin(math.radians(bearing)))
        y = int(self.cy - self.radius * math.cos(math.radians(bearing)))
        if full:
            layers = [(12, 15), (8, 35), (4, 90), (2, 180), (1, 255)]
        else:
            layers = [(4, 60), (1, 220)]
        self.glow.line((self.cx, self.cy), (x, y), NEON_GREEN, layers)
        self.glow.circle((x, y), 3, (200, 255, 210), [(4, 80), (0, 255)])

    def _dots(self, s, full):
        layers = ([(12, 15), (8, 30), (5, 60), (2, 120), (0, 255)] if full
                  else [(5, 50), (0, 255)])
        for o in s.get("objects", []):
            x, y = self.polar(o["bearing"], o["range"])
            moving = o.get("moving")
            d = o["range"]
            if moving:
                col = ORANGE
                base = 6
            elif d < DANGER_ZONE:
                col = DANGER_RED
                base = int(8 + abs(math.sin(time.time() * 5)) * 8)   # pulse
            elif d < WARNING_ZONE:
                col = DANGER_RED
                base = 6
            else:
                col = (150, 30, 30)
                base = 4
            # ghost trail (last positions)
            for gi, (gb, gr) in enumerate(o.get("ghosts", [])[-3:]):
                gx, gy = self.polar(gb, gr)
                gf = 0.4 ** (len(o.get("ghosts", [])[-3:]) - gi)
                self.glow.circle((gx, gy), 3, col, [(2, 80)], fade=gf)
            self.glow.circle((x, y), base, col, layers)

    def _arrows(self, s):
        for o in s.get("objects", []):
            if not (o.get("moving") and o.get("from")):
                continue
            x, y = self.polar(o["bearing"], o["range"])
            fb, fr = o["from"]
            fx, fy = self.polar(fb, fr)
            self.glow.line((fx, fy), (x, y), ORANGE, [(4, 60), (2, 200)])
            ang = math.atan2(y - fy, x - fx)
            for da in (math.radians(150), math.radians(-150)):
                hx = int(x + 12 * math.cos(ang + da))
                hy = int(y + 12 * math.sin(ang + da))
                self.glow.line((x, y), (hx, hy), ORANGE, [(3, 200)])

    def _dot_labels(self, s):
        for o in s.get("objects", []):
            x, y = self.polar(o["bearing"], o["range"])
            col = ORANGE if o.get("moving") else DANGER_RED
            tag = o.get("size", "?")[0] + (" MOV" if o.get("moving") else "")
            self.screen.blit(self.f_med.render(tag, True, col), (x + 10, y - 10))

    # ── panel ──────────────────────────────────────────────────────────────────

    def _panel(self, s):
        pg = self.pg
        g = self.glow
        px = self.radar_w
        pg.draw.rect(self.screen, PANEL_BG, (px, 0, self.panel_w, self.h))
        # glowing panel border
        pg.draw.rect(self.screen, PANEL_LINE, (px + 2, 2, self.panel_w - 4, self.h - 4), 2)

        x = px + 18
        y = 18
        g.text(self.screen, self.f_title, "RASPBOT RADAR", TEXT_HI, (x, y),
               glow_color=NEON_GREEN)
        y += 30
        g.text(self.screen, self.f_small, "v1.0  PPI SONAR ARRAY", TEXT_GREEN,
               (x, y), bloom=False)
        y += 22
        pg.draw.line(self.screen, PANEL_LINE, (x, y), (px + self.panel_w - 18, y), 1)
        y += 14

        dist = s.get("filtered_distance", 0.0)
        zone = s.get("zone", "CLEAR")
        zcol = {"DANGER": DANGER_RED, "WARNING": WARNING_YEL,
                "SAFE": SAFE_GREEN, "CLEAR": TEXT_HI}.get(zone, TEXT_HI)
        rows = [
            ("DISTANCE", f"{dist:05.1f} cm" if dist else "---.- cm", TEXT_HI),
            ("ANGLE", f"{s.get('current_angle', 0):+04.0f} deg", TEXT_HI),
            ("ZONE", zone, zcol),
            ("OBJECTS", f"{len(s.get('objects', [])):03d}", TEXT_HI),
            ("MOVING", f"{len(s.get('moving_objects', [])):03d}", ORANGE),
        ]
        for label, val, vc in rows:
            g.text(self.screen, self.f_small, label, TEXT_GREEN, (x, y),
                   glow_color=NEON_GREEN)
            self.screen.blit(self.f_data.render(val, True, vc), (x + 120, y - 2))
            y += 28

        y += 6
        pg.draw.line(self.screen, PANEL_LINE, (x, y), (px + self.panel_w - 18, y), 1)
        y += 14

        state = s.get("state", "SCANNING")
        g.text(self.screen, self.f_med, "STATUS", TEXT_GREEN, (x, y))
        y += 22
        scol = {"DANGER": DANGER_RED, "MOVING": ORANGE,
                "PAUSED": WARNING_YEL, "CLEAR": SAFE_GREEN}.get(state, TEXT_HI)
        self.screen.blit(self.f_data.render(f"* {state}", True, scol), (x, y))
        y += 30
        pg.draw.line(self.screen, PANEL_LINE, (x, y), (px + self.panel_w - 18, y), 1)
        y += 14

        # sweep progress bar
        frac = _clamp((s.get("current_angle", 0) + 90) / 180.0, 0, 1)
        bar_w = self.panel_w - 36
        pg.draw.rect(self.screen, PANEL_LINE, (x, y, bar_w, 14), 1)
        pg.draw.rect(self.screen, NEON_GREEN, (x + 1, y + 1, int((bar_w - 2) * frac), 12))
        y += 24

        muted = s.get("muted", False)
        ctrls = [
            f"[S] START/STOP : {'RUN' if s.get('sweep_on', True) else 'STOP'}",
            f"[M] MUTE       : {'MUTED' if muted else 'ON'}",
            f"[+] FASTER     : {s.get('speed', 2)}",
            "[-] SLOWER",
            "[R] RESET",
            "[Q] QUIT",
        ]
        for c in ctrls:
            g.text(self.screen, self.f_small, c, TEXT_GREEN, (x, y),
                   glow_color=NEON_GREEN)
            y += 22

        q = getattr(self.opt, "quality", "HIGH")
        self.screen.blit(self.f_small.render(f"QUALITY: {q}", True, DIM_GREEN),
                         (x, self.h - 26))

    # ── alert overlays ──────────────────────────────────────────────────────────

    def _alerts(self, s):
        state = s.get("state")
        if state == "DANGER":
            d = s.get("filtered_distance", 0)
            cx = self.radar_w // 2
            self.glow.text(self.screen, self.f_alert, "!! DANGER !!", DANGER_RED,
                           (cx - 150, int(self.h * 0.18)), glow_color=(255, 60, 60))
            self.glow.text(self.screen, self.f_big, f"OBJECT AT {d:.0f} cm",
                           DANGER_RED, (cx - 130, int(self.h * 0.18) + 52),
                           glow_color=(255, 60, 60))
        elif s.get("moving_objects"):
            if int(time.time() * 2) % 2 == 0:                  # 2 Hz flash
                cx = self.radar_w // 2
                self.glow.text(self.screen, self.f_med, "MOVEMENT DETECTED",
                               ORANGE, (cx - 110, 24), glow_color=(255, 160, 40))

    def _border(self, s):
        pg = self.pg
        state = s.get("state")
        if state == "DANGER":
            alpha = int(30 + abs(math.sin(time.time() * 8)) * 25 + 30)
            self._border_buf.fill((0, 0, 0, 0))
            pg.draw.rect(self._border_buf, (255, 0, 0, alpha), (0, 0, self.w, self.h), 8)
            self.screen.blit(self._border_buf, (0, 0))
        elif state == "MOVING":
            self._border_buf.fill((0, 0, 0, 0))
            pg.draw.rect(self._border_buf, (255, 140, 0, 110), (0, 0, self.w, self.h), 4)
            self.screen.blit(self._border_buf, (0, 0))
