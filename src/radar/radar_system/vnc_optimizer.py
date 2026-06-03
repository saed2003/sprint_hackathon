"""
vnc_optimizer.py — STEP 3: VNC performance optimization + FontManager.

Maximizes radar smoothness over a VNC link:
  • auto-detects a VNC/remote display and sets SDL env BEFORE pygame.init
  • benchmarks resolutions and picks the best one that holds ≥ 25 FPS
  • 16-bit colour mode (≈40% less VNC data)
  • adaptive quality (HIGH / MEDIUM / LOW) driven by live FPS
  • bandwidth estimate with auto-drop to LOW
  • writes an optimized vnc_settings.sh

    from vnc_optimizer import VNCOptimizer, FontManager
    VNCOptimizer.apply_sdl_env()        # call FIRST, before pygame.init()
    opt = VNCOptimizer()
    size = opt.choose_resolution()      # after pygame.init()
    ...
    opt.tick(fps)                       # each frame → adjusts quality level
"""

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from constants import WINDOW_SIZE, FONT_SIZES, FONT_LINKS, VNC_SETTINGS_SH

QUALITY_LEVELS = ["LOW", "MEDIUM", "HIGH"]


class VNCOptimizer:
    """Detects the display, benchmarks it, and manages adaptive quality."""

    def __init__(self, debug=True):
        self.debug = debug
        self.mode = self.detect_display()
        self.quality = "HIGH"
        self.window_size = WINDOW_SIZE
        self.color_depth = 16              # 16-bit for VNC
        self._fps_hist = []
        self._last_switch = 0.0
        self._bw_warned = False

    # ── detection ──────────────────────────────────────────────────────────────

    @staticmethod
    def detect_display():
        disp = os.environ.get("DISPLAY", "")
        is_vnc = False
        for k in ("VNCDESKTOP", "VNC_PORT", "RFB_PORT"):
            if os.environ.get(k):
                is_vnc = True
        # DISPLAY like :1 / :2 is typically a VNC virtual display
        if disp and disp not in (":0", ":0.0"):
            is_vnc = True
        mode = "VNC" if is_vnc else ("LOCAL" if disp else "HEADLESS")
        print(f"[vnc] display detected: {mode}  (DISPLAY='{disp or 'unset'}')")
        return mode

    @staticmethod
    def apply_sdl_env():
        """Set SDL env vars. MUST be called before pygame.init()."""
        os.environ.setdefault("SDL_VIDEODRIVER", "x11")
        os.environ.setdefault("SDL_AUDIODRIVER", "alsa")
        print("[vnc] SDL_VIDEODRIVER=x11  SDL_AUDIODRIVER=alsa")

    # ── resolution benchmark ─────────────────────────────────────────────────────

    def choose_resolution(self, candidates=((1280, 720), (1024, 600), (800, 600))):
        """Benchmark each candidate; pick the largest holding ≥ 25 FPS."""
        try:
            import pygame
        except Exception:
            self.window_size = candidates[-1]
            return self.window_size
        best = candidates[-1]
        for size in candidates:
            fps = self._bench(pygame, size)
            print(f"[vnc] {size[0]}x{size[1]} → {fps:.0f} FPS")
            if fps >= 25:
                best = size
                break
        self.window_size = best
        print(f"[vnc] optimal resolution: {best[0]}x{best[1]}")
        return best

    def _bench(self, pygame, size, frames=20):
        """Quick render benchmark at a resolution (off-screen-ish)."""
        try:
            screen = pygame.display.set_mode(size, 0, self.color_depth)
        except Exception:
            try:
                screen = pygame.display.set_mode(size)
            except Exception:
                return 0.0
        clock = pygame.time.Clock()
        t0 = time.time()
        for i in range(frames):
            screen.fill((0, 0, 0))
            pygame.draw.circle(screen, (0, 255, 70),
                               (size[0] // 2, size[1] // 2), 200, 2)
            pygame.display.flip()
            clock.tick(60)
        dt = time.time() - t0
        return frames / dt if dt > 0 else 0.0

    def set_mode(self):
        """Create the final display surface in 16-bit colour."""
        import pygame
        try:
            return pygame.display.set_mode(self.window_size, 0, self.color_depth)
        except Exception:
            return pygame.display.set_mode(self.window_size)

    # ── adaptive quality ──────────────────────────────────────────────────────

    def tick(self, fps):
        """Feed the measured FPS each frame; adjusts quality with hysteresis."""
        self._fps_hist.append(fps)
        if len(self._fps_hist) > 30:
            self._fps_hist.pop(0)
        avg = sum(self._fps_hist) / len(self._fps_hist)

        now = time.time()
        if now - self._last_switch < 2.0:     # don't thrash
            return self.quality
        idx = QUALITY_LEVELS.index(self.quality)
        if avg < 20 and idx > 0:
            self.quality = QUALITY_LEVELS[idx - 1]
            self._last_switch = now
            print(f"[vnc] FPS {avg:.0f} → quality {self.quality}")
        elif avg > 28 and idx < len(QUALITY_LEVELS) - 1:
            self.quality = QUALITY_LEVELS[idx + 1]
            self._last_switch = now
            print(f"[vnc] FPS {avg:.0f} → quality {self.quality}")
        return self.quality

    # quality flags the UI can read
    def glow_enabled(self):     return self.quality in ("MEDIUM", "HIGH")
    def full_glow(self):        return self.quality == "HIGH"
    def scanlines_enabled(self):return self.quality == "HIGH"

    # ── bandwidth monitor ─────────────────────────────────────────────────────

    def bandwidth_estimate(self, fps):
        """Rough bytes/s for the current resolution + depth + fps."""
        w, h = self.window_size
        bytes_per_px = self.color_depth / 8.0
        bw = w * h * bytes_per_px * fps
        if bw > 10e6 and not self._bw_warned:
            print(f"[vnc] WARNING: ~{bw/1e6:.0f} MB/s — dropping to LOW quality")
            self.quality = "LOW"
            self._bw_warned = True
        return bw

    # ── vnc_settings.sh generator ──────────────────────────────────────────────

    def write_vnc_settings(self, path=VNC_SETTINGS_SH):
        w, h = self.window_size
        script = (
            "#!/bin/bash\n"
            "# Optimal VNC settings for the RASPBOT radar\n"
            f"vncserver -geometry {w}x{h} -depth {self.color_depth} "
            "-compression 6 -quality 6\n"
        )
        try:
            with open(path, "w", newline="\n") as f:
                f.write(script)
            os.chmod(path, 0o755)
            print(f"[vnc] wrote {os.path.basename(path)} — "
                  f"run it to start an optimized VNC server")
        except Exception as e:
            print(f"[vnc] could not write settings: {e}")
        return path


# ═══════════════════════════════════════════════════════════════════════════════
#  FONT MANAGER  (military fonts with safe fallback)
# ═══════════════════════════════════════════════════════════════════════════════

class FontManager:
    """Loads Orbitron / Share Tech Mono .ttf from this folder, with fallback."""

    FILES = {
        "title": "Orbitron-Bold.ttf",
        "reg":   "Orbitron-Regular.ttf",
        "mono":  "ShareTechMono-Regular.ttf",
    }
    LINK = {
        "title": FONT_LINKS["orbitron"], "reg": FONT_LINKS["orbitron"],
        "mono":  FONT_LINKS["mono"],
    }

    def __init__(self):
        import pygame
        self.pg = pygame
        self.dir = _HERE
        self._cache = {}
        self._warned = set()

    def get(self, family, size):
        key = (family, size)
        if key in self._cache:
            return self._cache[key]
        pygame = self.pg
        path = os.path.join(self.dir, self.FILES.get(family, ""))
        font = None
        if os.path.exists(path):
            try:
                font = pygame.font.Font(path, size)
            except Exception:
                font = None
        if font is None:
            if family not in self._warned:
                print(f"[font] {self.FILES.get(family, family)} not found, "
                      f"using fallback. Download: {self.LINK.get(family, '')}")
                self._warned.add(family)
            font = pygame.font.SysFont("monospace", size,
                                       bold=(family == "title"))
        self._cache[key] = font
        return font

    # named convenience getters
    def title(self, size=FONT_SIZES["panel_title"]): return self.get("title", size)
    def data(self, size=FONT_SIZES["data"]):         return self.get("mono", size)
    def small(self, size=FONT_SIZES["small"]):       return self.get("mono", size)
    def alert(self, size=FONT_SIZES["alert"]):       return self.get("title", size)
    def reg(self, size=FONT_SIZES["med"]):           return self.get("reg", size)


# ── self-test ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    VNCOptimizer.apply_sdl_env()
    opt = VNCOptimizer()
    print("quality flags:", opt.glow_enabled(), opt.full_glow(), opt.scanlines_enabled())
    print("bandwidth @30fps:", f"{opt.bandwidth_estimate(30)/1e6:.1f} MB/s")
    opt.write_vnc_settings()
