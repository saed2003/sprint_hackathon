"""
sound_engine.py — STEP 5: intelligent audio feedback.

Tells you what the radar sees without looking at it: distance-driven beeps
(rate + pitch rise as objects approach), stereo panning by bearing, a
double-chirp for moving objects, and a rising danger alarm under 10 cm.

    snd = SoundEngine(master_volume=0.7)
    snd.start()                       # plays a warmup beep
    snd.update(distance=42, bearing=-30, moving=False)   # called each frame
    snd.toggle_mute()

All tones are generated programmatically with numpy — no audio files.
Runs in a dedicated thread; never blocks the UI.
"""

import os
import sys
import time
import math
import threading
import queue

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from constants import (
    MASTER_VOLUME, MIN_FREQ, MAX_FREQ, SND_SAMPLE_RATE, MAX_DISTANCE,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SoundEngine:
    """Threaded, queue-driven tone synthesizer for the radar."""

    def __init__(self, master_volume=MASTER_VOLUME):
        self.master = master_volume
        self.muted  = False
        self._fade  = 1.0                # current fade multiplier (mute ramp)
        self._ok    = False
        self._np    = None
        self._cache = {}                 # (freq, ms, lvol, rvol) -> Sound
        self._q     = queue.Queue()
        self._running = True

        # shared target state written by update()
        self._lock = threading.Lock()
        self._distance = None
        self._bearing  = 0.0
        self._moving   = False
        self._last_beep = 0.0
        self._state = "SILENT"

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def start(self):
        try:
            import numpy as np
            import pygame
            self._np = np
            pygame.mixer.pre_init(SND_SAMPLE_RATE, -16, 2, 512)  # stereo
            pygame.mixer.init()
            self._ok = True
            print("[sound] audio system ready")
            self._play(self._tone(440, 100, 0.2, 0.2))           # warmup beep
        except Exception as e:
            print(f"[sound] audio unavailable ({e}) — continuing silent")
            self._ok = False
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def toggle_mute(self):
        self.muted = not self.muted
        return self.muted

    def set_volume(self, v):
        self.master = _clamp(v, 0.0, 1.0)

    # ── per-frame state update (called by UI/main) ──────────────────────────────

    def update(self, distance, bearing=0.0, moving=False):
        with self._lock:
            self._distance = distance
            self._bearing  = bearing
            self._moving   = moving

    # ── tone synthesis ──────────────────────────────────────────────────────────

    def _tone(self, freq, ms, left_vol, right_vol):
        """Return a cached stereo Sound for (freq, duration, pan volumes)."""
        import pygame
        key = (int(freq), int(ms), round(left_vol, 2), round(right_vol, 2))
        if key in self._cache:
            return self._cache[key]
        np = self._np
        n = max(1, int(SND_SAMPLE_RATE * ms / 1000.0))
        t = np.linspace(0, ms / 1000.0, n, False)
        wave = np.sin(2 * np.pi * freq * t)
        fade = min(20, n // 2)                       # anti-click envelope
        if fade > 0:
            wave[:fade] *= np.linspace(0, 1, fade)
            wave[-fade:] *= np.linspace(1, 0, fade)
        left  = (wave * left_vol  * self.master * 32767).astype(np.int16)
        right = (wave * right_vol * self.master * 32767).astype(np.int16)
        stereo = np.ascontiguousarray(np.column_stack((left, right)))
        snd = pygame.sndarray.make_sound(stereo)
        if len(self._cache) > 256:
            self._cache.clear()
        self._cache[key] = snd
        return snd

    def _sweep_tone(self, f0, f1, ms, vol):
        """Rising/falling frequency sweep (used for the danger alarm)."""
        import pygame
        np = self._np
        n = max(1, int(SND_SAMPLE_RATE * ms / 1000.0))
        t = np.linspace(0, ms / 1000.0, n, False)
        freqs = np.linspace(f0, f1, n)
        phase = 2 * np.pi * np.cumsum(freqs) / SND_SAMPLE_RATE
        wave = np.sin(phase)
        fade = min(20, n // 2)
        wave[:fade] *= np.linspace(0, 1, fade)
        wave[-fade:] *= np.linspace(1, 0, fade)
        amp = (wave * vol * self.master * 32767).astype(np.int16)
        stereo = np.ascontiguousarray(np.column_stack((amp, amp)))
        return pygame.sndarray.make_sound(stereo)

    def _play(self, snd):
        if snd is not None and self._fade > 0.01:
            try:
                snd.set_volume(self._fade)
                snd.play()
            except Exception:
                pass

    # ── pan from bearing ─────────────────────────────────────────────────────────

    @staticmethod
    def _pan(bearing):
        """Stereo volumes from bearing (-90..+90). Left object → left louder."""
        # map bearing to 0..pi/2 so cos/sin split the energy
        b = math.radians(_clamp(bearing, -90, 90))
        # normalize to [0, pi/2]: -90→favor left, +90→favor right
        frac = (_clamp(bearing, -90, 90) + 90) / 180.0     # 0..1 (0=left)
        ang = frac * (math.pi / 2)
        left  = math.cos(ang)
        right = math.sin(ang)
        return left, right

    # ── distance → freq / interval (smooth) ─────────────────────────────────────

    @staticmethod
    def _freq(d):
        d = _clamp(d, 5, MAX_DISTANCE)
        return MIN_FREQ + ((100 - min(d, 100)) / 100.0) * (MAX_FREQ - MIN_FREQ)

    @staticmethod
    def _interval(d):
        """Seconds between beeps. 0 = continuous, None = silent."""
        if d >= 100:  return None
        if d < 5:     return 0.0
        # smooth shrink: interval ~ d/100, clamped to sane bounds
        return _clamp(d / 100.0, 0.1, 3.0)

    def _classify_state(self, d):
        if d is None or d >= 100:
            return "SILENT"
        if d < 10:
            return "ALARM"
        if d < 50:
            return "FAST_BEEP"
        return "BEEPING"

    # ── sound thread ──────────────────────────────────────────────────────────

    def _loop(self):
        while self._running:
            if not self._ok:
                time.sleep(0.2); continue

            # fade ramp for smooth mute/unmute
            target = 0.0 if self.muted else 1.0
            self._fade += (target - self._fade) * 0.25
            if self.muted and self._fade < 0.02:
                time.sleep(0.05); continue

            with self._lock:
                d = self._distance
                bearing = self._bearing
                moving = self._moving

            self._state = self._classify_state(d)

            if d is None or d >= 100:
                time.sleep(0.08); continue

            lvol, rvol = self._pan(bearing)

            if self._state == "ALARM":
                # rising danger sweep 400→1200 Hz, loud, repeats
                self._play(self._sweep_tone(400, 1200, 500, 0.8))
                time.sleep(0.5)
                continue

            freq = self._freq(d)
            dur  = 0.09
            self._play(self._tone(freq, int(dur * 1000), lvol, rvol))

            if moving:
                # distinctive double-chirp in addition to the distance beep
                time.sleep(0.06)
                chirp = 880 if d < 50 else 440
                self._play(self._tone(chirp, 70, lvol, rvol))

            iv = self._interval(d)
            time.sleep(0.1 if iv is None else max(dur, iv))


# ── self-test ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    s = SoundEngine()
    s.start()
    print("pitch @150:", round(s._freq(150)), "@40:", round(s._freq(40)),
          "@8:", round(s._freq(8)))
    print("interval @150:", s._interval(150), "@40:", s._interval(40),
          "@8:", s._interval(8), "@4:", s._interval(4))
    print("pan left(-60):", tuple(round(v, 2) for v in s._pan(-60)),
          "center(0):", tuple(round(v, 2) for v in s._pan(0)),
          "right(+60):", tuple(round(v, 2) for v in s._pan(60)))
    if s._ok:
        for d in (90, 60, 30, 15, 8):
            s.update(d, bearing=0, moving=False); time.sleep(1.0)
    s.stop()
