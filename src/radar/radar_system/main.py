"""
main.py — master controller. Ties every module together.

    python3 main.py            # run on the Pi desktop / VNC
    python3 main.py --demo     # fake data, no hardware (laptop preview)

Startup sequence:
    1. load + verify calibration.json   (runs servo_calibration if missing)
    2. init UltrasonicFilter
    3. VNCOptimizer — detect + benchmark
    4. SoundEngine — warmup beep
    5. SweepSync — latency + start
    6. RadarUI — launch

A thread-safe data_bus carries state between the scan thread and the UI.
Ctrl+C / window-close → clean shutdown (servo centered, sound stopped).
"""

import os
import sys
import time
import math
import signal
import argparse
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from constants import (
    SRC_DIR, PAN_CENTER, ARC_DEG, SWEEP_SPEED, MAX_DISTANCE, DANGER_ZONE,
    WARNING_ZONE, SAFE_ZONE, SETTLE_S,
)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from servo_calibration import load_calibration, correct_angle, run_calibration
from sensor_filter import UltrasonicFilter, classify_zone
from vnc_optimizer import VNCOptimizer, FontManager
from sweep_sync import SweepSync
from sound_engine import SoundEngine

# object clustering thresholds
OBJ_BREAK_CM = 25
OBJ_GAP_DEG  = SWEEP_SPEED * 2.2
MOVE_CM      = 5


def _ok(msg):   print(f"[OK] {msg}")
def _info(msg): print(f"     {msg}")


class RadarApp:
    def __init__(self, demo=False):
        self.demo = demo
        self.running = True

        # ── thread-safe shared bus ──────────────────────────────────────────
        self.data_lock = threading.Lock()
        self.data_bus = {
            "current_angle":     0.0,
            "filtered_distance": 0.0,
            "zone":              "CLEAR",
            "objects":           [],
            "moving_objects":    [],
            "state":             "SCANNING",
            "sweep_on":          True,
            "muted":             False,
            "speed":             SWEEP_SPEED,
        }

        self.calib = None
        self.bot = None
        self.filter = None
        self.opt = None
        self.fonts = None
        self.sync = None
        self.sound = None
        self.ui = None

        self._prev_objects = []     # last sweep's objects (movement compare)
        self._sweep_hits = {}       # bearing -> distance for in-progress sweep

    # ── startup ─────────────────────────────────────────────────────────────

    def startup(self):
        print("RASPBOT RADAR SYSTEM STARTING...\n")

        # 1. calibration
        self.calib = load_calibration(require=True)
        if self.calib is None:
            if self.demo:
                self.calib = {"left_limit": PAN_CENTER - ARC_DEG // 2,
                              "right_limit": PAN_CENTER + ARC_DEG // 2,
                              "center_offset": 0.0, "angle_corrections": {},
                              "servo_speed_dps": 300, "calibrated": True}
                _ok("Calibration: using demo defaults")
            else:
                print("[!] No valid calibration — launching servo_calibration...")
                self.calib = run_calibration()
        else:
            _ok("Calibration loaded")
            _info(f"limits {self.calib['left_limit']}–{self.calib['right_limit']}, "
                  f"{self.calib['servo_speed_dps']:.0f} dps")

        # 2. sensor filter
        self.filter = UltrasonicFilter()
        _ok("Sensor filter ready")

        # 3. VNC optimizer
        VNCOptimizer.apply_sdl_env()
        import pygame
        pygame.init()
        self.opt = VNCOptimizer()
        self.opt.choose_resolution()
        self.opt.write_vnc_settings()
        self.fonts = FontManager()
        _ok(f"VNC optimized: {self.opt.window_size[0]}x{self.opt.window_size[1]}")

        # 4. sound
        self.sound = SoundEngine()
        self.sound.start()
        _ok("Sound engine ready")

        # 5. hardware + sweep sync
        if not self.demo:
            from setup_and_api.api import RasBot
            self.bot = RasBot()
            self.bot.__enter__()
        self.sync = SweepSync(self.bot, calibration=self.calib, debug=False)
        if self.bot is not None:
            self.sync.measure_latency()
        # noise floor (real hardware only)
        if self.bot is not None:
            self.bot.set_pan(PAN_CENTER)
            time.sleep(0.4)
            self.filter.calibrate_noise_floor(self.bot.read_distance)
        self.sync.start()
        _ok("Sweep sync ready")

        # 6. UI
        from radar_ui import RadarUI
        self.ui = RadarUI(self.data_bus, self.data_lock, self.opt, self.fonts,
                          self.sync, self.on_key, sound=self.sound)
        _ok("Radar UI ready")
        print("\n[LAUNCHING] Radar UI...\n")

    # ── controls (called by UI) ─────────────────────────────────────────────

    def on_key(self, name):
        with self.data_lock:
            if name == "S":
                self.data_bus["sweep_on"] = not self.data_bus["sweep_on"]
            elif name == "M":
                self.data_bus["muted"] = self.sound.toggle_mute()
            elif name == "+":
                self.data_bus["speed"] = min(8, self.data_bus["speed"] + 1)
            elif name == "-":
                self.data_bus["speed"] = max(1, self.data_bus["speed"] - 1)
            elif name == "R":
                self.filter.reset()
                self._prev_objects = []
                self._sweep_hits = {}
                self.data_bus["objects"] = []
                self.data_bus["moving_objects"] = []

    # ── scan thread ──────────────────────────────────────────────────────────

    def _demo_distance(self, bearing):
        import random
        wall = 150.0 / max(math.cos(math.radians(bearing)), 0.4)
        person_b = 30 * math.sin(time.time() * 0.4)
        if abs(bearing - person_b) < 5:
            wall = min(wall, 55 + 20 * math.sin(time.time() * 0.4))
        d = wall + random.gauss(0, 2)
        if random.random() < 0.04:
            d = random.uniform(2, MAX_DISTANCE)
        if abs(bearing) > 78:
            return 0.0
        return max(0.0, min(MAX_DISTANCE, d))

    def scan_loop(self):
        last_dir = 1
        while self.running:
            with self.data_lock:
                sweep_on = self.data_bus["sweep_on"]
                speed = self.data_bus["speed"]
            if not sweep_on:
                time.sleep(0.05)
                continue

            self.sync.set_step(speed)

            if not self.sync.ready_to_read():
                time.sleep(0.003)
                continue

            bearing = self.sync.get_current_angle() - PAN_CENTER

            # read distance (filtered)
            if self.demo:
                raw = self._demo_distance(bearing)
            else:
                raw = self.bot.read_distance()
            res = self.filter.read(raw, angle=bearing)
            self.sync.mark_read()

            if res["valid"] and res["zone"] != "CLEAR":
                self._sweep_hits[round(bearing)] = res["distance"]
            else:
                self._sweep_hits.pop(round(bearing), None)

            # update live bus values
            with self.data_lock:
                self.data_bus["current_angle"] = bearing
                self.data_bus["filtered_distance"] = res["distance"]
                self.data_bus["zone"] = res["zone"]

            # advance to the next step; reversal = end of a sweep
            self.sync.advance()
            if self.sync.direction != last_dir:
                last_dir = self.sync.direction
                self._finish_sweep()
                self.filter.tick_sweep()

    def _finish_sweep(self):
        objs = self._cluster(dict(self._sweep_hits))
        for o in objs:
            o["moving"], o["from"] = self._movement(o)
            o["ghosts"] = self._ghosts(o)
        moving = [o for o in objs if o["moving"]]

        nearest = min(objs, key=lambda o: o["range"]) if objs else None
        if nearest and nearest["range"] < DANGER_ZONE:
            state = "DANGER"
        elif moving:
            state = "MOVING"
        elif not objs:
            state = "CLEAR"
        else:
            state = "SCANNING"
        with self.data_lock:
            if not self.data_bus["sweep_on"]:
                state = "PAUSED"
            self.data_bus["objects"] = objs
            self.data_bus["moving_objects"] = moving
            self.data_bus["state"] = state

        # feed sound: nearest object distance + bearing
        if nearest:
            self.sound.update(nearest["range"], nearest["bearing"],
                              any(o["moving"] for o in objs))
        else:
            self.sound.update(None, 0, False)

        self._prev_objects = objs

    def _cluster(self, readings):
        items = sorted(readings.items())
        objs, cur = [], []
        for bearing, dist in items:
            if cur:
                pb, pd = cur[-1]
                if (bearing - pb) > OBJ_GAP_DEG or abs(dist - pd) > OBJ_BREAK_CM:
                    objs.append(self._make(cur)); cur = []
            cur.append((bearing, dist))
        if cur:
            objs.append(self._make(cur))
        objs.sort(key=lambda o: o["range"])
        return objs

    def _make(self, pts):
        bs = [b for b, _ in pts]
        ds = [d for _, d in pts]
        span = (max(bs) - min(bs)) if len(bs) > 1 else 0
        size = "SMALL" if span <= 5 else "MEDIUM" if span <= 15 else "LARGE"
        return {"bearing": sum(bs) / len(bs), "range": min(ds),
                "span": span, "size": size, "moving": False,
                "from": None, "ghosts": []}

    def _movement(self, obj):
        best, bdb = None, 12.0
        for p in self._prev_objects:
            db = abs(p["bearing"] - obj["bearing"])
            if db < bdb:
                best, bdb = p, db
        if best and abs(best["range"] - obj["range"]) > MOVE_CM:
            return True, (best["bearing"], best["range"])
        return False, None

    def _ghosts(self, obj):
        if obj["from"] is None:
            return []
        return [obj["from"]]

    # ── run + shutdown ──────────────────────────────────────────────────────

    def run(self):
        self.startup()
        threading.Thread(target=self.scan_loop, daemon=True).start()
        try:
            self.ui.run()             # blocks until quit
        finally:
            self.shutdown()

    def shutdown(self):
        if not self.running:
            return
        self.running = False
        print("\nshutting down...")
        try:
            if self.sync:
                self.sync.stop()
            if self.sound:
                self.sound.stop()
            if self.bot is not None:
                try:
                    self.bot.look_center()
                except Exception:
                    pass
                self.bot.__exit__(None, None, None)
            import pygame
            pygame.quit()
        except Exception:
            pass
        print("RADAR SYSTEM SHUTDOWN COMPLETE")


def main():
    ap = argparse.ArgumentParser(description="RASPBOT radar master controller.")
    ap.add_argument("--demo", action="store_true",
                    help="fake data, no hardware (laptop preview)")
    args = ap.parse_args()

    app = RadarApp(demo=args.demo)

    def _sig(_signum, _frame):
        app.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sig)

    app.run()


if __name__ == "__main__":
    main()
