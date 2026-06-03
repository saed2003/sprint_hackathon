"""
game_engine.py — ROBO EXPLORER Game Engine
===========================================
Tracks score, XP, level, achievements, run history, and fog-of-war
robot position. Persists state to game_state.json next to this file.

Usage (from any other module):
    from game.game_engine import GameEngine
    ge = GameEngine.instance()
    ge.on_run_start()
    ge.on_checkpoint(scan_session_path)   # called from _do_scan
    ge.on_scan_360(ply_path)              # called after build
    ge.on_run_end()

The HTTP polling endpoint is served by control_server.py — just add:
    from game.game_engine import GameEngine
    @app.get("/api/game/state")
    def game_state(): return GameEngine.instance().get_state()
    @app.post("/api/game/event")
    def game_event(body): GameEngine.instance().handle_event(body); return {"ok": True}
"""

import json
import math
import time
import threading
from pathlib import Path
from datetime import datetime
from typing import Optional

# ── constants ────────────────────────────────────────────────────────────────
STATE_FILE   = Path(__file__).parent / "game_state.json"

XP_PER_CHECKPOINT  = 500
XP_PER_SCAN_360    = 1500
XP_PER_RUN_FINISH  = 800
XP_PER_MANUAL_SCAN = 300
BONUS_SPEED_RUN    = 1000   # awarded if full run < SPEED_RUN_SECONDS
SPEED_RUN_SECONDS  = 300    # 5 minutes

LEVEL_THRESHOLDS = [0, 500, 1500, 3500, 7000, 12000, 20000, 30000, 45000, 65000, 90000]


def _log(msg: str):
    """Print that can never crash the robot — survives consoles that can't
    encode emoji (e.g. Windows cp1252). Always a no-op on failure."""
    try:
        print(msg)
    except Exception:
        try:
            print(msg.encode("ascii", "replace").decode("ascii"))
        except Exception:
            pass

# Grid for fog-of-war  (units = robot body-widths ≈ 25 cm each)
FOG_GRID_W = 30
FOG_GRID_H = 30

# ── achievements ─────────────────────────────────────────────────────────────
ACHIEVEMENTS = {
    "first_discovery": {
        "title": "First Discovery",
        "desc":  "Complete your first checkpoint scan",
        "icon":  "🥇",
        "xp":    200,
    },
    "spin_master": {
        "title": "360° Master",
        "desc":  "Complete a full 360° rotation scan",
        "icon":  "🌀",
        "xp":    500,
    },
    "cartographer": {
        "title": "Cartographer",
        "desc":  "Reveal 80% of the fog-of-war map",
        "icon":  "🗺",
        "xp":    1000,
    },
    "speed_runner": {
        "title": "Speed Runner",
        "desc":  "Complete a full run in under 5 minutes",
        "icon":  "⚡",
        "xp":    1000,
    },
    "deep_scanner": {
        "title": "Deep Scanner",
        "desc":  "Complete 5 checkpoint scans in one run",
        "icon":  "🔭",
        "xp":    750,
    },
    "autonomous": {
        "title": "Fully Autonomous",
        "desc":  "Complete a full run without any manual override",
        "icon":  "🤖",
        "xp":    1200,
    },
    "point_cloud_collector": {
        "title": "Point Cloud Collector",
        "desc":  "Build 3 point clouds in total",
        "icon":  "☁️",
        "xp":    600,
    },
    "explorer_lvl5": {
        "title": "Elite Explorer",
        "desc":  "Reach level 5",
        "icon":  "🏆",
        "xp":    2000,
    },
}


def _level_for_xp(xp: int) -> int:
    for lvl, threshold in enumerate(reversed(LEVEL_THRESHOLDS)):
        if xp >= threshold:
            return len(LEVEL_THRESHOLDS) - 1 - lvl
    return 0


def _xp_for_next_level(current_xp: int) -> int:
    lvl = _level_for_xp(current_xp)
    if lvl + 1 < len(LEVEL_THRESHOLDS):
        return LEVEL_THRESHOLDS[lvl + 1]
    return LEVEL_THRESHOLDS[-1]


class GameEngine:
    """Singleton game engine. Use GameEngine.instance() everywhere."""

    _inst: Optional["GameEngine"] = None
    _lock = threading.Lock()

    @classmethod
    def instance(cls) -> "GameEngine":
        with cls._lock:
            if cls._inst is None:
                cls._inst = cls()
            return cls._inst

    # ── init ────────────────────────────────────────────────────────────────
    def __init__(self):
        self._lock = threading.Lock()
        self._state = self._load_or_default()
        self._run_start_time: Optional[float] = None
        self._manual_override_this_run = False
        self._pending_achievements: list[dict] = []   # queued for HUD popup

    # ── public event API ────────────────────────────────────────────────────
    def on_run_start(self, mode: str = "autonomous"):
        with self._lock:
            self._run_start_time = time.time()
            self._manual_override_this_run = (mode == "manual")
            s = self._state
            s["run_active"]       = True
            s["current_run_checkpoints"] = 0
            s["current_run_mode"] = mode
            s["current_run_start"] = datetime.now().isoformat()
            s["robot_x"] = FOG_GRID_W // 2   # start center
            s["robot_y"] = FOG_GRID_H // 2
            s["robot_heading"] = 0            # degrees, 0 = north
            # reveal starting cell
            self._reveal_fog(s["robot_x"], s["robot_y"], radius=2)
            self._save()
        _log("[GAME] Run started — mode: " + str(mode))

    def on_run_end(self):
        with self._lock:
            s = self._state
            s["run_active"] = False
            elapsed = time.time() - (self._run_start_time or time.time())

            # base completion XP
            xp = XP_PER_RUN_FINISH
            reason = "run complete"

            # speed run bonus
            if elapsed < SPEED_RUN_SECONDS and s["current_run_checkpoints"] >= 2:
                xp += BONUS_SPEED_RUN
                self._unlock_achievement("speed_runner")
                reason += " + speed bonus"

            # autonomous bonus
            if not self._manual_override_this_run and s["current_run_checkpoints"] >= 2:
                self._unlock_achievement("autonomous")

            self._add_xp(xp, reason)

            # log run to history
            s["run_history"].append({
                "date":        datetime.now().isoformat(),
                "checkpoints": s["current_run_checkpoints"],
                "clouds":      s.get("current_run_clouds", 0),
                "duration_s":  round(elapsed),
                "xp_earned":   xp,
            })
            if len(s["run_history"]) > 20:
                s["run_history"] = s["run_history"][-20:]

            s["current_run_checkpoints"] = 0
            s["current_run_clouds"]      = 0
            self._save()
        _log(f"[GAME] Run ended — {reason}")

    def on_checkpoint(self, session_path: str = ""):
        """Call this every time the robot hits a stop marker and scans."""
        with self._lock:
            s = self._state
            s["current_run_checkpoints"] = s.get("current_run_checkpoints", 0) + 1
            s["total_checkpoints"]       = s.get("total_checkpoints", 0) + 1
            cp = s["current_run_checkpoints"]

            self._add_xp(XP_PER_CHECKPOINT, f"checkpoint #{cp}")

            # advance robot on fog grid (move forward 2 cells per checkpoint)
            heading_rad = math.radians(s.get("robot_heading", 0))
            s["robot_x"] = max(0, min(FOG_GRID_W - 1,
                s["robot_x"] + round(2 * math.sin(heading_rad))))
            s["robot_y"] = max(0, min(FOG_GRID_H - 1,
                s["robot_y"] - round(2 * math.cos(heading_rad))))
            self._reveal_fog(s["robot_x"], s["robot_y"], radius=3)

            # achievements
            if s["total_checkpoints"] == 1:
                self._unlock_achievement("first_discovery")
            if cp >= 5:
                self._unlock_achievement("deep_scanner")

            # fog % check
            revealed = sum(1 for cell in s["fog_grid"] if cell)
            total    = FOG_GRID_W * FOG_GRID_H
            s["map_pct"] = round(revealed / total * 100)
            if s["map_pct"] >= 80:
                self._unlock_achievement("cartographer")

            self._save()
        _log(f"[GAME] Checkpoint #{cp} — session: {session_path}")

    def on_scan_360(self, ply_path: str = ""):
        """Call this after a 360° scan + cloud build completes."""
        with self._lock:
            s = self._state
            s["current_run_clouds"] = s.get("current_run_clouds", 0) + 1
            s["total_clouds"]       = s.get("total_clouds", 0) + 1

            self._add_xp(XP_PER_SCAN_360, "360° scan")
            self._unlock_achievement("spin_master")

            if s["total_clouds"] >= 3:
                self._unlock_achievement("point_cloud_collector")

            self._save()
        _log(f"[GAME] 360° scan complete — cloud: {ply_path}")

    def on_manual_scan(self, ply_path: str = ""):
        """Call this after a manual (V key) single capture."""
        with self._lock:
            self._manual_override_this_run = True
            self._add_xp(XP_PER_MANUAL_SCAN, "manual scan")
            self._save()

    def on_robot_move(self, heading_deg: float, distance_cells: float = 1.0):
        """Optional: update robot position on fog map during navigation."""
        with self._lock:
            s = self._state
            s["robot_heading"] = heading_deg % 360
            heading_rad = math.radians(heading_deg)
            s["robot_x"] = max(0, min(FOG_GRID_W - 1,
                s["robot_x"] + round(distance_cells * math.sin(heading_rad))))
            s["robot_y"] = max(0, min(FOG_GRID_H - 1,
                s["robot_y"] - round(distance_cells * math.cos(heading_rad))))
            self._reveal_fog(s["robot_x"], s["robot_y"], radius=2)
            self._save()

    def handle_event(self, body: dict):
        """Generic event dispatcher — used by the HTTP endpoint."""
        ev = body.get("event", "")
        if ev == "run_start":       self.on_run_start(body.get("mode", "autonomous"))
        elif ev == "run_end":       self.on_run_end()
        elif ev == "checkpoint":    self.on_checkpoint(body.get("session", ""))
        elif ev == "scan_360":      self.on_scan_360(body.get("ply", ""))
        elif ev == "manual_scan":   self.on_manual_scan(body.get("ply", ""))
        elif ev == "robot_move":
            self.on_robot_move(body.get("heading", 0), body.get("distance", 1.0))

    def get_state(self) -> dict:
        """Return full state for HTTP polling. Flushes pending achievements."""
        with self._lock:
            s = dict(self._state)
            s["pending_achievements"] = list(self._pending_achievements)
            self._pending_achievements.clear()
            s["level"] = _level_for_xp(s["total_xp"])
            s["xp_next_level"] = _xp_for_next_level(s["total_xp"])
            s["elapsed_s"] = (
                round(time.time() - self._run_start_time)
                if self._run_start_time and s["run_active"] else 0
            )
            return s

    def reset_run(self):
        """Reset only current-run stats (keep XP/achievements)."""
        with self._lock:
            s = self._state
            s["run_active"]              = False
            s["current_run_checkpoints"] = 0
            s["current_run_clouds"]      = 0
            s["robot_x"] = FOG_GRID_W // 2
            s["robot_y"] = FOG_GRID_H // 2
            s["robot_heading"] = 0
            s["fog_grid"] = [False] * (FOG_GRID_W * FOG_GRID_H)
            self._reveal_fog(s["robot_x"], s["robot_y"], radius=2)
            self._save()

    # ── internals ───────────────────────────────────────────────────────────
    def _add_xp(self, amount: int, reason: str = ""):
        s = self._state
        old_level = _level_for_xp(s["total_xp"])
        s["total_xp"] += amount
        s["score"]    += amount
        new_level = _level_for_xp(s["total_xp"])
        if new_level > old_level:
            s["level"] = new_level
            self._pending_achievements.append({
                "type":  "level_up",
                "title": f"Level Up! → {new_level}",
                "desc":  f"You reached level {new_level}",
                "icon":  "⬆️",
            })
            if new_level >= 5:
                self._unlock_achievement("explorer_lvl5")
        _log(f"[GAME] +{amount} XP ({reason}) — total: {s['total_xp']}")

    def _unlock_achievement(self, key: str):
        s = self._state
        if key in s["unlocked_achievements"]:
            return
        a = ACHIEVEMENTS.get(key)
        if not a:
            return
        s["unlocked_achievements"][key] = datetime.now().isoformat()
        self._add_xp(a["xp"], f"achievement: {a['title']}")
        self._pending_achievements.append({
            "type":  "achievement",
            "key":   key,
            "title": a["title"],
            "desc":  a["desc"],
            "icon":  a["icon"],
        })
        _log(f"[GAME] 🏆 Achievement unlocked: {a['title']}")

    def _reveal_fog(self, cx: int, cy: int, radius: int = 2):
        grid = self._state["fog_grid"]
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < FOG_GRID_W and 0 <= y < FOG_GRID_H:
                        grid[y * FOG_GRID_W + x] = True

    def _load_or_default(self) -> dict:
        if STATE_FILE.exists():
            try:
                return json.loads(STATE_FILE.read_text())
            except Exception:
                pass
        return self._default_state()

    def _save(self):
        try:
            STATE_FILE.write_text(json.dumps(self._state, indent=2))
        except Exception as e:
            _log(f"[GAME] Warning: could not save state: {e}")

    @staticmethod
    def _default_state() -> dict:
        return {
            "total_xp":               0,
            "score":                  0,
            "level":                  0,
            "run_active":             False,
            "current_run_checkpoints": 0,
            "current_run_clouds":     0,
            "current_run_mode":       "autonomous",
            "current_run_start":      None,
            "total_checkpoints":      0,
            "total_clouds":           0,
            "map_pct":                0,
            "robot_x":                FOG_GRID_W // 2,
            "robot_y":                FOG_GRID_H // 2,
            "robot_heading":          0,
            "fog_grid":               [False] * (FOG_GRID_W * FOG_GRID_H),
            "unlocked_achievements":  {},
            "run_history":            [],
        }
