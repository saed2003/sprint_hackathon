"""
sweep_sync.py — STEP 7: servo / UI sweep synchronization.

Makes the on-screen sweep arm match where the real (open-loop) servo is
actually pointing, using time-based angle estimation derived from the
calibrated servo speed.

    sync = SweepSync(bot)          # bot may be None in demo mode
    sync.start()
    while running:
        angle = sync.get_display_angle()   # smooth, real-time angle for the UI
        if sync.ready_to_read():           # servo has been still long enough
            d = bot.read_distance()
            sync.mark_read()

The RasBot servo has no position encoder, so "estimated" angle is computed
from elapsed time × calibrated degrees-per-second, with measured command
latency subtracted.
"""

import os
import sys
import time
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from constants import (
    PAN_CENTER, ARC_DEG, STILL_TIME, SERVO_SPEED_DPS_DEFAULT, SWEEP_SPEED,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SweepSync:
    """Time-based sweep tracker that keeps UI and servo in lockstep."""

    def __init__(self, bot, calibration=None, debug=False):
        self.bot = bot
        self.debug = debug
        cal = calibration or {}

        self.dps        = float(cal.get("servo_speed_dps", SERVO_SPEED_DPS_DEFAULT))
        self.left_limit  = int(cal.get("left_limit",  PAN_CENTER - ARC_DEG // 2))
        self.right_limit = int(cal.get("right_limit", PAN_CENTER + ARC_DEG // 2))
        self.center_off  = float(cal.get("center_offset", 0.0))

        self._angle       = float(self.left_limit)   # last commanded angle
        self._target      = float(self.right_limit)
        self._move_start  = time.time()
        self._start_angle = self._angle
        self.direction    = 1                         # +1 = toward right_limit
        self.step_deg     = SWEEP_SPEED               # degrees per advance()
        self.latency      = 0.0
        self._last_read   = 0.0
        self._still_since = time.time()
        self._running     = False
        self._lock        = threading.Lock()
        self._last_diag   = 0.0

    # ── latency calibration ────────────────────────────────────────────────────

    def measure_latency(self, samples=20, log=print):
        """Average the time it takes to issue a servo command."""
        if self.bot is None:
            self.latency = 0.0
            return 0.0
        times = []
        for _ in range(samples):
            t0 = time.time()
            try:
                self.bot.set_pan(int(self._angle))
            except Exception:
                pass
            times.append(time.time() - t0)
            time.sleep(0.005)
        self.latency = sum(times) / len(times)
        log(f"[sync] command latency ≈ {self.latency*1000:.1f} ms")
        return self.latency

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def start(self):
        """Begin at the left limit. Stepping is driven by advance() from the
        scan loop — there is no free-running servo thread (matches the
        step-and-settle model the sensor read gate expects)."""
        self._running = True
        self.step_deg = SWEEP_SPEED
        self.direction = 1
        self._command(self.left_limit)

    def stop(self):
        self._running = False

    def set_step(self, deg):
        self.step_deg = max(1, int(deg))

    def advance(self):
        """Move to the next discrete step, reversing at the limits.

        Called by the scan loop AFTER it has taken a reading at the current
        (settled) position.
        """
        nxt = self._angle + self.direction * self.step_deg
        if nxt >= self.right_limit:
            nxt = self.right_limit
            self.direction = -1
        elif nxt <= self.left_limit:
            nxt = self.left_limit
            self.direction = 1
        self._command(nxt)

    # ── command + estimate ─────────────────────────────────────────────────────

    def _estimate_nolock(self):
        """Estimate angle assuming the caller already holds self._lock."""
        start = self._start_angle
        target = self._angle
        t0 = self._move_start
        elapsed = max(0.0, time.time() - t0 - self.latency)
        travelled = self.dps * elapsed
        if target >= start:
            return min(target, start + travelled)
        return max(target, start - travelled)

    def _command(self, angle):
        angle = _clamp(angle, self.left_limit, self.right_limit)
        with self._lock:
            self._start_angle = self._estimate_nolock()   # lock-safe (no re-acquire)
            self._angle = angle
            self._move_start = time.time()
            self._still_since = 0.0
        if self.bot is not None:
            try:
                self.bot.set_pan(int(angle + self.center_off))
            except Exception:
                pass

    def estimated_angle(self):
        """Where the servo *should* be right now, from elapsed time × dps."""
        with self._lock:
            return self._estimate_nolock()

    def at_target(self):
        return abs(self.estimated_angle() - self._angle) < 0.5

    # ── sensor read gating ──────────────────────────────────────────────────────

    def ready_to_read(self):
        """True when the servo has reached the current step and been still
        for STILL_TIME (prevents motion blur). Manages its own settle timer."""
        if not self.at_target():
            self._still_since = 0.0
            return False
        if self._still_since == 0.0:
            self._still_since = time.time()
            return False
        return (time.time() - self._still_since) >= STILL_TIME

    def mark_read(self):
        self._last_read = time.time()

    # ── UI interface ──────────────────────────────────────────────────────────

    def get_current_angle(self):
        """Estimated true servo angle (commanded frame)."""
        return self.estimated_angle()

    def get_display_angle(self):
        """Bearing for the UI: 0 = straight ahead, + = right.

        Interpolated each frame so the UI is smooth even with coarse steps.
        """
        return self.estimated_angle() - PAN_CENTER

    def diagnostics(self, log=print):
        """Print a sync report at most every 5 s (debug mode)."""
        if not self.debug:
            return
        now = time.time()
        if now - self._last_diag < 5.0:
            return
        self._last_diag = now
        est = self.estimated_angle()
        log(f"[sync] commanded={self._angle:5.1f}°  estimated={est:5.1f}°  "
            f"display={est - PAN_CENTER:+5.1f}°  dir={'R' if self.direction>0 else 'L'}  "
            f"latency={self.latency*1000:.0f}ms")


# ── self-test (no hardware) ─────────────────────────────────────────────────────

if __name__ == "__main__":
    s = SweepSync(bot=None, calibration={"servo_speed_dps": 300,
                                          "left_limit": 0, "right_limit": 180},
                  debug=True)
    s.set_step(10)
    s.start()
    reads = 0
    t0 = time.time()
    while time.time() - t0 < 3 and reads < 25:
        if s.ready_to_read():
            print(f"READ @ display={s.get_display_angle():+6.1f}  "
                  f"dir={'R' if s.direction > 0 else 'L'}")
            s.mark_read()
            s.advance()
            reads += 1
        time.sleep(0.002)
    s.stop()
    print(f"total reads in 3s: {reads}")
