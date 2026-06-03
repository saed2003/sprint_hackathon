"""
sensor_filter.py — STEP 2: ultrasonic noise filtering.

Clean sensor data so the radar shows no ghost objects, no flickering dots
and no false alarms.

    from sensor_filter import UltrasonicFilter
    f = UltrasonicFilter()
    result = f.read(raw_cm, angle=87)
    # -> {"distance": 42.0, "zone": "WARNING", "valid": True, "persistent": True}

Pipeline per reading:
    raw → range check → median (last N) → spike rejection → EMA smoothing
        → zone classification → per-angle persistence
"""

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np
from collections import deque
from constants import (
    FILTER_ALPHA, MAX_JUMP, READINGS_PER_POS, READING_GAP_S, MAX_DISTANCE,
    MIN_DISTANCE, DANGER_ZONE, WARNING_ZONE, SAFE_ZONE, PERSIST_SWEEPS,
)


def classify_zone(distance):
    """Map a filtered distance (cm) to a zone label."""
    if distance is None or distance <= 0 or distance > SAFE_ZONE:
        return "CLEAR"
    if distance > WARNING_ZONE:
        return "SAFE"
    if distance > DANGER_ZONE:
        return "WARNING"
    return "DANGER"


class UltrasonicFilter:
    """Median + EMA + spike-rejection filter with per-angle persistence."""

    def __init__(self, alpha=FILTER_ALPHA, max_jump=MAX_JUMP,
                 max_range=MAX_DISTANCE, debug=False):
        self.alpha      = alpha
        self.max_jump   = max_jump
        self.max_range  = max_range
        self.debug      = debug

        self._window    = deque(maxlen=READINGS_PER_POS)  # recent valid raws
        self._last      = None         # last accepted filtered value
        self._reject_n  = 0            # consecutive spike rejections
        self._persist   = {}           # angle(int) -> remaining sweeps visible
        self.noise_floor = 0.0

    # ── noise floor calibration ───────────────────────────────────────────────

    def calibrate_noise_floor(self, read_fn, samples=20, log=print):
        """Sample the sensor with nothing in front to learn the baseline.

        read_fn() must return a raw distance in cm.
        """
        log("[filter] calibrating noise floor — keep the area clear...")
        vals = []
        for _ in range(samples):
            d = read_fn()
            if d and d > 0:
                vals.append(d)
            time.sleep(READING_GAP_S)
        if vals:
            # how far the quietest baseline sits from max range
            self.noise_floor = max(0.0, self.max_range - float(np.median(vals)))
            log(f"[filter] noise floor = {self.noise_floor:.1f} cm "
                f"(baseline median {np.median(vals):.1f} cm)")
        else:
            log("[filter] noise floor calibration got no valid reads")
        return self.noise_floor

    # ── helpers ────────────────────────────────────────────────────────────────

    def _in_range(self, raw):
        """Range / error rejection. Returns True if the raw value is usable."""
        if raw is None or raw == 0:          # 0 = common HC-SR04 error code
            return False
        if raw < MIN_DISTANCE:               # too close = sensor error
            return False
        if raw > self.max_range:             # beyond range = treat as clear
            return False
        # within noise floor of max range → effectively clear
        if self.noise_floor and raw >= self.max_range - self.noise_floor:
            return False
        return True

    def sample_and_read(self, read_fn, angle=None):
        """Take READINGS_PER_POS hardware reads (10 ms apart) then filter them."""
        raws = []
        for _ in range(READINGS_PER_POS):
            raws.append(read_fn())
            time.sleep(READING_GAP_S)
        # feed each into read(); the last result reflects the filtered state
        result = None
        for r in raws:
            result = self.read(r, angle=angle, _batch=True)
        return result

    # ── main entry ──────────────────────────────────────────────────────────────

    def read(self, raw_cm, angle=None, _batch=False):
        """Filter one raw reading. Returns a dict with distance/zone/valid/persistent."""
        valid_raw = self._in_range(raw_cm)
        spike_rejected = False

        if valid_raw:
            self._window.append(float(raw_cm))
            median = float(np.median(self._window))

            # spike rejection vs the last accepted value
            if self._last is not None and abs(median - self._last) > self.max_jump:
                self._reject_n += 1
                if self._reject_n < 3:
                    spike_rejected = True
                    median = self._last         # hold previous
                else:
                    self._reject_n = 0          # 3 in a row → genuine change
            else:
                self._reject_n = 0

            # exponential moving average
            if self._last is None:
                filtered = median
            else:
                filtered = self.alpha * median + (1.0 - self.alpha) * self._last
            self._last = filtered
            present = True
        else:
            # no usable echo this reading
            filtered = 0.0
            present = False

        zone = classify_zone(filtered if present else None)

        # ── per-angle persistence ─────────────────────────────────────────────
        persistent = False
        if angle is not None:
            a = int(round(angle))
            if present and zone != "CLEAR":
                self._persist[a] = PERSIST_SWEEPS
            cnt = self._persist.get(a, 0)
            persistent = cnt > 0

        if self.debug and not _batch:
            print(f"[filter] raw={raw_cm}  win={list(self._window)}  "
                  f"med={float(np.median(self._window)) if self._window else 0:.0f}  "
                  f"ema={filtered:.1f}  spike={'Y' if spike_rejected else 'N'}  "
                  f"zone={zone}  persist={'Y' if persistent else 'N'}")

        return {
            "distance":   round(filtered, 1) if present else 0.0,
            "zone":       zone,
            "valid":      present,
            "persistent": persistent,
            "spike":      spike_rejected,
        }

    def tick_sweep(self):
        """Call once at the end of every sweep to age persistence counters."""
        for a in list(self._persist.keys()):
            self._persist[a] -= 1
            if self._persist[a] <= 0:
                del self._persist[a]

    def reset(self):
        self._window.clear()
        self._last = None
        self._reject_n = 0
        self._persist.clear()


# ── self-test ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    f = UltrasonicFilter(debug=True)
    print("\nFeeding 42cm wall with a 180cm spike injected:")
    for raw in [42, 41, 180, 43, 42, 42, 41, 43]:
        f.read(raw, angle=90)
    print("\nObject vanishes (0/None readings):")
    for raw in [0, 0, 0]:
        print("  ", f.read(raw, angle=90))
