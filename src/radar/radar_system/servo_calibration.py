"""
servo_calibration.py — STEP 1: servo calibration.

Finds the exact servo positions so radar dots appear at the correct angles,
and measures the servo's sweep speed so the UI can stay in sync.

    python3 servo_calibration.py        # run interactively first

Writes calibration.json. Every other module calls load_calibration() at
startup and refuses to run unless calibrated == true.

NOTE: the RASPBOT V2 pan servo is OPEN-LOOP (no position encoder), so the
"actual angle" at each step is reported by you, the operator, watching the
servo. The script automates the sweep + timing; you confirm the angles.
"""

import os
import sys
import json
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from constants import (
    CALIBRATION_FILE, SRC_DIR, PAN_CENTER, ARC_DEG, SERVO_SPEED_DPS_DEFAULT,
)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import numpy as np

KEY_ANGLES = [0, 20, 45, 67, 90, 112, 135, 160, 180]


# ── shared loader (used by every other module) ───────────────────────────────

def load_calibration(require=True):
    """Load calibration.json. Returns the dict, or None if missing/invalid.

    If require=True and the file is missing or calibrated=false, returns None
    so callers can refuse to run.
    """
    if not os.path.exists(CALIBRATION_FILE):
        return None
    try:
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
    except Exception:
        return None
    if require and not data.get("calibrated", False):
        return None
    # rebuild a numpy interpolator for corrections
    return data


def correct_angle(data, commanded):
    """Apply the calibration correction table to a commanded angle."""
    if not data:
        return commanded
    table = data.get("angle_corrections", {})
    if not table:
        return commanded + data.get("center_offset", 0.0)
    xs = sorted(int(k) for k in table.keys())
    ys = [float(table[str(k)]) for k in xs]
    actual = float(np.interp(commanded, xs, ys))
    return actual


# ── interactive helpers ───────────────────────────────────────────────────────

def _ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return ""


def _ask_yes(prompt):
    return _ask(prompt + " (y/n): ").startswith("y")


# ── calibration phases ────────────────────────────────────────────────────────

def warmup(bot):
    print("Warming up servo... please wait")
    for ang in (0, 180, PAN_CENTER):
        bot.set_pan(ang)
        time.sleep(2.0)


def detect_limits(bot):
    """Sweep 0→180 in 1° steps; operator confirms the usable travel limits."""
    print("\n--- DEAD-ZONE / LIMIT DETECTION ---")
    print("Watch the servo. It will sweep slowly from 0° to 180°.")
    for ang in range(0, 181, 1):
        bot.set_pan(ang)
        time.sleep(0.02)
    left = 0
    right = 180
    if not _ask_yes("Did the servo reach the FAR-LEFT cleanly at 0°?"):
        try:
            left = int(_ask("Enter the lowest angle it actually reaches: ") or "0")
        except ValueError:
            left = 0
    if not _ask_yes("Did the servo reach the FAR-RIGHT cleanly at 180°?"):
        try:
            right = int(_ask("Enter the highest angle it actually reaches: ") or "180")
        except ValueError:
            right = 180
    print(f"LEFT_LIMIT  = {left}")
    print(f"RIGHT_LIMIT = {right}")
    return left, right


def angle_error_mapping(bot):
    """For 9 key angles, record the actual pointing angle vs commanded."""
    print("\n--- ANGLE ERROR MAPPING ---")
    corrections = {}
    for ang in KEY_ANGLES:
        bot.set_pan(ang)
        time.sleep(0.6)
        if _ask_yes(f"Is the servo pointing at {ang}°?"):
            corrections[str(ang)] = float(ang)
        else:
            try:
                actual = float(_ask(f"What angle is it ACTUALLY at? "))
            except ValueError:
                actual = float(ang)
            corrections[str(ang)] = actual
            print(f"  offset @ {ang}° = {actual - ang:+.1f}°")
    return corrections


def center_offset(bot):
    print("\n--- CENTER OFFSET ---")
    bot.set_pan(PAN_CENTER)
    time.sleep(0.6)
    if _ask_yes("Is the servo pointing STRAIGHT AHEAD at 90°?"):
        return 0.0
    try:
        actual = float(_ask("What angle is straight-ahead actually commanded as? "))
    except ValueError:
        actual = float(PAN_CENTER)
    off = actual - PAN_CENTER
    print(f"CENTER_OFFSET = {off:+.1f}°")
    return off


def speed_calibration(bot):
    """Measure degrees-per-second by timing a full left→right travel."""
    print("\n--- SPEED CALIBRATION ---")
    bot.set_pan(0)
    time.sleep(1.0)
    print("Press ENTER, watch the servo, then press ENTER again the INSTANT")
    print("it stops at the far right.")
    _ask("Ready? press ENTER to start the 0→180 move...")
    t0 = time.time()
    bot.set_pan(180)
    _ask("press ENTER when it has fully stopped at 180°...")
    elapsed = max(0.05, time.time() - t0)
    dps = 180.0 / elapsed
    print(f"SERVO_SPEED_DPS = {dps:.0f} (180° in {elapsed:.2f}s)")
    return dps


def verify(bot, data):
    """Sweep the full arc with corrections applied; operator confirms."""
    print("\n--- VERIFICATION ---")
    left = data["left_limit"]
    right = data["right_limit"]
    for ang in range(left, right + 1, 1):
        bot.set_pan(int(correct_angle(data, ang)))
        time.sleep(0.015)
        if ang % 30 == 0:
            print(f"  commanded {ang:3d}° → corrected "
                  f"{correct_angle(data, ang):5.1f}°")
    return _ask_yes("Does the sweep look accurate?")


# ── save ────────────────────────────────────────────────────────────────────

def save_calibration(data):
    data["calibration_date"] = datetime.now().isoformat(timespec="seconds")
    data["calibrated"] = True
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\nSaved → {CALIBRATION_FILE}")


# ── orchestration ─────────────────────────────────────────────────────────────

def run_calibration():
    print("=" * 60)
    print("SERVO CALIBRATION MODE — Yahboom RASPBOT V2")
    print("This will take ~3 minutes. Follow instructions.")
    print("=" * 60)

    from setup_and_api.api import RasBot

    with RasBot() as bot:
        while True:
            warmup(bot)
            left, right = detect_limits(bot)
            corrections = angle_error_mapping(bot)
            offset = center_offset(bot)
            dps = speed_calibration(bot)

            data = {
                "left_limit":        left,
                "right_limit":       right,
                "center_offset":     offset,
                "angle_corrections": corrections,
                "servo_speed_dps":   dps,
                "calibrated":        True,
            }
            save_calibration(data)

            if verify(bot, data):
                print("\nCalibration complete and verified ✔")
                bot.set_pan(PAN_CENTER)
                return data
            print("\nVerification failed — restarting calibration...\n")


def main():
    if SRC_DIR not in sys.path:
        sys.path.insert(0, SRC_DIR)
    try:
        run_calibration()
    except KeyboardInterrupt:
        print("\nCalibration aborted.")


if __name__ == "__main__":
    main()
