"""
Camera up/down controller for the RasBot tilt servo.

Tap a key to nudge the camera; the tilt servo moves in TILT_STEP increments
between TILT_MIN and TILT_MAX. Runs over bare SSH on the Pi (single-key reading
via termios, no desktop / pygame needed).

Controls (tap, no Enter needed)
  u     tilt camera up
  d     tilt camera down
  c     re-center (back to default tilt)
  q     quit (also Ctrl-C or ESC)

Run on the Pi:
  python3 camera_move/camera_move.py
"""

import os
import sys

# project root = the folder that contains camera_move/, wasd/, setup_and_api/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot
from setup_and_api.api.constants import TILT_MIN, TILT_MAX, TILT_DEFAULT

# ── tunables ────────────────────────────────────────────────────────────────
TILT_STEP = 10          # degrees per key tap


def _read_key() -> str:
    """Read a single keypress without waiting for Enter (Linux/Pi terminal)."""
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def main():
    print("Connecting to robot board...")
    with RasBot() as bot:
        tilt = TILT_DEFAULT
        bot.set_tilt(tilt)
        print("Connected. Tap  u = up   d = down   c = center   q = quit")
        print(f"tilt: {tilt}")

        while True:
            key = _read_key().lower()

            if key in ("q", "\x1b", "\x03"):      # q, ESC, Ctrl-C
                break
            elif key == "u":
                tilt = _clamp(tilt + TILT_STEP, TILT_MIN, TILT_MAX)
                bot.set_tilt(tilt)
            elif key == "d":
                tilt = _clamp(tilt - TILT_STEP, TILT_MIN, TILT_MAX)
                bot.set_tilt(tilt)
            elif key == "c":
                tilt = TILT_DEFAULT
                bot.set_tilt(tilt)
            else:
                continue

            print(f"tilt: {tilt}")

    print("\nStopped. Camera re-centered.")


if __name__ == "__main__":
    main()
