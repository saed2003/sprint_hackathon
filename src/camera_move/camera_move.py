"""
Camera pan/tilt controller for the RasBot servos.

Tap an arrow key to nudge the camera. The tilt servo (up/down) and pan servo
(left/right) move in fixed steps between their limits. Runs over bare SSH on the
Pi (single-key reading via termios, no desktop / pygame needed).

On start it runs a short self-test sweep so you can SEE the servos move. If
nothing moves during the self-test, the problem is wiring/firmware/power, not
this script.

Controls (tap, no Enter needed)
  Up / Down arrow      tilt camera up / down
  Left / Right arrow   pan camera left / right
  c                    re-center (back to defaults)
  q                    quit (also ESC or Ctrl-C)

Run on the Pi:
  python3 camera_move/camera_move.py
"""

import os
import sys
import time

# project root = the folder that contains camera_move/, wasd/, setup_and_api/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot
from setup_and_api.api.constants import (
    TILT_MIN, TILT_MAX, TILT_DEFAULT,
    PAN_MIN, PAN_MAX, PAN_DEFAULT,
)

# ── tunables ────────────────────────────────────────────────────────────────
TILT_STEP = 10          # degrees per Up/Down tap
PAN_STEP  = 10          # degrees per Left/Right tap

# arrow-key escape sequences
KEY_UP    = "\x1b[A"
KEY_DOWN  = "\x1b[B"
KEY_RIGHT = "\x1b[C"
KEY_LEFT  = "\x1b[D"


def _read_key() -> str:
    """Read one keypress (handles arrow escape sequences) without Enter."""
    import termios
    import tty
    import select

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # arrow keys arrive as ESC + '[' + letter; grab the rest if present
        if ch == "\x1b" and select.select([sys.stdin], [], [], 0.001)[0]:
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def self_test(bot) -> None:
    """Sweep both servos so the user can confirm the hardware moves."""
    print("Self-test: sweeping servos (watch the camera)...")
    for a in (TILT_DEFAULT + 25, TILT_DEFAULT - 15, TILT_DEFAULT):
        bot.set_tilt(_clamp(a, TILT_MIN, TILT_MAX))
        time.sleep(0.4)
    for a in (PAN_DEFAULT + 30, PAN_DEFAULT - 30, PAN_DEFAULT):
        bot.set_pan(_clamp(a, PAN_MIN, PAN_MAX))
        time.sleep(0.4)
    print("Self-test done.")


def main():
    print("Connecting to robot board...")
    with RasBot() as bot:
        self_test(bot)

        pan, tilt = PAN_DEFAULT, TILT_DEFAULT
        bot.set_pan(pan)
        bot.set_tilt(tilt)
        print("Ready. Arrows = move   c = center   q = quit")
        print(f"pan: {pan}    tilt: {tilt}")

        while True:
            key = _read_key()

            if key in ("q", "\x1b", "\x03"):          # q, lone ESC, Ctrl-C
                break
            elif key == KEY_UP:
                tilt = _clamp(tilt + TILT_STEP, TILT_MIN, TILT_MAX)
                bot.set_tilt(tilt)
            elif key == KEY_DOWN:
                tilt = _clamp(tilt - TILT_STEP, TILT_MIN, TILT_MAX)
                bot.set_tilt(tilt)
            elif key == KEY_LEFT:
                pan = _clamp(pan - PAN_STEP, PAN_MIN, PAN_MAX)
                bot.set_pan(pan)
            elif key == KEY_RIGHT:
                pan = _clamp(pan + PAN_STEP, PAN_MIN, PAN_MAX)
                bot.set_pan(pan)
            elif key == "c":
                pan, tilt = PAN_DEFAULT, TILT_DEFAULT
                bot.set_pan(pan)
                bot.set_tilt(tilt)
            else:
                continue

            print(f"pan: {pan}    tilt: {tilt}")

    print("\nStopped. Camera re-centered.")


if __name__ == "__main__":
    main()
