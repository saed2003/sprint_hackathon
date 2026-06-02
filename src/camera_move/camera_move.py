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
  q                    quit (also Ctrl-C)

Run on the Pi:
  python3 camera_move/camera_move.py
"""

import os
import sys
import time
import select
import termios
import tty
import contextlib

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
DEBUG     = bool(os.environ.get("CAM_DEBUG"))   # CAM_DEBUG=1 prints raw key bytes

# arrow-key escape sequences
KEY_UP    = "\x1b[A"
KEY_DOWN  = "\x1b[B"
KEY_RIGHT = "\x1b[C"
KEY_LEFT  = "\x1b[D"


@contextlib.contextmanager
def raw_mode(fd):
    """Put the terminal in raw mode for the whole session (set once)."""
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_key() -> str:
    """Read one keypress, assembling arrow escape sequences if present."""
    ch = sys.stdin.read(1)
    if ch == "\x1b":                                   # maybe an arrow sequence
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch += sys.stdin.read(1)                     # '['
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch += sys.stdin.read(1)                 # 'A'/'B'/'C'/'D'
    return ch


def out(msg: str) -> None:
    """Print a line while the terminal is in raw mode (needs explicit CR)."""
    sys.stdout.write(msg + "\r\n")
    sys.stdout.flush()


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

        with raw_mode(sys.stdin.fileno()):
            while True:
                key = read_key()
                if DEBUG:
                    out(f"key={key!r}")

                if key in ("q", "\x03", "\x04", ""):    # q, Ctrl-C, Ctrl-D, EOF
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

                out(f"pan: {pan}    tilt: {tilt}")

    print("\nStopped. Camera re-centered.")


if __name__ == "__main__":
    main()
