"""
WASD keyboard controller for the RasbotV2 Mecanum robot.

Movement (hold key = keep moving, release = stop)
  W / S       forward / backward
  A / D       strafe left / right
  Q / E       rotate counter-clockwise / clockwise

Camera servo
  Arrow UP / DOWN    tilt camera up / down
  Arrow LEFT / RIGHT pan camera left / right

Speed
  + or =      speed up  (step 20, max 255)
  -           slow down (step 20, min 40)

Other
  Space       stop motors immediately
  ESC / Ctrl-C  quit safely

Run from sprint_hackathon/:
  python3 drive.py
"""

import sys
import os
import select
import termios
import tty
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rasbot.api import RasBot, Color

# ── tunables ──────────────────────────────────────────────────────────────────
SPEED_DEFAULT = 120
SPEED_STEP    = 20
SPEED_MIN     = 40
SPEED_MAX     = 255
KEY_TIMEOUT   = 0.15   # seconds — motors stop if no key arrives within this window
PAN_STEP      = 10
TILT_STEP     = 10

# ── key tokens ────────────────────────────────────────────────────────────────
ESC    = '\x1b'
CTRL_C = '\x03'
SPACE  = ' '


def _read_key():
    """Read one keypress from raw stdin. Returns a string token."""
    ch = sys.stdin.read(1)
    if ch == ESC:
        # arrow key = ESC [ A/B/C/D — consume if present within 50 ms
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r2:
                    ch3 = sys.stdin.read(1)
                    return {'A': 'UP', 'B': 'DOWN',
                            'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, ESC)
        return ESC
    return ch


def _status(action, speed, pan, tilt):
    sys.stdout.write(
        f'\r\033[K  [{action:<14}]  speed={speed:3d}  pan={pan:3d}  tilt={tilt:3d}'
        '  (ESC/Ctrl-C = quit)'
    )
    sys.stdout.flush()


def main():
    print(__doc__)
    print('Connecting to robot board...')

    with RasBot() as bot:
        bot.set_all_leds_color(Color.GREEN)
        bot.beep(0.1)
        print('Connected. Robot is ready.\n')

        speed  = SPEED_DEFAULT
        pan    = 90
        tilt   = 25
        action = 'stopped'

        fd          = sys.stdin.fileno()
        old_termios = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)

            while True:
                # ── wait for a keypress (hold-to-move: timeout = stop) ────────
                readable, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)

                if not readable:
                    bot.stop()
                    action = 'stopped'
                    _status(action, speed, pan, tilt)
                    continue

                key = _read_key()

                # ── quit ──────────────────────────────────────────────────────
                if key in (CTRL_C, ESC):
                    break

                k = key.lower()

                # ── movement ──────────────────────────────────────────────────
                if k == 'w':
                    bot.forward(speed);      action = 'forward'
                elif k == 's':
                    bot.backward(speed);     action = 'backward'
                elif k == 'a':
                    bot.left(speed);         action = 'strafe left'
                elif k == 'd':
                    bot.right(speed);        action = 'strafe right'
                elif k == 'q':
                    bot.rotate_left(speed);  action = 'rotate CCW'
                elif k == 'e':
                    bot.rotate_right(speed); action = 'rotate CW'
                elif key == SPACE:
                    bot.stop();              action = 'stopped'

                # ── speed ─────────────────────────────────────────────────────
                elif key in ('+', '='):
                    speed  = min(speed + SPEED_STEP, SPEED_MAX)
                    action = f'speed={speed}'
                elif key == '-':
                    speed  = max(speed - SPEED_STEP, SPEED_MIN)
                    action = f'speed={speed}'

                # ── camera servos (arrow keys) ────────────────────────────────
                elif key == 'UP':
                    tilt = min(tilt + TILT_STEP, 100)
                    bot.set_tilt(tilt);  action = f'tilt up  ({tilt})'
                elif key == 'DOWN':
                    tilt = max(tilt - TILT_STEP, 0)
                    bot.set_tilt(tilt);  action = f'tilt down ({tilt})'
                elif key == 'LEFT':
                    pan = max(pan - PAN_STEP, 0)
                    bot.set_pan(pan);    action = f'pan left ({pan})'
                elif key == 'RIGHT':
                    pan = min(pan + PAN_STEP, 180)
                    bot.set_pan(pan);    action = f'pan right ({pan})'

                _status(action, speed, pan, tilt)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_termios)
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            time.sleep(0.3)
            bot.leds_off()
            print('\n\nStopped. Robot is safe.')


if __name__ == '__main__':
    main()
