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

Capture
  R           run a 360 capture: rotate in place and take 9 shots (40 deg each)
              -> captures/scan_<ts>/   (this only captures — it does NOT build the cloud)
  T           build the 3D point cloud from the last R capture, on the Pi
              -> captures/scan_<ts>/merged_360.ply
  Y           open the 3D point cloud in a window on the Pi screen (orbit with the mouse)
  V           take a single capture in place -> captures/<timestamp>/

Other
  Space       stop motors immediately
  ESC / Ctrl-C  quit safely

Run from sprint_hackathon/:
  python3 wasd/drive.py
"""

import sys
import os
import select
import termios
import tty
import time
import subprocess

# project root = the folder that contains wasd/, camera/, pointcloud/, rasbot/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color
from camera.rs_capture import StereoCapture
from pointcloud import scan360

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
        '  (R=capture  T=build cloud  Y=3D view  V=single  ESC=quit)'
    )
    sys.stdout.flush()


def _line(msg):
    """Print a full line while the terminal is in raw mode (needs explicit \\r\\n)."""
    sys.stdout.write(f'\r\033[K{msg}\r\n')
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
        cam    = None          # RealSense D405, opened lazily on first 'c'
        last_session = None    # folder from the last R capture (input for T)
        last_ply     = None    # cloud built by T (input for Y = 3D view)

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

                # ── R: 360 capture only — rotate and take 9 shots (build later with T) ──
                elif k == 'r':
                    bot.stop()
                    _status('360 capture', speed, pan, tilt)
                    try:
                        if cam is None:
                            cam = StereoCapture()
                        bot.set_all_leds_color(Color.BLUE)
                        _line('--- starting 360 capture: 9 shots, 40 deg each '
                              '(do not touch the robot) ---')
                        last_session = scan360.run_scan(bot, cam, log=_line)
                        last_ply = None
                        bot.set_all_leds_color(Color.GREEN)
                        bot.beep(0.15)
                        _line(f'--- capture done -> {last_session}  '
                              '(press T to build the cloud) ---')
                        action = f'captured {os.path.basename(last_session)}'
                    except Exception as e:
                        bot.stop()
                        bot.set_all_leds_color(Color.RED)
                        _line(f'  capture error: {e}')
                        action = 'capture error'

                # ── T: build the point cloud from the last R capture (on the Pi) ──
                elif k == 't':
                    bot.stop()
                    if last_session is None:
                        _line('  no capture yet — press R first to take the 9 shots')
                        action = 'no capture'
                    else:
                        _status('building cloud', speed, pan, tilt)
                        try:
                            bot.set_all_leds_color(Color.BLUE)
                            _line(f'--- building cloud from '
                                  f'{os.path.basename(last_session)} ---')
                            last_ply = scan360.build_from_session(last_session, log=_line)
                            bot.set_all_leds_color(Color.GREEN)
                            bot.beep(0.15)
                            _line(f'--- cloud ready -> {last_ply}  '
                                  '(press Y for the 3D view) ---')
                            action = 'cloud built'
                        except Exception as e:
                            bot.set_all_leds_color(Color.RED)
                            _line(f'  build error: {e}')
                            action = 'build error'

                # ── Y: open the 3D point-cloud viewer on the Pi screen ──
                elif k == 'y':
                    bot.stop()
                    if last_ply is None or not os.path.exists(last_ply):
                        _line('  no cloud yet — press R to capture, then T to build it')
                        action = 'no cloud'
                    else:
                        _line(f'--- opening 3D view: {last_ply}  '
                              '(orbit with the mouse; ESC closes it) ---')
                        try:
                            root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                            subprocess.Popen(
                                [sys.executable, os.path.join(root, 'pointcloud', 'view3d.py'), last_ply])
                            action = '3D view open'
                        except Exception as e:
                            _line(f'  3D view error: {e}')
                            action = '3D view error'

                # ── single capture in place (handy for testing) ──────────────
                elif k == 'v':
                    bot.stop()
                    _status('capturing', speed, pan, tilt)
                    try:
                        if cam is None:
                            cam = StereoCapture()
                        bot.set_all_leds_color(Color.BLUE)
                        folder = cam.save()
                        bot.set_all_leds_color(Color.GREEN)
                        if folder is None:
                            _line('  capture dropped a frame — try again')
                            action = 'capture failed'
                        else:
                            bot.beep(0.1)
                            _line(f'  saved -> {folder}')
                            action = f'saved {os.path.basename(folder)}'
                    except Exception as e:
                        bot.set_all_leds_color(Color.RED)
                        _line(f'  capture error: {e}')
                        action = 'capture error'

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
            if cam is not None:
                cam.close()
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            time.sleep(0.3)
            bot.leds_off()
            print('\n\nStopped. Robot is safe.')


if __name__ == '__main__':
    main()
