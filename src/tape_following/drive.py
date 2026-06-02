"""
RasBot controller — WASD manual drive + F to toggle line following.

HOLD a movement key to move, RELEASE to stop (real key-up detection).
Needs a desktop/display: run inside the Pi desktop over VNC, not bare SSH.
The control window must have keyboard FOCUS (click it) to receive keys.

Install pygame on the Pi once:
    sudo apt install -y python3-pygame

Movement (hold = move, release = stop; combine keys for diagonals)
  W / S       forward / backward
  A / D       strafe left / right
  Q / E       rotate counter-clockwise / clockwise

Line following
  F           toggle autonomous line following on/off

Camera servo (tap)
  Arrow UP / DOWN     tilt camera up / down
  Arrow LEFT / RIGHT  pan camera left / right

Speed (tap)
  + or =      speed up   (step 20, max 255)
  -           slow down  (step 20, min 40)

Capture (tap)
  R   360 capture  →  captures/scan_<ts>/
  T   build 3D cloud from last R capture
  Y   open 3D cloud viewer
  V   single capture in place

Quit          ESC or close the window

Run inside the VNC desktop:
  python3 src/tape_following/drive.py
"""

import os
import sys
import math
import time
import threading
import subprocess

# project root = the folder that contains tape_following/, camera/, pointcloud/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color
from camera.rs_capture import StereoCapture
from pointcloud import scan360
from tape_following import line_follow as lf

import pygame

# ── tunables ────────────────────────────────────────────────────────────────
SPEED_DEFAULT = 120
SPEED_STEP    = 20
SPEED_MIN     = 40
SPEED_MAX     = 255
PAN_STEP      = 10
TILT_STEP     = 10
FPS           = 60
# ─────────────────────────────────────────────────────────────────────────────

_TRANS = {pygame.K_w: (0, 1), pygame.K_s: (0, -1),
          pygame.K_d: (1, 0), pygame.K_a: (-1, 0)}
_ROT   = {pygame.K_q: +1, pygame.K_e: -1}


def desired_command(pressed, speed):
    vx  = sum(d[0] for k, d in _TRANS.items() if pressed[k])
    vy  = sum(d[1] for k, d in _TRANS.items() if pressed[k])
    rot = sum(v for k, v in _ROT.items() if pressed[k])
    if vx or vy:
        angle = round(math.degrees(math.atan2(vy, vx)))
        return ('move', angle, speed)
    if rot > 0:
        return ('rotate_left', speed)
    if rot < 0:
        return ('rotate_right', speed)
    return ('stop',)


def apply_command(bot, cmd):
    kind = cmd[0]
    if kind == 'move':
        bot.move(cmd[2], cmd[1])
    elif kind == 'rotate_left':
        bot.rotate_left(cmd[1])
    elif kind == 'rotate_right':
        bot.rotate_right(cmd[1])
    else:
        bot.stop()


def command_label(cmd):
    if cmd[0] == 'move':        return f'move {cmd[1]:>4}deg'
    if cmd[0] == 'rotate_left':  return 'rotate CCW'
    if cmd[0] == 'rotate_right': return 'rotate CW'
    return 'stopped'


def _ensure_cam(cam):
    return cam if cam is not None else StereoCapture()


def draw_hud(screen, font, action, speed, pan, tilt, follow_mode):
    screen.fill((18, 18, 22))
    follow_color = (80, 200, 255) if follow_mode else (120, 220, 120)
    follow_label = '*** LINE FOLLOWING (F=stop) ***' if follow_mode else 'manual drive'
    lines = [
        (f'RasBot  —  {follow_label}',                           follow_color),
        (f'action: {action}',                                     (235, 235, 235)),
        (f'speed: {speed}    pan: {pan}    tilt: {tilt}',        (200, 200, 210)),
        ('W/A/S/D move   Q/E rotate   F=follow   +/- speed',     (150, 150, 160)),
        ('R capture   T build   Y view   V single   ESC quit',   (150, 150, 160)),
        ('(click this window so it has keyboard focus)',           (120, 120, 130)),
    ]
    y = 14
    for text, color in lines:
        screen.blit(font.render(text, True, color), (16, y))
        y += 28
    pygame.display.flip()


def main():
    try:
        pygame.init()
        screen = pygame.display.set_mode((480, 210))
        pygame.display.set_caption('RasBot — WASD + line follow')
    except Exception as e:
        sys.exit('pygame could not open a window. Run INSIDE the Pi desktop (VNC).\n  ' + str(e))
    font  = pygame.font.SysFont('monospace', 16)
    clock = pygame.time.Clock()

    print('Connecting to robot board...')
    with RasBot() as bot:
        bot.set_all_leds_color(Color.GREEN)
        bot.beep(0.1)
        print('Connected. Click the window, then hold W/A/S/D to drive.  F = line follow.')

        speed        = SPEED_DEFAULT
        pan, tilt    = 90, 25
        cam          = None
        last_session = None
        last_ply     = None
        last_cmd     = None
        action       = 'stopped'
        running      = True

        # ── line-follow state ──────────────────────────────────────────────
        follow_stop   = threading.Event()
        follow_thread = None
        follow_mode   = False

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                    elif event.type == pygame.KEYDOWN:
                        key = event.key
                        if key == pygame.K_ESCAPE:
                            running = False

                        # ── F: toggle line following ───────────────────────
                        elif key == pygame.K_f:
                            follow_mode = not follow_mode
                            if follow_mode:
                                bot.stop(); last_cmd = ('stop',)
                                follow_stop.clear()
                                follow_thread = threading.Thread(
                                    target=lf.run,
                                    args=(bot,),
                                    kwargs={'stop_event': follow_stop},
                                    daemon=True,
                                )
                                follow_thread.start()
                                action = 'LINE FOLLOWING'
                                bot.set_all_leds_color(Color.BLUE)
                                print('--- line following started (F to stop) ---')
                            else:
                                follow_stop.set()
                                if follow_thread:
                                    follow_thread.join(timeout=3.0)
                                bot.stop(); last_cmd = ('stop',)
                                bot.set_all_leds_color(Color.GREEN)
                                action = 'stopped (manual)'
                                print('--- line following stopped ---')

                        # ── speed (only in manual mode) ────────────────────
                        elif not follow_mode:
                            if key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                                speed = min(speed + SPEED_STEP, SPEED_MAX)
                            elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                                speed = max(speed - SPEED_STEP, SPEED_MIN)

                            # ── camera servos ─────────────────────────────
                            elif key == pygame.K_UP:
                                tilt = min(tilt + TILT_STEP, 100); bot.set_tilt(tilt)
                            elif key == pygame.K_DOWN:
                                tilt = max(tilt - TILT_STEP, 0);   bot.set_tilt(tilt)
                            elif key == pygame.K_LEFT:
                                pan  = max(pan  - PAN_STEP,   0);   bot.set_pan(pan)
                            elif key == pygame.K_RIGHT:
                                pan  = min(pan  + PAN_STEP, 180);   bot.set_pan(pan)

                            # ── R: 360 capture ────────────────────────────
                            elif key == pygame.K_r:
                                bot.stop(); last_cmd = ('stop',)
                                cam = _ensure_cam(cam)
                                bot.set_all_leds_color(Color.BLUE)
                                draw_hud(screen, font, '360 capture...', speed, pan, tilt, False)
                                try:
                                    print(f'--- 360 capture: {scan360.SCAN_SHOTS} shots ---')
                                    last_session = scan360.run_scan(bot, cam, log=print)
                                    last_ply = None
                                    bot.set_all_leds_color(Color.GREEN); bot.beep(0.15)
                                    action = f'captured {os.path.basename(last_session)}'
                                    print(f'--- done -> {last_session} ---')
                                except Exception as e:
                                    bot.stop(); bot.set_all_leds_color(Color.RED)
                                    action = 'capture error'; print('capture error:', e)
                                pygame.event.clear()

                            # ── T: build cloud ────────────────────────────
                            elif key == pygame.K_t:
                                bot.stop(); last_cmd = ('stop',)
                                if last_session is None:
                                    action = 'no capture yet (press R)'
                                else:
                                    bot.set_all_leds_color(Color.BLUE)
                                    draw_hud(screen, font, 'building cloud...', speed, pan, tilt, False)
                                    try:
                                        last_ply = scan360.build_from_session(last_session, log=print)
                                        bot.set_all_leds_color(Color.GREEN); bot.beep(0.15)
                                        action = 'cloud built'
                                        print(f'--- cloud -> {last_ply} ---')
                                    except Exception as e:
                                        bot.set_all_leds_color(Color.RED)
                                        action = 'build error'; print('build error:', e)
                                pygame.event.clear()

                            # ── Y: open viewer ────────────────────────────
                            elif key == pygame.K_y:
                                if last_ply and os.path.exists(last_ply):
                                    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                                    subprocess.Popen([sys.executable,
                                                      os.path.join(root, 'pointcloud', 'view3d.py'),
                                                      last_ply])
                                    action = '3D view open'
                                else:
                                    action = 'no cloud yet (R then T)'

                            # ── V: single capture ─────────────────────────
                            elif key == pygame.K_v:
                                bot.stop(); last_cmd = ('stop',)
                                cam = _ensure_cam(cam)
                                bot.set_all_leds_color(Color.BLUE)
                                draw_hud(screen, font, 'capturing...', speed, pan, tilt, False)
                                try:
                                    folder = cam.save()
                                    bot.set_all_leds_color(Color.GREEN)
                                    if folder is None:
                                        action = 'capture dropped a frame'
                                    else:
                                        bot.beep(0.1)
                                        action = f'saved {os.path.basename(folder)}'
                                        print('saved ->', folder)
                                except Exception as e:
                                    bot.set_all_leds_color(Color.RED)
                                    action = 'capture error'; print('capture error:', e)
                                pygame.event.clear()

                # ── held-key motion (manual mode only) ────────────────────
                if not follow_mode:
                    pressed = pygame.key.get_pressed()
                    cmd = desired_command(pressed, speed)
                    if cmd != last_cmd:
                        apply_command(bot, cmd)
                        last_cmd = cmd
                        action   = command_label(cmd)

                draw_hud(screen, font, action, speed, pan, tilt, follow_mode)
                clock.tick(FPS)

        finally:
            follow_stop.set()
            if follow_thread and follow_thread.is_alive():
                follow_thread.join(timeout=2.0)
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            time.sleep(0.3)
            bot.leds_off()
            if cam is not None:
                cam.close()
            pygame.quit()
            print('\nStopped. Robot is safe.')


if __name__ == '__main__':
    main()
