"""
WASD teleop for the RasbotV2 Mecanum robot (pygame). HOLD a key to move, release to stop.
Needs a display, so run it inside the Pi desktop over VNC (not bare SSH), and click the
window so it has keyboard focus (losing focus stops the robot).

Keys: W/A/S/D move, Q/E rotate, arrows pan/tilt camera, +/- speed,
      R 360 capture, T build cloud, Y view cloud, V single capture, ESC quit.

  python3 wasd/drive.py
"""

import os
import sys
import math
import time
import subprocess

# project root on sys.path so `from camera...`, `from pointcloud...` resolve
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color
from camera.rs_capture import StereoCapture
from pointcloud import scan360

import pygame

SPEED_DEFAULT = 120
SPEED_STEP    = 20
SPEED_MIN     = 40
SPEED_MAX     = 255
PAN_STEP      = 10
TILT_STEP     = 10
FPS           = 60

# held key -> translation (vx: right +, vy: forward +) and rotation (+1 CCW, -1 CW)
_TRANS = {pygame.K_w: (0, 1), pygame.K_s: (0, -1),
          pygame.K_d: (1, 0), pygame.K_a: (-1, 0)}
_ROT = {pygame.K_q: +1, pygame.K_e: -1}


def desired_command(pressed, speed):
    """Held keys -> motion command tuple. Translation (W/A/S/D, blended) beats rotation
    (Q/E); speed rides along so the caller resends only when something changes."""
    vx = sum(d[0] for k, d in _TRANS.items() if pressed[k])
    vy = sum(d[1] for k, d in _TRANS.items() if pressed[k])
    rot = sum(v for k, v in _ROT.items() if pressed[k])      # cancels if Q+E both held
    if vx or vy:
        # bot.move() angle: 0=right, 90=forward, 180=left, 270=back
        return ('move', round(math.degrees(math.atan2(vy, vx))), speed)
    if rot > 0:
        return ('rotate_left', speed)
    if rot < 0:
        return ('rotate_right', speed)
    return ('stop',)


def apply_command(bot, cmd):
    if cmd[0] == 'move':
        bot.move(cmd[2], cmd[1])
    elif cmd[0] == 'rotate_left':
        bot.rotate_left(cmd[1])
    elif cmd[0] == 'rotate_right':
        bot.rotate_right(cmd[1])
    else:
        bot.stop()


def command_label(cmd):
    if cmd[0] == 'move':
        return f'move {cmd[1]:>4}deg'
    if cmd[0] == 'rotate_left':
        return 'rotate CCW'
    if cmd[0] == 'rotate_right':
        return 'rotate CW'
    return 'stopped'


def _ensure_cam(cam):
    """Open the D405 lazily on the first capture."""
    return cam if cam is not None else StereoCapture()


def draw_hud(screen, font, action, speed, pan, tilt):
    screen.fill((18, 18, 22))
    lines = [
        ('RasBot WASD  -  hold to move, release to stop', (120, 220, 120)),
        (f'action: {action}', (235, 235, 235)),
        (f'speed: {speed}    pan: {pan}    tilt: {tilt}', (200, 200, 210)),
        ('W/A/S/D move   Q/E rotate   arrows=servo   +/- speed', (150, 150, 160)),
        ('R capture   T build   Y view   V single   ESC quit', (150, 150, 160)),
        ('(click this window so it has keyboard focus)', (120, 120, 130)),
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
        pygame.display.set_caption('RasBot WASD - hold to move')
    except Exception as e:
        sys.exit('pygame could not open a window. Run this INSIDE the Pi desktop (VNC) as the '
                 'normal user, not bare SSH / not via sudo.\n  ' + str(e))
    font = pygame.font.SysFont('monospace', 16)
    clock = pygame.time.Clock()

    print('Connecting to robot board...')
    with RasBot() as bot:
        bot.set_all_leds_color(Color.GREEN)
        bot.beep(0.1)
        print('Connected. Click the control window, then hold W/A/S/D to drive.')

        speed = SPEED_DEFAULT
        pan, tilt = 90, 25
        cam = None
        last_session = None        # last R capture folder (input for T)
        last_ply = None            # cloud built by T (input for Y)
        last_cmd = None            # last motion command sent (resend only on change)
        action = 'stopped'
        running = True

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        key = event.key
                        if key == pygame.K_ESCAPE:
                            running = False
                        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                            speed = min(speed + SPEED_STEP, SPEED_MAX)
                        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                            speed = max(speed - SPEED_STEP, SPEED_MIN)
                        elif key == pygame.K_UP:
                            tilt = min(tilt + TILT_STEP, 100); bot.set_tilt(tilt)
                        elif key == pygame.K_DOWN:
                            tilt = max(tilt - TILT_STEP, 0); bot.set_tilt(tilt)
                        elif key == pygame.K_LEFT:
                            pan = max(pan - PAN_STEP, 0); bot.set_pan(pan)
                        elif key == pygame.K_RIGHT:
                            pan = min(pan + PAN_STEP, 180); bot.set_pan(pan)

                        elif key == pygame.K_r:        # 360 capture (blocking)
                            bot.stop(); last_cmd = ('stop',)
                            cam = _ensure_cam(cam)
                            bot.set_all_leds_color(Color.BLUE)
                            draw_hud(screen, font, '360 capture...', speed, pan, tilt)
                            try:
                                print(f'--- 360 capture: {scan360.SCAN_SHOTS} shots (do not touch the robot) ---')
                                last_session = scan360.run_scan(bot, cam, log=print)
                                last_ply = None
                                bot.set_all_leds_color(Color.GREEN); bot.beep(0.15)
                                action = f'captured {os.path.basename(last_session)}'
                                print(f'--- done -> {last_session} (press T to build) ---')
                            except Exception as e:
                                bot.stop(); bot.set_all_leds_color(Color.RED)
                                action = 'capture error'; print('capture error:', e)
                            pygame.event.clear()

                        elif key == pygame.K_t:        # build the cloud from the last capture
                            bot.stop(); last_cmd = ('stop',)
                            if last_session is None:
                                action = 'no capture yet (press R)'
                            else:
                                bot.set_all_leds_color(Color.BLUE)
                                draw_hud(screen, font, 'building cloud...', speed, pan, tilt)
                                try:
                                    last_ply = scan360.build_from_session(last_session, log=print)
                                    bot.set_all_leds_color(Color.GREEN); bot.beep(0.15)
                                    action = 'cloud built'
                                    print(f'--- cloud ready -> {last_ply} (press Y to view) ---')
                                except Exception as e:
                                    bot.set_all_leds_color(Color.RED)
                                    action = 'build error'; print('build error:', e)
                            pygame.event.clear()

                        elif key == pygame.K_y:        # open the 3D viewer (non-blocking)
                            if last_ply and os.path.exists(last_ply):
                                root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                                subprocess.Popen([sys.executable,
                                                  os.path.join(root, 'pointcloud', 'view3d.py'),
                                                  last_ply])
                                action = '3D view open'
                            else:
                                action = 'no cloud yet (R then T)'

                        elif key == pygame.K_v:        # single capture in place (blocking)
                            bot.stop(); last_cmd = ('stop',)
                            cam = _ensure_cam(cam)
                            bot.set_all_leds_color(Color.BLUE)
                            draw_hud(screen, font, 'capturing...', speed, pan, tilt)
                            try:
                                folder = cam.save()
                                bot.set_all_leds_color(Color.GREEN)
                                if folder is None:
                                    action = 'capture dropped a frame'
                                else:
                                    bot.beep(0.1); action = f'saved {os.path.basename(folder)}'
                                    print('saved ->', folder)
                            except Exception as e:
                                bot.set_all_leds_color(Color.RED)
                                action = 'capture error'; print('capture error:', e)
                            pygame.event.clear()

                # held-key motion: poll the live up/down state (no key-repeat dependence)
                pressed = pygame.key.get_pressed()
                cmd = desired_command(pressed, speed)
                if cmd != last_cmd:
                    apply_command(bot, cmd)
                    last_cmd = cmd
                    action = command_label(cmd)

                draw_hud(screen, font, action, speed, pan, tilt)
                clock.tick(FPS)

        finally:
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
