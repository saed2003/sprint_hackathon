"""
WASD controller for the RasbotV2 Mecanum robot — TRUE hold-to-move via pygame.

HOLD a movement key to move, RELEASE to stop (real key-up detection, no key-repeat
tricks). This needs a desktop/display, so run it INSIDE the Pi's desktop over VNC
(not bare SSH). It opens a small control window; that window must have keyboard FOCUS
(click it) to receive keys. If the window loses focus, the robot stops (a safety bonus).

Install pygame on the Pi once:
    sudo apt install -y python3-pygame        # or:  pip install pygame

Movement (hold = move, release = stop; combine keys for diagonals)
  W / S       forward / backward
  A / D       strafe left / right
  W+A, W+D …  diagonal / strafe blends
  Q / E       rotate counter-clockwise / clockwise (in place)

Camera servo (tap)
  Arrow UP / DOWN     tilt camera up / down
  Arrow LEFT / RIGHT  pan camera left / right

Speed (tap)
  + or =      speed up   (step 20, max 255)   — applies live while moving
  -           slow down  (step 20, min 40)

Capture (tap)
  R   run a 360 capture: rotate in place, take 10 shots -> captures/scan_<ts>/
  T   build the 3D point cloud from the last R capture -> .../merged_360.ply
  Y   open the 3D point cloud viewer (orbit with the mouse)
  V   take a single capture in place -> captures/<timestamp>/

Quit          ESC, or close the window

Run inside the VNC desktop:
  python3 wasd/drive.py
"""

import os
import sys
import math
import time
import subprocess

# project root = the folder that contains wasd/, camera/, pointcloud/, rasbot/, ...
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color
from camera.rs_capture import StereoCapture
from pointcloud import scan360

import pygame

# ── tunables ────────────────────────────────────────────────────────────────
SPEED_DEFAULT = 120
SPEED_STEP    = 20
SPEED_MIN     = 40
SPEED_MAX     = 255
PAN_STEP      = 10
TILT_STEP     = 10
FPS           = 60          # how often we poll the held keys and refresh the HUD

# held movement key -> translation contribution (vx: right +, vy: forward +)
_TRANS = {pygame.K_w: (0, 1), pygame.K_s: (0, -1),
          pygame.K_d: (1, 0), pygame.K_a: (-1, 0)}
# held movement key -> rotation (+1 = CCW / rotate_left, -1 = CW / rotate_right)
_ROT = {pygame.K_q: +1, pygame.K_e: -1}


def desired_command(pressed, speed):
    """Map the currently-held keys to a motion command tuple.

    Translation (W/A/S/D, blended into diagonals) takes priority over rotation
    (Q/E). Returns a tuple including the speed so the caller resends the command
    only when something actually changes (including a live speed change).
    """
    vx = sum(d[0] for k, d in _TRANS.items() if pressed[k])
    vy = sum(d[1] for k, d in _TRANS.items() if pressed[k])
    rot = sum(v for k, v in _ROT.items() if pressed[k])    # +1/-1, cancels if both held
    if vx or vy:
        # angle convention matches bot.move(): 0=right, 90=forward, 180=left, 270=back
        angle = round(math.degrees(math.atan2(vy, vx)))
        return ('move', angle, speed)
    if rot > 0:
        return ('rotate_left', speed)
    if rot < 0:
        return ('rotate_right', speed)
    return ('stop',)


def apply_command(bot, cmd):
    """Send a command tuple from desired_command() to the robot."""
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
        sys.exit('pygame could not open a window. Run this INSIDE the Pi desktop (VNC) as '
                 'the normal user, not bare SSH / not via sudo.\n  ' + str(e))
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
        last_session = None        # folder from the last R capture (input for T)
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

                        # ── speed ──
                        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                            speed = min(speed + SPEED_STEP, SPEED_MAX)
                        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                            speed = max(speed - SPEED_STEP, SPEED_MIN)

                        # ── camera servos ──
                        elif key == pygame.K_UP:
                            tilt = min(tilt + TILT_STEP, 100); bot.set_tilt(tilt)
                        elif key == pygame.K_DOWN:
                            tilt = max(tilt - TILT_STEP, 0); bot.set_tilt(tilt)
                        elif key == pygame.K_LEFT:
                            pan = max(pan - PAN_STEP, 0); bot.set_pan(pan)
                        elif key == pygame.K_RIGHT:
                            pan = min(pan + PAN_STEP, 180); bot.set_pan(pan)

                        # ── R: 360 capture only (blocking) ──
                        elif key == pygame.K_r:
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

                        # ── T: build the cloud from the last R capture (blocking) ──
                        elif key == pygame.K_t:
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

                        # ── Y: open the 3D viewer ──
                        elif key == pygame.K_y:
                            if last_ply and os.path.exists(last_ply):
                                root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                                subprocess.Popen([sys.executable,
                                                  os.path.join(root, 'pointcloud', 'view3d.py'),
                                                  last_ply])
                                action = '3D view open'
                            else:
                                action = 'no cloud yet (R then T)'

                        # ── V: single capture in place (blocking) ──
                        elif key == pygame.K_v:
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

                # ── held-key motion: poll the real key state every frame ──
                # (no key-repeat dependence — get_pressed() is the live up/down state)
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
