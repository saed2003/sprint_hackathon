"""
Pan/tilt servo controller via pygame — hold an arrow key to move the camera.

Needs a desktop/display, so run it INSIDE the Pi's desktop over VNC (not bare
SSH). It opens a small control window; click it so it has keyboard FOCUS.

Controls (hold to move, key-repeat sweeps)
  Up / Down arrow      tilt camera up / down
  Left / Right arrow   pan camera left / right
  c                    re-center
  ESC / close window   quit

Install pygame on the Pi once:
    sudo apt install -y python3-pygame      # or:  pip install pygame

Run inside the VNC desktop:
  python3 camera_move/pygame_servo.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color
from setup_and_api.api.constants import (
    TILT_MIN, TILT_MAX, TILT_DEFAULT,
    PAN_MIN, PAN_MAX, PAN_DEFAULT,
)

import pygame

# ── tunables ────────────────────────────────────────────────────────────────
TILT_STEP   = 5         # degrees per step
PAN_STEP    = 5
FPS         = 60
REPEAT_MS   = 30        # key-repeat interval while held (hold-to-sweep)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def draw_hud(screen, font, pan, tilt):
    screen.fill((18, 18, 22))
    lines = [
        ('RasBot servo control', (120, 220, 120)),
        (f'pan: {pan:>3}    tilt: {tilt:>3}', (235, 235, 235)),
        ('arrows = move   c = center   ESC = quit', (150, 150, 160)),
        ('(click this window so it has keyboard focus)', (120, 120, 130)),
    ]
    y = 16
    for text, color in lines:
        screen.blit(font.render(text, True, color), (16, y))
        y += 30
    pygame.display.flip()


def main():
    try:
        pygame.init()
        screen = pygame.display.set_mode((440, 170))
        pygame.display.set_caption('RasBot servo control')
    except Exception as e:
        sys.exit('pygame could not open a window. Run this INSIDE the Pi desktop '
                 '(VNC), not bare SSH / not via sudo.\n  ' + str(e))
    pygame.key.set_repeat(REPEAT_MS, REPEAT_MS)     # hold a key -> repeated KEYDOWN
    font = pygame.font.SysFont('monospace', 16)
    clock = pygame.time.Clock()

    print('Connecting to robot board...')
    with RasBot() as bot:
        bot.set_all_leds_color(Color.GREEN)
        pan, tilt = PAN_DEFAULT, TILT_DEFAULT
        bot.set_pan(pan)
        bot.set_tilt(tilt)
        print('Connected. Click the window, then hold the arrow keys.')

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
                        elif key == pygame.K_UP:
                            tilt = _clamp(tilt + TILT_STEP, TILT_MIN, TILT_MAX)
                            bot.set_tilt(tilt)
                        elif key == pygame.K_DOWN:
                            tilt = _clamp(tilt - TILT_STEP, TILT_MIN, TILT_MAX)
                            bot.set_tilt(tilt)
                        elif key == pygame.K_LEFT:
                            pan = _clamp(pan - PAN_STEP, PAN_MIN, PAN_MAX)
                            bot.set_pan(pan)
                        elif key == pygame.K_RIGHT:
                            pan = _clamp(pan + PAN_STEP, PAN_MIN, PAN_MAX)
                            bot.set_pan(pan)
                        elif key == pygame.K_c:
                            pan, tilt = PAN_DEFAULT, TILT_DEFAULT
                            bot.set_pan(pan)
                            bot.set_tilt(tilt)

                draw_hud(screen, font, pan, tilt)
                clock.tick(FPS)
        finally:
            bot.set_all_leds_color(Color.RED)
            pygame.quit()
            print('\nStopped. Camera re-centered.')


if __name__ == '__main__':
    main()
