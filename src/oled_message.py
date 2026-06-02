#!/usr/bin/env python3
"""Animated OLED splash for the demo. Line 1: a centered "SPRINT HACKATHON" title.
Line 2 (with a gap below): the team names cycling with a smooth slide transition.
Pi-only (drives the robot's 128x32 SSD1306 via RasBot).

  python3 oled_message.py            # run the splash (Ctrl-C to stop)
  python3 oled_message.py --clear    # blank the screen

We render our own frames into the OLED image buffer (RasBot.display_text can't center or
animate), then push them with oled.display().
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from setup_and_api.api import RasBot
from PIL import Image  # noqa: F401  (Image type is what RasBot's _oled_image already is)

WIDTH, HEIGHT = 128, 32
TITLE = "SPRINT HACKATHON"
NAMES = ["Mahmood Stitia", "Ahmad Khalifa", "Saleh Khalel", "Saed Abo Fool", "Ahmad Shalabi"]

TITLE_Y    = 0      # title sits on the top line
NAME_Y     = 19     # name line, leaving a gap under the title
HOLD_S     = 1.4    # seconds a name rests centered before sliding away
SLIDE_STEP = 6      # pixels moved per frame during a slide (bigger = faster)
FRAME_S    = 0.02   # delay between slide frames


def _text_w(draw, text, font):
    """Pixel width of text (Pillow >= 8 has textlength; older has textsize)."""
    try:
        return int(draw.textlength(text, font=font))
    except AttributeError:
        return draw.textsize(text, font=font)[0]


def run():
    bot = RasBot()
    bot._ensure_oled()                       # init the SSD1306 + its PIL image/draw/font
    oled, img, draw, font = bot._oled, bot._oled_image, bot._oled_draw, bot._oled_font

    title_x = (WIDTH - _text_w(draw, TITLE, font)) // 2
    center_x = lambda text: (WIDTH - _text_w(draw, text, font)) // 2

    def render(name_items):
        """Draw the centered title + each (name, x) on the name line, then show it."""
        draw.rectangle((0, 0, WIDTH, HEIGHT), fill=0)
        draw.text((title_x, TITLE_Y), TITLE, font=font, fill=255)
        for text, x in name_items:
            draw.text((x, NAME_Y), text, font=font, fill=255)
        oled.image(img)
        oled.display()

    print("OLED splash running — Ctrl-C to stop.")
    try:
        cur, cur_x = NAMES[0], center_x(NAMES[0])
        for shift in range(WIDTH - cur_x, -1, -SLIDE_STEP):     # slide name 1 in from the right
            render([(cur, cur_x + shift)])
            time.sleep(FRAME_S)

        i = 0
        while True:
            render([(cur, cur_x)])
            time.sleep(HOLD_S)
            nxt, nxt_x = NAMES[(i + 1) % len(NAMES)], center_x(NAMES[(i + 1) % len(NAMES)])
            for shift in range(0, (WIDTH - nxt_x) + 1, SLIDE_STEP):   # cur slides left, nxt in
                render([(cur, cur_x - shift), (nxt, WIDTH - shift)])
                time.sleep(FRAME_S)
            cur, cur_x, i = nxt, nxt_x, (i + 1) % len(NAMES)
    except KeyboardInterrupt:
        pass
    finally:
        bot.clear_display()
        print("\nstopped, screen cleared.")


def main():
    if "--clear" in sys.argv[1:]:
        RasBot().clear_display()
        print("OLED cleared.")
        return
    run()


if __name__ == "__main__":
    main()
