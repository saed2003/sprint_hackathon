#!/usr/bin/env python3
"""Show a static message on the robot's 128x32 OLED (4 lines, ~21 chars each). Pi-only.

  python3 oled_message.py                              # default Sprint splash
  python3 oled_message.py "SPRINT HACKATHON" "TAU"     # up to 4 custom lines
  python3 oled_message.py --clear                      # blank the screen

The text stays on the screen after this exits (we deliberately skip RasBot cleanup, which
would wipe it). Note: wasd/drive.py clears the OLED when it shuts down, so re-run this
afterwards if you want the message back.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from setup_and_api.api import RasBot

DEFAULT_LINES = ["SPRINT HACKATHON", "Street View Robot", "TAU", ""]
MAX_CHARS = 21          # the default OLED font is ~6 px wide on the 128 px display


def show(lines):
    """Write up to 4 lines to the OLED and leave them on screen."""
    lines = [str(t)[:MAX_CHARS] for t in (list(lines) + ["", "", "", ""])[:4]]
    bot = RasBot()          # NOT a `with` block on purpose: cleanup() would clear the OLED
    for i, text in enumerate(lines, start=1):
        bot.display_text(text, line=i)
    return lines


def main():
    args = sys.argv[1:]
    if "--clear" in args:
        RasBot().clear_display()
        print("OLED cleared.")
        return
    shown = show(args or DEFAULT_LINES)
    print("Sprint TAU:")
    for i, text in enumerate(shown, start=1):
        if text:
            print(f"  line {i}: {text}")
    print("(stays on screen; run with --clear to blank it)")


if __name__ == "__main__":
    main()
