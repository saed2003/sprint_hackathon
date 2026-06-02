"""
Dead-simple servo test — NO keyboard. Just sweeps both servos full range so you
can see whether the hardware moves at all.

If the camera does NOT physically move while running this, the problem is
power/wiring/protocol, not the keyboard code:
  - robot main battery ON (servos need battery power, not just USB)
  - pan/tilt servo plugs seated on the board (S1 = pan, S2 = tilt)

Run on the Pi:
  python3 camera_move/servo_test.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color


def main():
    print("Connecting...")
    with RasBot() as bot:
        bot.set_all_leds_color(Color.BLUE)
        print("Sweeping. Watch the camera. Ctrl-C to stop.\n")

        for i in range(3):
            print(f"--- round {i + 1} ---")

            print("TILT -> 0   (down)");   bot.set_tilt(0);    time.sleep(0.8)
            print("TILT -> 100 (up)");     bot.set_tilt(100);  time.sleep(0.8)
            print("TILT -> 25  (center)"); bot.set_tilt(25);   time.sleep(0.8)

            print("PAN  -> 0   (left)");   bot.set_pan(0);     time.sleep(0.8)
            print("PAN  -> 180 (right)");  bot.set_pan(180);   time.sleep(0.8)
            print("PAN  -> 90  (center)"); bot.set_pan(90);    time.sleep(0.8)

        bot.set_all_leds_color(Color.GREEN)
        print("\nDone. Did the camera move?")


if __name__ == "__main__":
    main()
