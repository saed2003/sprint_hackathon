import time
import sys
import os

# go up one level: tape_following/ -> src/ so setup_and_api is importable
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from setup_and_api.api import RasBot

# ── TUNED CONFIGURATION ─────────────────────────────────────────────
BASE_SPEED     =  80   # Slower for stability
GENTLE_TURN    =  100  # Outer wheel speed for minor corrections
GENTLE_REVERSE =  20   # Inner wheel speed for minor corrections (keeps it rolling forward)
SHARP_TURN     =  120  # Outer wheel speed for hard corners
SHARP_REVERSE  = -60   # Inner wheel speed for hard corners
LOOP_DELAY     = 0.02

def main():
    # Memory variable so it knows which way to search if it loses the line
    last_seen = "straight"

    with RasBot() as bot:
        print("Smooth line follower started. Ctrl+C to stop.")
        try:
            while True:
                L1, L2, R1, R2 = bot.read_line_sensors()

                if L2 and R1:
                    # Perfect center
                    bot._apply_motors(BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED)
                    last_seen = "straight"

                elif L1:
                    # Tape hit the FAR LEFT sensor -> Hard left correction
                    bot._apply_motors(SHARP_REVERSE, SHARP_REVERSE, SHARP_TURN, SHARP_TURN)
                    last_seen = "left"

                elif L2:
                    # Tape hit the INNER LEFT sensor -> Gentle left correction
                    bot._apply_motors(GENTLE_REVERSE, GENTLE_REVERSE, GENTLE_TURN, GENTLE_TURN)
                    last_seen = "left"

                elif R2:
                    # Tape hit the FAR RIGHT sensor -> Hard right correction
                    bot._apply_motors(SHARP_TURN, SHARP_TURN, SHARP_REVERSE, SHARP_REVERSE)
                    last_seen = "right"

                elif R1:
                    # Tape hit the INNER RIGHT sensor -> Gentle right correction
                    bot._apply_motors(GENTLE_TURN, GENTLE_TURN, GENTLE_REVERSE, GENTLE_REVERSE)
                    last_seen = "right"

                else:
                    # LOST THE LINE -> Use memory to sweep back and find it
                    if last_seen == "left":
                        bot._apply_motors(-60, -60, 80, 80)
                    elif last_seen == "right":
                        bot._apply_motors(80, 80, -60, -60)
                    else:
                        bot.stop()

                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            bot.stop()
            print("Motors off. Done.")

if __name__ == "__main__":
    main()
