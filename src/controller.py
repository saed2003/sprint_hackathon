"""
Terminal (SSH / PuTTY) robot controller.

  W / S        forward / backward     (hold to move, release to stop)
  A / D        rotate left / right    (hold to move, release to stop)
  F            toggle line following
  Q / ESC      quit

Run on the Pi:
    python3 src/controller.py
"""

import os, sys, time, threading, tty, termios, selectw

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color
import line_following.line_follow as lf

# ── tunables ──────────────────────────────────────────────────────────────────
MANUAL_SPEED  = 70     # motor speed for WASD (0–255)
STOP_DELAY_S  = 0.15   # stop motors this long after the last key repeat is seen
# ─────────────────────────────────────────────────────────────────────────────

_MOVE_KEYS = set('wasdWASD')


def _read_key(timeout=0.02):
    """Return the next character from stdin, or None if nothing arrives
    within `timeout` seconds.  Caller must have already set raw mode."""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def _apply_wasd(bot, key):
    k = key.lower()
    if k == 'w':
        bot.forward(MANUAL_SPEED)
    elif k == 's':
        bot.backward(MANUAL_SPEED)
    elif k == 'a':
        bot.rotate_left(MANUAL_SPEED)
    elif k == 'd':
        bot.rotate_right(MANUAL_SPEED)


def main():
    print("=" * 46)
    print("  RasBot Controller")
    print("  W/A/S/D  hold = move,  release = stop")
    print("  F        toggle line following")
    print("  Q / ESC  quit")
    print("=" * 46)

    follow_stop   = threading.Event()
    follow_thread = None
    follow_mode   = False

    with RasBot() as bot:
        bot.set_all_leds_color(Color.GREEN)
        bot.beep(0.1)

        fd           = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

        last_motion_key  = None
        last_key_time    = 0.0

        try:
            while True:
                now = time.time()

                # ── release detection: stop if key held but no repeat arrived ──
                if (not follow_mode
                        and last_motion_key is not None
                        and now - last_key_time > STOP_DELAY_S):
                    bot.stop()
                    last_motion_key = None

                key = _read_key()
                if key is None:
                    continue

                # ── quit ──────────────────────────────────────────────────────
                if key in ('q', 'Q', '\x1b'):
                    break

                # ── F: toggle line following ───────────────────────────────────
                if key in ('f', 'F'):
                    follow_mode = not follow_mode
                    if follow_mode:
                        bot.stop()
                        last_motion_key = None
                        follow_stop.clear()
                        follow_thread = threading.Thread(
                            target=lf.run,
                            args=(bot,),
                            kwargs={'stop_event': follow_stop},
                            daemon=True,
                        )
                        follow_thread.start()
                        sys.stdout.write('\r\n[FOLLOW] Line following — press F to stop\r\n')
                    else:
                        follow_stop.set()
                        if follow_thread:
                            follow_thread.join(timeout=3.0)
                        bot.stop()
                        bot.set_all_leds_color(Color.GREEN)
                        sys.stdout.write('\r\n[MANUAL] W/A/S/D to drive\r\n')
                    sys.stdout.flush()
                    continue

                # ── WASD: manual drive ─────────────────────────────────────────
                if not follow_mode and key in _MOVE_KEYS:
                    _apply_wasd(bot, key)
                    last_motion_key = key
                    last_key_time   = now

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            follow_stop.set()
            if follow_thread and follow_thread.is_alive():
                follow_thread.join(timeout=2.0)
            bot.stop()
            bot.set_all_leds_color(Color.RED)
            sys.stdout.write('\r\nStopped.\r\n')
            sys.stdout.flush()


if __name__ == '__main__':
    main()
