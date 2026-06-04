#!/usr/bin/env python3
"""
follow.py — simple, predictable BLACK-LINE follower for the Raspbot V2 (4 IR sensors).

A backup for src/tape_following (which got over-engineered: adaptive speed, path memory,
auto-tune -> wobbly + unpredictable). This is the opposite: ONE constant speed, a plain
proportional+small-derivative steer on TANK (differential) wheels, and a tiny bit of logic
for the two things a plain follower needs — recovering a lost line and choosing a branch at
a junction. Nothing adaptive. Built to drive the same known track for filming.

HOW IT STEERS (the important bit)
  read_line_sensors() -> (Lo, Li, Ri, Ro), True = sensor over the black tape.
  We turn that into one ERROR number (tape left = negative, right = positive), then drive the
  LEFT wheels and RIGHT wheels at different speeds (tank steering via _apply_motors) so the
  robot curves smoothly toward the tape. (NOTE: the API's drift() puts the rotation on the
  front/rear axle, not left/right, so it can't steer a moving robot — tank steering is the fix,
  same as the old archive/p_follow.py.)

THE KNOWN TRACK (yours): straight -> left -> straight -> left FORK (main road = left) -> turn.
  Normal corners (left OR right) are handled automatically by the error steer. A real FORK /
  cross lights up 3+ sensors at once; there we follow JUNCTION_PLAN (default: take the left/main
  road). Edit JUNCTION_PLAN if a junction should go right.

RUN (on the Pi):
    python3 follow.py                 # drive the track at the default speed
    python3 follow.py --speed 140     # faster
    python3 follow.py --plan left,left,right   # decision at each junction, in order
    python3 follow.py --test          # just print the sensors live (place it on the line first)
Stop with Ctrl+C (or it stops itself after LOST_TIMEOUT s with no line = track end).
"""
import os
import sys
import time
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))   # reach src/setup_and_api

from setup_and_api.api import RasBot, Color

# ── tuning — ALL CONSTANT (no adaptive speed) ───────────────────────────────────────
BASE_SPEED      = 120     # cruise speed (0-255). Raise for faster, lower if it overshoots.
KP              = 60.0    # proportional gain: how hard it steers toward the line
KD              = 14.0    # FIXED derivative: damps side-to-side wobble (not adaptive speed)
MAX_SPEED       = 255
WEIGHTS         = (-2.5, -1.0, 1.0, 2.5)   # error weight of (Lo, Li, Ri, Ro)
LOOP_DELAY      = 0.01    # ~100 Hz control loop
LOST_SWEEP      = 3.0     # error magnitude used to sweep back toward a lost line
LOST_TIMEOUT    = 2.0     # s with no line at all -> assume end of track and stop
# junctions / forks (3+ sensors lit at once = a crossing bar, not a normal corner)
JUNCTION_MIN_ON = 3
JUNCTION_PLAN   = ["left", "left", "left", "left", "left"]   # what to do at each junction in order
JUNCTION_COMMIT = 0.35    # s to commit a junction turn before resuming normal follow


def clamp(v, lo=-MAX_SPEED, hi=MAX_SPEED):
    return max(lo, min(hi, int(v)))


def tank(bot, left, right):
    """Differential drive: both left wheels at `left`, both right wheels at `right`."""
    bot._apply_motors(clamp(left), clamp(left), clamp(right), clamp(right))


def sensor_error(on):
    """on = (Lo,Li,Ri,Ro) as 0/1. Returns (error, n_lit). error<0 = tape left, >0 = right."""
    n = sum(on)
    if n == 0:
        return None, 0
    return sum(w * o for w, o in zip(WEIGHTS, on)) / n, n


def run_test(bot):
    """Live sensor view — drive nothing, just print which sensors see the tape."""
    print("SENSOR TEST — place the robot on the line. Ctrl+C to quit.")
    names = ("Lo", "Li", "Ri", "Ro")
    while True:
        s = bot.read_line_sensors()
        bar = " ".join(f"{nm}:{'#' if v else '.'}" for nm, v in zip(names, s))
        e, n = sensor_error([1 if v else 0 for v in s])
        print(f"  {bar}   n={n}  error={'lost' if e is None else f'{e:+.2f}'}   ", end="\r")
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=int, default=BASE_SPEED)
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--kd", type=float, default=KD)
    ap.add_argument("--plan", type=str, default=",".join(JUNCTION_PLAN),
                    help="comma list of left/right decisions per junction")
    ap.add_argument("--junction-min", type=int, default=JUNCTION_MIN_ON)
    ap.add_argument("--no-leds", action="store_true")
    ap.add_argument("--test", action="store_true", help="just print sensors, don't drive")
    a = ap.parse_args()
    plan = [d.strip().lower() for d in a.plan.split(",") if d.strip()]

    with RasBot() as bot:
        if a.test:
            try:
                run_test(bot)
            except KeyboardInterrupt:
                print()
            return

        print(f"line follow: speed={a.speed} kp={a.kp} kd={a.kd} plan={plan}. Ctrl+C to stop.")
        last_error = 0.0
        lost_since = None
        j_idx = 0
        in_junction = False
        j_dir = -1
        j_until = 0.0
        led = None

        def set_led(color):
            nonlocal led
            if not a.no_leds and color is not led:
                try:
                    bot.set_all_leds_color(color)
                except Exception:
                    pass
                led = color

        try:
            while True:
                now = time.time()
                on = [1 if v else 0 for v in bot.read_line_sensors()]
                n = sum(on)

                # 1) at a fork/cross (3+ lit): commit the planned turn
                if not in_junction and n >= a.junction_min:
                    d = plan[j_idx] if j_idx < len(plan) else "left"
                    j_idx += 1
                    in_junction = True
                    j_dir = -1.0 if d == "left" else 1.0
                    j_until = now + JUNCTION_COMMIT
                    set_led(Color.BLUE)
                if in_junction:
                    corr = a.kp * (2.5 * j_dir)            # hard pivot toward the chosen branch
                    tank(bot, a.speed + corr, a.speed - corr)
                    if now >= j_until and n <= 2:          # committed + back on a single line
                        in_junction = False
                        last_error = 2.5 * j_dir
                    time.sleep(LOOP_DELAY)
                    continue

                # 2) lost the line: sweep toward where it last was; stop if gone too long
                if n == 0:
                    if lost_since is None:
                        lost_since = now
                    elif now - lost_since > LOST_TIMEOUT:
                        print("\nno line for a while — stopping (end of track?).")
                        break
                    sgn = -1.0 if last_error < 0 else 1.0
                    corr = a.kp * (LOST_SWEEP * sgn)
                    tank(bot, a.speed + corr, a.speed - corr)
                    set_led(Color.RED)
                    time.sleep(LOOP_DELAY)
                    continue

                # 3) normal proportional + derivative steer on the tape
                lost_since = None
                e, _ = sensor_error(on)
                corr = a.kp * e + a.kd * (e - last_error)
                last_error = e
                tank(bot, a.speed + corr, a.speed - corr)
                set_led(Color.GREEN)
                time.sleep(LOOP_DELAY)
        except KeyboardInterrupt:
            print("\nstopped.")
        finally:
            bot.stop()
            if not a.no_leds:
                try:
                    bot.leds_off()
                except Exception:
                    pass
            print("motors off.")


if __name__ == "__main__":
    main()
