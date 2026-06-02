"""
ROBOT ORBIT object scan — stop-and-shoot (Pi only).  ★ the ambitious path ★

The robot walks around a stationary object in steps; at each station it (1) re-aims
the D405 at the object with the PAN SERVO using vision (closing the aim loop, since
the base motion is open-loop and sloppy), (2) measures the object distance, (3) saves
a standard capture folder + angle.txt, then (4) moves one step around the circle.
The merge (build_object.py) then absorbs the open-loop drift with ICP + loop closure.

WHY STOP-AND-SHOOT + TURN-DRIVE-TURN
------------------------------------
The Raspbot has no encoders/IMU, so we can't drive a smooth circle. Instead each step
is the classic "orbit a point" move, built from primitives scan360 already trusts:
    turn in place Δ/2  →  drive forward the chord (2·R·sin(Δ/2))  →  turn in place Δ/2
After it, the robot has advanced Δ degrees around the object and is re-aimed at it.
Pan-servo centring fixes the residual aim error every stop. It is still the riskiest
path — if it misbehaves, use the rock-solid turntable path (capture_session.py); both
feed the exact same merge.

CALIBRATE TWO NUMBERS on the real robot/floor (battery + grip change them):
    SEC_PER_DEG : in-place rotation timing  (reuse scan360's value; --calibrate-turn)
    SEC_PER_M   : forward-drive timing       (--calibrate-fwd to measure)

RUN (on the Pi, D405 + object ~25 cm in front, textured + lit):
    python3 capture_orbit.py                       # 18 shots, 20 deg each, R=0.25 m
    python3 capture_orbit.py --shots 24 --radius 0.22
    python3 capture_orbit.py --calibrate-fwd 3.0   # drive 3 s fwd, measure cm -> SEC_PER_M
    python3 capture_orbit.py --calibrate-turn 3.0  # spin 3 s, measure deg -> SEC_PER_DEG
Then build on the laptop:  python build_object.py <session> --mesh   (try --dir -1 if smeared)
"""
import os
import sys
import time
import math

import numpy as np

# the D405 capturer (aligned colour, standard folder) lives next door
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture_session import ColorStereoCapture, default_out_root

# the robot API lives in the project's src/ (Pi only — pulls smbus)
SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
sys.path.insert(0, SRC)

# ── tunables (object-specific defaults come from config.py) ──────────────────────
try:
    from config import DEFAULT as _CFG
except Exception:
    _CFG = dict(radius=0.40, shots=24, zmin=0.30, zmax=0.52)

ORBIT_SHOTS   = _CFG["shots"]    # views around the object (24 -> 15 deg each)
ORBIT_RADIUS  = _CFG["radius"]   # metres from the camera to the object centre (~0.40)
ROTATE_SPEED  = 40          # in-place turn speed
FORWARD_SPEED = 40          # forward drive speed
SEC_PER_DEG   = 2.73 / 360  # in-place rotation timing (same default as scan360)
SEC_PER_M     = 2.2         # forward timing: seconds to drive 1 m (CALIBRATE!)
SETTLE        = 0.4         # pause for the chassis to settle before a shot
ORBIT_DIR     = 1           # +1 = orbit one way, -1 the other (merge --dir must match)

# vision auto-centring (pan servo) + radius hold
ZMIN, ZMAX    = _CFG["zmin"], _CFG["zmax"]   # object depth gate (the close thing in frame)
PAN_GAIN      = 0.04        # deg of pan per pixel of centring error
PAN_SIGN      = +1          # flip to -1 if the camera turns AWAY from the object
CENTER_ITERS  = 3           # aim refinement passes per stop
RADIUS_TOL    = 0.05        # only correct distance if off target by more than this (m)
RADIUS_MAX_STEP = 0.10      # cap a single in/out correction drive (m), for safety


def object_centroid_and_distance(depth_m, ppx):
    """From a depth frame, find the close object's horizontal centroid pixel + distance.

    Returns (u_centroid, distance_m, n_pixels) or (None, None, 0) if nothing close.
    """
    mask = (depth_m > ZMIN) & (depth_m < ZMAX)
    n = int(mask.sum())
    if n < 500:
        return None, None, 0
    ys, xs = np.where(mask)
    u = float(xs.mean())
    d = float(np.median(depth_m[mask]))
    return u, d, n


def auto_center(bot, cam, pan_angle, log=print):
    """Nudge the PAN servo so the object sits in the horizontal centre. Returns
    (new_pan_angle, distance_m). Vision closes the aim loop the open-loop base can't."""
    ppx = cam.intr.ppx
    dist = None
    for _ in range(CENTER_ITERS):
        depth = cam.grab_depth()
        if depth is None:
            break
        u, d, n = object_centroid_and_distance(depth, ppx)
        if u is None:
            log("    (no object in close range — check distance/lighting)")
            break
        dist = d
        err = u - ppx                                   # +err: object is to the right
        if abs(err) < 8:
            break
        pan_angle = int(max(0, min(180, pan_angle + PAN_SIGN * PAN_GAIN * err)))
        bot.set_pan(pan_angle)
        time.sleep(0.25)
    return pan_angle, dist


def hold_radius(bot, cam, radius, log=print):
    """Use the measured object distance to keep the orbit radius ~constant (camera helps
    'find the way'). Drives forward/back if the figure has drifted nearer/farther than
    `radius`. Returns the corrected distance, or None if the object wasn't seen."""
    depth = cam.grab_depth()
    if depth is None:
        return None
    _, d, n = object_centroid_and_distance(depth, cam.intr.ppx)
    if d is None:
        return None
    err = d - radius                                # +err: too far -> drive forward
    if abs(err) > RADIUS_TOL:
        move = max(-RADIUS_MAX_STEP, min(RADIUS_MAX_STEP, err))
        if move > 0:
            _forward(bot, move)
        else:
            _backward(bot, -move)
        log(f"    radius hold: was {d*100:.0f}cm, nudged {move*100:+.0f}cm toward {radius*100:.0f}cm")
        depth = cam.grab_depth()
        if depth is not None:
            _, d2, _ = object_centroid_and_distance(depth, cam.intr.ppx)
            d = d2 or d
    return d


def _rotate(bot, deg, speed=ROTATE_SPEED, sec_per_deg=SEC_PER_DEG, direction=ORBIT_DIR):
    """Time-pulse an in-place turn of `deg` degrees (sign via direction)."""
    spin = bot.rotate_left if direction >= 0 else bot.rotate_right
    spin(speed)
    time.sleep(sec_per_deg * abs(deg))
    bot.stop()


def _forward(bot, metres, speed=FORWARD_SPEED, sec_per_m=SEC_PER_M):
    """Time-pulse a forward drive of `metres`."""
    bot.forward(speed)
    time.sleep(sec_per_m * metres)
    bot.stop()


def _backward(bot, metres, speed=FORWARD_SPEED, sec_per_m=SEC_PER_M):
    """Time-pulse a backward drive of `metres`."""
    bot.backward(speed)
    time.sleep(sec_per_m * metres)
    bot.stop()


def orbit_step(bot, step_deg, radius, log=print):
    """Advance one step around the object: turn Δ/2, drive the chord, turn Δ/2."""
    chord = 2.0 * radius * math.sin(math.radians(step_deg) / 2.0)
    _rotate(bot, step_deg / 2.0); time.sleep(0.1)
    _forward(bot, chord);         time.sleep(0.1)
    _rotate(bot, step_deg / 2.0)
    log(f"    orbit step: turn {step_deg/2:.0f}, fwd {chord*100:.1f}cm, turn {step_deg/2:.0f}")


def run_orbit(bot, cam, shots=ORBIT_SHOTS, radius=ORBIT_RADIUS, out_root=None, log=print):
    """Stop-and-shoot orbit. Returns the session folder of shot_NN captures."""
    out_root = out_root or default_out_root()
    session = os.path.join(out_root, "orbit_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    step = 360.0 / shots
    if cam.pipeline is None:
        log("Opening D405 (warming up)..."); cam.start(); log(cam.info())

    log(f"robot orbit: {shots} shots, {step:.0f} deg each, R={radius*100:.0f}cm")
    pan = 90
    bot.set_pan(pan)
    cumulative = 0.0
    for i in range(shots):
        bot.stop(); time.sleep(SETTLE)
        pan, dist = auto_center(bot, cam, pan, log=log)     # vision: aim the camera
        held = hold_radius(bot, cam, radius, log=log)       # vision: keep ~constant radius
        dist = held or dist
        pan, _ = auto_center(bot, cam, pan, log=log)        # re-aim after any in/out nudge
        folder = os.path.join(session, f"shot_{i:02d}")
        ok = cam.save_to(folder)
        with open(os.path.join(folder, "angle.txt"), "w") as f:
            f.write(f"{cumulative:.3f}\n")
        log(f"  shot {i+1}/{shots} (~{cumulative:.0f} deg, d={dist*100 if dist else float('nan'):.0f}cm)"
            + ("" if ok else "  FRAME DROPPED"))
        try:
            bot.beep(0.05)
        except Exception:
            pass
        if i < shots - 1:
            orbit_step(bot, step, radius, log=log)
            cumulative += step
    bot.stop()
    log(f"  orbit complete -> {session}")
    log(f"  build on the laptop:  python build_object.py {session} --mesh")
    return session


# ── calibration helpers ──────────────────────────────────────────────────────────

def calibrate_forward(seconds):
    """Drive forward `seconds` at FORWARD_SPEED; measure the distance to set SEC_PER_M."""
    from setup_and_api.api import RasBot
    print(f"Driving forward {seconds:.1f}s at speed {FORWARD_SPEED}. Measure the distance moved.")
    with RasBot() as bot:
        bot.forward(FORWARD_SPEED); time.sleep(seconds); bot.stop()
    print(f"If it moved D metres, set SEC_PER_M = {seconds:.1f} / D   (e.g. 0.9 m -> {seconds/0.9:.2f}).")


def calibrate_turn(seconds):
    """Spin in place `seconds`; measure degrees to set SEC_PER_DEG (same as scan360)."""
    from setup_and_api.api import RasBot
    print(f"Spinning {seconds:.1f}s at speed {ROTATE_SPEED}. Measure the degrees turned.")
    with RasBot() as bot:
        (bot.rotate_left if ORBIT_DIR >= 0 else bot.rotate_right)(ROTATE_SPEED)
        time.sleep(seconds); bot.stop()
    print(f"If it turned D deg, set SEC_PER_DEG = {seconds:.1f} / D   (e.g. 300 -> {seconds/300:.5f}).")


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag); v = cast(args[i + 1]); del args[i:i + 2]; return v
    return None


def capture(shots=ORBIT_SHOTS, radius=ORBIT_RADIUS, log=print):
    """Set up the robot + D405, run the orbit, tear down. Returns the session folder.
    Used by run.py and by main(). Pi only (imports RasBot)."""
    from setup_and_api.api import RasBot, Color
    cam = ColorStereoCapture()
    log("Connecting to robot board...")
    with RasBot() as bot:
        try:
            bot.set_all_leds_color(Color.BLUE)
        except Exception:
            pass
        try:
            session = run_orbit(bot, cam, shots=shots, radius=radius, log=log)
            bot.set_all_leds_color(Color.GREEN); bot.beep(0.15)
            return session
        except Exception as e:
            bot.stop(); bot.set_all_leds_color(Color.RED)
            log(f"orbit error: {e}"); raise
        finally:
            bot.stop(); cam.close()


def main():
    args = sys.argv[1:]
    cf = _pop(args, "--calibrate-fwd", float)
    if cf is not None:
        calibrate_forward(cf); return
    ct = _pop(args, "--calibrate-turn", float)
    if ct is not None:
        calibrate_turn(ct); return

    shots = _pop(args, "--shots", int) or ORBIT_SHOTS
    radius = _pop(args, "--radius", float) or ORBIT_RADIUS
    capture(shots=shots, radius=radius)


if __name__ == "__main__":
    main()
