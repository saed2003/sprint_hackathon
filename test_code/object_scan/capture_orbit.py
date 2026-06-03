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
ORBIT_RADIUS  = _CFG["radius"]   # metres from the camera to the object centre (~0.35)
ROTATE_SPEED  = 40          # in-place turn speed (calibration helper)
FORWARD_SPEED = 40          # forward/back drive speed (radius hold)
STRAFE_SPEED  = 40          # sideways strafe speed (orbit advance)
SEC_PER_DEG   = 3.0 / 540   # in-place rotation timing (calibrated: 540 deg in 3.0s @ speed 40)
SEC_PER_M     = 3.0 / 1.2   # forward timing (calibrated: 1.2 m in 3.0s @ speed 40)
STRAFE_EFF    = 0.7         # mecanum strafe covers ~70% of forward distance per second
SETTLE        = 0.4         # pause for the chassis to settle before a shot
ORBIT_DIR     = 1           # +1 = strafe/orbit one way, -1 the other (merge --dir must match)

# vision: steer the BASE to FACE the object (closed loop) + hold the radius.
# The object is always the closest thing in the depth gate, so its horizontal
# centroid tells us how far to turn. We turn until it is centred — open-loop turn
# accuracy doesn't matter. TUNE FACE_SPEED/FACE_PULSE on the real robot.
ZMIN, ZMAX    = _CFG["zmin"], _CFG["zmax"]   # object depth gate (the close thing in frame)
# The vision loop tracks in a slightly WIDER gate than the build's segment gate, so a car
# that drifted a little is still seen and pulled back instead of being lost (runaway).
TRACK_ZMIN    = max(0.10, ZMIN - 0.03)
TRACK_ZMAX    = ZMAX + 0.12
# Floor removal: the depth gate also catches the floor (it fills the lower frame and is
# within range), which would hijack the aim/range loop. We RANSAC the dominant plane and,
# if it's big + ~horizontal, drop it so only the OBJECT (sitting on it) drives the loop.
FLOOR_THRESH  = 0.015       # plane inlier distance (m) — points within this are "on the plane"
FLOOR_MIN_FRAC= 0.30        # only treat a plane as floor if it's at least this fraction of pts
FLOOR_VERT    = 0.6         # ...and its normal is this vertical (|n_y|) — excludes walls
REACQUIRE_PULSES = 5        # base sweeps to re-find the object if it falls out of the gate
FACE_SPEED    = 35          # base turn speed while facing the object (raise if it won't move)
FACE_PULSE    = 0.10        # seconds per corrective turn pulse (lower if it overshoots)
FACE_TOL_PX   = 50          # object counts as 'centred' within this many px of centre
FACE_ITERS    = 8           # max correction pulses per stop
TURN_SIGN     = +1          # flip to -1 if the base turns AWAY from the object
RADIUS_TOL    = 0.05        # only correct distance if off target by more than this (m)
RADIUS_MAX_STEP = 0.10      # cap a single in/out correction drive (m), for safety


def _remove_floor(pts):
    """Return a boolean mask of points that are NOT on the floor.

    RANSAC the dominant plane in `pts` (Nx3, camera frame, +y = down). If that plane is
    large (>= FLOOR_MIN_FRAC of points) AND roughly horizontal (|normal_y| >= FLOOR_VERT),
    it's the floor → keep everything off it. Otherwise (object fills the frame, or only a
    wall was found) keep everything. Pure numpy, so it runs on the Pi (no Open3D)."""
    n = len(pts)
    if n < 200:
        return np.ones(n, bool)
    rng = np.random.default_rng(0)                       # deterministic
    s = pts if n <= 4000 else pts[rng.choice(n, 4000, replace=False)]
    best_cnt, best_nrm, best_p0 = 0, None, None
    for _ in range(80):
        p0, p1, p2 = s[rng.choice(len(s), 3, replace=False)]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm = nrm / ln
        cnt = int((np.abs((s - p0) @ nrm) < FLOOR_THRESH).sum())
        if cnt > best_cnt:
            best_cnt, best_nrm, best_p0 = cnt, nrm, p0
    if best_nrm is None:
        return np.ones(n, bool)
    if best_cnt >= FLOOR_MIN_FRAC * len(s) and abs(best_nrm[1]) >= FLOOR_VERT:
        return np.abs((pts - best_p0) @ best_nrm) >= FLOOR_THRESH   # keep = off the floor
    return np.ones(n, bool)                              # no convincing floor — keep all


def _largest_blob(vs, us, keep, shape):
    """Restrict `keep` (a mask over the gated pixels at rows `vs`, cols `us`) to its
    largest connected blob — the object — so a stray wall/background patch can't drag the
    centroid. Uses cv2 (present on the Pi); returns `keep` unchanged if cv2 is missing."""
    try:
        import cv2
    except Exception:
        return keep
    img = np.zeros(shape, np.uint8)
    img[vs[keep], us[keep]] = 255
    num, lab, stats, _ = cv2.connectedComponentsWithStats(img, 8)
    if num <= 1:
        return keep
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))   # skip background label 0
    return keep & (lab[vs, us] == best)


def object_centroid_and_distance(depth_m, intr):
    """Find the object's horizontal centroid pixel + distance, IGNORING the floor.

    Back-projects the depth-gated pixels to 3D, removes the floor plane (see
    `_remove_floor`), and returns the centroid/median of what's left — the object on its
    stand. Without this the floor (~85k px in our captures) dominated the mask and the
    aim/range loop tracked the floor while the car drifted out of frame.

    Returns (u_centroid, distance_m, n_pixels) or (None, None, 0) if nothing close.
    """
    gate = (depth_m > TRACK_ZMIN) & (depth_m < TRACK_ZMAX)
    if int(gate.sum()) < 500:
        return None, None, 0
    vs, us = np.where(gate)
    z = depth_m[gate]
    x = (us - intr.ppx) * z / intr.fx
    y = (vs - intr.ppy) * z / intr.fy                    # +y = down
    keep = _remove_floor(np.stack([x, y, z], axis=1))
    if int(keep.sum()) < 300:                            # floor removal ate the object — distrust it
        keep = np.ones(len(z), bool)
    keep = _largest_blob(vs, us, keep, depth_m.shape)    # object only, drop background slivers
    if int(keep.sum()) < 200:
        return None, None, 0
    u = float(us[keep].mean())
    d = float(np.median(z[keep]))
    return u, d, int(keep.sum())


def face_object(bot, cam, log=print):
    """Turn the ROBOT BASE (closed loop, vision) until the object is horizontally
    centred — this is the camera 'following' the DB5. Bang-bang on the object's depth
    centroid: pulse a small in-place turn toward it, re-measure, repeat. It does NOT
    rely on open-loop turn accuracy, so it works even when small timed turns are
    unreliable. Returns the object distance (m), or None if the object was lost."""
    dist = None
    for _ in range(FACE_ITERS):
        depth = cam.grab_depth()
        if depth is None:
            break
        u, d, n = object_centroid_and_distance(depth, cam.intr)
        if u is None:
            log("    (object lost — check distance/lighting/ZMIN-ZMAX)")
            break
        dist = d
        err = u - cam.intr.ppx                          # +err: object is to the right
        if abs(err) < FACE_TOL_PX:
            break
        turn_right = (err > 0)                          # object right -> turn base right
        if TURN_SIGN < 0:
            turn_right = not turn_right
        (bot.rotate_right if turn_right else bot.rotate_left)(FACE_SPEED)
        time.sleep(FACE_PULSE)
        bot.stop()
        time.sleep(0.15)                                # settle before re-measuring
    return dist


def hold_radius(bot, cam, radius, log=print):
    """Use the measured object distance to keep the orbit radius ~constant (camera helps
    'find the way'). Drives forward/back if the figure has drifted nearer/farther than
    `radius`. Returns the corrected distance, or None if the object wasn't seen."""
    depth = cam.grab_depth()
    if depth is None:
        return None
    _, d, n = object_centroid_and_distance(depth, cam.intr)
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
            _, d2, _ = object_centroid_and_distance(depth, cam.intr)
            d = d2 or d
    return d


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


def orbit_strafe(bot, chord, log=print):
    """Advance one step AROUND the object by strafing sideways (mecanum) by `chord`
    metres — that's tangent to the orbit, so the camera keeps looking at the object
    instead of driving into it. The next face_object() re-centres the small residual
    bearing change. Strafe covers ~STRAFE_EFF of the forward distance per second."""
    t = (SEC_PER_M / max(0.1, STRAFE_EFF)) * chord
    (bot.right if ORBIT_DIR >= 0 else bot.left)(STRAFE_SPEED)
    time.sleep(t)
    bot.stop()
    log(f"    orbit strafe: {'right' if ORBIT_DIR >= 0 else 'left'} {chord*100:.1f}cm ({t:.2f}s)")


def reacquire_object(bot, cam, log=print):
    """If the object fell out of the gate, sweep the base in widening left/right pulses to
    bring it back BEFORE advancing — otherwise the orbit runs away tracking nothing.
    Returns True once the object is back in the gate, else False after REACQUIRE_PULSES."""
    log("    object lost — sweeping to re-acquire")
    for k in range(1, REACQUIRE_PULSES + 1):
        depth = cam.grab_depth()
        if depth is not None and object_centroid_and_distance(depth, cam.intr)[0] is not None:
            return True
        (bot.rotate_right if k % 2 else bot.rotate_left)(FACE_SPEED)   # alternate, widening
        time.sleep(FACE_PULSE * k)
        bot.stop(); time.sleep(0.15)
    return False


def _bearing_deg(cam):
    """Object's horizontal bearing off the optical axis, in degrees (signed; + = object to
    the right). None if not seen. Used to MEASURE the orbit advance per step: after a
    tangential strafe the object swings off-centre by ~the orbit angle covered
    (atan(chord/R) ≈ θ), so the camera closes the ANGLE loop too — strafe slip can no
    longer cut the circle short."""
    depth = cam.grab_depth()
    if depth is None:
        return None
    u, d, n = object_centroid_and_distance(depth, cam.intr)
    if u is None:
        return None
    return math.degrees(math.atan2(u - cam.intr.ppx, cam.intr.fx))


def run_orbit(bot, cam, shots=ORBIT_SHOTS, radius=ORBIT_RADIUS, out_root=None, log=print):
    """Stop-and-shoot orbit. Drives until a FULL 360° **measured by the camera** (object
    bearing swing per step), NOT a fixed shot count — so mecanum strafe slip can't stop it
    half-way. `shots` only sets the strafe chord (nominal step size). Returns the session."""
    out_root = out_root or default_out_root()
    session = os.path.join(out_root, "orbit_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    step = 360.0 / shots                                # nominal step -> sets the strafe chord
    chord = 2.0 * radius * math.sin(math.radians(step) / 2.0)
    max_shots = shots * 3                               # safety cap if per-step advance is tiny
    if cam.pipeline is None:
        log("Opening D405 (warming up)..."); cam.start(); log(cam.info())

    log(f"robot orbit: target 360°, ~{step:.0f}°/step (chord={chord*100:.1f}cm), "
        f"R={radius*100:.0f}cm, cap {max_shots} shots  (vision-measured advance)")
    bot.set_pan(90)                                     # camera fixed forward; the BASE aims
    face_object(bot, cam, log=log)                      # initial aim (not counted as advance)
    cumulative = 0.0
    i = 0
    while True:
        bot.stop(); time.sleep(SETTLE)
        dist = face_object(bot, cam, log=log)           # vision: turn the base to face the DB5
        if dist is None and reacquire_object(bot, cam, log=log):
            dist = face_object(bot, cam, log=log)       # re-found it -> face it again
        held = hold_radius(bot, cam, radius, log=log)   # vision: keep ~constant radius
        dist = held or dist
        face_object(bot, cam, log=log)                  # re-face after any in/out nudge
        if dist is None:
            log("    object still not in view — capturing anyway; the merge drops bad views")
        folder = os.path.join(session, f"shot_{i:02d}")
        ok = cam.save_to(folder)
        with open(os.path.join(folder, "angle.txt"), "w") as f:
            f.write(f"{cumulative:.3f}\n")              # MEASURED cumulative angle (better prior)
        log(f"  shot {i+1} (~{cumulative:.0f}/360°, d={dist*100 if dist else float('nan'):.0f}cm)"
            + ("" if ok else "  FRAME DROPPED"))
        try:
            bot.beep(0.05)
        except Exception:
            pass
        if cumulative >= 360.0 - 0.5 * step or i >= max_shots - 1:
            break
        b0 = _bearing_deg(cam) or 0.0                   # ~0 (just centred)
        orbit_strafe(bot, chord, log=log)              # advance one tangential step
        time.sleep(0.15)
        b1 = _bearing_deg(cam)                          # how far the object swung = orbit advance
        if b1 is None:
            adv = step                                  # lost after strafe -> assume nominal
        else:
            adv = abs(b1 - b0)
            if adv < 0.3 * step or adv > 3.0 * step:    # implausible measurement -> nominal
                adv = step
        cumulative += adv
        log(f"    advanced ~{adv:.1f}° (total {cumulative:.0f}/360°)")
        i += 1
    bot.stop()
    log(f"  orbit complete: {i + 1} shots, {cumulative:.0f}° -> {session}")
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
