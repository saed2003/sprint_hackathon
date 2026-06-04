"""
vision_follow.py — AI vision-based line follower (USB webcam + OpenCV).

Replaces IR sensors with real computer vision. The robot "sees" the black tape
through the USB camera and not only tracks it, but understands the road ahead:
turns, T-junctions, crosses, forks, and dead-ends — then decides which way to go.

HOW IT SEES (multi-band scanning)
  The bottom slice of the frame (the floor ahead) is split into several
  horizontal BANDS, near->far. In every band we find the tape segment(s):
    • the NEAR band gives the immediate steering error (PD control)
    • comparing band centers near->far reveals a CURVE / hard TURN
    • counting segments + which screen edges the tape touches reveals
      JUNCTIONS:  left/right 90 turns, T-junctions, 4-way crosses, forks,
      and dead-ends.
  At a junction the follower PICKS a branch (policy below) and commits a
  latched maneuver to drive onto it — it never dithers or U-turns.

DECISION POLICY at a junction (tunable)
  Prefer STRAIGHT if open; otherwise take the JUNCTION_DIR side (+1=right,
  -1=left) if open; otherwise the only branch available. A 4-way cross uses
  JUNCTION_DIR directly. Dead-end -> spin to search, then stop after timeout.

Run:  python3 src/tape_following/vision_follow.py        # offline self-test
      In drive.py: press G to toggle vision mode (camera preview in the HUD)
"""

import os
import sys
import time
import threading
import numpy as np
import cv2

# ============================================================================
#   TUNING CONSTANTS  (group everything you adjust here)
# ============================================================================

# ── Control (mirrors best_follow.py scale; vision error is normalized [-1,+1])
SPEED         = 185     # cruise PWM
Kp            = 95      # proportional gain (error is small [-1,+1] -> larger gain)
Kd            = 55      # derivative gain — damps wobble
SMOOTH        = 0.35    # motor EMA smoothing (0.15 silky -> 0.5 snappy)

# ── Tape detection (black tape on a lighter floor)
ROI_TOP_FRAC  = 0.45    # use the bottom 55% of the frame (the floor ahead)
BLUR_K        = 7       # Gaussian kernel (odd)
THRESH_MODE   = "otsu"  # "otsu"  -> auto cutoff (BEST for black tape on light floor)
                        # "adaptive" -> per-region (only for strong lighting gradients;
                        #               note: washes out THICK tape, use thin lines)
                        # "fixed"  -> hard THRESH_VALUE cutoff
ADAPT_BLOCK   = 31      # adaptive threshold neighborhood (odd)
ADAPT_C       = 7       # adaptive threshold bias
THRESH_VALUE  = 90      # fixed-threshold cutoff (THRESH_MODE="fixed")
MORPH_K       = 5       # morphology kernel to clean speckle / close gaps
MIN_AREA      = 600     # ignore tape blobs smaller than this (noise)

# ── Multi-band scanning (near -> far)
N_BANDS       = 5       # horizontal slices across the ROI
COL_FILL_FRAC = 0.30    # a column counts as "tape" if this frac of band height is black
MIN_SEG_FRAC  = 0.04    # ignore tape segments narrower than this frac of width
EDGE_FRAC     = 0.12    # within this frac of a side = "touches that edge" (a branch)
CENTER_FRAC   = 0.22    # a segment within this frac of center = "straight open"
BAR_WIDTH_FRAC= 0.55    # a single segment wider than this = a perpendicular BAR
CURVE_TRIG    = 0.45    # |near-far center shift| above this = a hard turn ahead

# ── Junction decision
JUNCTION_DIR  = +1      # tie-break / cross choice: +1 = RIGHT, -1 = LEFT
PREFER_STRAIGHT = True  # at a fork/T, go straight if that branch is open

# ── Turn / junction commit (latched maneuvers, open-loop)
PIVOT_FWD     = 140     # outer wheel during a pivot
PIVOT_REV     = -90     # inner wheel during a pivot (reverse)
TURN_LATCH    = 60      # ticks committed to a 90 turn (~0.48 s at 8 ms)
JUNCTION_LATCH= 45      # ticks to drive THROUGH a junction onto the chosen branch
REACQUIRE_OK  = 0.30    # |error| below this re-acquires the line -> end the latch

# ── Lost line / dead-end
LOST_SPIN     = 110     # in-place spin speed while searching
DEBOUNCE      = 2       # blank reads before we treat the line as lost
END_LOST_SEC  = 3.0     # stop the run after the tape is gone this long

# ── Overlay
PREVIEW_W     = 320     # camera overlay width in the pygame HUD
PREVIEW_H     = 180     # camera overlay height

LOOP_DELAY    = 0.008   # 125 Hz control loop
PWM_MIN, PWM_MAX = -255, 255


def clamp(val, lo=PWM_MIN, hi=PWM_MAX):
    return max(lo, min(int(val), hi))


# ============================================================================
#   THREAD-SAFE FRAME SHARING (vision thread -> pygame HUD thread)
# ============================================================================

_frame_lock  = threading.Lock()
_frame_store = [None]


def _set_frame(f):
    with _frame_lock:
        _frame_store[0] = f.copy() if f is not None else None


def _latest_frame():
    with _frame_lock:
        return _frame_store[0]


# ============================================================================
#   PERCEPTION RESULT
# ============================================================================

class Perception:
    """Everything the vision pipeline understood about the road ahead."""
    def __init__(self):
        self.error         = None     # normalized steering error [-1,+1] or None
        self.band_centers  = []       # near->far: normalized x of main tape, or None
        self.left_open     = False    # a branch exits the LEFT side ahead
        self.right_open    = False    # a branch exits the RIGHT side ahead
        self.straight_open = False    # tape continues straight ahead
        self.bar           = False    # a perpendicular bar (cross / T / stop marker)
        self.curve_dir     = 0        # -1 left, +1 right, 0 straight (gentle curve)
        self.junction      = "none"   # see classify(): straight/turn_*/T/cross/fork_*/dead_end
        self.n_branches     = 0       # how many ways you could go from here


# ============================================================================
#   VISION PIPELINE
# ============================================================================

class _VisionPipeline:
    """Turns a camera frame into a Perception + an annotated frame."""

    def __init__(self, frame_w=640, frame_h=480):
        self.W = frame_w
        self.H = frame_h
        self.cx = frame_w // 2
        self.roi_row = int(frame_h * ROI_TOP_FRAC)
        self._morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_K, MORPH_K))

    # ── mask the black tape ──────────────────────────────────────────────
    def _mask(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (BLUR_K, BLUR_K), 0)
        if THRESH_MODE == "adaptive":
            mask = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, ADAPT_BLOCK, ADAPT_C)
        elif THRESH_MODE == "fixed":
            _, mask = cv2.threshold(blurred, THRESH_VALUE, 255, cv2.THRESH_BINARY_INV)
        else:  # "otsu" — auto cutoff between dark tape and light floor
            _, mask = cv2.threshold(blurred, 0, 255,
                                    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        # clean speckle and close small gaps in the tape
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._morph)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph)
        return mask

    # ── group a band's tape columns into segments ────────────────────────
    def _segments(self, band_mask):
        """Return list of (x0, x1, center, width_frac) tape runs in one band."""
        h, w = band_mask.shape
        col = (band_mask > 0).sum(axis=0)                 # tape pixels per column
        on  = col >= (COL_FILL_FRAC * h)                  # column is "tape"
        segs = []
        x = 0
        min_w = MIN_SEG_FRAC * w
        while x < w:
            if on[x]:
                x0 = x
                while x < w and on[x]:
                    x += 1
                x1 = x - 1
                if (x1 - x0 + 1) >= min_w:
                    segs.append((x0, x1, (x0 + x1) / 2.0, (x1 - x0 + 1) / float(w)))
            else:
                x += 1
        return segs

    # ── full analysis ────────────────────────────────────────────────────
    def process(self, bgr_frame):
        if bgr_frame is None:
            return self._lost_perception(), self._no_signal_frame()

        annotated = bgr_frame.copy()
        H, W = bgr_frame.shape[:2]
        roi = bgr_frame[self.roi_row:, :]
        roi_h = roi.shape[0]
        mask = self._mask(roi)

        p = Perception()
        band_h = roi_h // N_BANDS
        bands = []           # (band_index, y0_abs, y1_abs, segments)

        for b in range(N_BANDS):
            y0 = b * band_h
            y1 = roi_h if b == N_BANDS - 1 else (b + 1) * band_h
            segs = self._segments(mask[y0:y1, :])
            bands.append((b, self.roi_row + y0, self.roi_row + y1, segs))

        # band 0 = nearest (bottom of image), band N-1 = farthest ahead
        # NB: image rows increase downward, so the bottom band is the LAST one.
        # We want near = bottom = highest y. Re-order near->far:
        bands_near_far = sorted(bands, key=lambda t: -t[1])

        # ── steering error from the nearest band that has tape ──
        near_center = None
        for (_, _, _, segs) in bands_near_far:
            if segs:
                main = max(segs, key=lambda s: s[3])      # widest segment
                near_center = main[2]
                break

        # ── record each band's main center near->far (for curve sensing) ──
        for (_, _, _, segs) in bands_near_far:
            if segs:
                main = max(segs, key=lambda s: s[3])
                p.band_centers.append((main[2] - self.cx) / self.cx)
            else:
                p.band_centers.append(None)

        if near_center is not None:
            p.error = (near_center - self.cx) / self.cx

        # ── branch / bar sensing from the FAR half of the ROI ──
        # left/right openings and the crossing bar can appear anywhere in the
        # far half (the junction may be near or a little farther).
        far_bands = bands_near_far[len(bands_near_far) // 2:]   # upper (far) bands
        for (_, _, _, segs) in far_bands:
            for (x0, x1, c, wf) in segs:
                if x0 <= EDGE_FRAC * W:
                    p.left_open = True
                if x1 >= (1 - EDGE_FRAC) * W:
                    p.right_open = True
                if wf >= BAR_WIDTH_FRAC:
                    p.bar = True

        # ── straight-open: ONLY if the line reaches the farthest band as a
        # narrow, roughly-centered run (i.e. the road truly continues ahead) —
        # a turn or T-junction has no such run at the far edge of view.
        far_segs = bands_near_far[-1][3]
        for (x0, x1, c, wf) in far_segs:
            if wf < BAR_WIDTH_FRAC and abs(c - self.cx) <= CENTER_FRAC * W:
                p.straight_open = True

        # ── curvature: how far the far center drifts from the near center ──
        valid = [c for c in p.band_centers if c is not None]
        if len(valid) >= 2:
            shift = valid[-1] - valid[0]                  # far - near
            if abs(shift) >= CURVE_TRIG:
                p.curve_dir = 1 if shift > 0 else -1

        p.junction = self._classify(p)
        p.n_branches = int(p.left_open) + int(p.straight_open) + int(p.right_open)

        self._annotate(annotated, mask, bands, p)
        return p, annotated

    # ── classify the road ahead ──────────────────────────────────────────
    def _classify(self, p):
        L, S, R = p.left_open, p.straight_open, p.right_open
        if not (L or S or R):
            return "dead_end"
        if L and R and S:
            return "cross"               # 4-way (straight + both sides)
        if L and R and not S:
            return "T"                   # T-junction: left or right, no straight
        if S and L and not R:
            return "fork_left"           # go straight or left
        if S and R and not L:
            return "fork_right"          # go straight or right
        if L and not S and not R:
            return "turn_left"           # 90 left
        if R and not S and not L:
            return "turn_right"          # 90 right
        return "straight"                # S only

    # ── draw the rich presentation overlay ───────────────────────────────
    def _annotate(self, img, mask, bands, p):
        H, W = img.shape[:2]
        # ROI boundary
        cv2.rectangle(img, (0, self.roi_row), (W - 1, H - 1), (0, 120, 0), 1)
        # center reference line
        for y in range(self.roi_row, H, 12):
            cv2.line(img, (self.cx, y), (self.cx, min(y + 6, H)), (90, 90, 90), 1)

        # band separators + detected segment centers
        for (b, y0a, y1a, segs) in bands:
            cv2.line(img, (0, y0a), (W - 1, y0a), (40, 40, 40), 1)
            ymid = (y0a + y1a) // 2
            for (x0, x1, c, wf) in segs:
                col = (0, 255, 255) if wf < BAR_WIDTH_FRAC else (0, 0, 255)  # cyan / red bar
                cv2.line(img, (x0, ymid), (x1, ymid), col, 3)
                cv2.circle(img, (int(c), ymid), 4, (255, 255, 0), -1)

        # nearest-band steering arrow
        if p.error is not None:
            tip_x = int(self.cx + p.error * self.cx)
            base_y = H - 12
            cv2.arrowedLine(img, (self.cx, base_y), (tip_x, base_y - 40),
                            (0, 140, 255), 3, tipLength=0.3)

        # branch availability badges (which ways you can go)
        def badge(txt, on, x):
            color = (60, 230, 60) if on else (70, 70, 70)
            cv2.putText(img, txt, (x, self.roi_row - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        badge("<L", p.left_open,     12)
        badge("^S", p.straight_open, W // 2 - 16)
        badge("R>", p.right_open,    W - 52)

        # headline: junction class + steering
        head = f"{p.junction.upper()}"
        if p.error is not None:
            head += f"  err:{p.error:+.2f}"
        cv2.putText(img, head, (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # ── helpers ──
    def _lost_perception(self):
        p = Perception()
        p.junction = "dead_end"
        return p

    def _no_signal_frame(self):
        f = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        cv2.putText(f, "NO SIGNAL", (self.W // 2 - 90, self.H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        return f


# ============================================================================
#   VISION FOLLOWER  (state machine mirrors best_follow._Follower)
# ============================================================================

class _VisionFollower:
    def __init__(self):
        self.actual_L = 0.0
        self.actual_R = 0.0
        self.last_error = 0.0
        self.latch_ticks = 0          # committed turn / junction maneuver
        self.latch_dir   = 0          # -1 left, +1 right
        self._last_turn_dir = 0
        self.lost_ticks  = 0
        self._lost_since = None
        self.finished = False
        self.finish_reason = None
        self.state = "INIT"
        self._pipeline = _VisionPipeline()
        self._tick = 0

    # ── main step ─────────────────────────────────────────────────────────
    def step(self, bot):
        self._tick += 1
        now = time.time()

        # capture + perceive
        try:
            frame = bot.capture_usb_frame()
            p, annotated = self._pipeline.process(frame)
        except Exception as e:
            print(f"[vision] capture error: {e}")
            p, annotated = self._pipeline._lost_perception(), self._pipeline._no_signal_frame()

        # overlay the running state, then publish for the HUD
        cv2.putText(annotated, self.state, (10, annotated.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 200, 255) if p.error is None else (0, 230, 0), 2)
        _set_frame(annotated)

        # ── committed turn / junction maneuver: finish it first ──
        if self.latch_ticks > 0:
            reacquired = (p.error is not None and abs(p.error) <= REACQUIRE_OK
                          and p.junction in ("straight", "none"))
            if reacquired:
                self.latch_ticks = 0
            else:
                self.latch_ticks -= 1
                self._pivot(bot, self.latch_dir)
                self._lost_since = None
                return

        # ── junction: pick a branch and commit ──
        if p.junction in ("cross", "T", "fork_left", "fork_right",
                           "turn_left", "turn_right"):
            d = self._choose(p)
            self.state = f"{p.junction.upper()}->{'R' if d > 0 else 'L' if d < 0 else 'S'}"
            if d == 0:
                # go straight through the junction: drive forward briefly
                self.latch_ticks = 0
                self._lost_since = None
                self._drive(bot, SPEED, SPEED)
                self.last_error = 0.0
                return
            self.latch_dir = d
            self.latch_ticks = (JUNCTION_LATCH if p.junction in
                                ("cross", "T", "fork_left", "fork_right") else TURN_LATCH)
            self._last_turn_dir = d
            self._pivot(bot, d)
            self.last_error = float(d)
            self._lost_since = None
            return

        # ── dead-end / lost line ──
        if p.error is None or p.junction == "dead_end":
            self.lost_ticks += 1
            if self._lost_since is None:
                self._lost_since = now
            if now - self._lost_since >= END_LOST_SEC:
                self.finished = True
                self.finish_reason = "tape lost / dead-end"
                self.state = "END"
                bot.stop()
                return
            if self.lost_ticks < DEBOUNCE:
                self.state = "COAST"
                self._drive(bot, self.actual_L, self.actual_R)
                return
            # search toward the side we last steered/turned
            side = self._last_turn_dir if self._last_turn_dir != 0 else (
                -1 if self.last_error < 0 else 1)
            self.state = f"SEARCH({'R' if side > 0 else 'L'})"
            self._pivot(bot, side, speed_fwd=LOST_SPIN, speed_rev=-LOST_SPIN)
            return

        # ── on the line ──
        self.lost_ticks = 0
        self._lost_since = None
        error = p.error

        # hard curve ahead but no branch -> nudge harder (not a full pivot)
        if p.curve_dir != 0 and abs(error) < REACQUIRE_OK:
            error += 0.35 * p.curve_dir

        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = p.error

        target_L = SPEED + correction
        target_R = SPEED - correction
        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH
        self._drive(bot, self.actual_L, self.actual_R)

        if abs(p.error) < 0.08:        self.state = "STRAIGHT"
        elif abs(p.error) < 0.35:      self.state = "SLIGHT"
        else:                          self.state = "CURVE"

    # ── junction decision policy ────────────────────────────────────────
    def _choose(self, p):
        """Return -1 (left), +1 (right), or 0 (straight)."""
        if p.junction == "turn_left":   return -1
        if p.junction == "turn_right":  return +1
        if p.junction == "cross":       # 4-way: prefer straight, else configured
            if PREFER_STRAIGHT and p.straight_open:
                return 0
            return JUNCTION_DIR
        # T / fork
        if PREFER_STRAIGHT and p.straight_open:
            return 0
        # take JUNCTION_DIR side if open, else whatever is open
        if JUNCTION_DIR > 0 and p.right_open:  return +1
        if JUNCTION_DIR < 0 and p.left_open:   return -1
        if p.right_open:                       return +1
        if p.left_open:                        return -1
        return 0

    # ── motor helpers ───────────────────────────────────────────────────
    def _drive(self, bot, l, r):
        self.actual_L, self.actual_R = float(l), float(r)
        bot._apply_motors(clamp(l), clamp(l), clamp(r), clamp(r))

    def _pivot(self, bot, direction, speed_fwd=PIVOT_FWD, speed_rev=PIVOT_REV):
        if direction < 0:   # pivot LEFT: left wheel reverse, right wheel forward
            self._drive(bot, speed_rev, speed_fwd)
        else:               # pivot RIGHT
            self._drive(bot, speed_fwd, speed_rev)


# ============================================================================
#   ENTRY POINT  (signature matches best_follow.run)
# ============================================================================

def run(bot, stop_event=None, **kwargs):
    follower = _VisionFollower()
    print("=== VISION FOLLOWER (turns + junctions) STARTED ===")
    print(f"Speed={SPEED} Kp={Kp} Kd={Kd}  bands={N_BANDS}  junction_dir={JUNCTION_DIR:+d}")
    try:
        while stop_event is None or not stop_event.is_set():
            follower.step(bot)
            if follower.finished:
                print(f"\n*** VISION RUN COMPLETE: {follower.finish_reason} ***")
                break
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        bot.stop()


# ============================================================================
#   SELF-TEST  (synthetic frames; no robot, no camera)
# ============================================================================

def _make_frame(kind):
    """Build a synthetic 640x480 white frame with black tape in some layout.

    Zones up the ROI:  NEAR (bottom)  ->  JUNCTION (middle)  ->  AHEAD (top).
    A line that CONTINUES straight reaches the AHEAD zone; a line that turns or
    ends at a junction stops in the JUNCTION zone.
    """
    W, H = 640, 480
    f = np.ones((H, W, 3), dtype=np.uint8) * 230
    roi_y = int(H * ROI_TOP_FRAC)
    cx = W // 2
    span = H - roi_y
    y_junc  = roi_y + int(span * 0.30)        # top of the junction zone
    y_near  = roi_y + int(span * 0.55)        # where NEAR begins
    y_bar0, y_bar1 = y_junc, y_junc + 40      # crossing-bar rows

    def vbar(y0, y1, x=cx):                    # vertical tape segment
        f[y0:y1, x - 25:x + 25] = 20
    def hbar(x0, x1):                          # perpendicular crossing bar
        f[y_bar0:y_bar1, x0:x1] = 20

    if kind == "straight":
        vbar(roi_y, H)                                   # full height -> continues
    elif kind == "curve_right":
        for y in range(roi_y, H, 8):
            off = int(150 * (1 - (y - roi_y) / span))    # bends right going up
            f[y:y + 8, cx + off - 25: cx + off + 25] = 20
    elif kind == "turn_left":
        vbar(y_junc, H)                                  # approach stops at junction
        hbar(0, cx + 25)                                 # exits LEFT only
    elif kind == "turn_right":
        vbar(y_junc, H)
        hbar(cx - 25, W)                                 # exits RIGHT only
    elif kind == "T":
        vbar(y_near, H)                                  # approach ends at the bar
        hbar(0, W)                                       # full bar: L & R, no straight
    elif kind == "cross":
        vbar(roi_y, H)                                   # straight continues through
        hbar(0, W)                                       # full bar across
    elif kind == "fork_right":
        vbar(roi_y, H)                                   # straight continues
        hbar(cx - 25, W)                                 # AND a branch to the RIGHT
    elif kind == "dead_end":
        vbar(y_near, H)                                  # short stub, NEAR only
    return f


if __name__ == "__main__":
    pipe = _VisionPipeline()
    cases = ["straight", "curve_right", "turn_left", "turn_right",
             "T", "cross", "fork_right", "dead_end"]
    print(f"{'case':12s} {'junction':10s} {'err':>6s}  L S R  curve  branches")
    print("-" * 60)
    show = "--show" in sys.argv
    for kind in cases:
        frame = _make_frame(kind)
        p, annotated = pipe.process(frame)
        err = f"{p.error:+.2f}" if p.error is not None else "  -- "
        print(f"{kind:12s} {p.junction:10s} {err:>6s}  "
              f"{int(p.left_open)} {int(p.straight_open)} {int(p.right_open)}  "
              f"{p.curve_dir:+d}      {p.n_branches}")
        if show:
            cv2.imshow(kind, annotated)
    if show:
        print("\nPress any key to close windows...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    print("\n[OK] self-test complete  (run with --show to view annotated frames)")
