"""
vision_follow.py — AI vision-based line follower (USB webcam + OpenCV).

Replaces IR sensors with real computer vision. The robot "sees" the tape via
the USB camera, detects the dark tape against the light floor, and steers
based on where the tape's centroid is. Same PD control as best_follow.py.

Run:  python3 src/tape_following/vision_follow.py
      Or: press G in drive.py to toggle vision mode (WASD + camera preview HUD)
"""

import os
import sys
import time
import threading
import numpy as np
import cv2

# ============================================================================
#   TUNING CONSTANTS (all adjustable for tape type / lighting / floor)
# ============================================================================

# ── Control (mirrors best_follow.py proportionally)
SPEED         = 185     # cruise PWM (same as IR follower)
Kp            = 0.45    # proportional gain (vision error is normalized [-1, +1])
Kd            = 0.25    # derivative gain — dampens oscillation
SMOOTH        = 0.35    # EMA motor smoothing (identical to best_follow)

# ── Vision pipeline
ROI_TOP_FRAC  = 0.60    # keep bottom 40% (row = int(H * ROI_TOP_FRAC))
BLUR_K        = 7       # Gaussian kernel (must be odd)
THRESH_VALUE  = 90      # binary threshold: tape (dark) = 0, floor (light) = 255
THRESH_INV    = True    # True = dark tape on light floor
MIN_AREA      = 800     # minimum contour area (noise filter)
ADAPTIVE_THRESH = True  # if True, use adaptive threshold (robust to uneven lighting)

# ── Lost-line recovery
END_LOST_SEC  = 3.0     # stop run after tape is missing this long
LOST_SPIN     = 110     # in-place spin speed when searching for tape

# ── Overlay tuning
PREVIEW_W     = 320     # camera overlay width in pygame HUD
PREVIEW_H     = 180     # camera overlay height

# ── Loop rate
LOOP_DELAY    = 0.008   # 125 Hz control loop (same as best_follow)
PWM_MIN, PWM_MAX = -255, 255


def clamp(val, lo=PWM_MIN, hi=PWM_MAX):
    return max(lo, min(val, hi))


# ============================================================================
#   THREAD-SAFE FRAME SHARING (between vision thread and pygame thread)
# ============================================================================

_frame_lock = threading.Lock()
_frame_store = [None]


def _set_frame(f):
    """Store the latest annotated frame (thread-safe). Call from vision thread."""
    with _frame_lock:
        _frame_store[0] = f.copy() if f is not None else None


def _latest_frame():
    """Retrieve the latest annotated frame (thread-safe). Call from pygame thread."""
    with _frame_lock:
        return _frame_store[0]


# ============================================================================
#   VISION PIPELINE
# ============================================================================

class _VisionPipeline:
    """Processes a USB camera frame to detect the tape and compute steering error."""

    def __init__(self, frame_w=640, frame_h=480):
        self.W = frame_w
        self.H = frame_h
        self.cx_center = frame_w // 2
        self.roi_row = int(frame_h * ROI_TOP_FRAC)

    def process(self, bgr_frame) -> tuple:
        """
        Process one USB camera frame.
        Returns (normalized_error, annotated_bgr):
          normalized_error: float in [-1, +1], or None if tape not found
          annotated_bgr: 640x480 BGR with visual feedback overlays
        """
        if bgr_frame is None:
            return None, self._no_signal_frame()

        annotated = bgr_frame.copy()
        H, W = bgr_frame.shape[:2]

        # 1. Extract ROI (bottom 40%)
        roi = bgr_frame[self.roi_row:, :]
        roi_h, roi_w = roi.shape[:2]

        # 2. Preprocess: grayscale → blur → threshold
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (BLUR_K, BLUR_K), 0)

        if ADAPTIVE_THRESH:
            # Adaptive threshold: robust to uneven lighting
            mask = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 31, 5
            )
        else:
            # Fixed threshold
            _, mask = cv2.threshold(
                blurred, THRESH_VALUE, 255,
                cv2.THRESH_BINARY_INV if THRESH_INV else cv2.THRESH_BINARY
            )

        # 3. Find contours and pick the largest
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]

        state = "TRACKING"
        error = None

        if contours:
            largest = max(contours, key=cv2.contourArea)

            # 4. Compute centroid
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy_roi = int(M["m01"] / M["m00"])

                # 5. Normalized error (range [-1, +1])
                error = (cx - self.cx_center) / self.cx_center

                # 6. Draw annotations on the full frame
                # Green ROI box
                cv2.rectangle(annotated, (0, self.roi_row), (W, H), (0, 255, 0), 2)

                # Cyan contour outline (shifted to full-frame coords)
                contour_shifted = largest + np.array([0, self.roi_row])
                cv2.drawContours(annotated, [contour_shifted], 0, (255, 255, 0), 2)

                # Red centroid dot (shifted)
                centroid_x = cx
                centroid_y = cy_roi + self.roi_row
                cv2.circle(annotated, (centroid_x, centroid_y), 8, (0, 0, 255), -1)

                # Orange error arrow from frame-center to centroid
                center_y = self.roi_row + roi_h // 2
                cv2.arrowedLine(
                    annotated,
                    (self.cx_center, center_y),
                    (centroid_x, centroid_y),
                    (0, 140, 255), 2, tipLength=0.3
                )

                # Vertical dashed center line for reference
                for y in range(self.roi_row, H, 10):
                    cv2.line(annotated, (self.cx_center, y), (self.cx_center, min(y + 5, H)),
                            (100, 100, 100), 1)

                # Text annotations
                cv2.putText(annotated, f"err: {error:+.3f}", (10, self.roi_row + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        else:
            state = "LOST"

        # Add state text
        color = (0, 255, 0) if error is not None else (0, 165, 255)
        cv2.putText(annotated, state, (10, self.roi_row + 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return error, annotated

    def _no_signal_frame(self):
        """Return a 'NO SIGNAL' frame."""
        frame = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        cv2.putText(frame, "NO SIGNAL", (self.W // 2 - 80, self.H // 2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        return frame


# ============================================================================
#   VISION FOLLOWER (mirrors best_follow._Follower)
# ============================================================================

class _VisionFollower:
    """PD line follower using USB camera vision input."""

    def __init__(self):
        self.actual_L = 0.0
        self.actual_R = 0.0
        self.last_error = 0.0
        self._lost_since = None
        self.finished = False
        self.finish_reason = None
        self._pipeline = _VisionPipeline()
        self.latest_annotated = None
        self._tick = 0

    def step(self, bot) -> None:
        """Main control loop iteration."""
        self._tick += 1

        # Capture USB camera frame
        try:
            frame = bot.capture_usb_frame()
            if frame is None:
                error = None
                annotated = self._pipeline._no_signal_frame()
            else:
                error, annotated = self._pipeline.process(frame)
        except Exception as e:
            print(f"[vision] capture error: {e}")
            error = None
            annotated = self._pipeline._no_signal_frame()

        self.latest_annotated = annotated
        _set_frame(annotated)

        # ── LOST LINE (all white) ──
        if error is None:
            if self._lost_since is None:
                self._lost_since = time.time()
            lost_duration = time.time() - self._lost_since

            if lost_duration < 0.5:
                # Debounce: coast briefly
                nudge = clamp(int(Kp * (self.last_error if self.last_error else 0)))
                self._apply(bot, SPEED * 0.6 + nudge, SPEED * 0.6 - nudge)
            elif lost_duration < END_LOST_SEC:
                # Spin toward last-known direction
                spin_dir = 1 if self.last_error < 0 else -1
                self._apply(bot, spin_dir * LOST_SPIN, -spin_dir * LOST_SPIN)
            else:
                # Give up
                self.finished = True
                self.finish_reason = "lost tape"
                bot.stop()
            return

        self._lost_since = None

        # ── ON TAPE: PD CONTROL ──
        derivative = error - self.last_error
        correction = Kp * error + Kd * derivative
        self.last_error = error

        target_L = SPEED + correction * SPEED
        target_R = SPEED - correction * SPEED

        # EMA smoothing
        self.actual_L += (target_L - self.actual_L) * SMOOTH
        self.actual_R += (target_R - self.actual_R) * SMOOTH

        self._apply(bot, self.actual_L, self.actual_R)

    def _apply(self, bot, l, r):
        """Apply clamped PWM to motors."""
        bot._apply_motors(clamp(l), clamp(l), clamp(r), clamp(r))


# ============================================================================
#   ENTRY POINT
# ============================================================================

def run(bot, stop_event=None, **kwargs):
    """
    Main vision-following routine. Called by drive.py G-key.
    Signature matches best_follow.run() exactly.
    """
    follower = _VisionFollower()
    print("=== VISION FOLLOWER STARTED ===")
    print(f"Speed={SPEED}  Kp={Kp}  Kd={Kd}  Smooth={SMOOTH}")

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
#   SELF-TEST / UNIT TEST
# ============================================================================

if __name__ == "__main__":
    import sys

    # Synthetic test frame: white background + black off-center horizontal band
    test_w, test_h = 640, 480
    test_frame = np.ones((test_h, test_w, 3), dtype=np.uint8) * 255
    # Black band (tape) offset to the right (positive error expected)
    roi_y = int(test_h * ROI_TOP_FRAC)
    test_frame[roi_y + 50:roi_y + 100, 350:450] = 0  # dark band at x=[350:450]

    pipeline = _VisionPipeline(test_w, test_h)
    error, annotated = pipeline.process(test_frame)

    print(f"Test result: error={error}")
    if error is not None:
        expected_cx = 400  # band center
        expected_error = (expected_cx - 320) / 320
        print(f"Expected: ~{expected_error:.2f}")
        assert abs(error - expected_error) < 0.05, f"Error mismatch: got {error}, want ~{expected_error}"
        print("[OK] Pipeline test passed")
    else:
        print("[WARN] Pipeline did not detect tape")

    # Optionally display the annotated result
    try:
        cv2.namedWindow("Vision Pipeline Test", cv2.WINDOW_NORMAL)
        cv2.imshow("Vision Pipeline Test", annotated)
        print("Press any key to close...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"[info] cv2.imshow not available (headless or no display): {e}")
