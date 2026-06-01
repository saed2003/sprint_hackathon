"""Live view from the Raspbot V2 USB camera. Press Q to quit.

Run on the Raspberry Pi inside a GUI/VNC session (window needs a display).

To hit 30 fps the script first tries MJPG (compressed, low USB bandwidth).
Some cams return black frames under MJPG; if that happens it auto-falls back
to the camera's native (uncompressed) format.
"""
import sys
import time
import cv2

W, H, FPS = 640, 480, 30
CAM_INDEX = 0          # /dev/video0 ; bump if wrong device


def open_cam(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)        # default backend fallback
    return cap


def configure(cap, use_mjpg):
    """Set format/res/fps. Returns True if frames look valid (not black)."""
    if use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    # warm up + sanity-check brightness; black MJPG decode -> mean ~0
    ok = False
    for _ in range(5):
        ok, frame = cap.read()
    return ok and frame is not None and frame.mean() > 5.0


cap = open_cam(CAM_INDEX)
if not cap.isOpened():
    sys.exit(f"Cannot open camera index {CAM_INDEX}. "
             f"Check `v4l2-ctl --list-devices` and try a different CAM_INDEX.")

# try MJPG (for 30 fps); if it returns black frames, reopen in native format
if configure(cap, use_mjpg=True):
    mode = "MJPG"
else:
    cap.release()
    cap = open_cam(CAM_INDEX)
    configure(cap, use_mjpg=False)
    mode = "native"

aw  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
ah  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
afp = cap.get(cv2.CAP_PROP_FPS)
print(f"Camera open: {aw}x{ah} @ {afp:.0f}fps  mode={mode}. Press Q to quit.")

prev = time.time()
fps = 0.0
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed; retrying ...")
            continue

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)   # smoothed measured fps
        cv2.putText(frame, f"{fps:4.1f} FPS", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Raspbot V2 - live (Q=quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
