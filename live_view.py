"""Live view from the Raspbot V2 USB camera. Press Q to quit.

Run on the Raspberry Pi inside a GUI/VNC session (window needs a display).
"""
import sys
import time
import cv2

W, H, FPS = 640, 480, 30
CAM_INDEX = 0          # /dev/video0 ; try 1, 2 ... if wrong device

# V4L2 backend is the right one on Raspberry Pi / Linux
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    # fall back to default backend (and let OpenCV pick)
    cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    sys.exit(f"Cannot open camera index {CAM_INDEX}. "
             f"Check `ls /dev/video*` and try a different CAM_INDEX.")

# MJPG lets cheap USB cams hit higher res/fps over USB2
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS,          FPS)

aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera open: {aw}x{ah}. Press Q to quit.")

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
            fps = 0.9 * fps + 0.1 * (1.0 / dt)   # smoothed
        cv2.putText(frame, f"{fps:4.1f} FPS", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Raspbot V2 — live (Q=quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
