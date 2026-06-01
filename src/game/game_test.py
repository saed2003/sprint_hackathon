"""
Street View Robot — VNC Game
=============================
Run AFTER the robot has completed its scanning run.

The game shows a split screen on the VNC display:
  LEFT  — live camera feed (what the robot sees right now)
  RIGHT — top-down map built from all captured point clouds

The player drives the real robot with WASD/QE and must physically
navigate it to the TARGET location (the X-marked cross on the floor).
When all 4 IR sensors detect the cross → YOU WIN!

Run from the project root on the Pi:
    python3 src/game/game.py

Requirements:
    pip install pygame
    (numpy and opencv-python are already installed)
"""

import os
import sys
import glob
import time

import numpy as np
import cv2
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ── settings ──────────────────────────────────────────────────────────────────
SCREEN_W   = 1280
SCREEN_H   = 720
DRIVE_SPEED = 110   # robot forward/back/strafe speed
ROT_SPEED   = 80    # robot rotation speed
FPS         = 30
GAME_TIME   = 90    # seconds before time is up (0 = no limit)

# ── colours ───────────────────────────────────────────────────────────────────
C_BLACK  = (  0,   0,   0)
C_WHITE  = (255, 255, 255)
C_GREEN  = (  0, 200,   0)
C_RED    = (220,  50,  50)
C_YELLOW = (255, 210,   0)
C_CYAN   = (  0, 200, 220)
C_DARK   = ( 18,  18,  18)
C_PANEL  = ( 25,  25,  40)
C_GREY   = (120, 120, 120)
# ─────────────────────────────────────────────────────────────────────────────

CAPTURES_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "captures",
)


# ── point-cloud helpers ───────────────────────────────────────────────────────

def _read_ply_xyz(path):
    """Return Nx3 float32 xyz from a binary PLY file, or None on failure."""
    try:
        with open(path, "rb") as f:
            header = b""
            while True:
                line = f.readline()
                header += line
                if line.strip() == b"end_header":
                    break
            n = 0
            for ln in header.decode("ascii", errors="ignore").split("\n"):
                if ln.startswith("element vertex"):
                    n = int(ln.split()[-1])
                    break
            if n == 0:
                return None
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                           ("r", "u1"),  ("g", "u1"),  ("b", "u1")])
            raw = np.frombuffer(f.read(n * dt.itemsize), dtype=dt)
            return np.stack([raw["x"], raw["y"], raw["z"]], axis=1)
    except Exception:
        return None


def build_topdown_map(size=480):
    """Load all merged_360.ply files and project them to a top-down 2D image."""
    plys = sorted(glob.glob(os.path.join(CAPTURES_ROOT, "scan_*", "merged_360.ply")))
    if not plys:
        # fall back to any ply
        plys = sorted(glob.glob(os.path.join(CAPTURES_ROOT, "**", "*.ply"),
                                recursive=True))

    clouds = [_read_ply_xyz(p) for p in plys]
    clouds = [c for c in clouds if c is not None and len(c) > 0]

    img = np.full((size, size, 3), 15, dtype=np.uint8)

    if not clouds:
        cv2.putText(img, "No scans yet", (20, size // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 80, 80), 2)
        return img, plys

    pts = np.concatenate(clouds, axis=0)
    x, z = pts[:, 0], pts[:, 2]   # top-down = X (left/right) × Z (depth)

    margin = 30
    x_min, x_max = x.min(), x.max()
    z_min, z_max = z.min(), z.max()
    span = max(x_max - x_min, z_max - z_min, 0.001)
    scale = (size - 2 * margin) / span

    xi = ((x - x_min) * scale + margin).astype(np.int32)
    zi = ((z - z_min) * scale + margin).astype(np.int32)

    valid = (xi >= 0) & (xi < size) & (zi >= 0) & (zi < size)
    img[zi[valid], xi[valid]] = (0, 160, 220)

    # mark each scan origin with a coloured circle
    colours = [(0,255,0),(255,128,0),(255,0,128),(0,255,255),(255,255,0)]
    for idx, cloud in enumerate(clouds):
        cx = int((cloud[:, 0].mean() - x_min) * scale + margin)
        cz = int((cloud[:, 2].mean() - z_min) * scale + margin)
        col = colours[idx % len(colours)]
        cv2.circle(img, (cx, cz), 8, col, -1)
        cv2.putText(img, str(idx + 1), (cx + 10, cz + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    return img, plys


# ── camera helpers ────────────────────────────────────────────────────────────

def open_camera():
    """Try D405 color first, fall back to USB camera."""
    try:
        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(cfg)
        return ("rs", pipeline)
    except Exception:
        pass
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            return ("usb", cap)
    except Exception:
        pass
    return None


def grab_frame(cam):
    """Return a BGR numpy frame or None."""
    if cam is None:
        return None
    kind, dev = cam
    if kind == "rs":
        try:
            frames = dev.wait_for_frames(timeout_ms=80)
            f = frames.get_color_frame()
            return np.asanyarray(f.get_data()) if f else None
        except Exception:
            return None
    else:
        ret, frame = dev.read()
        return frame if ret else None


def close_camera(cam):
    if cam is None:
        return
    kind, dev = cam
    if kind == "rs":
        dev.stop()
    else:
        dev.release()


# ── pygame helpers ────────────────────────────────────────────────────────────

def cv2_to_surf(img):
    """Convert a BGR OpenCV image to a pygame Surface."""
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return pygame.surfarray.make_surface(rgb.swapaxes(0, 1))


def text(screen, msg, font, colour, center):
    surf = font.render(msg, True, colour)
    screen.blit(surf, surf.get_rect(center=center))


# ── main game class ───────────────────────────────────────────────────────────

class StreetViewGame:

    PANEL = SCREEN_W // 2          # width of each half
    BAR_H = 90                     # bottom status bar height
    CAM_H = SCREEN_H - BAR_H       # camera / map panel height

    def __init__(self, bot):
        self.bot  = bot
        self.cam  = open_camera()

        print("Building top-down map from captured scans…")
        self.map_img, self.scan_paths = build_topdown_map()
        print(f"  {len(self.scan_paths)} scan(s) loaded.")

        pygame.init()
        pygame.display.set_caption("Street View Robot — Find the X!")
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        self.clock  = pygame.time.Clock()

        self.f_huge  = pygame.font.SysFont("monospace", 60, bold=True)
        self.f_big   = pygame.font.SysFont("monospace", 40, bold=True)
        self.f_med   = pygame.font.SysFont("monospace", 26)
        self.f_small = pygame.font.SysFont("monospace", 19)

        self.state      = "intro"
        self.start_time = None
        self.win_time   = None

    # ── state: intro ──────────────────────────────────────────────────────────

    def _draw_intro(self):
        self.screen.fill(C_DARK)

        # title
        text(self.screen, "STREET VIEW ROBOT", self.f_big, C_CYAN,
             (SCREEN_W // 2, 120))
        text(self.screen, "Find the  X  on the floor!", self.f_med, C_WHITE,
             (SCREEN_W // 2, 195))

        # map preview
        map_small = cv2.resize(self.map_img, (360, 360))
        self.screen.blit(cv2_to_surf(map_small),
                         map_small.shape[1::-1] and
                         (SCREEN_W // 2 - 180, 240))

        # scan count
        n = len(self.scan_paths)
        txt = f"{n} location{'s' if n != 1 else ''} scanned" if n else "No scans yet — run line_follow.py first"
        text(self.screen, txt, self.f_small, C_GREY, (SCREEN_W // 2, 625))

        # controls
        text(self.screen, "WASD = move   Q/E = rotate   ESC = quit",
             self.f_small, C_GREY, (SCREEN_W // 2, 655))

        # start prompt (blink)
        if int(time.time() * 2) % 2 == 0:
            text(self.screen, "Press SPACE to start", self.f_med, C_YELLOW,
                 (SCREEN_W // 2, 690))

    # ── state: playing ────────────────────────────────────────────────────────

    def _draw_game(self):
        self.screen.fill(C_DARK)
        elapsed   = time.time() - self.start_time
        remaining = max(0.0, GAME_TIME - elapsed) if GAME_TIME else None

        # ── left panel: camera ────────────────────────────────────────────────
        frame = grab_frame(self.cam)
        if frame is not None:
            cam_resized = cv2.resize(frame, (self.PANEL, self.CAM_H))
            self.screen.blit(cv2_to_surf(cam_resized), (0, 0))
        else:
            pygame.draw.rect(self.screen, C_PANEL, (0, 0, self.PANEL, self.CAM_H))
            text(self.screen, "No camera", self.f_med, C_GREY,
                 (self.PANEL // 2, self.CAM_H // 2))

        # label
        text(self.screen, "LIVE CAMERA", self.f_small, C_CYAN, (self.PANEL // 2, 18))

        # ── right panel: map ──────────────────────────────────────────────────
        map_big = cv2.resize(self.map_img, (self.PANEL, self.CAM_H))
        self.screen.blit(cv2_to_surf(map_big), (self.PANEL, 0))

        # map overlay labels
        text(self.screen, "ROOM MAP  (top-down)", self.f_small, C_CYAN,
             (self.PANEL + self.PANEL // 2, 18))
        text(self.screen, "Numbered dots = scanned stops",
             self.f_small, C_GREY, (self.PANEL + self.PANEL // 2, 42))
        text(self.screen, "Drive to the TARGET X to WIN!",
             self.f_small, C_YELLOW, (self.PANEL + self.PANEL // 2, 66))

        # divider
        pygame.draw.line(self.screen, C_GREY,
                         (self.PANEL, 0), (self.PANEL, self.CAM_H), 2)

        # ── bottom bar ────────────────────────────────────────────────────────
        bar_y = self.CAM_H
        pygame.draw.rect(self.screen, (10, 10, 20),
                         (0, bar_y, SCREEN_W, self.BAR_H))
        pygame.draw.line(self.screen, C_GREY,
                         (0, bar_y), (SCREEN_W, bar_y), 1)

        # timer
        if remaining is not None:
            t_col = C_RED if remaining < 10 else C_YELLOW if remaining < 30 else C_WHITE
            text(self.screen, f"{int(remaining):02d}s", self.f_big, t_col,
                 (70, bar_y + 45))
        else:
            text(self.screen, f"{int(elapsed)}s", self.f_med, C_WHITE,
                 (70, bar_y + 45))

        # controls
        controls = "W/S fwd·back   A/D strafe   Q/E rotate   SPACE stop"
        text(self.screen, controls, self.f_small, C_GREY,
             (SCREEN_W // 2, bar_y + 28))

        # goal
        text(self.screen, "FIND THE  X  MARKER!", self.f_med, C_YELLOW,
             (SCREEN_W // 2, bar_y + 65))

    # ── state: win ────────────────────────────────────────────────────────────

    def _draw_win(self):
        self.screen.fill((0, 25, 0))

        # flash effect
        alpha = int(abs(time.time() % 1.0 - 0.5) * 510)
        text(self.screen, "YOU WIN!", self.f_huge,
             (alpha, 255, alpha), (SCREEN_W // 2, 220))

        text(self.screen, f"Time: {self.win_time:.1f} seconds",
             self.f_big, C_WHITE, (SCREEN_W // 2, 350))

        stars = "★" * min(5, max(1, int((GAME_TIME - self.win_time) / (GAME_TIME / 5)) + 1)) if GAME_TIME else "★★★★★"
        text(self.screen, stars, self.f_big, C_YELLOW,
             (SCREEN_W // 2, 430))

        if int(time.time() * 2) % 2 == 0:
            text(self.screen, "Press SPACE to play again",
                 self.f_med, C_GREY, (SCREEN_W // 2, 560))

    # ── state: time up ────────────────────────────────────────────────────────

    def _draw_timeup(self):
        self.screen.fill((30, 0, 0))
        text(self.screen, "TIME'S UP!", self.f_huge, C_RED,
             (SCREEN_W // 2, 250))
        text(self.screen, "The X escaped you this time...",
             self.f_med, C_WHITE, (SCREEN_W // 2, 370))
        if int(time.time() * 2) % 2 == 0:
            text(self.screen, "Press SPACE to try again",
                 self.f_med, C_GREY, (SCREEN_W // 2, 520))

    # ── robot driving ─────────────────────────────────────────────────────────

    def _handle_drive(self):
        keys = pygame.key.get_pressed()
        if   keys[pygame.K_w]: self.bot.forward(DRIVE_SPEED)
        elif keys[pygame.K_s]: self.bot.backward(DRIVE_SPEED)
        elif keys[pygame.K_a]: self.bot.left(DRIVE_SPEED)
        elif keys[pygame.K_d]: self.bot.right(DRIVE_SPEED)
        elif keys[pygame.K_q]: self.bot.rotate_left(ROT_SPEED)
        elif keys[pygame.K_e]: self.bot.rotate_right(ROT_SPEED)
        else:                  self.bot.stop()

    # ── win detection ─────────────────────────────────────────────────────────

    def _check_win(self):
        try:
            lo, li, ri, ro = self.bot.read_line_sensors()
            if lo and li and ri and ro:          # all 4 on the X marker
                self.win_time = time.time() - self.start_time
                self.state    = "win"
                self.bot.stop()
                self.bot.beep(0.1)
                time.sleep(0.1)
                self.bot.beep(0.3)              # double beep = win
                self.bot.set_all_leds_color(Color.GREEN)
        except Exception:
            pass

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self):
        running = True
        while running:
            self.clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    if event.key == pygame.K_SPACE:
                        if self.state == "intro":
                            self.state      = "playing"
                            self.start_time = time.time()
                            self.bot.set_all_leds_color(Color.GREEN)
                        elif self.state in ("win", "timeup"):
                            self.state = "intro"
                            self.bot.set_all_leds_color(Color.BLUE)

            if self.state == "intro":
                self._draw_intro()

            elif self.state == "playing":
                self._handle_drive()
                self._check_win()
                # check time limit
                if GAME_TIME and (time.time() - self.start_time) >= GAME_TIME:
                    self.state = "timeup"
                    self.bot.stop()
                    self.bot.set_all_leds_color(Color.RED)
                self._draw_game()

            elif self.state == "win":
                self.bot.stop()
                self._draw_win()

            elif self.state == "timeup":
                self.bot.stop()
                self._draw_timeup()

            pygame.display.flip()

        self.bot.stop()
        close_camera(self.cam)
        pygame.quit()


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    print("Starting Street View Robot Game…")
    with RasBot() as bot:
        bot.set_all_leds_color(Color.BLUE)
        bot.beep(0.1)
        game = StreetViewGame(bot)
        try:
            game.run()
        except KeyboardInterrupt:
            pass
        finally:
            bot.stop()
            bot.set_all_leds_color(Color.RED)
    print("Game over.")


if __name__ == "__main__":
    main()
