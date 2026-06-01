"""
Street View Robot — Room Explorer Game
=======================================
Split-screen VNC game with live coverage mapping.

LEFT  panel : live camera feed from the robot.
RIGHT panel : top-down coverage map of the room.
               GREEN  = area the D405 has already scanned.
               RED    = area not yet scanned.

How to play:
  • Drive the robot with WASD / QE.
  • Stop the robot on a RED-tape cross → robot scans → that area
    turns GREEN on the map.
  • Goal: turn the whole map green (explore the full room).
  • Win when coverage reaches WIN_COVERAGE_PCT (default 80 %).

Run from the project root on the Pi:
    python3 src/game/game_test.py

Requirements:
    pip install pygame       (already done)
    numpy + opencv-python    (already installed)
"""

import os, sys, glob, time, threading

import numpy as np
import cv2
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_and_api.api import RasBot, Color

# ── settings ──────────────────────────────────────────────────────────────────
SCREEN_W        = 1280
SCREEN_H        = 720
DRIVE_SPEED     = 110
ROT_SPEED       = 80
FPS             = 30
GAME_TIME       = 0          # seconds, 0 = no time limit
WIN_COVERAGE_PCT = 80.0      # % of map green to win

CAPTURES_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "captures",
)

# ── colours (pygame RGB) ──────────────────────────────────────────────────────
C_BLACK   = (  0,   0,   0)
C_WHITE   = (255, 255, 255)
C_GREEN   = (  0, 210,   0)
C_RED     = (210,  40,  40)
C_YELLOW  = (255, 210,   0)
C_CYAN    = (  0, 200, 220)
C_DARK    = ( 14,  14,  22)
C_GREY    = (100, 100, 100)
# ─────────────────────────────────────────────────────────────────────────────


# ── PLY reader ────────────────────────────────────────────────────────────────

def _read_ply_xyz(path):
    """Return Nx3 float32 xyz from a binary PLY, or None on failure."""
    try:
        with open(path, "rb") as f:
            hdr = b""
            while True:
                line = f.readline()
                hdr += line
                if line.strip() == b"end_header":
                    break
            n = 0
            for ln in hdr.decode("ascii", errors="ignore").split("\n"):
                if ln.startswith("element vertex"):
                    n = int(ln.split()[-1])
                    break
            if n == 0:
                return None
            dt = np.dtype([("x","<f4"),("y","<f4"),("z","<f4"),
                           ("r","u1"),("g","u1"),("b","u1")])
            raw = np.frombuffer(f.read(n * dt.itemsize), dtype=dt)
            return np.stack([raw["x"], raw["y"], raw["z"]], axis=1)
    except Exception:
        return None


# ── coverage map ──────────────────────────────────────────────────────────────

class CoverageMap:
    """Builds and renders a green/red top-down coverage map.

    GREEN cells  = areas seen by the D405 in any scan session.
    RED   cells  = areas inside the estimated room boundary that
                   have NOT been scanned yet.
    """

    CELL    = 0.07    # grid resolution in metres (7 cm)
    MARGIN  = 0.6     # red padding around known scanned area (m)

    def __init__(self):
        self._lock         = threading.Lock()
        self._known        = set()          # paths already loaded
        self._cells        = set()          # (ix, iz) scanned cells
        self._scan_origins = []             # (mean_x, mean_z) per scan
        self.scan_count    = 0
        self.coverage_pct  = 0.0
        self._x_min = self._x_max = None
        self._z_min = self._z_max = None
        self._img          = self._blank()
        self._load_all()

    # ── public ────────────────────────────────────────────────────────────────

    def refresh(self):
        """Call periodically to pick up newly saved PLY files."""
        plys = sorted(glob.glob(
            os.path.join(CAPTURES_ROOT, "scan_*", "merged_360.ply")))
        new = [p for p in plys if p not in self._known]
        if new:
            for p in new:
                self._ingest(p)
            self._rebuild()
            return True
        return False

    @property
    def image(self):
        with self._lock:
            return self._img.copy()

    # ── internal ──────────────────────────────────────────────────────────────

    def _blank(self, size=500):
        img = np.full((size, size, 3), 14, dtype=np.uint8)
        cv2.putText(img, "No scans yet —", (30, size // 2 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 70, 70), 2)
        cv2.putText(img, "drive to a marker!", (20, size // 2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 70, 70), 2)
        return img

    def _load_all(self):
        plys = sorted(glob.glob(
            os.path.join(CAPTURES_ROOT, "scan_*", "merged_360.ply")))
        for p in plys:
            self._ingest(p)
        self._rebuild()

    def _ingest(self, path):
        pts = _read_ply_xyz(path)
        if pts is None or len(pts) == 0:
            self._known.add(path)
            return
        self._known.add(path)
        self.scan_count += 1

        x, z = pts[:, 0], pts[:, 2]
        # Update world bounds
        xmn, xmx = float(x.min()), float(x.max())
        zmn, zmx = float(z.min()), float(z.max())
        if self._x_min is None:
            self._x_min, self._x_max = xmn, xmx
            self._z_min, self._z_max = zmn, zmx
        else:
            self._x_min = min(self._x_min, xmn)
            self._x_max = max(self._x_max, xmx)
            self._z_min = min(self._z_min, zmn)
            self._z_max = max(self._z_max, zmx)

        # Record grid cells
        ix = (x / self.CELL).astype(np.int32)
        iz = (z / self.CELL).astype(np.int32)
        for cx, cz in zip(ix, iz):
            self._cells.add((int(cx), int(cz)))

        # Store scan origin (centroid)
        self._scan_origins.append((float(x.mean()), float(z.mean())))

    def _rebuild(self, size=500):
        if self._x_min is None:
            with self._lock:
                self._img = self._blank(size)
            self.coverage_pct = 0.0
            return

        m = self.MARGIN
        wx0, wx1 = self._x_min - m, self._x_max + m
        wz0, wz1 = self._z_min - m, self._z_max + m
        wx_span  = max(wx1 - wx0, 0.001)
        wz_span  = max(wz1 - wz0, 0.001)
        pad      = 15

        def w2p(wx, wz):
            px = int((wx - wx0) / wx_span * (size - 2*pad) + pad)
            pz = int((wz - wz0) / wz_span * (size - 2*pad) + pad)
            return px, pz

        cell_px = max(2, int(self.CELL / wx_span * (size - 2*pad)))

        img = np.full((size, size, 3), 14, dtype=np.uint8)

        # ── draw all grid cells ───────────────────────────────────────────────
        ix0 = int(wx0 / self.CELL) - 1
        ix1 = int(wx1 / self.CELL) + 1
        iz0 = int(wz0 / self.CELL) - 1
        iz1 = int(wz1 / self.CELL) + 1

        total = green = 0
        for ix in range(ix0, ix1):
            for iz in range(iz0, iz1):
                wx = ix * self.CELL
                wz = iz * self.CELL
                if wx0 <= wx <= wx1 and wz0 <= wz <= wz1:
                    px, pz = w2p(wx, wz)
                    if pad <= px < size-pad and pad <= pz < size-pad:
                        total += 1
                        if (ix, iz) in self._cells:
                            color = (0, 170, 0)     # BGR green
                            green += 1
                        else:
                            color = (30, 30, 140)   # BGR dark red
                        cv2.rectangle(img,
                                      (px, pz), (px + cell_px, pz + cell_px),
                                      color, -1)

        # ── draw scan-origin markers ──────────────────────────────────────────
        colours = [(0,255,0),(255,180,0),(0,255,255),(255,0,200),(180,255,0)]
        for i, (ox, oz) in enumerate(self._scan_origins):
            px, pz = w2p(ox, oz)
            col = colours[i % len(colours)]
            cv2.circle(img, (px, pz), 7, col, -1)
            cv2.circle(img, (px, pz), 7, (255,255,255), 1)
            cv2.putText(img, str(i+1), (px+9, pz+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

        # ── border ────────────────────────────────────────────────────────────
        cv2.rectangle(img, (pad-2, pad-2), (size-pad+2, size-pad+2),
                      (60, 60, 60), 1)

        self.coverage_pct = (green / total * 100) if total > 0 else 0.0
        with self._lock:
            self._img = img


# ── camera helpers ────────────────────────────────────────────────────────────

def open_camera():
    """Try USB camera first, then D405 colour stream."""
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
            return ("usb", cap)
    except Exception:
        pass
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg  = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe.start(cfg)
        return ("rs", pipe)
    except Exception:
        pass
    return None


def grab_frame(cam):
    if cam is None:
        return None
    kind, dev = cam
    if kind == "usb":
        ret, frame = dev.read()
        return frame if ret else None
    try:
        frames = dev.wait_for_frames(timeout_ms=80)
        f = frames.get_color_frame()
        return np.asanyarray(f.get_data()) if f else None
    except Exception:
        return None


def close_camera(cam):
    if cam is None:
        return
    kind, dev = cam
    if kind == "usb":
        dev.release()
    else:
        try:
            dev.stop()
        except Exception:
            pass


# ── pygame helpers ────────────────────────────────────────────────────────────

def cv2surf(img):
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return pygame.surfarray.make_surface(rgb.swapaxes(0, 1))


def txt(screen, msg, font, colour, center):
    s = font.render(msg, True, colour)
    screen.blit(s, s.get_rect(center=center))


# ── scan helper ───────────────────────────────────────────────────────────────

def run_scan(bot, coverage_map, log):
    """Stop-marker scan: capture 360° and update the coverage map."""
    bot.set_all_leds_color(Color.BLUE)
    bot.beep(0.1)
    log("Stop marker → running 360° scan…")
    try:
        from camera.rs_capture import StereoCapture
        from pointcloud        import scan360
        cam = StereoCapture()
        try:
            _, ply = scan360.scan_and_build(bot, cam, log=log)
            log(f"Scan complete → {ply}")
            bot.beep(0.15)
        finally:
            cam.close()
    except Exception as exc:
        log(f"Scan error: {exc}")
    coverage_map.refresh()
    bot.set_all_leds_color(Color.GREEN)


# ── main game class ───────────────────────────────────────────────────────────

class RoomExplorerGame:
    PANEL  = SCREEN_W // 2
    BAR_H  = 85
    CAM_H  = SCREEN_H - BAR_H

    def __init__(self, bot):
        self.bot      = bot
        self.cam      = open_camera()
        self.coverage = CoverageMap()

        pygame.init()
        pygame.display.set_caption("Room Explorer — Scan the World!")
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        self.clock  = pygame.time.Clock()

        self.f_huge  = pygame.font.SysFont("monospace", 58, bold=True)
        self.f_big   = pygame.font.SysFont("monospace", 36, bold=True)
        self.f_med   = pygame.font.SysFont("monospace", 24)
        self.f_small = pygame.font.SysFont("monospace", 17)

        self.state       = "intro"
        self.start_time  = None
        self.win_time    = None
        self.log_lines   = []          # recent status messages
        self._scan_lock  = False       # prevent double-scan trigger
        self._debounce   = 0.0

    # ── logging ───────────────────────────────────────────────────────────────

    def _log(self, msg):
        self.log_lines = ([msg] + self.log_lines)[:4]

    # ── intro screen ──────────────────────────────────────────────────────────

    def _draw_intro(self):
        self.screen.fill(C_DARK)
        cx = SCREEN_W // 2

        txt(self.screen, "ROOM EXPLORER", self.f_big, C_CYAN, (cx, 110))
        txt(self.screen, "Drive the robot. Scan the room.", self.f_med, C_WHITE, (cx, 175))

        # Legend
        pygame.draw.rect(self.screen, (0,170,0), (cx-160, 220, 20, 20))
        txt(self.screen, "= D405 scanned area", self.f_small, C_WHITE, (cx+35, 230))
        pygame.draw.rect(self.screen, (30,30,140), (cx-160, 255, 20, 20))
        txt(self.screen, "= not scanned yet", self.f_small, C_WHITE, (cx+20, 265))

        # Map preview
        if self.coverage.scan_count > 0:
            m = cv2.resize(self.coverage.image, (300, 300))
            self.screen.blit(cv2surf(m), (cx - 150, 295))
            txt(self.screen, f"{self.coverage.scan_count} scan(s) loaded  "
                f"({self.coverage.coverage_pct:.0f}% covered)",
                self.f_small, C_GREY, (cx, 610))
        else:
            txt(self.screen, "No scans yet — drive to a stop marker first!",
                self.f_small, C_GREY, (cx, 450))

        txt(self.screen, "WASD = drive   QE = rotate   ESC = quit",
            self.f_small, C_GREY, (cx, 650))
        if int(time.time() * 2) % 2 == 0:
            txt(self.screen, "Press SPACE to start", self.f_med, C_YELLOW, (cx, 690))

    # ── game screen ───────────────────────────────────────────────────────────

    def _draw_game(self):
        self.screen.fill(C_DARK)
        elapsed   = time.time() - self.start_time
        remaining = max(0.0, GAME_TIME - elapsed) if GAME_TIME else None

        # ── left: live camera ─────────────────────────────────────────────────
        frame = grab_frame(self.cam)
        if frame is not None:
            resized = cv2.resize(frame, (self.PANEL, self.CAM_H))
            self.screen.blit(cv2surf(resized), (0, 0))
        else:
            pygame.draw.rect(self.screen, (20,20,35), (0, 0, self.PANEL, self.CAM_H))
            txt(self.screen, "No camera", self.f_med, C_GREY,
                (self.PANEL//2, self.CAM_H//2))

        txt(self.screen, "LIVE CAMERA", self.f_small, C_CYAN,
            (self.PANEL//2, 16))

        # ── right: coverage map ───────────────────────────────────────────────
        map_img = cv2.resize(self.coverage.image, (self.PANEL, self.CAM_H))
        self.screen.blit(cv2surf(map_img), (self.PANEL, 0))

        # Map overlays
        txt(self.screen, "ROOM COVERAGE MAP", self.f_small, C_CYAN,
            (self.PANEL + self.PANEL//2, 16))

        # Green / Red legend
        pygame.draw.rect(self.screen, (0,170,0),   (self.PANEL+12, 35, 14, 14))
        pygame.draw.rect(self.screen, (30,30,140),  (self.PANEL+100, 35, 14, 14))
        s = self.f_small.render("= scanned", True, C_WHITE)
        self.screen.blit(s, (self.PANEL+30, 34))
        s = self.f_small.render("= not yet", True, C_WHITE)
        self.screen.blit(s, (self.PANEL+118, 34))

        # Coverage %
        pct = self.coverage.coverage_pct
        pct_col = C_GREEN if pct >= WIN_COVERAGE_PCT else (C_YELLOW if pct > 30 else C_RED)
        txt(self.screen, f"{pct:.1f}% covered  |  {self.coverage.scan_count} scans",
            self.f_med, pct_col,
            (self.PANEL + self.PANEL//2, self.CAM_H - 28))

        # Divider
        pygame.draw.line(self.screen, C_GREY,
                         (self.PANEL, 0), (self.PANEL, self.CAM_H), 2)

        # ── bottom bar ────────────────────────────────────────────────────────
        bar_y = self.CAM_H
        pygame.draw.rect(self.screen, (10,10,18),
                         (0, bar_y, SCREEN_W, self.BAR_H))
        pygame.draw.line(self.screen, C_GREY,
                         (0, bar_y), (SCREEN_W, bar_y), 1)

        # Timer / elapsed
        if remaining is not None:
            t_col = C_RED if remaining < 10 else (C_YELLOW if remaining < 30 else C_WHITE)
            txt(self.screen, f"{int(remaining):02d}s", self.f_big, t_col, (55, bar_y+42))
        else:
            txt(self.screen, f"{int(elapsed)}s", self.f_med, C_WHITE, (55, bar_y+42))

        # Log messages
        for i, line in enumerate(self.log_lines):
            s = self.f_small.render(line, True, (180, 180, 180))
            self.screen.blit(s, (120, bar_y + 8 + i * 18))

        # Controls
        txt(self.screen, "WASD=drive  QE=rotate  Stop on RED cross → scan!",
            self.f_small, C_GREY, (SCREEN_W//2 + 80, bar_y + 68))

    # ── win / timeup screens ──────────────────────────────────────────────────

    def _draw_win(self):
        self.screen.fill((0, 22, 0))
        cx = SCREEN_W // 2
        a = int(abs(time.time() % 1.0 - 0.5) * 510)
        txt(self.screen, "ROOM FULLY SCANNED!", self.f_huge, (a, 255, a), (cx, 200))
        txt(self.screen, f"Time: {self.win_time:.1f} s  |  "
            f"{self.coverage.scan_count} locations",
            self.f_big, C_WHITE, (cx, 330))
        txt(self.screen, f"Coverage: {self.coverage.coverage_pct:.1f}%",
            self.f_big, C_YELLOW, (cx, 420))
        if int(time.time()*2)%2==0:
            txt(self.screen, "Press SPACE to play again",
                self.f_med, C_GREY, (cx, 560))

    def _draw_timeup(self):
        self.screen.fill((25, 0, 0))
        cx = SCREEN_W // 2
        txt(self.screen, "TIME'S UP!", self.f_huge, C_RED, (cx, 230))
        txt(self.screen, f"Coverage reached: {self.coverage.coverage_pct:.1f}%",
            self.f_big, C_WHITE, (cx, 360))
        if int(time.time()*2)%2==0:
            txt(self.screen, "Press SPACE to try again",
                self.f_med, C_GREY, (cx, 520))

    # ── robot control ─────────────────────────────────────────────────────────

    def _handle_drive(self):
        k = pygame.key.get_pressed()
        if   k[pygame.K_w]: self.bot.forward(DRIVE_SPEED)
        elif k[pygame.K_s]: self.bot.backward(DRIVE_SPEED)
        elif k[pygame.K_a]: self.bot.left(DRIVE_SPEED)
        elif k[pygame.K_d]: self.bot.right(DRIVE_SPEED)
        elif k[pygame.K_q]: self.bot.rotate_left(ROT_SPEED)
        elif k[pygame.K_e]: self.bot.rotate_right(ROT_SPEED)
        else:               self.bot.stop()

    def _check_stop_marker(self):
        """Trigger scan when all 4 IR sensors hit the cross."""
        if self._scan_lock or time.time() < self._debounce:
            return
        try:
            lo, li, ri, ro = self.bot.read_line_sensors()
            if lo and li and ri and ro:
                self._scan_lock = True
                self.bot.stop()
                self._log("Stop marker detected — scanning…")

                # Run scan in background so game stays responsive
                def _scan():
                    run_scan(self.bot, self.coverage, self._log)
                    # Check win after scan
                    if self.coverage.coverage_pct >= WIN_COVERAGE_PCT:
                        self.win_time = time.time() - self.start_time
                        self.state = "win"
                    self._debounce  = time.time() + 2.0
                    self._scan_lock = False

                threading.Thread(target=_scan, daemon=True).start()
        except Exception:
            pass

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self):
        # Refresh map every 5 s in background
        def _refresh_loop():
            while True:
                self.coverage.refresh()
                time.sleep(5)
        threading.Thread(target=_refresh_loop, daemon=True).start()

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
                            self.state     = "playing"
                            self.start_time = time.time()
                            self.bot.set_all_leds_color(Color.GREEN)
                            self._log("Game started — drive to stop markers!")
                        elif self.state in ("win", "timeup"):
                            self.state = "intro"
                            self.bot.set_all_leds_color(Color.BLUE)

            if self.state == "intro":
                self._draw_intro()

            elif self.state == "playing":
                if not self._scan_lock:
                    self._handle_drive()
                    self._check_stop_marker()
                # Time limit check
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
    print("Starting Room Explorer Game…")
    with RasBot() as bot:
        bot.set_all_leds_color(Color.BLUE)
        bot.beep(0.1)
        game = RoomExplorerGame(bot)
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
