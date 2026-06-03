"""
constants.py — shared constants for the RASPBOT radar system.
Imported by every other module so all files agree on one configuration.
"""

import os

# ── paths ─────────────────────────────────────────────────────────────────────
RADAR_DIR        = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(RADAR_DIR, "calibration.json")
VNC_SETTINGS_SH  = os.path.join(RADAR_DIR, "vnc_settings.sh")
# `src/` — needed so `from setup_and_api.api import RasBot` resolves
SRC_DIR          = os.path.dirname(os.path.dirname(RADAR_DIR))

# ── display ─────────────────────────────────────────────────────────────────
WINDOW_SIZE   = (1280, 720)
FPS_TARGET    = 30
RADAR_RADIUS  = 300            # px
MAX_DISTANCE  = 200            # cm — display radius + valid-echo cutoff
MIN_DISTANCE  = 2              # cm — below this = sensor error

# ── zones (cm) ──────────────────────────────────────────────────────────────
DANGER_ZONE   = 20
WARNING_ZONE  = 80
SAFE_ZONE     = 100

# ── servo ───────────────────────────────────────────────────────────────────
PAN_CENTER    = 90             # commanded angle pointing straight ahead
ARC_DEG       = 180            # total swept arc
SWEEP_SPEED   = 2              # degrees per step
STILL_TIME    = 0.015          # s the servo must be still before a reading
SETTLE_S      = 0.04           # s settle after a move command
SERVO_SPEED_DPS_DEFAULT = 300  # fallback if calibration missing

# ── filter ──────────────────────────────────────────────────────────────────
FILTER_ALPHA     = 0.3         # EMA smoothing factor
MAX_JUMP         = 50          # cm — bigger single-step change = spike
READINGS_PER_POS = 5           # median window per position
READING_GAP_S    = 0.01        # 10 ms between the 5 reads
PERSIST_SWEEPS   = 3           # keep a detection visible this many sweeps

# ── sound ───────────────────────────────────────────────────────────────────
MASTER_VOLUME   = 0.7
MIN_FREQ        = 220
MAX_FREQ        = 1100
SND_SAMPLE_RATE = 44100

# ── colors (R, G, B) ────────────────────────────────────────────────────────
NEON_GREEN  = (0, 255, 70)
DANGER_RED  = (255, 30, 30)
WARNING_YEL = (255, 200, 0)
SAFE_GREEN  = (0, 200, 50)
ORANGE      = (255, 140, 0)
BLACK       = (0, 0, 0)
DIM_GREEN   = (0, 90, 30)
TEXT_GREEN  = (90, 220, 110)
TEXT_HI     = (200, 255, 210)
PANEL_BG    = (4, 14, 4)
PANEL_LINE  = (0, 80, 0)

# ── fonts ───────────────────────────────────────────────────────────────────
FONT_TITLE = "Orbitron-Bold.ttf"
FONT_REG   = "Orbitron-Regular.ttf"
FONT_DATA  = "ShareTechMono-Regular.ttf"
FONT_SIZES = {"title": 28, "data": 18, "small": 13, "alert": 48,
              "med": 16, "big": 36, "panel_title": 22}

FONT_LINKS = {
    "orbitron": "fonts.google.com/specimen/Orbitron",
    "mono":     "fonts.google.com/specimen/Share+Tech+Mono",
}
