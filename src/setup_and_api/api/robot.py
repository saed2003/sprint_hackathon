"""RasBot hardware API for students.

Provides control of:
- Mecanum wheel movement (forward, backward, strafe, rotate, omnidirectional)
- Pan/tilt servo for camera aiming
- USB camera capture
- RealSense D405 stereo camera (color, depth, stereo IR)
- Ultrasonic distance sensor
- Line-tracking sensors (4x IR)
- RGB LEDs, buzzer, OLED display
- Audio recording and playback
"""

import math
import time
import wave
import threading
import subprocess
import logging
from typing import NamedTuple, Tuple

import smbus
import numpy as np

from .constants import (
    I2C_ADDRESS, I2C_BUS, Register, Color, Motor,
    PAN_SERVO_ID, TILT_SERVO_ID, PAN_DEFAULT, TILT_DEFAULT,
    PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX, LED_COUNT,
    AUDIO_CHUNK, AUDIO_CHANNELS, AUDIO_RATE, CHASSIS_HALF_WIDTH,
    RS_FRAME_WIDTH, RS_FRAME_HEIGHT, RS_FPS,
    RS_DEPTH_MIN_MM, RS_DEPTH_MAX_MM,
)

logger = logging.getLogger(__name__)


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


class RealSenseFrames(NamedTuple):
    """All frames from a single synchronized RealSense capture."""
    color: np.ndarray      # BGR uint8, shape (H, W, 3)
    depth: np.ndarray      # uint16 millimeters, shape (H, W)
    ir_left: np.ndarray    # uint8 grayscale, shape (H, W)
    ir_right: np.ndarray   # uint8 grayscale, shape (H, W)


class RasBot:
    """Unified controller for the RasbotV2 Mecanum-wheel robot."""

    # ── lifecycle ──────────────────────────────────────────────

    def __init__(
        self,
        i2c_address: int = I2C_ADDRESS,
        i2c_bus: int = I2C_BUS,
    ) -> None:
        self._addr = i2c_address
        self._bus = smbus.SMBus(i2c_bus)
        self._i2c_lock = threading.Lock()
        self._rs_pipeline = None
        self._rs_config = None
        self._rs_depth_scale = None
        self._usb_camera = None
        self._oled = None
        self._oled_draw = None
        self._oled_image = None
        self._ultrasonic_enabled = False
        self._ir_enabled = False

    def __enter__(self) -> "RasBot":
        return self

    def __exit__(self, *exc) -> None:
        self.cleanup()

    def cleanup(self) -> None:
        """Stop motors, turn off LEDs/buzzer, release cameras, clear OLED."""
        self.stop()
        self.leds_off()
        self.buzzer_off()
        if self._ultrasonic_enabled:
            self._write_block(Register.ULTRASONIC_SWITCH, [0])
            self._ultrasonic_enabled = False
        if self._ir_enabled:
            self._write_block(Register.IR_SWITCH, [0])
            self._ir_enabled = False
        self.look_center()
        self.release_camera()
        self.release_usb_camera()
        if self._oled is not None:
            self.clear_display()

    # ── I2C primitives (private) ───────────────────────────────

    def _write_block(self, register: int, data: list) -> None:
        with self._i2c_lock:
            self._bus.write_i2c_block_data(self._addr, register, data)

    def _read_block(self, register: int, length: int) -> list:
        with self._i2c_lock:
            return self._bus.read_i2c_block_data(self._addr, register, length)

    def _set_motor(self, motor_id: int, speed: int) -> None:
        """Set a single motor. speed: -255 to 255 (negative = backward)."""
        speed = _clamp(speed, -255, 255)
        direction = 1 if speed < 0 else 0
        self._write_block(Register.MOTOR, [motor_id, direction, abs(speed)])

    def _apply_motors(self, lf: int, lr: int, rf: int, rr: int) -> None:
        """Send speed values to all four motors."""
        self._set_motor(Motor.LEFT_FRONT, lf)
        self._set_motor(Motor.LEFT_REAR, lr)
        self._set_motor(Motor.RIGHT_FRONT, rf)
        self._set_motor(Motor.RIGHT_REAR, rr)

    # ── mecanum kinematics (private) ───────────────────────────

    def _compute_mecanum_speeds(
        self,
        speed: int,
        angle_deg: float,
        rotation: float = 0.0,
    ) -> Tuple[int, int, int, int]:
        """Decompose speed + direction into per-wheel speeds.

        Coordinate system:
            90 (forward)
        180 ──┤── 0 (right)
            270 (backward)
        """
        speed = _clamp(speed, 0, 255)
        rad = math.radians(angle_deg)
        vx = speed * math.cos(rad)
        vy = speed * math.sin(rad)
        vp = -rotation * CHASSIS_HALF_WIDTH

        lf = int(vy + vx - vp)
        lr = int(vy - vx + vp)
        rf = int(vy - vx - vp)
        rr = int(vy + vx + vp)
        return lf, lr, rf, rr

    # ── movement ───────────────────────────────────────────────

    def forward(self, speed: int = 100) -> None:
        """Move forward."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 90)
        self._apply_motors(lf, lr, rf, rr)

    def backward(self, speed: int = 100) -> None:
        """Move backward."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 270)
        self._apply_motors(lf, lr, rf, rr)

    def left(self, speed: int = 100) -> None:
        """Strafe left (lateral movement)."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 180)
        self._apply_motors(lf, lr, rf, rr)

    def right(self, speed: int = 100) -> None:
        """Strafe right (lateral movement)."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 0)
        self._apply_motors(lf, lr, rf, rr)

    def rotate_left(self, speed: int = 100) -> None:
        """In-place counter-clockwise rotation."""
        speed = _clamp(speed, 0, 255)
        self._apply_motors(-speed, -speed, speed, speed)

    def rotate_right(self, speed: int = 100) -> None:
        """In-place clockwise rotation."""
        speed = _clamp(speed, 0, 255)
        self._apply_motors(speed, speed, -speed, -speed)

    def diagonal_left_front(self, speed: int = 100) -> None:
        """Move diagonally forward-left."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 135)
        self._apply_motors(lf, lr, rf, rr)

    def diagonal_right_front(self, speed: int = 100) -> None:
        """Move diagonally forward-right."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 45)
        self._apply_motors(lf, lr, rf, rr)

    def diagonal_left_back(self, speed: int = 100) -> None:
        """Move diagonally backward-left."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 225)
        self._apply_motors(lf, lr, rf, rr)

    def diagonal_right_back(self, speed: int = 100) -> None:
        """Move diagonally backward-right."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, 315)
        self._apply_motors(lf, lr, rf, rr)

    def move(self, speed: int, angle_degrees: float) -> None:
        """Omnidirectional movement at any angle.

        angle: 0=right, 90=forward, 180=left, 270=backward.
        """
        lf, lr, rf, rr = self._compute_mecanum_speeds(speed, angle_degrees)
        self._apply_motors(lf, lr, rf, rr)

    def drift(
        self,
        speed: int,
        angle_degrees: float,
        rotation_rate: float,
    ) -> None:
        """Move in a direction while simultaneously rotating."""
        lf, lr, rf, rr = self._compute_mecanum_speeds(
            speed, angle_degrees, rotation_rate
        )
        self._apply_motors(lf, lr, rf, rr)

    def stop(self) -> None:
        """Stop all motors immediately."""
        for motor_id in Motor:
            self._write_block(Register.MOTOR, [motor_id, 0, 0])

    # ── servos ─────────────────────────────────────────────────

    def set_pan(self, angle: int) -> None:
        """Set horizontal servo angle (0-180, default 90)."""
        angle = _clamp(angle, PAN_MIN, PAN_MAX)
        self._write_block(Register.SERVO, [PAN_SERVO_ID, angle])

    def set_tilt(self, angle: int) -> None:
        """Set vertical servo angle (0-100, default 25)."""
        angle = _clamp(angle, TILT_MIN, TILT_MAX)
        self._write_block(Register.SERVO, [TILT_SERVO_ID, angle])

    def look_center(self) -> None:
        """Reset both servos to default positions."""
        self.set_pan(PAN_DEFAULT)
        self.set_tilt(TILT_DEFAULT)

    def nod(self, cycles: int = 2, delay: float = 0.3) -> None:
        """Tilt servo nod gesture."""
        for _ in range(cycles):
            self.set_tilt(100)
            time.sleep(delay)
            self.set_tilt(TILT_DEFAULT)
            time.sleep(delay)

    def shake_head(self, cycles: int = 2, delay: float = 0.3) -> None:
        """Pan servo shake gesture."""
        for _ in range(cycles):
            self.set_pan(60)
            time.sleep(delay)
            self.set_pan(120)
            time.sleep(delay)
        self.set_pan(PAN_DEFAULT)

    # ── LEDs ───────────────────────────────────────────────────

    def set_all_leds(self, r: int, g: int, b: int) -> None:
        """Set all 14 LEDs to an RGB color (0-255 each)."""
        r, g, b = _clamp(r, 0, 255), _clamp(g, 0, 255), _clamp(b, 0, 255)
        self._write_block(Register.LED_RGB_ALL, [r, g, b])

    def set_led(self, index: int, r: int, g: int, b: int) -> None:
        """Set a single LED (1-14) to an RGB color."""
        index = _clamp(index, 1, LED_COUNT)
        r, g, b = _clamp(r, 0, 255), _clamp(g, 0, 255), _clamp(b, 0, 255)
        self._write_block(Register.LED_RGB_SINGLE, [index, r, g, b])

    def set_all_leds_color(self, color: Color) -> None:
        """Set all LEDs to a preset color."""
        self._write_block(Register.LED_ALL, [1, int(color)])

    def set_led_color(self, index: int, color: Color) -> None:
        """Set a single LED (1-14) to a preset color."""
        index = _clamp(index, 1, LED_COUNT)
        self._write_block(Register.LED_SINGLE, [index, 1, int(color)])

    def leds_off(self) -> None:
        """Turn off all LEDs."""
        self._write_block(Register.LED_ALL, [0, 0])

    # ── buzzer ─────────────────────────────────────────────────

    def buzzer_on(self) -> None:
        """Turn buzzer on."""
        self._write_block(Register.BUZZER, [1])

    def buzzer_off(self) -> None:
        """Turn buzzer off."""
        self._write_block(Register.BUZZER, [0])

    def beep(self, duration: float = 0.2) -> None:
        """Short beep for the given duration in seconds."""
        self.buzzer_on()
        time.sleep(duration)
        self.buzzer_off()

    # ── sensors ────────────────────────────────────────────────

    def read_distance(self) -> float:
        """Read ultrasonic distance in centimeters.

        Auto-enables the sensor on first call.
        """
        if not self._ultrasonic_enabled:
            self._write_block(Register.ULTRASONIC_SWITCH, [1])
            self._ultrasonic_enabled = True
            time.sleep(0.1)

        high = self._read_block(Register.ULTRASONIC_HIGH, 1)[0]
        low = self._read_block(Register.ULTRASONIC_LOW, 1)[0]
        distance_mm = (high << 8) | low
        return distance_mm / 10.0

    def read_line_sensors(self) -> Tuple[bool, bool, bool, bool]:
        """Read the 4 line-tracking sensors.

        Returns (left_outer, left_inner, right_inner, right_outer).
        True means line detected (sensor over dark line).
        """
        data = self._read_block(Register.LINE_TRACK, 1)[0]
        left_inner = not bool((data >> 3) & 0x01)
        left_outer = not bool((data >> 2) & 0x01)
        right_inner = not bool((data >> 1) & 0x01)
        right_outer = not bool(data & 0x01)
        return (left_outer, left_inner, right_inner, right_outer)

    def read_ir(self) -> int:
        """Read IR remote receiver value. Returns raw byte (0-255)."""
        if not self._ir_enabled:
            self._write_block(Register.IR_SWITCH, [1])
            self._ir_enabled = True
            time.sleep(0.05)
        return self._read_block(Register.IR_DATA, 1)[0]

    def read_button(self) -> bool:
        """Read button state. Returns True if pressed."""
        return bool(self._read_block(Register.BUTTON, 1)[0])

    # ── RealSense D405 stereo camera ──────────────────────────

    def _ensure_camera(self) -> None:
        """Lazily initialize the RealSense D405 pipeline."""
        if self._rs_pipeline is not None:
            return
        import pyrealsense2 as rs

        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(
            rs.stream.color, RS_FRAME_WIDTH, RS_FRAME_HEIGHT,
            rs.format.bgr8, RS_FPS,
        )
        config.enable_stream(
            rs.stream.depth, RS_FRAME_WIDTH, RS_FRAME_HEIGHT,
            rs.format.z16, RS_FPS,
        )
        config.enable_stream(
            rs.stream.infrared, 1, RS_FRAME_WIDTH, RS_FRAME_HEIGHT,
            rs.format.y8, RS_FPS,
        )
        config.enable_stream(
            rs.stream.infrared, 2, RS_FRAME_WIDTH, RS_FRAME_HEIGHT,
            rs.format.y8, RS_FPS,
        )

        try:
            profile = pipeline.start(config)
        except Exception as e:
            raise RuntimeError(f"Failed to start RealSense pipeline: {e}") from e

        self._rs_pipeline = pipeline
        self._rs_config = config
        self._rs_depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    def capture_frame(self) -> np.ndarray:
        """Capture a single color frame from the RealSense (BGR numpy array)."""
        self._ensure_camera()
        frames = self._rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("Failed to capture color frame")
        return np.asanyarray(color_frame.get_data())

    def capture_depth(self) -> np.ndarray:
        """Capture a depth frame in millimeters (uint16 numpy array)."""
        self._ensure_camera()
        frames = self._rs_pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            raise RuntimeError("Failed to capture depth frame")
        raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_mm = raw * self._rs_depth_scale * 1000.0
        return depth_mm.astype(np.uint16)

    def capture_depth_colorized(self) -> np.ndarray:
        """Capture depth and return a JET-colorized visualization (BGR uint8)."""
        import cv2

        depth_mm = self.capture_depth()
        depth_clipped = np.clip(depth_mm, RS_DEPTH_MIN_MM, RS_DEPTH_MAX_MM)
        depth_normalized = (
            (depth_clipped - RS_DEPTH_MIN_MM).astype(np.float32)
            / (RS_DEPTH_MAX_MM - RS_DEPTH_MIN_MM)
            * 255.0
        ).astype(np.uint8)
        depth_normalized[depth_mm == 0] = 0
        return cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

    def capture_stereo(self) -> Tuple[np.ndarray, np.ndarray]:
        """Capture the stereo infrared pair from the D405.

        Returns (ir_left, ir_right) as uint8 grayscale numpy arrays.
        """
        self._ensure_camera()
        frames = self._rs_pipeline.wait_for_frames()
        ir_left = frames.get_infrared_frame(1)
        ir_right = frames.get_infrared_frame(2)
        if not ir_left or not ir_right:
            raise RuntimeError("Failed to capture stereo infrared frames")
        return (
            np.asanyarray(ir_left.get_data()),
            np.asanyarray(ir_right.get_data()),
        )

    def capture_all(self) -> RealSenseFrames:
        """Capture all RealSense streams in a single synchronized frameset.

        Returns a RealSenseFrames namedtuple with color, depth,
        ir_left, and ir_right fields.
        """
        self._ensure_camera()
        frames = self._rs_pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        ir_left = frames.get_infrared_frame(1)
        ir_right = frames.get_infrared_frame(2)

        if not color_frame or not depth_frame or not ir_left or not ir_right:
            raise RuntimeError("Failed to capture one or more RealSense frames")

        raw_depth = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_mm = (raw_depth * self._rs_depth_scale * 1000.0).astype(np.uint16)

        return RealSenseFrames(
            color=np.asanyarray(color_frame.get_data()),
            depth=depth_mm,
            ir_left=np.asanyarray(ir_left.get_data()),
            ir_right=np.asanyarray(ir_right.get_data()),
        )

    def get_stereo_baseline(self) -> float:
        """Get the stereo baseline distance in millimeters.

        Starts the camera pipeline if not already running.
        """
        self._ensure_camera()
        import pyrealsense2 as rs
        profile = self._rs_pipeline.get_active_profile()
        ir1_stream = profile.get_stream(rs.stream.infrared, 1)
        ir2_stream = profile.get_stream(rs.stream.infrared, 2)
        extrinsics = ir1_stream.get_extrinsics_to(ir2_stream)
        baseline_m = abs(extrinsics.translation[0])
        return baseline_m * 1000.0

    def get_stereo_intrinsics(self):
        """Get the factory intrinsics for the left IR camera.

        Returns an rs2_intrinsics object with width, height, fx, fy, ppx, ppy.
        Starts the camera pipeline if not already running.
        """
        self._ensure_camera()
        import pyrealsense2 as rs
        profile = self._rs_pipeline.get_active_profile()
        ir1_stream = profile.get_stream(rs.stream.infrared, 1)
        return ir1_stream.as_video_stream_profile().get_intrinsics()

    def release_camera(self) -> None:
        """Stop the RealSense pipeline if it was started."""
        if self._rs_pipeline is not None:
            self._rs_pipeline.stop()
            self._rs_pipeline = None
            self._rs_config = None

    # ── USB camera ─────────────────────────────────────────────

    def _ensure_usb_camera(self) -> None:
        """Lazily initialize a standard USB camera via OpenCV."""
        if self._usb_camera is None:
            import cv2
            self._usb_camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
            self._usb_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._usb_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self._usb_camera.isOpened():
                self._usb_camera = None
                raise RuntimeError("Failed to open USB camera")
            for _ in range(30):
                self._usb_camera.read()

    def capture_usb_frame(self) -> np.ndarray:
        """Capture a frame from the USB camera (BGR numpy array)."""
        self._ensure_usb_camera()
        ret, frame = self._usb_camera.read()
        if not ret:
            raise RuntimeError("Failed to capture USB camera frame")
        return frame

    def release_usb_camera(self) -> None:
        """Release the USB camera if it was opened."""
        if self._usb_camera is not None:
            self._usb_camera.release()
            self._usb_camera = None

    # ── OLED display ───────────────────────────────────────────

    def _ensure_oled(self) -> None:
        if self._oled is None:
            import Adafruit_SSD1306
            from PIL import Image, ImageDraw, ImageFont
            self._oled = Adafruit_SSD1306.SSD1306_128_32(
                rst=None, i2c_bus=I2C_BUS, gpio=1
            )
            self._oled.begin()
            self._oled.clear()
            self._oled.display()
            self._oled_image = Image.new("1", (128, 32))
            self._oled_draw = ImageDraw.Draw(self._oled_image)
            self._oled_font = ImageFont.load_default()

    def display_text(self, text: str, line: int = 1) -> None:
        """Display text on one of the 4 OLED lines (1-4)."""
        self._ensure_oled()
        line = _clamp(line, 1, 4)
        y = 8 * (line - 1)
        self._oled_draw.rectangle([0, y, 127, y + 7], fill=0)
        self._oled_draw.text((0, y - 2), text, font=self._oled_font, fill=255)
        self._oled.image(self._oled_image)
        self._oled.display()

    def clear_display(self) -> None:
        """Clear the OLED display."""
        if self._oled is not None:
            self._oled_draw.rectangle([0, 0, 127, 31], fill=0)
            self._oled.image(self._oled_image)
            self._oled.display()

    # ── audio ──────────────────────────────────────────────────

    def play_sound(self, file_path: str) -> None:
        """Play a WAV/audio file through system audio (non-blocking)."""
        subprocess.Popen(
            ["aplay", file_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def record_audio(
        self,
        duration: float = 3.0,
        output_path: str = "recording.wav",
    ) -> str:
        """Record audio from microphone. Returns path to saved WAV file."""
        import pyaudio

        pa = pyaudio.PyAudio()
        stream = pa.open(
            format=pyaudio.paInt16,
            channels=AUDIO_CHANNELS,
            rate=AUDIO_RATE,
            input=True,
            frames_per_buffer=AUDIO_CHUNK,
        )

        frames = []
        num_chunks = int(AUDIO_RATE / AUDIO_CHUNK * duration)
        for _ in range(num_chunks):
            data = stream.read(AUDIO_CHUNK, exception_on_overflow=False)
            frames.append(data)

        stream.stop_stream()
        stream.close()
        pa.terminate()

        with wave.open(output_path, "wb") as wf:
            wf.setnchannels(AUDIO_CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(AUDIO_RATE)
            wf.writeframes(b"".join(frames))

        return output_path
