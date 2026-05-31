from enum import IntEnum


# I2C configuration
I2C_ADDRESS = 0x2B
I2C_BUS = 1


class Register(IntEnum):
    """I2C register map for the RasbotV2 controller board."""
    MOTOR = 0x01
    SERVO = 0x02
    LED_ALL = 0x03
    LED_SINGLE = 0x04
    IR_SWITCH = 0x05
    BUZZER = 0x06
    ULTRASONIC_SWITCH = 0x07
    LED_RGB_ALL = 0x08
    LED_RGB_SINGLE = 0x09
    LINE_TRACK = 0x0A
    IR_DATA = 0x0C
    BUTTON = 0x0D
    ULTRASONIC_LOW = 0x1A
    ULTRASONIC_HIGH = 0x1B


class Color(IntEnum):
    """Preset color codes for WS2812 LEDs."""
    RED = 0
    GREEN = 1
    BLUE = 2
    YELLOW = 3
    PURPLE = 4
    CYAN = 5
    WHITE = 6


class Motor(IntEnum):
    """Motor IDs mapped to physical wheel positions."""
    LEFT_FRONT = 0
    LEFT_REAR = 1
    RIGHT_FRONT = 2
    RIGHT_REAR = 3


# Servo configuration
PAN_SERVO_ID = 1
TILT_SERVO_ID = 2
PAN_DEFAULT = 90
TILT_DEFAULT = 25
PAN_MIN, PAN_MAX = 0, 180
TILT_MIN, TILT_MAX = 0, 100

# LED count
LED_COUNT = 14

# Chassis geometry (mm half-widths averaged) for drift/rotation calculations
CHASSIS_HALF_WIDTH = (117 + 132) / 2

# Audio defaults
AUDIO_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_CHUNK = 1024

# RealSense D405 camera defaults
RS_FRAME_WIDTH = 640
RS_FRAME_HEIGHT = 480
RS_FPS = 30
RS_DEPTH_MIN_MM = 70     # D405 minimum usable range
RS_DEPTH_MAX_MM = 500    # D405 optimal max range
