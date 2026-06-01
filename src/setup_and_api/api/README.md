# RasBot API Reference

The `rasbot.api` package gives you one class — `RasBot` — that controls every part
of the RasbotV2 Mecanum-wheel robot: the wheels, the camera servos, the cameras, the
sensors, the LEDs, the buzzer, the OLED screen, and the microphone/speaker.

You only ever need to import one thing:

```python
from rasbot.api import RasBot, Color
```

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Creating and Cleaning Up a Robot](#creating-and-cleaning-up-a-robot)
3. [Driving the Wheels](#driving-the-wheels)
4. [Camera Servos (Pan / Tilt)](#camera-servos-pan--tilt)
5. [LEDs](#leds)
6. [Buzzer](#buzzer)
7. [Sensors](#sensors)
8. [RealSense D405 Depth Camera](#realsense-d405-depth-camera)
9. [USB Camera](#usb-camera)
10. [OLED Display](#oled-display)
11. [Audio](#audio)
12. [Constants Reference](#constants-reference)

---

## Quick Start

```python
import time
from rasbot.api import RasBot, Color

# `with` makes sure the robot stops and cleans up, even if your code crashes.
with RasBot() as bot:
    bot.set_all_leds_color(Color.GREEN)   # turn the LED strip green
    bot.forward(speed=100)                # drive forward
    time.sleep(1.0)                       # ...for one second
    bot.stop()                            # stop the wheels

    distance = bot.read_distance()        # how far is the wall ahead?
    print(f"Wall is {distance:.1f} cm away")
```

When the `with` block ends, the robot automatically stops the motors, turns off the
LEDs and buzzer, re-centers the camera, and releases the cameras for you.

---

## Creating and Cleaning Up a Robot

### `RasBot(i2c_address=0x2B, i2c_bus=1)`

Creates the robot controller and opens the I2C connection to the controller board.

| Parameter     | Default | Meaning |
|---------------|---------|---------|
| `i2c_address` | `0x2B`  | I2C address of the RasbotV2 board. You almost never change this. |
| `i2c_bus`     | `1`     | I2C bus number on the Raspberry Pi. |

**Recommended use — the `with` statement:**

```python
with RasBot() as bot:
    ...  # use the robot here
# cleanup() runs automatically here
```

**Manual use** (only if you can't use `with`):

```python
bot = RasBot()
try:
    ...
finally:
    bot.cleanup()   # ALWAYS call this so the robot stops safely
```

### `bot.cleanup()`

Stops the motors, turns off all LEDs and the buzzer, disables the ultrasonic and IR
sensors, re-centers the camera servos, releases both cameras, and clears the OLED.
Called automatically when a `with RasBot()` block ends.

> ⚠️ **Always clean up.** If your program ends without calling `cleanup()` (or using
> `with`), the motors can keep spinning and the robot may drive off the table.

---

## Driving the Wheels

The robot has four [Mecanum wheels](https://en.wikipedia.org/wiki/Mecanum_wheel), which
let it move in *any* direction — including sideways and diagonally — without turning.

**Speed** is an integer from `0` to `255`. Higher is faster. A good starting value is
around `100`.

### Simple directions

| Method | What it does |
|--------|--------------|
| `bot.forward(speed=100)`  | Drive straight forward |
| `bot.backward(speed=100)` | Drive straight backward |
| `bot.left(speed=100)`     | Strafe (slide) left without turning |
| `bot.right(speed=100)`    | Strafe (slide) right without turning |
| `bot.rotate_left(speed=100)`  | Spin counter-clockwise in place |
| `bot.rotate_right(speed=100)` | Spin clockwise in place |

### Diagonal directions

| Method | What it does |
|--------|--------------|
| `bot.diagonal_left_front(speed=100)`  | Slide diagonally forward-left |
| `bot.diagonal_right_front(speed=100)` | Slide diagonally forward-right |
| `bot.diagonal_left_back(speed=100)`   | Slide diagonally backward-left |
| `bot.diagonal_right_back(speed=100)`  | Slide diagonally backward-right |

### Stopping

| Method | What it does |
|--------|--------------|
| `bot.stop()` | Stop all four motors immediately |

> **Important:** Movement commands do **not** stop on their own. `bot.forward()` keeps
> the robot moving until you call `bot.stop()` (or another movement command). The usual
> pattern is: move, `time.sleep(...)`, then `stop()`.

### Advanced: move at any angle

#### `bot.move(speed, angle_degrees)`

Drive in *any* direction. The angle uses this compass:

```
        90  (forward)
         |
 180 ----+---- 0   (right)
         |
        270 (backward)
```

```python
bot.move(speed=120, angle_degrees=45)   # forward-right diagonal
bot.move(speed=120, angle_degrees=90)   # same as bot.forward(120)
```

#### `bot.drift(speed, angle_degrees, rotation_rate)`

Move in a direction *and* spin at the same time (like a controlled skid). `rotation_rate`
is positive for one spin direction and negative for the other.

```python
bot.drift(speed=100, angle_degrees=90, rotation_rate=0.5)
```

---

## Camera Servos (Pan / Tilt)

The USB camera sits on a two-servo pan/tilt mount. (The D405 depth camera is fixed and
does **not** move.)

| Method | Range | Default | What it does |
|--------|-------|---------|--------------|
| `bot.set_pan(angle)`  | `0`–`180` | `90` | Turn camera left/right (90 = straight ahead) |
| `bot.set_tilt(angle)` | `0`–`100` | `25` | Tilt camera up/down |
| `bot.look_center()`   | —         | —    | Return both servos to their default positions |

Out-of-range values are automatically clamped to the valid range.

### Gestures

| Method | What it does |
|--------|--------------|
| `bot.nod(cycles=2, delay=0.3)`        | Nod the camera up and down (a "yes" gesture) |
| `bot.shake_head(cycles=2, delay=0.3)` | Pan the camera side to side (a "no" gesture) |

```python
bot.set_pan(45)        # look to the right
bot.set_tilt(60)       # look up
bot.nod()              # nod twice
bot.look_center()      # back to center
```

---

## LEDs

The robot has a strip of **14 RGB LEDs**, numbered `1` to `14`. You can set them to any
RGB color, or use a preset `Color`.

### Custom RGB colors (0–255 per channel)

| Method | What it does |
|--------|--------------|
| `bot.set_all_leds(r, g, b)`        | Set every LED to one RGB color |
| `bot.set_led(index, r, g, b)`      | Set one LED (`index` 1–14) to an RGB color |

```python
bot.set_all_leds(255, 0, 0)     # all red
bot.set_led(1, 0, 0, 255)       # first LED blue
```

### Preset colors

Use the `Color` enum: `RED`, `GREEN`, `BLUE`, `YELLOW`, `PURPLE`, `CYAN`, `WHITE`.

| Method | What it does |
|--------|--------------|
| `bot.set_all_leds_color(color)`     | Set every LED to a preset color |
| `bot.set_led_color(index, color)`   | Set one LED (`index` 1–14) to a preset color |

```python
from rasbot.api import Color
bot.set_all_leds_color(Color.PURPLE)
bot.set_led_color(7, Color.CYAN)
```

### Turning them off

| Method | What it does |
|--------|--------------|
| `bot.leds_off()` | Turn off all LEDs |

---

## Buzzer

| Method | What it does |
|--------|--------------|
| `bot.buzzer_on()`         | Turn the buzzer on (stays on) |
| `bot.buzzer_off()`        | Turn the buzzer off |
| `bot.beep(duration=0.2)`  | Beep once for `duration` seconds, then stop |

```python
bot.beep()           # short beep
bot.beep(1.0)        # one-second beep
```

---

## Sensors

### `bot.read_distance() -> float`

Reads the front ultrasonic distance sensor and returns the distance to the nearest
object **in centimeters**. The sensor turns itself on automatically the first time you
call this.

```python
if bot.read_distance() < 20:
    bot.stop()
    print("Obstacle ahead!")
```

### `bot.read_line_sensors() -> (bool, bool, bool, bool)`

Reads the four downward-facing line-tracking sensors. Returns a tuple in this order:

```
(left_outer, left_inner, right_inner, right_outer)
```

Each value is `True` when that sensor is over a **dark line**, `False` over a light
surface. Useful for line-following projects.

```python
lo, li, ri, ro = bot.read_line_sensors()
if li and ri:
    bot.forward(80)        # line is centered, go straight
elif li:
    bot.rotate_left(60)    # line drifted left, correct
elif ri:
    bot.rotate_right(60)   # line drifted right, correct
```

### `bot.read_ir() -> int`

Reads the infrared remote-control receiver. Returns the raw byte value (`0`–`255`) of
the last button received. The receiver turns itself on automatically on first use.

### `bot.read_button() -> bool`

Returns `True` while the on-board button is pressed, `False` otherwise.

```python
print("Press the button to start...")
while not bot.read_button():
    time.sleep(0.05)
print("Go!")
```

---

## RealSense D405 Depth Camera

The Intel RealSense D405 is a **fixed** stereo depth camera (it does not move with the
servos). It can return color images, depth maps, and the raw left/right infrared images
used for stereo vision.

All capture methods start the camera automatically the first time you call them.

> 📏 **Depth units:** every depth value returned by this API is in **millimeters**. The
> conversion from the camera's raw units is handled for you.

### `bot.capture_frame() -> ndarray`

Captures one **color** image as a BGR NumPy array of shape `(480, 640, 3)`.

```python
import cv2
img = bot.capture_frame()
cv2.imwrite("photo.png", img)
```

### `bot.capture_depth() -> ndarray`

Captures a **depth** image as a `uint16` NumPy array of shape `(480, 640)`. Each pixel
is the distance to that point **in millimeters** (`0` means "no reading").

```python
depth = bot.capture_depth()
center_mm = depth[240, 320]      # distance at the center pixel
print(f"Object in front is {center_mm} mm away")
```

### `bot.capture_depth_colorized() -> ndarray`

Captures the depth image and returns a colorful BGR visualization (close = one color,
far = another) that's easy to look at or save. Good for debugging.

```python
import cv2
cv2.imwrite("depth_view.png", bot.capture_depth_colorized())
```

### `bot.capture_stereo() -> (ndarray, ndarray)`

Captures the **left and right infrared** images as a pair of `uint8` grayscale arrays,
`(ir_left, ir_right)`. These are the raw stereo images for doing your own stereo-vision
or disparity work.

### `bot.capture_all() -> RealSenseFrames`

Captures **everything at once**, all synchronized to the same instant. Returns a
`RealSenseFrames` named tuple with four fields:

| Field | Type | Description |
|-------|------|-------------|
| `.color`    | BGR `uint8` `(480,640,3)` | Color image |
| `.depth`    | `uint16` `(480,640)`      | Depth in millimeters |
| `.ir_left`  | `uint8` `(480,640)`       | Left infrared image |
| `.ir_right` | `uint8` `(480,640)`       | Right infrared image |

```python
frames = bot.capture_all()
print(frames.color.shape, frames.depth.shape)
center_distance = frames.depth[240, 320]
```

### Camera geometry (for stereo math)

| Method | Returns |
|--------|---------|
| `bot.get_stereo_baseline()`   | Distance between the two IR cameras, in **millimeters** |
| `bot.get_stereo_intrinsics()` | Factory intrinsics for the left IR camera (`fx`, `fy`, `ppx`, `ppy`, `width`, `height`) |

```python
baseline = bot.get_stereo_baseline()
intr = bot.get_stereo_intrinsics()
print(f"baseline={baseline:.1f} mm, fx={intr.fx:.1f}")
```

### `bot.release_camera()`

Stops the RealSense pipeline. Usually you don't call this directly — `cleanup()` does it
for you.

---

## USB Camera

A standard USB webcam is also available (this is the one on the pan/tilt mount).

### `bot.capture_usb_frame() -> ndarray`

Captures one frame from the USB camera as a BGR NumPy array. Starts the camera
automatically on first use.

```python
frame = bot.capture_usb_frame()
```

### `bot.release_usb_camera()`

Releases the USB camera. Handled for you by `cleanup()`.

---

## OLED Display

A small 128×32 OLED screen with **4 text lines** (numbered 1–4).

### `bot.display_text(text, line=1)`

Writes `text` on the given line (`1`–`4`), replacing whatever was there.

```python
bot.display_text("RasBot ready", line=1)
bot.display_text(f"Dist: {bot.read_distance():.0f} cm", line=2)
```

### `bot.clear_display()`

Clears the entire screen.

---

## Audio

### `bot.play_sound(file_path)`

Plays a WAV file through the speaker. **Non-blocking** — your program keeps running while
the sound plays.

```python
bot.play_sound("beep.wav")
```

### `bot.record_audio(duration=3.0, output_path="recording.wav") -> str`

Records from the microphone for `duration` seconds, saves it to `output_path`, and
returns that path. **Blocking** — your program waits until recording finishes.

```python
path = bot.record_audio(duration=5.0, output_path="my_voice.wav")
bot.play_sound(path)
```

---

## Constants Reference

These live in `rasbot.api.constants`. Most students only need `Color`.

### `Color` (preset LED colors)

```python
from rasbot.api import Color
```

`Color.RED`, `Color.GREEN`, `Color.BLUE`, `Color.YELLOW`, `Color.PURPLE`,
`Color.CYAN`, `Color.WHITE`

### Useful default values

| Constant | Value | Meaning |
|----------|-------|---------|
| `LED_COUNT`     | `14`  | Number of addressable LEDs |
| `PAN_DEFAULT`   | `90`  | Centered pan angle |
| `TILT_DEFAULT`  | `25`  | Default tilt angle |
| `PAN_MIN`/`PAN_MAX`   | `0`/`180` | Pan servo range |
| `TILT_MIN`/`TILT_MAX` | `0`/`100` | Tilt servo range |
| `RS_FRAME_WIDTH` × `RS_FRAME_HEIGHT` | `640` × `480` | RealSense resolution |
| `RS_FPS`        | `30`  | RealSense frame rate |

> The `Register` and `Motor` enums are used internally by `RasBot` to talk to the
> hardware. You normally never need them directly.
