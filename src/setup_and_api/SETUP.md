# RasBot Raspberry Pi Setup

Fresh Raspberry Pi OS (Bookworm/Trixie) setup instructions for the RasbotV2 robot.

## 1. Flash OS

Use Raspberry Pi Imager to flash **64-bit** Raspberry Pi OS to the SD card. The 64-bit variant is required for Intel RealSense support (Pi 3B+ and later all support 64-bit).

## 2. Enable I2C

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

## 3. Core Dependencies

```bash
sudo apt update
sudo apt install -y python3-numpy python3-opencv python3-smbus i2c-tools
```

## 4. OLED Display Support (optional)

```bash
sudo apt install -y python3-pil python3-pip
pip install Adafruit-SSD1306 --break-system-packages
```

## 5. Audio Recording/Playback (optional)

```bash
sudo apt install -y python3-pyaudio portaudio19-dev alsa-utils
```

## 6. Intel RealSense D405 Camera (optional)

Must be built from source on ARM. Requires **64-bit** Raspberry Pi OS — the NEON optimizations
in librealsense use AArch64-only intrinsics that fail to compile on 32-bit armhf.

```bash
# Build dependencies
sudo apt install -y git cmake libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Clone and build
cd ~
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Set up udev rules (allows camera access without root)
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Build with Python bindings
mkdir build && cd build
cmake ../ \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DFORCE_RSUSB_BACKEND=true
make -j$(nproc)
sudo make install
```

If `import pyrealsense2` fails after install, add the library path:

```bash
echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages' >> ~/.bashrc
source ~/.bashrc
```

**If stuck on 32-bit:** remove the NEON source before building:
```bash
sed -i '/image-neon.cpp/d' src/proc/CMakeLists.txt
```
Then re-run cmake and make as above. This falls back to the C implementation (slightly slower but functional).

## 7. Verify Installation

```bash
# Check I2C sees the robot board (should show device at 0x2B)
i2cdetect -y 1

# Check Python imports
python3 -c "import smbus; import numpy; import cv2; print('All good')"

# Check RealSense (if installed)
python3 -c "import pyrealsense2 as rs; print(rs)"
```

## 8. Copy RasBot Code

Copy the `rasbot/` directory to the Pi and run tests:

```bash
cd /path/to/rasbot
python3 tests.py
```

Edit `TESTS_TO_RUN` in `tests.py` to select which tests to run.