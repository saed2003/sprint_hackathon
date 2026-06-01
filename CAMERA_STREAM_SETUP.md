# RasotV2 Camera Stream & Control Dashboard Setup

## Quick Start

### 1. **On the Raspberry Pi** — Start the streaming server

```bash
cd /path/to/sprint_hackathon/src/camera/
python stream_server.py
```

You should see:
```
============================================================
MJPEG Stream Server Running
============================================================
Stream URL: http://192.168.x.x:8000/stream.mjpg
Dashboard:  http://192.168.x.x:8000/
============================================================
```

### 2. **On Your Control Computer** — Open the dashboard

#### Option A: Direct Stream Access
```
http://raspberrypi-ip:8000/
```
(The streaming server has a built-in web interface)

#### Option B: Use the Control Dashboard
```
http://your-computer/robot_control_dashboard.html?stream=192.168.x.x:8000
```

Or open `robot_control_dashboard.html` locally and use the **⚙ Config** button to enter the stream URL.

---

## Features

### **Camera Stream (Right Panel)**
- Live MJPEG feed from the RealSense D405 camera
- Auto-detects common local addresses (localhost, raspberrypi.local, robot.local)
- Configurable via the Config panel or URL parameter

### **Control Keyboard (Left Panel)**
#### Movement (Blue)
- `W` — Forward
- `A` — Strafe Left  
- `S` — Backward
- `D` — Strafe Right

#### Rotation (Orange)
- `Q` — Rotate Left
- `E` — Rotate Right

#### Actions (Green)
- `SPACE` — Capture 360° Point Cloud

#### Mode Control (Purple)
- `M` — Toggle Manual ↔ Autonomous

### **Status Displays**
- Current driving mode (Manual/Autonomous)
- Hotkey legend with color-coded categories
- Stream connection status

---

## Technical Details

### Stream Server (`stream_server.py`)
- **Protocol**: MJPEG over HTTP
- **Port**: 8000 (configurable)
- **Resolution**: 1280×720 @ 30fps
- **Encoding**: Tries MJPG codec first, falls back to native format for reliability
- **FPS Overlay**: Shows real-time frame rate on feed

### Dashboard (`robot_control_dashboard.html`)
- Pure HTML/CSS/JavaScript (no dependencies)
- Auto-detects streaming server on local network
- Keyboard event tracking with visual feedback
- Dark theme optimized for robot operation

---

## Troubleshooting

### Stream won't connect
1. Ensure streaming server is running: `python stream_server.py`
2. Check firewall allows port 8000
3. Verify Raspberry Pi IP address: `hostname -I`
4. Manually enter IP in Config panel

### Black frames in stream
- Server auto-detects and switches from MJPG to native format
- Check camera permissions: `ls -l /dev/video0`

### Camera not found
- Check with: `v4l2-ctl --list-devices`
- Update `CAM_INDEX` in `stream_server.py` if needed

### High latency
- Reduce JPEG quality in `stream_server.py` (line ~94, `cv2.IMWRITE_JPEG_QUALITY`)
- Ensure camera is connected to high-speed USB port

---

## Running as a Service (Optional)

Create `/etc/systemd/system/rasbot-stream.service`:
```ini
[Unit]
Description=Rasbot V2 Camera Stream Server
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/sprint_hackathon/src/camera
ExecStart=/usr/bin/python3 stream_server.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable rasbot-stream.service
sudo systemctl start rasbot-stream.service
```

View logs:
```bash
sudo journalctl -u rasbot-stream.service -f
```
