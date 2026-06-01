# RasotV2 Camera Stream & Control Dashboard Setup

## Quick Start (3 Steps)

### 1. **On the Raspberry Pi** — Start the control server

```bash
cd /path/to/sprint_hackathon/src/
python control_server.py
```

You should see:
```
============================================================
RasotV2 Control Server
============================================================
Control Server: http://192.168.x.x:9000/
Stream Server:  http://192.168.x.x:8000/stream.mjpg
============================================================
Press Ctrl+C to stop
```

### 2. **On Your Control Computer** — Open the dashboard

Open the HTML file in your browser:
```
sprint_hackathon/top/robot_control_dashboard.html
```

Or visit directly:
```
file:///path/to/sprint_hackathon/top/robot_control_dashboard.html
```

### 3. **Configure and Connect**

1. Click the **⚙** button (bottom-right of video area)
2. Enter your Raspberry Pi IP address:
   - **Control Server**: `192.168.x.x:9000`
   - **Stream Server**: `192.168.x.x:8000`
3. Click **Save**
4. Click **Connect** button — the control server will automatically start the camera stream

That's it! 🚀 Live video should appear in the dashboard.

---

## Architecture

### **Control Server** (`control_server.py`) — Port 9000
Manages all robot services:
- `POST /api/stream/start` — Start camera streaming
- `POST /api/stream/stop` — Stop camera streaming
- `GET /api/stream/status` — Check if stream is running
- `GET /health` — Server health check

### **Stream Server** (`stream_server.py`) — Port 8000
Streams camera feed via MJPEG:
- Captures from RealSense D405 camera
- Encodes as JPEG per frame
- Streams via HTTP boundary chunks (MJPEG)
- Displays FPS overlay

### **Dashboard** (`robot_control_dashboard.html`)
- Displays live camera feed
- Connects to control server via HTTP API
- Shows keyboard layout with color-coded controls
- Tracks key presses and logs actions
- Configurable server IP/port

---

## Features

### **One-Click Connection**
- Click **Connect** button to start everything
- Control server launches streaming automatically
- No manual server management needed

### **Camera Stream (Right Panel)**
- Live MJPEG feed from the camera
- Real-time FPS counter overlay
- Status indicator (connected/disconnected)
- Connect/Disconnect buttons

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
- Stream connection status with visual indicator
- Server health check on startup

---

## Technical Details

### Control Server (`control_server.py`)
- **Language**: Python 3
- **Protocol**: HTTP REST API
- **Port**: 9000
- **Functionality**:
  - Spawns `stream_server.py` as subprocess
  - Monitors process health
  - Provides status endpoints
  - Gracefully manages startup/shutdown

### Stream Server (`stream_server.py`)
- **Protocol**: MJPEG over HTTP
- **Port**: 8000
- **Resolution**: 1280×720 @ 30fps
- **Encoding**: Tries MJPG codec first, falls back to native format
- **FPS Overlay**: Shows real-time frame rate on feed
- **Buffer**: Single-frame buffer to minimize latency

### Dashboard (`robot_control_dashboard.html`)
- **Technology**: HTML5/CSS3/JavaScript (no dependencies)
- **Storage**: LocalStorage for configuration persistence
- **API Calls**: Fetch API for control server communication
- **Keyboard**: Native DOM events for real-time key tracking
- **Theme**: Dark mode optimized for robot operation centers

---

## Troubleshooting

### Connect button not working / Control server not reachable
1. Ensure control server is running: `python control_server.py`
2. Check firewall allows port 9000
3. Verify correct Raspberry Pi IP: `hostname -I` on the Pi
4. Test connection: `curl http://192.168.x.x:9000/health`
5. Try clicking **⚙ Config** to manually enter IP and click **Save**

### "Control server not connected" warning
- Control server health check failed
- Solutions:
  1. Check if control server is still running
  2. Verify network connection between computer and Pi
  3. Try pinging the Pi: `ping 192.168.x.x`
  4. Check if firewall is blocking port 9000

### Stream won't start even after clicking Connect
1. Check control server output for error messages
2. Verify `stream_server.py` exists at `src/camera/stream_server.py`
3. Test stream server manually: `python src/camera/stream_server.py`
4. Check camera is accessible: `v4l2-ctl --list-devices`

### Black frames in stream
- Server auto-detects and switches from MJPG to native format
- Check camera permissions: `ls -l /dev/video0`
- Ensure USB camera is working: `python src/camera/live_view.py`

### Camera not found
- Check with: `v4l2-ctl --list-devices`
- Update `CAM_INDEX` in `stream_server.py` (line 14) if needed

### High latency / low frame rate
- Reduce JPEG quality in `stream_server.py` (line ~94, `cv2.IMWRITE_JPEG_QUALITY`)
- Ensure camera is connected to high-speed USB port
- Reduce resolution in `stream_server.py` constants (W, H)

---

## Running as a Service (Optional)

To auto-start the control server on Raspberry Pi boot:

Create `/etc/systemd/system/rasbot-control.service`:
```ini
[Unit]
Description=Rasbot V2 Control Server
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/sprint_hackathon/src
ExecStart=/usr/bin/python3 control_server.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable rasbot-control.service
sudo systemctl start rasbot-control.service
```

Check status:
```bash
sudo systemctl status rasbot-control.service
```

View logs in real-time:
```bash
sudo journalctl -u rasbot-control.service -f
```

Stop the service:
```bash
sudo systemctl stop rasbot-control.service
```

---

## File Structure

```
sprint_hackathon/
├── src/
│   ├── control_server.py          ← Main control server (start this!)
│   ├── camera/
│   │   ├── stream_server.py       ← Streaming server (launched by control_server)
│   │   ├── live_view.py           ← Camera test script
│   │   └── ...
│   └── ...
├── top/
│   └── robot_control_dashboard.html ← Web interface
├── CAMERA_STREAM_SETUP.md         ← This file
└── ...
```
