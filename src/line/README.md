# Line Follower

Simple line follower that uses the robot's 4 color sensors to navigate over black tape.

## Sensors

The robot has 4 color sensors in a single bar (70mm wide):
- **L1** (outer-left)
- **L2** (inner-left)  
- **R1** (inner-right)
- **R2** (outer-right)

Each sensor returns:
- **True (1)** = sees BLACK tape
- **False (0)** = sees no tape (white/light background)

## Usage

### Run the follower

```bash
cd /Users/saleh/onMyMac/sprint/sprint_hackathon
python3 src/line/follow.py
```

The robot will:
1. Detect the black tape
2. Use PD control to keep the tape centered between sensors
3. Drive forward smoothly
4. Stop when the tape is lost for 2 seconds

### Calibrate / test sensors

```bash
python3 src/line/follow.py --calibrate
```

This will continuously read and display all 4 sensor values so you can verify they're working correctly. Place tape under the sensors and watch the values change.

### Disable debug output

```bash
python3 src/line/follow.py --no-debug
```

## How it works

The follower uses **PD (Proportional-Derivative) control** to steer:

1. **Read sensors** → determine which side the tape is
2. **Compute error** → how far left/right the tape is
3. **Apply PD control** → adjust left/right wheel speeds proportionally
4. **Smooth movement** → exponential moving average to avoid jerkiness

The algorithm:
- If both inner sensors (L2, R1) see tape → centered, go straight
- If tape drifts left → reduce left wheel speed, increase right wheel speed (turn right to recenter)
- If tape drifts right → opposite
- If all sensors go dark → line is lost, stop after 2 seconds

## Tuning

Edit these parameters in `follow.py` to adjust behavior:

```python
SPEED          = 150      # how fast to drive (0-255)
Kp             = 20       # how aggressively to steer (proportional)
Kd             = 12       # smoothing factor (derivative)
SMOOTH         = 0.3      # motor smoothing (0.1=smooth, 0.5=snappy)
END_LOST_SEC   = 2.0      # how long to wait before stopping after losing tape
```

- **Increase Kp** to steer harder (but risk oscillation)
- **Increase Kd** to dampen oscillation
- **Increase SMOOTH** for snappier response, decrease for smoother movement
