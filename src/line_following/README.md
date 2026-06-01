# Line Following (Mode 2) — placeholder

This folder is reserved for **Mode 2** from the project brief: the robot follows a
dark tape path on the floor using its **4 IR line-tracking sensors**, and at each
**stop marker** (a perpendicular cross-mark that trips all four sensors at once) it
halts, runs the 360 scan, and resumes to the next marker.

## Status

**Not implemented / not tested on hardware yet.** [line_follow.py](line_follow.py) is a
scaffold only — it wires up the real `RasBot` API and the existing `pointcloud/scan360`
capture routine the same way [wasd/drive.py](../wasd/drive.py) does, with a first-draft
steering loop. The thresholds, speeds, sensor order/polarity, and stop-marker debounce
all still need to be worked out and tuned on the real robot.

## How it hooks into the rest of the project

- **Sensors:** `bot.read_line_sensors()` → `(left_outer, left_inner, right_inner, right_outer)`,
  each `True` when over the dark line. See [setup_and_api/api/README.md](../setup_and_api/api/README.md).
- **Movement:** `bot.forward()`, `bot.rotate_left/right()`, `bot.stop()`.
- **Capture at a marker:** `scan360.scan_and_build(bot, cam)` — the same sweep + on-Pi
  merge that `drive.py` runs on the `R`+`T` keys.

## Run (on the Pi, once it's implemented)

```bash
python3 line_following/line_follow.py
```
