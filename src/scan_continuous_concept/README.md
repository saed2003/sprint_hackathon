# Continuous-rotation 360 scan — concept / experiment

Isolated sandbox for the **alternative** 360 idea (spin while streaming, subsample frames,
build afterward). **Nothing outside this folder is modified** — it uses its own copy of the
camera helper and only *reads* the unchanged merge in `pointcloud/scan360.py`. Delete this
folder to fully revert to the shipped pipeline.

## The idea (from the chat)
> With the D405, spin in place while the camera streams (30/15 fps), keep every ~5th frame,
> then build one 3D map from all the kept depth/IR frames.

## Can the D405 do it?
Yes. It streams depth + **hardware-synced** stereo IR at 30 fps, so depth is valid even while
moving. The only motion cost is **IR motion blur**, which a short locked exposure
(`CONT_EXPOSURE_US = 6000`) keeps small at a slow spin.

## How this version stays accurate
The robot has **no IMU/encoders**, so the timing-based pose in the original `CLAUDE.md`
continuous plan would drift/overshoot — that's exactly why the shipped scan went discrete +
vision-measured. So here we **don't trust the spin timing for geometry**: frames are saved
**without `angle.txt`**, which makes `scan360.build_from_session()` recover each step's yaw
from the **overlap between consecutive IR frames** (ORB → homography `H = K R K⁻¹` → yaw).
A continuous sweep makes consecutive frames only ~5–6° apart with ~95% overlap — the regime
where that homography is *most* robust (much better than the shipped scan's 36° jumps). The
spin timing only decides **when to stop** (~one full turn); it never enters the cloud.

## Honest trade-offs vs the shipped `scan360.py`
- ✅ faster, smoother, no start/stop shake, dense overlap → robust per-step vision angle.
- ⚠️ each frame has some motion blur → per-frame depth a touch noisier than a dead-stop shot.
- ⚠️ many frames (~50–70) → the on-Pi merge takes longer (ORB on each consecutive pair).
- ❌ at a **fast** spin it degrades (more blur + bigger gaps). Keep the spin slow.

**Will the D405 always give bad pics?** No — passive stereo just needs *texture + light* and
*short exposure while moving*. Blank walls and a fast spin are what hurt it, not motion itself.

## Run it
```bash
# on the robot (SSH or VNC; no display needed):
python3 scan_continuous_concept/scan_continuous.py                 # spin, capture, build
python3 scan_continuous_concept/scan_continuous.py --seconds 16    # slower full turn
python3 scan_continuous_concept/scan_continuous.py --speed 30 --sub 6 --exposure 5000
python3 scan_continuous_concept/scan_continuous.py --no-exposure   # let auto-exposure run

# rebuild from frames already captured (no robot):
python3 scan_continuous_concept/scan_continuous.py captures/cscan_<timestamp>

# view the result (shipped viewer):
python3 pointcloud/view3d.py captures/cscan_<timestamp>/merged_360.ply
```

## Calibrate the one number
`CONT_FULL_ROTATION_SEC` = seconds for one full 360 at `CONT_SPIN_SPEED`. Tape the floor,
spin, time one turn by stopwatch, set it. It only controls when the spin stops, so it's not
critical — a little past a full turn is fine.

## If you like it, wire a key (optional, later)
This runs as a standalone command on purpose (keeps `wasd/drive.py` untouched). To add an
`O` key later, mirror the `R` handler in `drive.py`:
```python
elif key == pygame.K_o:
    import sys, os
    sys.path.insert(0, os.path.join(root, 'scan_continuous_concept'))
    import scan_continuous
    last_session = scan_continuous.run_continuous_scan(bot, cam, log=print)
    # then press T to build, as usual
```
