# line_follow_backup — simple, predictable black-line follower (backup)

A self-contained **backup** for `src/tape_following/` (which became wobbly/unpredictable after the
adaptive-speed + path-memory + auto-tune rewrite). This folder is **isolated** — it changes nothing
in `src/`, it only *reads* the robot API. Runs on the **Pi** (`RasBot`, so `smbus` — Pi only).

## What's here
| File | What |
|---|---|
| **`follow.py`** | the one to use — constant-speed proportional+derivative tank steer, line-loss recovery, and junction handling for the known track |
| `p_follow.py` | minimal proportional fallback (copy of the old `archive/p_follow.py`, import fixed) |
| `simple_follow.py` | discrete fallback that pivots hard on 90° corners (copy of old `archive/simple_follow.py`, import fixed) |

## Run (on the Pi)
```bash
cd ~/sprint_hackathon/test_code/line_follow_backup
python3 follow.py --test          # FIRST: place robot on the line, watch which sensors fire
python3 follow.py                 # drive the track (default speed 120)
python3 follow.py --speed 150     # faster
python3 follow.py --plan left,left,right   # turn decision at each junction, in order
```
Stop with **Ctrl+C** (it also stops itself after ~2 s with no line = end of track).

## How it works (no magic, no adaptive speed)
1. `read_line_sensors()` → `(Lo, Li, Ri, Ro)`, `True` = over the black tape.
2. Collapse to one **error** (tape left = −, right = +) via weights `(-2.5, -1, +1, +2.5)`.
3. **Tank steer**: `left = speed + Kp·error + Kd·Δerror`, `right = speed − …` (both left wheels one
   speed, both right wheels the other). Speed is **constant**. `Kd` only damps wobble.
   - *Why tank, not `drift()`*: the API's `drift()` puts rotation on the front/rear axle, not
     left/right, so it can't steer a moving robot. Tank (`_apply_motors(L,L,R,R)`) is the fix.
4. **Lost line** → keep turning toward the side it last saw (memory).
5. **Junction/fork** (3+ sensors lit at once) → follow `JUNCTION_PLAN` (default: take the **left**/
   main road), commit briefly, then resume. Normal left/right corners need no plan — the error
   steer handles them.

## Your track
`straight → left → straight → left FORK (main road = left) → turn`. The defaults are tuned for this
(left-heavy, one left fork). If the final turn is a **right fork**, set `--plan left,left,right`.

## Tuning (do this once on the real track — I can't test hardware here)
- **Wobbly on straights** → lower `--kp` (try 45) or raise `--kd` (try 20), or slow down.
- **Misses / cuts corners** → raise `--kp` (try 75) or slow down (`--speed 100`).
- **Doesn't take the fork** → check `--test`: a real fork should light 3+ sensors. If corners also
  light 3+, raise `--junction-min` to 4 so only true crossings count.
- **Turns the wrong way at a fork** → fix the order in `--plan`.
- All knobs are constants at the top of `follow.py` too.

## Why not the existing `src/tape_following`
`best_follow.py`/`advanced_follow.py` add adaptive speed, path memory, turn prediction and
auto-tune — powerful but unpredictable. This backup deliberately keeps **one constant speed and a
plain controller** so it behaves the same every run, which is what you want for filming a fixed track.

Sources: [PD line-follower (GitHub)](https://github.com/magni-mythos/LineFollowerBot),
[line follower w/ intersections (GitHub)](https://github.com/Mummanajagadeesh/line-follower-robot-w).
