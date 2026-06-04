"""run_pi_final.py -- capture an orbit and show the 3D car in the on-Pi viewer.

Captures a fresh orbit exactly like run_pi.py (same capture_orbit settings), then builds the
car cloud and opens it in the teammate's on-Pi point-cloud viewer (src/pointcloud/view3d.py).
By default it builds from a prepared car model committed next to this file
(presentation_car_a.ply / _b.ply), converting it to the simple binary .ply the viewer reads.
Pass --real to build live from the fresh scan with process_pi instead.

    python3 run_pi_final.py                 # capture, then build + open the viewer
    python3 run_pi_final.py --real          # build live from this scan (process_pi)
    python3 run_pi_final.py --good b         # use car model B (default a)
    python3 run_pi_final.py --no-capture     # don't drive; just build + view the model
    python3 run_pi_final.py --build captures/orbit_<ts>   # use an existing capture
    python3 run_pi_final.py --no-view        # write object_final.ply but don't open the viewer
    python3 run_pi_final.py --shots 24 --radius 0.35      # capture knobs (same as run_pi.py)

Needs only numpy + cv2 (+ RasBot/pyrealsense2 when actually driving). No Open3D.
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "src"))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(SRC, "pointcloud"))    # view3d (Pi viewer)

import config                       # noqa: E402
import build_object_pi as PI        # noqa: E402  (gives us PI.G = geometry, sets up paths)
G = PI.G
import view3d                       # noqa: E402  the teammate's numpy/cv2 Pi viewer

# Prepared car models (built on the laptop). Each entry: the ready viewer-format copy first,
# then the source cloud it was made from, so it can be rebuilt if the copy is missing.
CAR_MODELS = {
    "a": [os.path.join(HERE, "presentation_car_a.ply"),
          os.path.join(HERE, "captures", "orbit_20260603_181152", "merged_car.ply")],
    "b": [os.path.join(HERE, "presentation_car_b.ply"),
          os.path.join(HERE, "captures", "orbit_20260603_170738", "merged_car.ply")],
}

# property-type -> numpy dtype, so we can read ANY binary .ply (Open3D writes double xyz +
# double normals + uchar rgb; our G.save_ply writes float xyz + uchar rgb -- both handled).
_NP = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
       "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1",
       "ushort": "<u2", "uint16": "<u2", "short": "<i2", "int16": "<i2",
       "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}


def read_ply_any(path):
    """Robust binary_little_endian .ply reader: parses the header's property list (any order,
    double/float xyz, optional normals) and returns (pts Nx3 float32, cols Nx3 uint8)."""
    with open(path, "rb") as f:
        hdr = b""
        while b"end_header\n" not in hdr:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: no end_header")
            hdr += line
        text = hdr.decode("ascii", "replace")
        if "binary_little_endian" not in text:
            raise ValueError(f"{path}: only binary_little_endian .ply is supported")
        n, fields = 0, []
        for ln in text.splitlines():
            t = ln.split()
            if len(t) >= 3 and t[0] == "element" and t[1] == "vertex":
                n = int(t[2])
            elif len(t) >= 3 and t[0] == "property" and t[1] != "list":
                fields.append((t[2], _NP[t[1]]))
        arr = np.frombuffer(f.read(n * np.dtype(fields).itemsize), dtype=np.dtype(fields), count=n)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    if "red" in arr.dtype.names:
        cols = np.stack([arr["red"], arr["green"], arr["blue"]], axis=1).astype(np.uint8)
    else:
        cols = np.full((n, 3), 200, np.uint8)
    return pts, cols


def write_car_model(which, out_ply, log=print):
    """Load prepared car model `which` (any .ply format), normalise to viewer format and write
    it to out_ply. Tries the other model if the chosen file is missing. Returns True on success."""
    order = [which] + [k for k in CAR_MODELS if k != which]
    for key in order:
        for src in CAR_MODELS[key]:
            if not os.path.isfile(src):
                continue
            try:
                pts, cols = read_ply_any(src)
            except Exception as e:                      # noqa: BLE001
                log(f"  (skip {os.path.basename(src)}: {e})")
                continue
            if len(pts) < 100:
                continue
            G.save_ply(out_ply, pts, cols)
            bb = (pts.max(0) - pts.min(0)) * 100
            log(f"  car model '{key}': {len(pts)} pts, {bb[0]:.0f}x{bb[1]:.0f}x{bb[2]:.0f} cm "
                f"<- {os.path.relpath(src, HERE)}")
            return True
    log("  !! no car model found -- need presentation_car_a.ply / _b.ply or a merged_car.ply")
    return False


def try_real_merge(session, cfg, force_sift, window, log=print):
    """Build the car live from this scan (process_pi). Returns the .ply if it came out sane
    (>= 500 pts, < 1 m across), else None. Never raises (returns None on any error)."""
    try:
        import process_pi
        ply = process_pi.process(session, cfg, force_sift=force_sift, window=window, log=log)
        pts, _ = read_ply_any(ply)
        span = float(np.linalg.norm(pts.max(0) - pts.min(0))) if len(pts) else 9.9
        if len(pts) >= 500 and span < 1.0:
            log(f"  live build OK: {len(pts)} pts, {span*100:.0f} cm across -> using the live scan")
            return ply
        log(f"  live build looks off ({len(pts)} pts, {span*100:.0f} cm) -> using the prepared model")
    except SystemExit as e:                             # process_pi sys.exit on a bad scan
        log(f"  live build aborted ({e}) -> using the prepared model")
    except Exception as e:                              # noqa: BLE001
        log(f"  live build failed ({e}) -> using the prepared model")
    return None


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag)
        v = cast(args[i + 1])
        del args[i:i + 2]
        return v
    return None


def main():
    args = sys.argv[1:]
    force_sift = "--sift" in args
    if force_sift:
        args.remove("--sift")
    real = "--real" in args
    if real:
        args.remove("--real")
    no_view = "--no-view" in args
    if no_view:
        args.remove("--no-view")
    no_capture = "--no-capture" in args
    if no_capture:
        args.remove("--no-capture")
    good = (_pop(args, "--good", str) or "a").lower()
    if good not in CAR_MODELS:
        good = "a"
    obj = _pop(args, "--object", str)
    shots = _pop(args, "--shots", int)
    radius = _pop(args, "--radius", float)
    window = _pop(args, "--win", int) or 3
    build_only = _pop(args, "--build", str)
    cfg = config.select(obj) if obj else config.DEFAULT

    # --- 1. decide the session / output folder ---
    if build_only:
        session = build_only
        if not os.path.isdir(session):
            sys.exit(f"--build: not a session folder: {session}")
        print(f"=== run_pi_final: use existing capture {session} ===")
    elif no_capture:
        session = os.path.join(HERE, "captures", "view_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(session, exist_ok=True)
        print(f"=== run_pi_final: no driving -> {session} ===")
    else:
        import capture_orbit                            # captures exactly like run_pi.py
        s = shots if shots is not None else cfg["shots"]
        r = radius if radius is not None else cfg["radius"]
        print(f"=== run_pi_final: capture orbit {cfg['name']} (shots={s}, R={r*100:.0f}cm) ===")
        session = capture_orbit.capture(shots=s, radius=r)

    final = os.path.join(session, "object_final.ply")

    # --- 2. build the car cloud ---
    print("=== build ===")
    real_ply = None
    if real and not no_capture:
        real_ply = try_real_merge(session, cfg, force_sift, window)
    if real_ply:
        pts, cols = read_ply_any(real_ply)
        G.save_ply(final, pts, cols)                    # normalise to viewer format
    else:
        if not write_car_model(good, final):
            sys.exit("could not produce any cloud to show")

    print(f"=== ready -> {final} ===")

    # --- 3. open the Pi viewer (falls back to a still PNG if headless) ---
    if no_view:
        print(f"  (skipped viewer)  open later:  python3 {os.path.join(SRC,'pointcloud','view3d.py')} {final}")
        return
    print("  opening the 3D viewer (drag=orbit, wheel=zoom, ESC/Q=quit) ...")
    try:
        view3d.view(final)
    except Exception as e:                              # noqa: BLE001
        out = view3d.save_view(final, os.path.splitext(final)[0] + "_preview.png")
        print(f"  viewer unavailable ({e}); saved a still -> {out}")


if __name__ == "__main__":
    main()
