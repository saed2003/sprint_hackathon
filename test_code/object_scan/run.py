#!/usr/bin/env python3
"""
ONE command to run the whole object scan: capture -> merge -> mesh -> preview.

Pick HOW the views are captured; everything after is identical. Two demo objects are
defined in config.py (default = the LEGO DB5 car); choose with --object.

    python run.py                 # SELF-TEST: synthetic object, NO hardware (start here)
    python run.py synth           # same, explicit
    python run.py turntable       # laptop + D405: you spin the OBJECT (most reliable)
    python run.py orbit           # Pi: the ROBOT drives around the object
    python run.py build <session> # just rebuild from already-captured shots
    python run.py clean           # delete capture sessions (--yes, --outputs, or a name)

Pick the object (default db5):
    python run.py --object db5    # LEGO 007 Aston Martin DB5 #76911  (recommended)
    python run.py --object teemo  # Funko Pop Teemo with Mushroom #1138

Common flags (after the mode):
    --shots N      override the number of views
    --radius M     orbit radius in metres (orbit mode; default from config)
    --dir -1       flip turn direction if the model comes out mirrored/smeared
    --no-loop      partial scan (not a full 360)
    --no-mesh      skip Poisson meshing (point cloud only)

Output (all in the session folder):
    merged_object.ply        the fused, cleaned point cloud
    merged_object_mesh.ply   the watertight textured mesh  (the presentation 'wow')
    *_preview.png            headless 4-view images of both
"""
import os
import sys
import glob
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
CAPTURES = os.path.join(HERE, "captures")     # where all scans (synth_/obj_/orbit_) live

import config
# NOTE: build_object / mesh / preview need Open3D (laptop only). They are imported
# lazily in build_and_mesh() so the CAPTURE-only modes still run on the Pi (no Open3D).


def _pop(args, flag, cast):
    if flag in args:
        i = args.index(flag); v = cast(args[i + 1]); del args[i:i + 2]; return v
    return None


def clean(args):
    """Delete scratch capture data. stdlib only, so it works on the Pi too.

        run.py clean              # ALL sessions in captures/        (asks first)
        run.py clean --yes        # ...without asking                (alias -y)
        run.py clean --outputs    # keep raw shots, delete only built .ply / preview .png
        run.py clean orbit_<ts>   # just one (or more) named session(s)
    """
    assume_yes = "--yes" in args or "-y" in args
    outputs_only = "--outputs" in args
    names = [a for a in args[1:] if not a.startswith("-")]    # args[0] == "clean"

    sessions = ([os.path.join(CAPTURES, n) for n in names] if names
                else sorted(glob.glob(os.path.join(CAPTURES, "*"))))
    sessions = [s for s in sessions if os.path.isdir(s)]

    if outputs_only:
        gen = ("merged_object.ply", "merged_object_mesh.ply",
               "merged_object_preview.png", "merged_object_mesh_preview.png", "cloud.ply")
        targets = [p for s in sessions for pat in gen
                   for p in glob.glob(os.path.join(s, "**", pat), recursive=True)]
        label = "built clouds/meshes/previews (raw shots kept)"
    else:
        targets = sessions
        label = "ALL capture sessions (raw shots + built output)"

    if not targets:
        print("Nothing to clean.")
        return
    print(f"About to delete {label}:")
    for t in targets:
        print("  -", os.path.relpath(t, HERE))
    if not assume_yes and input("\nDelete these? [y/N] ").strip().lower() not in ("y", "yes"):
        print("Cancelled. Nothing deleted.")
        return
    for t in targets:
        shutil.rmtree(t) if os.path.isdir(t) else os.remove(t)
    print(f"Deleted {len(targets)} item(s).")


def capture_views(mode, shots, radius):
    """Return a session folder of shot_NN captures for the chosen mode."""
    if mode == "synth":
        import _synth_test
        return _synth_test.make_session(shots=shots)
    if mode == "turntable":
        import capture_session
        return capture_session.run_session(shots=shots)
    if mode == "orbit":
        import capture_orbit
        return capture_orbit.capture(shots=shots or capture_orbit.ORBIT_SHOTS,
                                     radius=radius or capture_orbit.ORBIT_RADIUS)
    sys.exit(f"unknown mode '{mode}' (use synth | turntable | orbit | build <dir>)")


def build_and_mesh(session, CFG, loop, direction, do_mesh):
    """The Open3D part (laptop): merge -> mesh -> preview. Imported lazily so the
    Pi (no Open3D) can still capture and just hand off the session."""
    import build_object
    import mesh as mesh_mod
    import preview as preview_mod

    print(f"\n=== merge (session: {session}) ===")
    ply = build_object.build(session, voxel=CFG["voxel"], zmin=CFG["zmin"], zmax=CFG["zmax"],
                             crop=CFG["crop"], loop=loop, direction=direction)
    outputs = [ply]
    if do_mesh:
        print("\n=== mesh ===")
        outputs.append(mesh_mod.poisson_mesh(ply, depth=CFG["poisson_depth"]))

    print("\n=== preview ===")
    previews = []
    for f in outputs:
        try:
            previews.append(preview_mod.preview(f))
        except Exception as e:
            print(f"  (preview skipped for {os.path.basename(f)}: {e})")

    print("\n=== done ===")
    for f in outputs + previews:
        print("  " + f)
    if do_mesh:
        print("\nview the mesh interactively (laptop):")
        print(f"  {sys.executable} -c \"import open3d as o3d; "
              f"o3d.visualization.draw_geometries([o3d.io.read_triangle_mesh('{outputs[-1]}')])\"")


def main():
    args = sys.argv[1:]
    if args and args[0] == "clean":          # independent of object/Open3D
        clean(args)
        return
    obj = _pop(args, "--object", str)
    CFG = config.select(obj) if obj else config.DEFAULT       # set active object BEFORE lazy imports
    do_mesh = "--no-mesh" not in args and CFG.get("mesh", True)
    if "--no-mesh" in args:
        args.remove("--no-mesh")
    loop = "--no-loop" not in args
    if not loop:
        args.remove("--no-loop")
    shots = _pop(args, "--shots", int)
    radius = _pop(args, "--radius", float)
    direction = _pop(args, "--dir", int) or 1

    mode = args[0] if args else "synth"

    # 1. get a session of captured views
    if mode == "build":
        if len(args) < 2:
            sys.exit("usage: python run.py build <session_dir> [--object db5|teemo]")
        session = args[1]
    else:
        print(f"=== object scan: {CFG['name']} | mode={mode} ===")
        session = capture_views(mode, shots, radius)

    # 2. merge + mesh + preview (needs Open3D). On the Pi (orbit capture) Open3D isn't
    #    installed, so we degrade gracefully: keep the capture, build it on the laptop.
    try:
        build_and_mesh(session, CFG, loop, direction, do_mesh)
    except ImportError as e:
        obj = CFG.get("key", "db5")
        print(f"\n=== captured: {session} ===")
        print(f"Open3D isn't available here ({e}) — this is normal on the Pi.")
        print("Copy the session to the laptop and finish the build there:")
        print(f"  scp -r {session} <you>@laptop:~/  # then, in object_scan/:")
        print(f"  ../../../.venv/bin/python run.py build {os.path.basename(session)} --object {obj}")


if __name__ == "__main__":
    main()
