"""
Delete captured data so you can start fresh.

Removes:
  - every folder inside captures/   (depth.npy, ir_*.png, cloud.ply, ...)
  - merged.ply and merged_views.png in the project folder

Usage:
  python3 clean_captures.py            # asks before deleting
  python3 clean_captures.py --yes      # delete without asking
  python3 clean_captures.py --clouds   # keep raw captures, only delete generated
                                       #   clouds (cloud.ply / *_preview.png / merged*)

Only needs the standard library, so it runs on the Pi or the laptop.
"""

import os
import sys
import glob
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    args = sys.argv[1:]
    assume_yes = "--yes" in args or "-y" in args
    clouds_only = "--clouds" in args

    capture_dirs = sorted(d for d in glob.glob(os.path.join(HERE, "captures", "*"))
                          if os.path.isdir(d))
    merged = [os.path.join(HERE, f) for f in ("merged.ply", "merged_views.png")
              if os.path.exists(os.path.join(HERE, f))]

    if clouds_only:
        # keep the raw captures (depth.npy / ir_*.png), delete only generated clouds —
        # at any depth, so it covers both flat captures/<ts>/ and scan_<ts>/shot_*/.
        gen_patterns = ("cloud.ply", "cloud_preview.png", "cloud_views.png", "merged_360.ply")
        targets = merged + [
            p for pat in gen_patterns
            for p in glob.glob(os.path.join(HERE, "captures", "**", pat), recursive=True)
        ]
        label = "generated clouds (raw captures kept)"
    else:
        targets = capture_dirs + merged
        label = "ALL captures + merged output"

    if not targets:
        print("Nothing to clean.")
        return

    print(f"About to delete {label}:")
    for t in targets:
        print("  -", os.path.relpath(t, HERE))

    if not assume_yes:
        if input("\nDelete these? [y/N] ").strip().lower() not in ("y", "yes"):
            print("Cancelled. Nothing deleted.")
            return

    for t in targets:
        if os.path.isdir(t):
            shutil.rmtree(t)
        else:
            os.remove(t)
    print(f"Deleted {len(targets)} item(s).")


if __name__ == "__main__":
    main()
