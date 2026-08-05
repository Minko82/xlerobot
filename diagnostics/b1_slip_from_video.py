#!/usr/bin/env python3
"""B1 — measure object slip from a phone video using two ArUco markers.

Marker ID 0 goes on the gripper jaw, ID 1 on the object. Slip is the CHANGE in
separation between them, not either marker's absolute motion -- so camera shake,
panning and the robot's own travel all cancel, provided both markers stay in
frame and roughly in the same plane.

Scale comes from the markers themselves: the printed side length is known, so
pixels convert to millimetres per frame. That also absorbs changes in
camera-to-object distance, which a fixed mm-per-pixel constant would not.

Needs OpenCV with the `aruco` module, which is present in the Jetson environment
but often not in a desktop install -- so this usually runs on the robot. Copy the
clip over first::

    scp clip.mov <user>@<robot-host>:/tmp/
    python diagnostics/b1_slip_from_video.py /tmp/clip.mov --marker-mm 30 \\
        --out results/B1/base_0.5ms2_600g

Pass the marker's MEASURED printed size, not its nominal one -- printers scale,
and that single number sets the scale of every result.

Reported honestly: if markers are missing for part of the clip, the gap is stated
rather than interpolated over.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

#: Frames at the start used to establish the unslipped baseline separation.
BASELINE_FRAMES = 30

#: Displacement beyond this counts as slip. The protocol leaves the threshold
#: open ("[X] mm"); 2 mm is roughly the point at which slip is visible by eye.
DEFAULT_SLIP_MM = 2.0


def corners_to_centre(c):
    return c.reshape(4, 2).mean(axis=0)


def marker_side_px(c):
    """Mean side length in pixels — the per-frame scale reference."""
    p = c.reshape(4, 2)
    return float(np.mean([np.linalg.norm(p[i] - p[(i + 1) % 4]) for i in range(4)]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("video", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="Output directory.")
    ap.add_argument("--marker-mm", type=float, required=True,
                    help="Printed marker side length in mm. MEASURE IT after printing — "
                         "this single number sets the scale of every result.")
    ap.add_argument("--gripper-id", type=int, default=0)
    ap.add_argument("--object-id", type=int, default=1)
    ap.add_argument("--slip-mm", type=float, default=DEFAULT_SLIP_MM)
    ap.add_argument("--dict", default="DICT_4X4_50")
    args = ap.parse_args()

    if not args.video.exists():
        print(f"FATAL: {args.video} not found")
        return 1
    args.out.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        print(f"FATAL: could not open {args.video}")
        return 1
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"\n  {args.video.name}: {w}x{h}, {fps:.1f} fps, {total} frames "
          f"({total / fps:.2f}s)" if fps else "")
    if fps and fps < 100:
        print(f"  [warn] {fps:.0f} fps is below the protocol's 120 fps. Fast slip may be "
              "missed; use the phone's slow-motion mode.")

    dic = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector = aruco.ArucoDetector(dic, params)

    rows, missing = [], 0
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        found = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                found[int(i)] = c
        if args.gripper_id in found and args.object_id in found:
            g, o = found[args.gripper_id], found[args.object_id]
            sep_px = float(np.linalg.norm(corners_to_centre(o) - corners_to_centre(g)))
            # Average both markers' apparent size for the scale.
            px_per_mm = ((marker_side_px(g) + marker_side_px(o)) / 2.0) / args.marker_mm
            rows.append(dict(frame=idx, t=idx / fps if fps else idx,
                             sep_px=sep_px, sep_mm=sep_px / px_per_mm,
                             px_per_mm=px_per_mm))
        else:
            missing += 1
        idx += 1
    cap.release()

    if len(rows) < BASELINE_FRAMES + 5:
        print(f"\n  FAILED: only {len(rows)} frames had both markers visible "
              f"({missing} missing). Cannot measure. Re-shoot with both markers in frame.")
        return 2

    baseline = statistics.median(r["sep_mm"] for r in rows[:BASELINE_FRAMES])
    for r in rows:
        r["disp_mm"] = r["sep_mm"] - baseline

    peak = max(rows, key=lambda r: abs(r["disp_mm"]))
    final = rows[-1]["disp_mm"]
    slipped = abs(peak["disp_mm"]) >= args.slip_mm

    with open(args.out / "slip_trace.csv", "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["frame", "time_s", "separation_px", "separation_mm",
                     "displacement_mm", "px_per_mm"])
        for r in rows:
            wr.writerow([r["frame"], f"{r['t']:.4f}", f"{r['sep_px']:.2f}",
                         f"{r['sep_mm']:.3f}", f"{r['disp_mm']:+.3f}", f"{r['px_per_mm']:.3f}"])

    pct = 100.0 * missing / max(idx, 1)
    print("\n" + "=" * 58)
    print(f"  frames analysed   {len(rows)} of {idx}  ({pct:.1f}% missing a marker)")
    print(f"  baseline sep      {baseline:.2f} mm  (median of first {BASELINE_FRAMES} frames)")
    print(f"  peak displacement {peak['disp_mm']:+.2f} mm at t={peak['t']:.3f}s")
    print(f"  final displacement{final:+.2f} mm")
    print(f"  threshold         {args.slip_mm:.1f} mm")
    print(f"  VERDICT           {'SLIP' if slipped else 'NO SLIP'}")
    if pct > 10:
        print(f"  [warn] {pct:.0f}% of frames lost a marker — treat the peak as a lower bound.")
    print(f"  data              {args.out / 'slip_trace.csv'}")
    print("=" * 58)
    return 0


if __name__ == "__main__":
    sys.exit(main())
