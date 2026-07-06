# Autonomous vision demos

Demos for the `xlerobot_vision` pipeline: an Intel RealSense camera detects a
colored object, the detection is transformed into the arm's base frame, and
the differential-IK solver drives the SO-101 arm to it.

```
RealSense capture ─▶ color detection ─▶ frame transform ─▶ IK ─▶ motor bus
```

Install the vision extras first (`pip install -r requirements-vision.txt`) and
validate without hardware: `python tests/vision_offline_suite.py` (expected:
49/49 passed).

## Demos

- **`grab_cube.py`** — the main demo: detect a cube, plan with IK, grasp it.
  Requires a RealSense camera and one SO-101 arm.
- **`scripted/`** — pre-programmed single-arm motions (no camera needed):
  `so101_simple.py`, `so101_reach_forward.py`, `so101_pen_pickup.py`,
  `so101_pen_drop.py`, `so101_grab_cube.py`, `so101_grab_apple.py`,
  `so101_hang.py`.

All demos use the calibration in `calibration/single_bus.json` (see
`xlerobot_vision.config`) and default to the bus on `/dev/ttyACM0`.
