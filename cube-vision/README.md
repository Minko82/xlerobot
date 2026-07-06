# cube-vision — perception + IK manipulation pipeline

Autonomous cube detection and grasping for the XLeRobot-Pro: an Intel RealSense
depth camera detects a colored cube, the detection is transformed from the
camera frame into the arm's base frame, and a differential-IK solver drives the
SO-101 arm to grasp it.

```
RealSense capture ─▶ color_detect ─▶ frame_transform ─▶ ik_solver ─▶ motor bus
   (RGB + depth)      (HSV → 3-D       (camera frame       (pink/       (Feetech
                       centroid)        → Base frame)       pinocchio)    bus)
```

## Quick start

```bash
# From the repo root, after the base install (docs/setup.md):
pip install -r cube-vision/requirements.txt

# Validate everything without hardware:
cd cube-vision
python test_offline.py    # expected: 49/49 passed

# Live grasp demo (RealSense + one SO-101 arm on /dev/ttyACM0):
python control_cube.py
```

## Layout

| Path | Purpose |
| --- | --- |
| `assets/` | MJCF robot model + meshes used by the IK solver |
| `frame_transform/` | Camera-to-base-frame transforms (FK via pinocchio); `xlerobot/` holds the URDF/MJCF models |
| `ik_solver/` | Differential IK for the SO-101 arm (pink); includes a slimmed SO-ARM100 simulation model |
| `pincer_transform/` | Head-camera offset calibration utilities |
| `calibration/` | Stored motor calibrations for this robot |
| `calibrate.py` | Motor calibration (standalone or as a module) |
| `color_detect.py` | HSV color detection → 3-D centroid in camera frame |
| `realsense_capture.py` | RealSense RGB + aligned-depth capture |
| `point_cloud.py` / `visualize_point_cloud.py` | Point-cloud utilities (needs `open3d`) |
| `control_cube.py` | **Main demo:** detect cube → IK → grasp |
| `control.py` / `control_single_bus.py` | Earlier control loop variants |
| `visualize_ik.py` / `visualize_mujoco.py` / `visualize_color_detect.py` | Debug visualizers |
| `test_offline.py` / `test_vision_transform.py` | Hardware-free pipeline tests |
| `so101-*.py` | Scripted single-arm demos (pen pickup, cube grab, etc.) |
| `check_acm*.py`, `read_motors.py`, `debug_*.py`, `diagnose*.py` | Bus/motor diagnostics |
| `lock_motors.py` / `unlock-motors.py` | Torque enable/disable helpers |

## Notes

- Scripts assume the arm bus on `/dev/ttyACM0` (see constants at the top of
  each script to change ports).
- On Jetson, `open3d` may need `LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1`.
- Generated artifacts are written to `outputs/` (gitignored).
