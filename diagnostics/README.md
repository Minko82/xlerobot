# Diagnostics

Standalone tools for debugging the robot hardware and the vision/IK pipeline.
All of them import from the `xlerobot_pro` and `lerobot` libraries, so run
them from anywhere after the install in the [main README](../README.md#getting-started).
Scripts default to the port in `xlerobot_pro.config.BUS_PORT`
(`/dev/ttyACM0`, override with `XLEROBOT_BUS_PORT` or `--port` where offered).

## Bus and motors

| Script | Purpose |
| --- | --- |
| `check_bus.py` | Verify which motors respond on a bus (`--port`, `--motors arm\|head`) |
| `read_motors.py` | Print current calibrated motor positions |
| `read_head.py` | Print calibrated head motor positions (for URDF zero offsets) |
| `read_head_robot.py` | Same, via the `XLerobot` robot class |
| `lock_motors.py` / `unlock_motors.py` | Enable / release torque |
| `verify_motor_limits.py` | Check configured joint limits against the hardware |
| `debug_motors.py` | Measure neutral offsets and joint directions |

## Frame transforms and vision

| Script | Purpose |
| --- | --- |
| `check_vision_live.py` | Live end-to-end check: camera detection vs. transformed base coordinates |
| `diagnose_fk_vs_vision.py` | Place the gripper on the target, compare FK vs. vision position |
| `debug_transform_chain.py` | Dump every intermediate matrix in the camera-to-base transform |
| `calibrate_head_offsets.py` | Calibrate head-camera offsets (writes `calibration/t_base_camera.json`) |
| `measure_ee.py` | Measure the end-effector position with the arm posed by hand |
| `debug_pincer_transform.py` | Debug the reduced-arm-model transform helpers |

## Visualizers

| Script | Purpose |
| --- | --- |
| `visualize_mujoco.py` | Simulate the full transform + IK pipeline in MuJoCo |
| `visualize_color_detect.py` | Overlay color-detection results on the last capture |
| `visualize_point_cloud.py` | Display the last captured point cloud (needs `open3d`) |

Capture artifacts are read from and written to `./outputs/` in the current
working directory (gitignored).
