# XLeRobot-Pro Software Setup

This guide covers installing the software stack and bringing up the robot after the
hardware is assembled. For the hardware build (3D printing, assembly, motor IDs,
wiring), follow the
[Build Guide on the project website](https://minko82.github.io/xlerobot-pro-website/build-guide.html).

## 1. Prerequisites

- **OS:** Ubuntu 22.04/24.04 (x86_64 or NVIDIA Jetson) or macOS. The onboard
  compute target is an NVIDIA Jetson Orin Nano.
- **Python:** 3.10 or newer.
- **Hardware access:** the Feetech motor buses enumerate as USB serial devices
  (`/dev/ttyACM*` on Linux, `/dev/tty.usbmodem*` on macOS).

## 2. Installation

Clone the repository and install in a fresh environment (conda or venv):

```bash
git clone https://github.com/Minko82/xlerobot-pro.git
cd xlerobot-pro

# with conda
conda create -y -n lerobot python=3.10
conda activate lerobot

pip install -e ".[all]"        # or: make install
```

`.[all]` installs the vendored LeRobot fork plus all robot, camera, and teleop
extras. For an install that exactly matches a tested machine, use the pinned
lockfiles instead:

```bash
pip install -r requirements-ubuntu.txt   # Ubuntu 24.04 (x86_64)
pip install -r requirements-macos.txt    # macOS (arm64)
```

Extra dependencies for specific components:

- **Vision pipeline (`cube-vision/`):** `pyrealsense2` (installed via `.[all]`
  through the `intelrealsense` extra), and `open3d` for the point-cloud
  visualizers: `pip install open3d`.
- **VR teleoperation:** requires the XLeVR/TeleVuer stack; see the
  [XLeRobot VR documentation](https://xlerobot.readthedocs.io/en/latest/software/getting_started/teleop_vr.html).

Verify the install without any hardware:

```bash
make smoke     # compiles all code and runs the hardware-free test suite
```

## 3. Serial port access (Linux)

```bash
# Find the ports the motor buses are on
lerobot-find-port

# Grant access (per boot)
sudo chmod 666 /dev/ttyACM0

# Or permanently, add yourself to the dialout group and re-login
sudo usermod -aG dialout $USER
```

## 4. Motor IDs

Each motor on a bus needs a unique ID (see the
[Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html)
for the per-bus ID map). With **only one motor connected at a time**:

```bash
python set_motor_id.py --port /dev/ttyACM0 --id 7 --model sts3215
```

## 5. Calibration

Calibrate each arm once, then reuse the stored calibration:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=right_arm
```

- LeRobot stores calibrations under `~/.cache/huggingface/lerobot/calibration/`.
- Reference calibrations for this robot are versioned in `calibration/`
  (`left_arm.json`, `right_arm.json`) and `cube-vision/calibration/`.

## 6. Validate the build

Run the bring-up examples in order (details in `examples/README.md`):

```bash
cd examples
python 0_so100_keyboard_joint_control.py            # single-arm joint control
python provided_examples/1_so100_keyboard_ee_control.py  # end-effector IK control
python 9_dual_wrist_camera.py                       # wrist camera check
```

## 7. Autonomous cube manipulation (vision + IK)

The `cube-vision/` directory contains the RealSense-based perception →
frame-transform → IK pipeline. Validate it offline first:

```bash
cd cube-vision
python test_offline.py            # full pipeline, no hardware needed
python control_cube.py            # live: detect a cube and grasp it
```

See `cube-vision/README.md` for the module layout and each script's purpose.

## 8. Jetson notes

- If `open3d` fails to import with an OpenMP error, preload libgomp:

  ```bash
  echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1' >> ~/.bashrc
  ```

- Install the Jetson-specific PyTorch wheel from NVIDIA before
  `pip install -e ".[all]"` so pip does not replace it with the CPU build.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `Permission denied: /dev/ttyACM0` | Section 3 above |
| Motor not found on bus | Re-check IDs (section 4) and that only one bus terminator is powered |
| `ModuleNotFoundError: lerobot` | Activate the environment and re-run `pip install -e ".[all]"` |
| RealSense frames time out | Replug USB 3.0 cable; check `rs-enumerate-devices` |
