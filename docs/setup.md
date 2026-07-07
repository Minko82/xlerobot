# XLeRobot-Pro Software Setup

This guide covers installing the software stack and bringing up the robot after the
hardware is assembled. For the hardware build (3D printing, assembly, motor IDs,
wiring), follow the
[Build Guide on the project website](https://minko82.github.io/xlerobot-pro-website/build-guide.html).

## 1. Prerequisites

- **Target platform:** NVIDIA **Jetson Orin Nano Super** with JetPack 6
  (Ubuntu 22.04, Python 3.10) — the robot's onboard computer. Any Ubuntu
  22.04/24.04 or macOS machine also works for development.
- **Python:** 3.10 or newer.
- **Hardware access:** the Feetech motor buses enumerate as USB serial devices
  (`/dev/ttyACM*` on Linux, `/dev/tty.usbmodem*` on macOS).

## 2. Installation

Clone the repository and install in a fresh environment (venv or conda):

```bash
git clone https://github.com/Minko82/xlerobot-pro.git
cd xlerobot-pro

python3 -m venv ~/.venvs/xlerobot-pro
source ~/.venvs/xlerobot-pro/bin/activate
pip install --upgrade pip
```

**On the Jetson, install NVIDIA's PyTorch build first** — the generic PyPI
wheel is CPU-only on aarch64:

```bash
pip install torch torchvision --index-url https://pypi.jetson-ai-lab.dev/jp6/cu126
```

Then install the project:

```bash
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

- **Vision + IK stack (`src/xlerobot_pro/`):** `pip install -r requirements-vision.txt`
  (pinocchio, pink + quadprog for IK, MuJoCo, RealSense, and the optional
  visualizers).
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
  (`left_arm.json`, `right_arm.json`, `head.json`, `single_bus.json`).

## 6. Validate the build

Run the bring-up examples in order (details in `examples/README.md`):

```bash
cd examples
python 0_so100_keyboard_joint_control.py            # single-arm joint control
python provided_examples/1_so100_keyboard_ee_control.py  # end-effector IK control
python 9_dual_wrist_camera.py                       # wrist camera check
```

## 7. Autonomous cube manipulation (vision + IK)

The `xlerobot_pro` library implements the RealSense-based perception →
frame-transform → IK pipeline; `examples/vision/` contains the runnable demos.
Validate the pipeline offline first:

```bash
pip install -r requirements-vision.txt

python tests/vision_offline_suite.py     # full pipeline, no hardware (49/49 tests)
python examples/vision/grab_cube.py      # live: detect a cube and grasp it
```

See `examples/vision/README.md` for the demos and `diagnostics/README.md` for
bus/motor/transform debugging tools.

## 8. Jetson Orin Nano Super notes

- Verify the GPU-enabled PyTorch is active:
  `python -c "import torch; print(torch.cuda.is_available())"` should print
  `True`. If not, reinstall from the JetPack index (section 2).
- If `open3d` fails to import with an OpenMP error, preload libgomp:

  ```bash
  echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1' >> ~/.bashrc
  ```

- For maximum inference performance, set the board to MAXN power mode:
  `sudo nvpmodel -m 0 && sudo jetson_clocks`. On battery (Tri-Bus compute
  rail), prefer the default mode to stay inside the power envelope.

## Troubleshooting

| Symptom                           | Fix                                                                  |
| --------------------------------- | -------------------------------------------------------------------- |
| `Permission denied: /dev/ttyACM0` | Section 3 above                                                      |
| Motor not found on bus            | Re-check IDs (section 4) and that only one bus terminator is powered |
| `ModuleNotFoundError: lerobot`    | Activate the environment and re-run `pip install -e ".[all]"`        |
| RealSense frames time out         | Replug USB 3.0 cable; check `rs-enumerate-devices`                   |
