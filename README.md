# XLeRobot-Pro

<img width="894" height="455" alt="VR-Teleoperated precision during EV battery disassembly" src="https://github.com/user-attachments/assets/d8b911e5-4e95-4397-965e-9f6c243c8a38" />

**\*VR-Teleoperated Precision:** The platform performing screw extraction during an EV battery disassembly process.\*

<br>

## Overview

The XLeRobot-Pro is an accessible bimanual mobile manipulator. Featuring an optimized 3D-printed frame, safety power envelopes, and NVIDIA Jetson Orin compute for high-end research and education.

It builds on the open-source [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) project ([docs](https://xlerobot.readthedocs.io/)). Everything we have modified or added — the stiffness-optimized structural redesign, the Tri-Bus power topology, and the onboard GPU-accelerated autonomy stack (the `xlerobot_pro` library in this repository) — carries the **XLeRobot-Pro** name, tailored for accessible high-performance research and hands-on robotics education.

<br>

<p align="center">
  <a href="https://minko82.github.io/xlerobot-pro-website/">
    <img src="https://img.shields.io/badge/Project_Website-008080?style=for-the-badge&logo=google-chrome&logoColor=white" alt="Project Website">
  </a>
  <a href="https://arxiv.org/abs/2603.09051">
    <img src="https://img.shields.io/badge/Our_Paper-B31B1B?style=for-the-badge&logo=arxiv&logoColor=white" alt="Our Paper">
  </a>
</p>

<br>

### Key Features

- **Accessible Platform:** Total Bill of Materials (BOM) under $1,300.
- **Tri-Bus Power Topology:** Prevents compute brownouts by isolating high-transient motor loads.
- **Onboard Intelligence:** NVIDIA Jetson Orin Nano for autonomous SLAM and 67 TOPS edge inference.
- **Bimanual Flexibility:** 1kg payload per arm with modular, 3D-printed structural design.
- **Intuitive Control:** Low-latency VR interface with handtracking for human-in-the-loop coordination.

<br>

---

## Setup Guide

This guide takes you from an empty workbench all the way to an autonomous cube grasp. **Work through the steps in order** — each one builds on the last.

**Contents**

1. [Build the hardware](#1-build-the-hardware)
2. [Prerequisites](#2-prerequisites)
3. [Clone and create an environment](#3-clone-and-create-an-environment)
4. [Install PyTorch (Jetson only)](#4-install-pytorch-jetson-only)
5. [Install XLeRobot-Pro](#5-install-xlerobot-pro)
6. [Grant serial port access](#6-grant-serial-port-access)
7. [Verify the install — no hardware needed](#7-verify-the-install--no-hardware-needed)
8. [Assign motor IDs](#8-assign-motor-ids)
9. [Calibrate the arms](#9-calibrate-the-arms)
10. [First motion](#10-first-motion)
11. [Autonomous vision demo](#11-autonomous-vision-demo)
12. [Firmware safety limits](#12-firmware-safety-limits)
13. [Jetson tuning notes](#13-jetson-tuning-notes)
14. [Troubleshooting](#14-troubleshooting)

<br>

### 1. Build the hardware

Before any software, assemble the robot. The **[Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html)** on the project website walks through 3D printing, mechanical assembly, the per-bus motor ID map, and the Tri-Bus power wiring.

You are ready for the software setup once the arms, head, and mobile base are assembled and the motor buses are wired to the onboard computer over USB.

<br>

### 2. Prerequisites

- **Onboard computer:** NVIDIA **Jetson Orin Nano Super** running JetPack 6 (Ubuntu 22.04, Python 3.10). Any Ubuntu 22.04/24.04 or macOS (arm64) machine also works for development without the robot.
- **Python:** 3.10 or newer.
- **Motor buses:** the Feetech buses enumerate as USB serial devices — `/dev/ttyACM*` on Linux, `/dev/tty.usbmodem*` on macOS.

<br>

### 3. Clone and create an environment

```bash
git clone https://github.com/Minko82/xlerobot-pro.git
cd xlerobot-pro

python3 -m venv ~/.venvs/xlerobot-pro
source ~/.venvs/xlerobot-pro/bin/activate
pip install --upgrade pip
```

> Re-activate the environment (`source ~/.venvs/xlerobot-pro/bin/activate`) in every new terminal before running the robot.

<br>

### 4. Install PyTorch (Jetson only)

**Do this before anything else on the Jetson.** The generic PyPI `torch` wheel is CPU-only on aarch64, so install NVIDIA's JetPack 6 build first — otherwise pip will pull the wrong one:

```bash
pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

On a laptop/desktop for development, skip this step — the standard `torch` wheel is installed automatically in the next step.

<br>

### 5. Install XLeRobot-Pro

Install the robot stack (the vendored LeRobot fork plus all robot, camera, and teleop extras) and then the vision + IK stack:

```bash
pip install -e ".[all]"                                # robot stack (or: make install)
pip install -r requirements/requirements-vision.txt    # vision + IK: pinocchio, pink, MuJoCo, RealSense
```

**Reproducible installs.** To match a known-good machine exactly, use the pinned lockfiles instead of `.[all]`:

```bash
pip install -r requirements/requirements-ubuntu.txt    # Ubuntu 24.04 (x86_64)
pip install -r requirements/requirements-macos.txt     # macOS (arm64)
```

**VR teleoperation** additionally needs the XLeVR/TeleVuer stack — see the [XLeRobot VR docs](https://xlerobot.readthedocs.io/en/latest/software/getting_started/teleop_vr.html).

<br>

### 6. Grant serial port access

The motor buses are USB serial devices, so your user needs permission to open them. On Linux:

```bash
sudo usermod -aG dialout $USER   # permanent — then log out and back in
# — or, just for this boot —
sudo chmod 666 /dev/ttyACM0
```

On macOS no extra permission is needed; the buses appear as `/dev/tty.usbmodem*`.

<br>

### 7. Verify the install — no hardware needed

Confirm the software is healthy before touching the robot:

```bash
make smoke                             # compiles the whole codebase + runs the test suite
python tests/vision_offline_suite.py   # full perception → transform → IK → FK pipeline (expect 49/49)
```

If both pass, the vision and IK stack is working end to end without any motors or camera.

<br>

### 8. Assign motor IDs

Every motor on a bus needs a unique ID (the per-bus map is in the [Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html)). Connect **only one motor at a time**, then write its ID:

```bash
python set_motor_id.py --port /dev/ttyACM0 --id 7 --model sts3215
```

Repeat for each motor. This is normally done once during the hardware build.

<br>

### 9. Calibrate the arms

Calibrate each arm once; the calibration is stored and reused afterwards:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=right_arm
```

- LeRobot stores calibrations under `~/.cache/huggingface/lerobot/calibration/`.
- Reference calibrations for this robot are versioned in [`calibration/`](calibration/) (`left_arm.json`, `right_arm.json`, `head.json`, `single_bus.json`) — useful as a sanity check or fallback.

<br>

### 10. First motion

With the motors wired, powered, and calibrated, find the buses and run the bring-up examples **in order** (details in [`examples/README.md`](examples/README.md)):

```bash
lerobot-find-port                                                  # identify the motor buses

python examples/0_so100_keyboard_joint_control.py                 # single-arm joint control
python examples/xlerobot_examples/1_so100_keyboard_ee_control.py  # end-effector (IK) control
python examples/9_dual_wrist_camera.py                            # wrist camera check
```

Start with joint control to confirm each motor moves in the expected direction before moving on to end-effector control.

<br>

### 11. Autonomous vision demo

The `xlerobot_pro` library implements the full RealSense perception → frame-transform → IK pipeline; [`examples/vision/`](examples/vision/README.md) holds the runnable demos. You already validated it offline in step 7 — now run it live:

```bash
python examples/vision/grab_cube.py    # detect a colored cube and grasp it
```

Requires a RealSense camera and one SO-101 arm. The scripted motions in `examples/vision/scripted/` run without a camera if you just want to exercise the arm.

<br>

### 12. Firmware safety limits

System-wide maximum torque, acceleration, and speed live in one place: [`src/xlerobot_pro/firmware_limits.py`](src/xlerobot_pro/firmware_limits.py). The defaults implement the paper's firmware saturation settings so each power bus stays inside its fuse rating:

- **Wheels / neck:** torque 650 (~1.91 N·m), acceleration 20
- **Arms:** torque 450 (~1.32 N·m), acceleration 40

They are applied automatically when a robot connects and by the vision demos. To raise or lower the platform maximums, edit the commented constants in that file, then confirm what the motors actually stored:

```bash
python diagnostics/verify_motor_limits.py
```

<br>

### 13. Jetson tuning notes

- **Confirm GPU PyTorch is active:** `python -c "import torch; print(torch.cuda.is_available())"` should print `True`. If not, reinstall from the JetPack index (step 4).
- **`open3d` OpenMP error:** preload libgomp —

  ```bash
  echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1' >> ~/.bashrc
  ```

- **Maximum inference performance:** `sudo nvpmodel -m 0 && sudo jetson_clocks`. On battery (the Tri-Bus compute rail), prefer the default power mode to stay inside the power envelope.

<br>

### 14. Troubleshooting

| Symptom                           | Fix                                                                  |
| --------------------------------- | ------------------------------------------------------------------- |
| `Permission denied: /dev/ttyACM0` | Grant serial access — [step 6](#6-grant-serial-port-access)         |
| Motor not found on bus            | Re-check IDs ([step 8](#8-assign-motor-ids)); ensure only one bus terminator is powered |
| `ModuleNotFoundError: lerobot`    | Activate the environment and re-run `pip install -e ".[all]"`        |
| RealSense frames time out         | Replug the USB 3.0 cable; check `rs-enumerate-devices`              |

For bus, motor, and frame-transform debugging tools, see [`diagnostics/README.md`](diagnostics/README.md).

<br>

---

## Repository Layout

| Path                                     | Contents                                                                         |
| ---------------------------------------- | -------------------------------------------------------------------------------- |
| [`src/lerobot/`](src/lerobot/)           | Vendored LeRobot fork with XLeRobot-Pro robots, teleoperators, and power tooling |
| [`examples/`](examples/README.md)        | Bring-up, teleoperation, policy, and autonomous-vision examples                  |
| [`src/xlerobot_pro/`](src/xlerobot_pro/) | Vision + IK library, robot model, and system-wide firmware safety limits         |
| [`diagnostics/`](diagnostics/README.md)  | Bus, motor, and frame-transform debugging tools                                  |
| [`calibration/`](calibration/)           | Reference motor calibrations for the two arms                                    |
| [`requirements/`](requirements/)         | Base spec, platform lockfiles, and the vision dependency list                    |
| [`set_motor_id.py`](set_motor_id.py)     | Motor ID assignment tool used during the hardware build                          |
| [`tests/`](tests/)                       | Hardware-free smoke tests (`make smoke`)                                         |

<br>

---

## Citation & Contributions

We hope this platform helps accelerate your research! If you find "Cutting the Cord: XLeRobot-Pro" useful for your work, please cite our [paper](https://arxiv.org/abs/2603.09051).

We are committed to fostering a collaborative ecosystem. If you have improved the structural design, optimized the power topology, or developed new manipulation behaviors, we strongly encourage you to submit a Pull Request. We would love to see how you evolve this foundation!

<br>

---

### Acknowledgements

We would like to extend our sincere gratitude to the creators of the original [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) ([docs](https://xlerobot.readthedocs.io/)) and [LeRobot](https://huggingface.co/docs/lerobot) projects. Their open-source contributions provided the essential foundation that made this evolution possible.
