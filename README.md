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

## Getting Started

**Hardware build first:** follow the [Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html) on the project website for assembly, 3D printing, motor IDs, and the Tri-Bus wiring.

### Software setup (Jetson Orin Nano Super)

The robot's onboard computer is an NVIDIA **Jetson Orin Nano Super** running JetPack 6 (Ubuntu 22.04, Python 3.10). The same steps work on any Ubuntu/macOS machine for development.

**1. Clone and create an environment**

```bash
git clone https://github.com/Minko82/xlerobot-pro.git
cd xlerobot-pro
python3 -m venv ~/.venvs/xlerobot-pro
source ~/.venvs/xlerobot-pro/bin/activate
pip install --upgrade pip
```

**2. Install PyTorch (Jetson only, before anything else)**

The generic PyPI `torch` wheel does not use the Jetson GPU. Install NVIDIA's JetPack 6 build first so pip doesn't replace it:

```bash
pip install torch torchvision --index-url https://pypi.jetson-ai-lab.dev/jp6/cu126
```

**3. Install XLeRobot-Pro**

```bash
pip install -e ".[all]"                  # robot stack (vendored LeRobot fork + extras)
pip install -r requirements-vision.txt   # vision + IK stack (pinocchio, pink, MuJoCo, RealSense)
```

**4. Grant serial port access (one time)**

```bash
sudo usermod -aG dialout $USER   # then log out and back in
```

**5. Verify — no hardware needed**

```bash
make smoke                             # compiles everything + runs the test suite
python tests/vision_offline_suite.py   # full vision→IK pipeline check (expect 49/49)
```

**6. Bring up the robot**

With the motors wired and powered per the Build Guide:

```bash
lerobot-find-port                                  # find the motor buses
python examples/0_so100_keyboard_joint_control.py  # first motion test
python examples/vision/grab_cube.py                # autonomous cube grasp demo
```

For motor ID assignment, calibration, Jetson quirks (open3d/libgomp), and troubleshooting, see the full guide: **[docs/setup.md](docs/setup.md)**.

<br>

---

## Repository Layout

| Path                                     | Contents                                                                         |
| ---------------------------------------- | -------------------------------------------------------------------------------- |
| [`docs/setup.md`](docs/setup.md)         | Software setup and bring-up guide                                                |
| [`src/lerobot/`](src/lerobot/)           | Vendored LeRobot fork with XLeRobot-Pro robots, teleoperators, and power tooling |
| [`examples/`](examples/README.md)        | Bring-up, teleoperation, policy, and autonomous-vision examples                  |
| [`src/xlerobot_pro/`](src/xlerobot_pro/) | Vision + IK library: perception, frame transforms, differential IK, robot model  |
| [`diagnostics/`](diagnostics/README.md)  | Bus, motor, and frame-transform debugging tools                                  |
| [`calibration/`](calibration/)           | Reference motor calibrations for the two arms                                    |
| [`scripts/`](scripts/README.md)          | Utilities (Tri-Bus power-budget calculator)                                      |
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
