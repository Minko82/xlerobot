# XLeRobot-Pro

<img width="894" height="455" alt="VR-Teleoperated precision during EV battery disassembly" src="https://github.com/user-attachments/assets/d8b911e5-4e95-4397-965e-9f6c243c8a38" />

**\*VR-Teleoperated Precision:** The platform performing screw extraction during an EV battery disassembly process.\*

<br>

## Overview

The XLeRobot-Pro is an accessible bimanual mobile manipulator. Featuring an optimized 3D-printed frame, safety power envelopes, and NVIDIA Jetson Orin compute for high-end research and education.

It is an evolution of the [XLeRobot](https://xlerobot.readthedocs.io/en/latest/software/getting_started/install.html) ecosystem. We have advanced the platform by integrating a stiffness-optimized structural redesign, a novel Tri-Bus power topology, and onboard GPU-accelerated autonomy, specifically tailored for accessible high-performance research and hands-on robotics education.

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

To get started with the XLeRobot-Pro, follow the instructions below:

- **Hardware Build:** For detailed assembly instructions, parts lists, and 3D-printing guides, follow the [Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html) on the project website.
- **Software Setup:** For installation, serial port setup, motor ID assignment, calibration, and validation, see [docs/setup.md](docs/setup.md).

```bash
git clone https://github.com/Minko82/xlerobot-pro.git
cd xlerobot-pro
pip install -e ".[all]"
make smoke   # verify the install — no hardware needed
```

<br>

---

## Repository Layout

| Path                                    | Contents                                                                         |
| --------------------------------------- | -------------------------------------------------------------------------------- |
| [`docs/setup.md`](docs/setup.md)        | Software setup and bring-up guide                                                |
| [`src/lerobot/`](src/lerobot/)          | Vendored LeRobot fork with XLeRobot-Pro robots, teleoperators, and power tooling |
| [`examples/`](examples/README.md)       | Tested bring-up and teleoperation examples (keyboard, Xbox, VR)                  |
| [`cube-vision/`](cube-vision/README.md) | Autonomous perception + IK cube-manipulation pipeline                            |
| [`calibration/`](calibration/)          | Reference motor calibrations for the two arms                                    |
| [`scripts/`](scripts/README.md)         | Utilities (Tri-Bus power-budget calculator)                                      |
| [`set_motor_id.py`](set_motor_id.py)    | Motor ID assignment tool used during the hardware build                          |
| [`tests/`](tests/)                      | Hardware-free smoke tests (`make smoke`)                                         |

<br>

---

## Citation & Contributions

We hope this platform helps accelerate your research! If you find "Cutting the Cord: XLeRobot-Pro" useful for your work, please cite our [paper](https://arxiv.org/abs/2603.09051).

We are committed to fostering a collaborative ecosystem. If you have improved the structural design, optimized the power topology, or developed new manipulation behaviors, we strongly encourage you to submit a Pull Request. We would love to see how you evolve this foundation!

<br>

---

### Acknowledgements

We would like to extend our sincere gratitude to the creators of the original [XLeRobot](https://xlerobot.readthedocs.io/en/latest/index.html) and [LeRobot](https://huggingface.co/docs/lerobot) projects. Their open-source contributions provided the essential foundation that made this evolution possible.
