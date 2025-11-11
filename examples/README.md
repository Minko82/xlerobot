# XLeRobot Examples

This folder contains a small set of **tested example scripts** to help you bring up the hardware, verify communication, and practice basic control of the SO100/SO101 arms and wrist cameras.

All examples assume:

- You have installed XLeRobot and its dependencies.
- Your `lerobot` Conda environment is activated.
- The robot arms and/or cameras are powered and plugged in as described in the main `README.md`.

<br>

---
## How to run

From the repo root:

```bash
cd examples
python3 0_so100_keyboard_joint_control.py
```

<br>

---

## Examples

- **`0_so100_keyboard_joint_control.py`** → basic single-arm joint control.
- **`2_dual_so100_keyboard_ee_control.py`** → dual-arm, joint + end-effector control.
- **`9_dual_wrist_camera.py`** → dual wrist camera verification.


- Additional, unverified examples may be available in `examples/provided_examples`; they are for reference only and may require updates to work seamlessly with XLeRobot.


