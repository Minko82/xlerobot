# XLeRobot-Pro Examples

This folder contains a small set of **tested example scripts** to help you bring up the hardware, verify communication, and practice basic control of the SO100/SO101 arms and wrist cameras.

All examples assume:

- You have installed the software stack ([docs/setup.md](../docs/setup.md)).
- Your `lerobot` environment is activated.
- The robot arms and/or cameras are powered and wired as described in the [Build Guide](https://minko82.github.io/xlerobot-pro-website/build-guide.html).

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

## Tested examples

- **`0_so100_keyboard_joint_control.py`** → basic single-arm joint control.
- **`2_dual_so100_keyboard_ee_control.py`** → dual-arm, joint + end-effector control.
- **`9_dual_wrist_camera.py`** → dual wrist camera verification.

## Other examples

- **`5_xlerobot_teleop_xbox.py`** / **`5_xlerobot_teleop_xbox_new_wiring.py`** → full-robot Xbox-controller teleoperation (the `new_wiring` variant matches the Tri-Bus wiring from the Build Guide).
- **`teleop_hand_and_arm.py`** → VR hand-tracking teleoperation (requires the XLeVR/TeleVuer stack).
- **`smolvla_policy_control.py`**, **`diffusion_policy_control.py`**, **`serve_policy.py`** → running trained policies on the robot.
- **`so100_precision_test.py`** → repeatability measurement (results in `precision_test_results.csv`).

- Additional, unverified examples live in `examples/provided_examples/`; they are upstream references and may require updates to work with XLeRobot-Pro.
