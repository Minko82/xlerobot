#!/usr/bin/env python3
"""
Motor calibration module for SO-101 robot.

Usage as standalone (always recalibrates):
    python -m xlerobot_pro.calibration
    python -m xlerobot_pro.calibration --file calibration/my_calib.json

Usage as module (reuses existing calibration file if present):
    from xlerobot_pro import load_or_run_calibration
    load_or_run_calibration(bus)
"""

import argparse
import json
from pathlib import Path
from typing import cast

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from xlerobot_pro.config import BUS_PORT, CALIBRATION_DIR, DEFAULT_CALIBRATION_FILE  # noqa: F401

# Bus 1 (ARMS_PORT): right arm, IDs 7-12. The left arm occupies IDs 1-6 on the
# same bus and is not driven by the vision/IK stack.
ARM_MOTOR_DEFS = {
    "shoulder_pan": Motor(7, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(8, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(9, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(10, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(11, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
}

# Bus 2 (HEAD_PORT): head, IDs 1-2. Names match XLerobotNewWiring so a single
# calibration file serves both the robot class and this stack.
#   head_motor_1 = ID 1 = tilt (up/down)
#   head_motor_2 = ID 2 = pan  (left/right)
HEAD_MOTOR_DEFS = {
    "head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
}

# Original single-bus map (head IDs 1-2 + arm IDs 7-12 on one port). The
# vision/IK scripts build their bus from this; left untouched.
MOTOR_DEFS = {
    "head_motor_1": Motor(2, "sts3215", MotorNormMode.DEGREES),  # pan (ID 2)
    "head_motor_2": Motor(1, "sts3215", MotorNormMode.DEGREES),  # tilt (ID 1)
    "shoulder_pan": Motor(7, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(8, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(9, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(10, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(11, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
}


def load_calibration(bus: FeetechMotorsBus, filepath: Path) -> dict:
    """Load calibration from JSON and apply it to the bus.

    Returns the raw calibration dict.
    """
    with open(filepath) as f:
        calib_raw = json.load(f)
    bus.calibration = {name: MotorCalibration(**vals) for name, vals in calib_raw.items()}
    print(f"Loaded calibration from {filepath}")
    return calib_raw


#: Calibration written by XLerobotNewWiring, in the LeRobot cache location.
ROBOT_CALIBRATION_FILE = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/xlerobot_new_wiring/xlerobot.json"
)

#: XLerobotNewWiring prefixes arm joints by side; this stack uses bare names.
ARM_NAME_PREFIX = "right_arm_"


def load_robot_calibration(
    bus: FeetechMotorsBus,
    motor_defs: dict,
    filepath: Path = ROBOT_CALIBRATION_FILE,
    prefix: str = "",
) -> dict:
    """Apply an XLerobotNewWiring calibration file to ``bus``.

    Lets the vision/IK stack reuse the whole-robot calibration instead of
    keeping a second, separately-recorded copy. ``prefix`` bridges the robot
    class's ``right_arm_<joint>`` naming to the bare names used here.
    """
    with open(filepath) as f:
        raw = json.load(f)

    calib = {}
    for name in motor_defs:
        key = f"{prefix}{name}"
        if key not in raw:
            raise KeyError(f"{key!r} not found in {filepath}")
        calib[name] = MotorCalibration(**raw[key])

    bus.calibration = calib
    print(f"Loaded calibration for {len(calib)} motors from {filepath}")
    return calib


def run_interactive_calibration(bus: FeetechMotorsBus, filepath: Path) -> dict:
    """Run the interactive two-step calibration and save to JSON.

    Steps:
        1. User moves all motors to middle -> set homing offsets
        2. User moves all motors through full range -> record min/max

    Returns the raw calibration dict.
    """
    motor_names = list(bus.motors.keys())
    bus.disable_torque(motor_names)

    input("\n>>> Move ALL motors to the MIDDLE of their range of motion, then press ENTER...")
    homing_offsets = bus.set_half_turn_homings(cast("list[str | int]", motor_names))
    print(f"Homing offsets set: {homing_offsets}")

    print("\n>>> Move ALL motors through their FULL range of motion.")
    input("    Move each joint to both extremes. Press ENTER when done...")
    range_mins, range_maxes = bus.record_ranges_of_motion(cast("list[str | int]", motor_names))
    print(f"Range mins: {range_mins}")
    print(f"Range maxes: {range_maxes}")

    calib_raw: dict[str, dict] = {}
    for name in motor_names:
        motor = bus.motors[name]
        calib_raw[name] = {
            "id": motor.id,
            "drive_mode": 0,
            "homing_offset": homing_offsets[name],
            "range_min": range_mins[name],
            "range_max": range_maxes[name],
        }

    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "w") as f:
        json.dump(calib_raw, f, indent=4)
    print(f"Calibration saved to {filepath}")

    bus.calibration = {name: MotorCalibration(**vals) for name, vals in calib_raw.items()}
    return calib_raw


def load_or_run_calibration(
    bus: FeetechMotorsBus,
    filepath: Path = DEFAULT_CALIBRATION_FILE,
    force: bool = False,
) -> dict:
    """Load existing calibration or run interactive calibration if missing.

    Args:
        bus: Connected FeetechMotorsBus instance.
        filepath: Path to calibration JSON file.
        force: If True, re-run calibration even if file exists.

    Returns the raw calibration dict.
    """
    if filepath.exists() and not force:
        return load_calibration(bus, filepath)
    else:
        if force:
            print("Force recalibration requested.")
        else:
            print("No calibration file found. Running calibration...")
        return run_interactive_calibration(bus, filepath)


def main():
    parser = argparse.ArgumentParser(description="Calibrate SO-101 motors")
    parser.add_argument(
        "--file",
        type=Path,
        default=DEFAULT_CALIBRATION_FILE,
        help=f"Calibration file path (default: {DEFAULT_CALIBRATION_FILE})",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Recalibrate even if calibration file already exists",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=BUS_PORT,
        help=f"Serial port (default: {BUS_PORT})",
    )
    args = parser.parse_args()

    bus = FeetechMotorsBus(port=args.port, motors=MOTOR_DEFS)
    bus.connect()

    try:
        # Always recalibrate when run standalone; --force is kept for explicitness
        load_or_run_calibration(bus, filepath=args.file, force=True)

        # Print current positions to verify
        print("\nCalibrated positions:")
        positions = bus.sync_read("Present_Position", list(bus.motors.keys()))
        for name, val in positions.items():
            print(f"  {name}: {float(val):.2f}")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
