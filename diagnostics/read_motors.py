#!/usr/bin/env python3
"""Read and print current motor positions (degrees).
Disable torque first so you can manually position the arm."""

from lerobot.motors.feetech import FeetechMotorsBus
from xlerobot_pro import ARM_MOTOR_DEFS, ARM_NAME_PREFIX, ARMS_PORT, load_robot_calibration

bus = FeetechMotorsBus(port=ARMS_PORT, motors=ARM_MOTOR_DEFS)
bus.connect()
load_robot_calibration(bus, ARM_MOTOR_DEFS, prefix=ARM_NAME_PREFIX)

arm_motors = list(ARM_MOTOR_DEFS)
bus.disable_torque(arm_motors)

input("Move the gripper directly above the cube, then press ENTER...")

positions = bus.sync_read("Present_Position", list(bus.motors.keys()))
print("\nMotor positions (degrees):")
for name, val in positions.items():
    print(f"  {name:20s}: {float(val):.2f}")

bus.disconnect()
