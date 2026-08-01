"""Debug script: measure neutral offsets and joint directions."""
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
import numpy as np

from xlerobot_pro import (
    ARM_MOTOR_DEFS,
    ARM_NAME_PREFIX,
    ARMS_PORT,
    HEAD_MOTOR_DEFS,
    HEAD_PORT,
    load_robot_calibration,
)

# Bus 1: arm (IDs 7-12).  Bus 2: head (ID 1 = tilt, ID 2 = pan).
bus = FeetechMotorsBus(port=ARMS_PORT, motors=ARM_MOTOR_DEFS)
bus.connect()
load_robot_calibration(bus, ARM_MOTOR_DEFS, prefix=ARM_NAME_PREFIX)

head_bus = FeetechMotorsBus(port=HEAD_PORT, motors=HEAD_MOTOR_DEFS)
head_bus.connect()
load_robot_calibration(head_bus, HEAD_MOTOR_DEFS)

arm_joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

print("=== Joint direction test ===")
print("For each joint, move it in the POSITIVE MJCF direction:")
print("  - shoulder_pan:  rotate the base counter-clockwise (top view)")
print("  - shoulder_lift: tilt the upper arm backward (away from front)")
print("  - elbow_flex:    bend the elbow (fold the forearm up)")
print("  - wrist_flex:    tilt the wrist up")
print("  - wrist_roll:    roll the wrist counter-clockwise")
print()

input(">>> First, put arm in NEUTRAL (straight up). Press ENTER...")
neutral = bus.sync_read("Present_Position", arm_joints)
neutral_vals = {name: float(neutral[name]) for name in arm_joints}
print("Neutral readings:")
for name in arm_joints:
    print(f"  {name:20s} = {neutral_vals[name]:8.2f} deg")

print()
for joint in arm_joints:
    input(f">>> Move ONLY {joint} in the POSITIVE direction, then press ENTER...")
    pos = bus.sync_read("Present_Position", arm_joints)
    delta = float(pos[joint]) - neutral_vals[joint]
    direction = "SAME" if delta > 0 else "REVERSED"
    print(f"  {joint}: moved {delta:+.2f} deg -> direction is {direction}")
    print()

print("\n=== Summary: neutral offsets ===")
for name in arm_joints:
    print(f"  {name:20s} = {neutral_vals[name]:8.2f} deg")

bus.disconnect()
head_bus.disconnect()
