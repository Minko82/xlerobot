"""Read calibrated head motor positions. Use to find URDF zero offsets."""

from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro import HEAD_MOTOR_DEFS, HEAD_PORT, load_robot_calibration

# Bus 2 carries the head: ID 1 = tilt (up/down), ID 2 = pan (left/right).
head_bus = FeetechMotorsBus(port=HEAD_PORT, motors=HEAD_MOTOR_DEFS)
head_bus.connect()
load_robot_calibration(head_bus, HEAD_MOTOR_DEFS)

pos = head_bus.sync_read("Present_Position", list(HEAD_MOTOR_DEFS))
tilt = float(pos["head_motor_1"])
pan = float(pos["head_motor_2"])
print(f"pan={pan:.2f}°, tilt={tilt:.2f}°")
print("\nThese values are the URDF zero offsets.")
print("Add these to frame_transform._head_motor_to_mjcf to correct the transform.")

head_bus.disconnect()
