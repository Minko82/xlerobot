"""Release torque on the arm motors by cycling a connect/disconnect.

Usage:
    python diagnostics/unlock_motors.py
"""

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

from xlerobot_vision.config import BUS_PORT

config = SO101FollowerConfig(port=BUS_PORT, use_degrees=True)
robot = SO101Follower(config)
robot.connect()
robot.disconnect()
