"""Enable torque on all bus-1 motors so the robot holds its pose.

Usage:
    python diagnostics/lock_motors.py
"""

from lerobot.robots.xlerobot import XLerobot, XLerobotConfig

from xlerobot_pro.config import BUS_PORT

config = XLerobotConfig(port1=BUS_PORT, use_degrees=True)
robot = XLerobot(config)
robot.bus1.connect()
robot.bus1.enable_torque()
