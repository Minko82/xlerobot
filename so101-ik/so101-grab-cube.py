from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig

# Connect to robot
config = SO100FollowerConfig(port="/dev/ttyACM0")
robot = SO100Follower(config)
robot.connect()

# Move individual joints (degrees)
action = {
    "shoulder_pan.pos": 45.0,
    "shoulder_lift.pos": 30.0,
    "elbow_flex.pos": -20.0,
    "wrist_flex.pos": 10.0,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 50.0,  # 0=closed, 100=open
}
robot.send_action(action)
