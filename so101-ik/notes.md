  1. To release from stiff hold (disable torque):
  robot.bus.disable_torque()

  This makes all motors freely movable. You can also disconnect entirely, which disables torque by default:
  robot.disconnect()  # This also disables torque by default

  2. To return to original position:
  There's no built-in "home" method, but you can save the starting position and return to it:

  # At the start, save the initial position
  start_obs = robot.get_observation()
  start_positions = {}
  for key, value in start_obs.items():
      if key.endswith('.pos'):
          motor_name = key.removesuffix('.pos')
          start_positions[motor_name] = value

  # ... do your movements ...

  # Return to start position
  action = {f"{motor}.pos": pos for motor, pos in start_positions.items()}
  robot.send_action(action)

  3. Complete example for your script:
  from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
  import time

  # Connect to robot
  config = SO100FollowerConfig(port="/dev/ttyACM0")
  robot = SO100Follower(config)
  robot.connect()

  # Save starting position
  start_obs = robot.get_observation()
  start_positions = {key.removesuffix('.pos'): val
                     for key, val in start_obs.items()
                     if key.endswith('.pos')}

  # Your movements
  action = {
      "shoulder_pan.pos": 45.0,
      "shoulder_lift.pos": 30.0,
      "elbow_flex.pos": -20.0,
      "wrist_flex.pos": 10.0,
      "wrist_roll.pos": 0.0,
      "gripper.pos": 50.0,
  }
  robot.send_action(action)
  time.sleep(2)  # Wait for movement

  # Return to start position
  return_action = {f"{motor}.pos": pos for motor, pos in start_positions.items()}
  robot.send_action(return_action)
  time.sleep(2)  # Wait for return movement

  # Release from stiff hold
  robot.bus.disable_torque()

  The example at /Users/justincosta/xle-robot/examples/provided_examples/0_so100_keyboard_joint_control.py:123-174
  shows a more sophisticated P-control approach for smooth returns if you need that.
