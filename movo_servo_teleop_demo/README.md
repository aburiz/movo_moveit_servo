# MOVO Servo Teleop Demo (ROS 1 Noetic)

RViz-only proof of concept for realtime MOVO arm teleoperation using MoveIt Servo and Xbox controller input.

## What this demo does

- Uses the archived `kinova-movo` MoveIt model/config for 7-DOF Jaco arms.
- Runs in ROS 1 Noetic on Ubuntu 20.04.
- Uses MoveIt Servo for realtime collision-aware motion.
- Uses Xbox (`joy_node`) input for Cartesian end-effector teleop.
- Publishes dummy Kinova-like outputs for inspection:
  - `/right_arm_driver/in/joint_velocity`
  - `/left_arm_driver/in/joint_velocity`
  - `/right_arm_driver/in/cartesian_velocity`
  - `/left_arm_driver/in/cartesian_velocity`
- Adds a test obstacle box into the planning scene.

## Architecture

- `move_group` + MOVO MoveIt config: planning scene and collision model.
- `right_servo_server` and `left_servo_server`: MoveIt Servo nodes.
- `xbox_servo_mapper.py`: `/joy` -> arm-specific `TwistStamped` commands + deadman/home/toggle logic.
- `fake_kinova_command_bridge.py`: integrates Servo joint velocities to `/joint_states` (RViz motion) and mirrors dummy Kinova-like command topics.
- `add_test_obstacles.py`: inserts a fixed box obstacle in front of the robot.

## Controls (default mapping)

- Deadman is disabled by default (`deadman_button: -1`) for quick testing
- Arm toggle is disabled by default (`toggle_arm_button: -1`) so right arm stays active
- `Y`: reset active arm to home pose
- Left stick: Cartesian XY translation
- Right stick vertical: Cartesian Z translation
- Right stick horizontal: yaw angular velocity
- `LB`/`RB`: roll angular velocity

## Launch

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch
```

If joystick is on a different device:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch joy_dev:=/dev/input/js1
```
