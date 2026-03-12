# movo_servo

MoveIt Servo setup for real-time Cartesian teleoperation of MOVO Jaco arms in ROS Noetic.

## Quick Start

1. Source your workspace:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/movo_servo_ws/devel/setup.bash
   ```
2. Launch MOVO simulation + MoveIt + Servo (right arm by default):
   ```bash
   roslaunch movo_servo movo_servo_sim.launch arm:=right
   ```
3. In a new terminal, start the Twist-to-TwistStamped bridge:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/movo_servo_ws/devel/setup.bash
   roslaunch movo_servo movo_servo_input.launch
   ```
4. In a third terminal, run keyboard teleop:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/movo_servo_ws/devel/setup.bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/movo_servo/cmd_vel
   ```

## Left Arm

Use:
```bash
roslaunch movo_servo movo_servo_sim.launch arm:=left
```

## Useful Runtime Checks

Servo status:
```bash
rostopic echo /servo_server/status
```

Outgoing trajectory commands:
```bash
rostopic hz /movo/right_arm_controller/command
```
