# MOVO Servo Teleop Demo

ROS 1 Noetic MoveIt Servo teleop for MOVO with two operating modes:

- `RViz only`: the original fake bridge keeps `/joint_states` moving so RViz mirrors Servo output.
- `RViz + real arms`: the same Servo joint velocity outputs also stream to the classic Kinova Jaco2 Gen2 drivers at 100 Hz.

This integration is built for the classic Kinova ROS stack used in the reference `movo_arm_ws`, not Kortex / Gen3.

## ROS Bridge Compatibility

The copied Humble bridge folder in
`/home/abu/Downloads/ros-humble-ros1-bridge-builder(DO NOT MODIFY ONLY REFER)`
was used as a read-only reference only.

What it is doing:

- runs `ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`
- points the ROS2 bridge container at the ROS1 master with `ROS_MASTER_URI`
- includes custom `kinova_msgs` support for:
  - `PoseVelocity`
  - `JointVelocity`
  - `HomeArm`
  - `Start`
  - `Stop`
  - `ClearTrajectories`
  - `ArmJointAngles`
  - `SetFingersPosition`

What this repo now does for that bridge:

- keeps the bridge-facing ROS1 master lean by splitting the system into:
  - public/shared master on `.101:11311`
  - private/local compute master on `127.0.0.1:11312`
- keeps the old public ROS1-side Cartesian Kinova command topics alive on the public/shared master:
  - `/right_arm/right_arm_driver/in/cartesian_velocity`
  - `/left_arm/left_arm_driver/in/cartesian_velocity`
- keeps the old public ROS1-side Kinova services alive on the public/shared master:
  - `/right_arm/right_arm_driver/in/start`
  - `/right_arm/right_arm_driver/in/stop`
  - `/right_arm/right_arm_driver/in/home_arm`
  - `/right_arm/right_arm_driver/in/clear_trajectories`
  - `/left_arm/left_arm_driver/in/start`
  - `/left_arm/left_arm_driver/in/stop`
  - `/left_arm/left_arm_driver/in/home_arm`
  - `/left_arm/left_arm_driver/in/clear_trajectories`
- keeps the classic Kinova actions alive on the public/shared master:
  - `/right_arm/right_arm_driver/fingers_action/finger_positions`
  - `/right_arm/right_arm_driver/joints_action/joint_angles`
  - `/right_arm/right_arm_driver/pose_action/tool_pose`
  - `/left_arm/left_arm_driver/fingers_action/finger_positions`
  - `/left_arm/left_arm_driver/joints_action/joint_angles`
  - `/left_arm/left_arm_driver/pose_action/tool_pose`
- mirrors only lightweight feedback back onto the public/shared master:
  - `/right_arm/right_arm_driver/out/joint_state`
  - `/right_arm/right_arm_driver/out/joint_angles`
  - `/right_arm/right_arm_driver/out/finger_position`
  - `/right_arm/right_arm_driver/out/tool_pose`
  - `/left_arm/left_arm_driver/out/joint_state`
  - `/left_arm/left_arm_driver/out/joint_angles`
  - `/left_arm/left_arm_driver/out/finger_position`
  - `/left_arm/left_arm_driver/out/tool_pose`
- routes public Cartesian velocity commands into MoveIt Servo on the private/local master instead of letting them go straight to the hardware driver
- keeps D455, MoveIt, Servo, octomap, RViz, fake bridge, and the real Kinova drivers on the private/local master only
- keeps camera/depth/octomap topics off the bridge-facing master entirely, so `dynamic_bridge --bridge-all-topics` does not forward them

Important limitation:

- direct legacy `/<arm>/<driver>/in/joint_velocity` is intentionally not exposed on the public/shared master in split-master mode
- that avoids bypassing MoveIt Servo through the bridge
- the collision-aware public replacement path is the Cartesian command interface above

## Architecture

- `xbox_servo_mapper.py`
  `/joy` -> per-arm Servo twist commands, gripper commands, active-arm selection, start/stop/home events.
- `fake_kinova_command_bridge.py`
  Keeps the original RViz path independent by integrating Servo joint velocities into `/joint_states`.
- `real_kinova_dual_arms.launch`
  Brings up the classic Kinova Ethernet drivers under:
  - `/right_arm/right_arm_driver`
  - `/left_arm/left_arm_driver`
- `private_compute_stack.py`
  Starts a hidden local ROS1 master on `127.0.0.1:11312` and launches the private compute stack there.
- `movo_servo_private_compute.launch`
  Brings up the private/local compute master stack:
  - D455
  - MoveIt
  - MoveIt Servo
  - fake bridge
  - real Kinova drivers
  - real command bridge
  - optional RViz / joystick
- `rosbridge_private_gateway.py`
  Lives on the private/local master and accepts lean localhost commands from the public/shared master.
- `rosbridge_public_gateway.py`
  Lives on the public/shared master and preserves the classic Kinova command surface expected by the Humble bridge.
- `real_kinova_command_bridge.py`
  Subscribes to the existing Servo outputs:
  - `/movo_servo_teleop_demo/right/joint_velocity_cmd`
  - `/movo_servo_teleop_demo/left/joint_velocity_cmd`
  Converts `rad/s` from MoveIt Servo into `deg/s` for `kinova_msgs/JointVelocity`, then streams to the real drivers at 100 Hz.
- `/movo_servo_teleop_demo/real_arm_status`
  Publishes `diagnostic_msgs/DiagnosticArray` with per-arm connection, start/stop, estop, feedback, and command-age status.

## Important Networking Note

`local_machine_ip` is intentionally **not hardcoded** to a machine-specific value.

- Default placeholder:
  [`config/real_arms.yaml`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/config/real_arms.yaml)
  sets `local_machine_ip: ""`
- Launch override:
  [`launch/movo_servo_teleop_demo.launch`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/launch/movo_servo_teleop_demo.launch)
  and
  [`launch/real_kinova_dual_arms.launch`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/launch/real_kinova_dual_arms.launch)
  both accept `local_machine_ip:=<CURRENT_PC_IP>`

If `use_real_arms:=true` and `local_machine_ip` is empty, launch fails loudly instead of silently reusing a stale host IP.

## Current Hardware Network Configuration

- Right arm serial: `PJ00900006509075-0`
- Right arm IP: `192.168.131.30`
- Right arm port: `25000`
- Left arm serial: `PJ00900006432489-0`
- Left arm IP: `192.168.131.20`
- Left arm port: `65535`
- Mask: `255.255.255.0`
- Robot type: `j2s7s300`
- Driver local ports used by this launch:
  - right: `25000` / `25005`
  - left: `25010` / `25015`

## Environment Setup

These commands match the folders in this machine right now:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_PACKAGE_PATH=/home/abu/Downloads/movo/movo_moveit_servo:/home/abu/Downloads/movo_arm_ws-main/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/home/abu/Downloads/movo_arm_ws-main/devel:$CMAKE_PREFIX_PATH
```

Recommended long term: put both the Kinova driver stack and this repo in the same catkin workspace and build them together.

For the stock Kinova ROS installation/use flow that this setup is modeled after, see the official reference:
https://github.com/Kinovarobotics/kinova-ros?tab=readme-ov-file#installation

If `rospack find kinova_driver` fails, the reference workspace was likely moved after it was built and its `devel/.catkin` file still points at the old source tree. In this machine's current state, that stale path is `/home/movo/bimanual_ws/src`, so the explicit `ROS_PACKAGE_PATH` export above is required unless you rebuild `movo_arm_ws-main` in its current location.

The real-arm launch uses the bundled Kinova SDK package in [`kinova_api/`](/home/abu/Downloads/movo/movo_moveit_servo/kinova_api). Right now that is [`KinovaAPI-6.1.0-amd64.deb`](/home/abu/Downloads/movo/movo_moveit_servo/kinova_api/KinovaAPI-6.1.0-amd64.deb). Its shared libraries are extracted locally under `kinova_api/runtime/usr/lib`, and [`real_kinova_dual_arms.launch`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/launch/real_kinova_dual_arms.launch) exports that directory as `LD_LIBRARY_PATH` for the Kinova driver nodes.

If you need to recreate that runtime folder, run:

```bash
rosrun movo_servo_teleop_demo extract_local_kinova_sdk.sh
```

## Launch Commands

For every launch below, first run:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_PACKAGE_PATH=/home/abu/Downloads/movo/movo_moveit_servo:/home/abu/Downloads/movo_arm_ws-main/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/home/abu/Downloads/movo_arm_ws-main/devel:$CMAKE_PREFIX_PATH
```

### Most Common Run Commands

If this computer is the ROS1 master and its Ethernet IP is `192.168.131.101`, use:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_MASTER_URI=http://192.168.131.101:11311
export ROS_IP=192.168.131.101
export ROS_PACKAGE_PATH=/home/abu/Downloads/movo/movo_moveit_servo:/home/abu/Downloads/movo_arm_ws-main/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/home/abu/Downloads/movo_arm_ws-main/devel:$CMAKE_PREFIX_PATH
```

All-in-one local bringup on this PC:

- RViz
- MoveIt Servo collision checking
- Xbox controller
- Intel RealSense D455 depth perception
- real Kinova arms over Ethernet

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=true \
  launch_realsense_d455:=true \
  launch_joy:=true \
  joy_dev:=/dev/input/js0 \
  load_test_obstacle:=false \
  local_machine_ip:=192.168.131.101
```

Same setup but RViz/sim only, no real arms:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=false \
  launch_realsense_d455:=true \
  launch_joy:=true \
  joy_dev:=/dev/input/js0 \
  load_test_obstacle:=false
```

Quick D455 health check while the launch is running:

```bash
rostopic hz /d455/aligned_depth_to_color/image_raw
rosrun movo_servo_teleop_demo check_d455_moveit_perception.py
```

RViz only:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch
```

RViz with an Intel RealSense D455 feeding the MoveIt planning scene in Phase 1:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  launch_realsense_d455:=true
```

What the Phase 1 D455 path does:

- starts Intel's ROS1 `realsense2_camera` wrapper
- enables depth, color, and aligned depth output
- attaches the D455 frame tree to the existing `kinect2_link` mount with a static transform
- feeds MoveIt Octomap directly from the aligned D455 depth image:
  - `/d455/aligned_depth_to_color/image_raw`
- keeps the D455 depth stream out to `6.0 m` so removed obstacles still have background data available for clearing
- limits the MoveIt depth-octomap updater itself to `0.8 m` so only nearby obstacles become occupied voxels
- raises the Octomap update cap to `30 Hz` for faster clearing of transient obstacles
- runs a lightweight octomap live-refresh node so stale occupied cells are periodically dropped and rebuilt from the current depth view
- keeps the D455 point cloud off by default because MoveIt is using the aligned depth image directly
- keeps the D455 topics local-only in split-master rosbridge mode so the bridge does not forward camera traffic

This avoids a broad URDF/SRDF rewrite while still letting MoveIt Servo plan around live depth data from the D455. The aligned depth-image updater is also less prone to leaving behind "ghost" occupied voxels after a transient obstacle such as a hand moves away.

If you want the octomap to behave like a live obstacle layer instead of a persistent map, leave the default refresh enabled. The key launch args are:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  launch_realsense_d455:=true \
  realsense_clip_distance:=6.0 \
  octomap_max_range:=0.8 \
  live_octomap_refresh:=true \
  live_octomap_refresh_period_sec:=0.15
```

The current defaults are tuned for faster disappearance of stale octomap voxels:

- depth-image octomap update rate: `30 Hz`
- live octomap refresh period: `0.15 s`
- RViz planning-scene display time: `0.05 s`

If you want to see farther into the scene anyway, you can still raise the D455 depth range at launch, but that will also pull more room clutter into the planning scene:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  launch_realsense_d455:=true \
  realsense_clip_distance:=5.0
```

If the camera package is not installed yet:

```bash
sudo apt-get install -y ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

RViz only with a different joystick device:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch joy_dev:=/dev/input/js1
```

Real drivers only, no MoveIt / RViz:

```bash
roslaunch movo_servo_teleop_demo real_kinova_dual_arms.launch local_machine_ip:=192.168.131.101
```

If you need to override the SDK library location manually:

```bash
roslaunch movo_servo_teleop_demo real_kinova_dual_arms.launch \
  local_machine_ip:=192.168.131.101 \
  kinova_sdk_lib_dir:=/path/to/kinova/sdk/lib
```

RViz + real arms together:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=true \
  local_machine_ip:=192.168.131.101
```

RViz + real arms + D455 + Xbox controller together:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=true \
  launch_realsense_d455:=true \
  launch_joy:=true \
  joy_dev:=/dev/input/js0 \
  load_test_obstacle:=false \
  local_machine_ip:=192.168.131.101
```

`.101` replaces `.10` while `.100` stays the base/bridge machine:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_MASTER_URI=http://192.168.131.101:11311
export ROS_IP=192.168.131.101
export ROS_PACKAGE_PATH=/home/abu/Downloads/movo/movo_moveit_servo:/home/abu/Downloads/movo_arm_ws-main/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/home/abu/Downloads/movo_arm_ws-main/devel:$CMAKE_PREFIX_PATH

roslaunch movo_servo_teleop_demo movo_servo_rosbridge_ready.launch
```

What that wrapper does:

- keeps `.101:11311` as the public/shared ROS1 master for the Humble bridge
- starts a private/local ROS1 compute master on `127.0.0.1:11312`
- runs Kinova drivers, D455, MoveIt, Servo, fake bridge, real bridge, and optional RViz on the private/local master
- runs only the lean public Kinova-compatible gateway on the public/shared master
- defaults `launch_joy:=false`
- defaults `load_test_obstacle:=false` so bringup is not born in the demo collision box
- uses `ROS_IP` as the default `local_machine_ip`
- keeps D455, depth, point cloud, and octomap topics off the bridge-facing master

Most likely replacement-PC command when you want the base bridge on `.100` to keep talking to the same ROS1-side arm interface:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_MASTER_URI=http://192.168.131.101:11311
export ROS_IP=192.168.131.101
export ROS_PACKAGE_PATH=/home/abu/Downloads/movo/movo_moveit_servo:/home/abu/Downloads/movo_arm_ws-main/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/home/abu/Downloads/movo_arm_ws-main/devel:$CMAKE_PREFIX_PATH

roslaunch movo_servo_teleop_demo movo_servo_rosbridge_ready.launch \
  local_machine_ip:=192.168.131.101 \
  launch_realsense_d455:=true \
  rviz:=true
```

On `.100`, keep the bridge there but point it at `.101` instead of `.10`:

```bash
docker run --rm -it --name ros_bridge \
  --network host \
  --ipc host \
  -v /etc/cyclonedds/cyclonedds.xml:/etc/cyclonedds/cyclonedds.xml \
  -e ROS_MASTER_URI=http://192.168.131.101:11311/ \
  -e ROS_IP=192.168.131.100 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml \
  kinova_arms_bridge:latest \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    source /kinova_msgs_custom/kinova_msgs_ros2/install/setup.bash && \
    source /ros-humble-ros1-bridge/install/local_setup.bash && \
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
  "
```

Lean split-master checks after the wrapper is up:

Public/shared master should stay camera-free:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_MASTER_URI=http://192.168.131.101:11311
export ROS_IP=192.168.131.101
rostopic list | rg 'd455|aligned_depth|image_raw|points|octomap'
```

Expected: no D455 / depth / point-cloud topics on the public/shared master.

Private/local compute checks:

```bash
source /opt/ros/noetic/setup.bash
source /home/abu/Downloads/movo_arm_ws-main/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP
unset ROS_HOSTNAME
export ROS_PACKAGE_PATH=/home/abu/Downloads/movo/movo_moveit_servo:/home/abu/Downloads/movo_arm_ws-main/src:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/home/abu/Downloads/movo_arm_ws-main/devel:$CMAKE_PREFIX_PATH

rostopic hz /d455/aligned_depth_to_color/image_raw
rosrun movo_servo_teleop_demo check_d455_moveit_perception.py
```

RViz + real arms with explicit port / serial overrides:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=true \
  local_machine_ip:=192.168.131.101 \
  right_arm_ip:=192.168.131.30 \
  left_arm_ip:=192.168.131.20 \
  right_local_cmd_port:=25000 \
  right_local_broadcast_port:=25005 \
  left_local_cmd_port:=25010 \
  left_local_broadcast_port:=25015 \
  right_arm_serial:=PJ00900006509075-0 \
  left_arm_serial:=PJ00900006432489-0
```

Optional standalone arm-state publishers from the Kinova side:

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=true \
  local_machine_ip:=192.168.131.101 \
  real_arms_use_urdf:=true
```

## Controller Guide

- `A`
  Enable motion and call real-arm `start` on both drivers.
- `B`
  Latch estop, publish zeros, and call real-arm `stop` on both drivers.
- `X`
  Toggle active teleop arm between left and right.
- `RB`
  Move the current active arm to the configured custom home joint pose.
- `LB` double tap
  Move both arms to the configured custom home joint poses.
- Left stick vertical
  Cartesian X.
- Left stick horizontal
  Cartesian Y.
- Right stick vertical
  Cartesian Z.
- Right stick horizontal
  Yaw.
- D-pad up/down
  Pitch.
- D-pad left/right
  Roll.
- `RT`
  Close gripper command for the active arm.
- `LT`
  Open gripper command for the active arm.

## Diagnostics And Safe Identification

Non-motion diagnostics:

```bash
rosrun movo_servo_teleop_demo ping_real_arm.py --arm right
rosrun movo_servo_teleop_demo ping_real_arm.py --arm left
```

Optional small identification pulse:

```bash
rosrun movo_servo_teleop_demo ping_real_arm.py --arm right --motion
rosrun movo_servo_teleop_demo ping_real_arm.py --arm left --motion
```

What `ping_real_arm.py` prints:

- requested arm
- target driver namespace
- target robot IP
- configured `local_machine_ip`
- whether the robot IP responds to `ping`
- whether `start` and `stop` services exist
- whether the custom home joint-angle action exists
- whether feedback is arriving on `.../out/joint_state`

Best practice for `--motion`:

- Use it against `real_kinova_dual_arms.launch` by itself.
- In the combined teleop launch, the real bridge is continuously streaming commands, so the diagnostic pulse is less isolated.

## Actuator-1 Correction

Only actuator 1 is corrected now.

- The MOVO URDF and MoveIt model stay on the legacy geometry.
- The real-arm bridge flips joint 1 velocity before publishing to the Kinova driver.
- The fake RViz bridge negates joint 1 when converting custom home angles from Kinova native degrees so the simulated home pose still matches the real robot.

## Runtime Topics And Services

Single-master local teleop mode publishes real driver joint velocities here:

- `/right_arm/right_arm_driver/in/joint_velocity`
- `/left_arm/left_arm_driver/in/joint_velocity`

Single-master local teleop mode uses these real driver services:

- `/right_arm/right_arm_driver/in/start`
- `/right_arm/right_arm_driver/in/stop`
- `/left_arm/left_arm_driver/in/start`
- `/left_arm/left_arm_driver/in/stop`

Split-master rosbridge mode exposes these public bridge-facing interfaces instead:

- `/right_arm/right_arm_driver/in/cartesian_velocity`
- `/right_arm/right_arm_driver/in/cartesian_velocity_with_fingers`
- `/right_arm/right_arm_driver/in/cartesian_velocity_with_finger_velocity`
- `/right_arm/right_arm_driver/in/start`
- `/right_arm/right_arm_driver/in/stop`
- `/right_arm/right_arm_driver/in/home_arm`
- `/right_arm/right_arm_driver/in/clear_trajectories`
- `/right_arm/right_arm_driver/fingers_action/finger_positions`
- `/right_arm/right_arm_driver/joints_action/joint_angles`
- `/right_arm/right_arm_driver/pose_action/tool_pose`
- `/left_arm/left_arm_driver/in/cartesian_velocity`
- `/left_arm/left_arm_driver/in/cartesian_velocity_with_fingers`
- `/left_arm/left_arm_driver/in/cartesian_velocity_with_finger_velocity`
- `/left_arm/left_arm_driver/in/start`
- `/left_arm/left_arm_driver/in/stop`
- `/left_arm/left_arm_driver/in/home_arm`
- `/left_arm/left_arm_driver/in/clear_trajectories`
- `/left_arm/left_arm_driver/fingers_action/finger_positions`
- `/left_arm/left_arm_driver/joints_action/joint_angles`
- `/left_arm/left_arm_driver/pose_action/tool_pose`

Custom home actions used internally by the teleop bridge:

- `/right_arm/right_arm_driver/joints_action/joint_angles`
- `/left_arm/left_arm_driver/joints_action/joint_angles`

Status topic:

```bash
rostopic echo /movo_servo_teleop_demo/real_arm_status
```

Useful checks:

```bash
rostopic hz /right_arm/right_arm_driver/in/joint_velocity
rostopic hz /left_arm/left_arm_driver/in/joint_velocity
rostopic echo -n 1 /right_arm/right_arm_driver/out/joint_state
rostopic echo -n 1 /left_arm/left_arm_driver/out/joint_state
```

## How To Test Every Requested Feature

### 1. RViz-only regression check

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch
```

Verify:

- RViz still loads.
- `fake_kinova_command_bridge.py` still moves `/joint_states`.
- Servo teleop works exactly like before.

### 2. Launch guard for missing `local_machine_ip`

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch use_real_arms:=true
```

Expected:

- launch fails immediately
- error explains that `local_machine_ip:=<CURRENT_PC_IP>` is required

### 3. Driver-only hardware bringup

```bash
roslaunch movo_servo_teleop_demo real_kinova_dual_arms.launch local_machine_ip:=192.168.131.101
```

Verify:

- both Kinova drivers come up under the namespaced paths above
- `ping_real_arm.py --arm right`
- `ping_real_arm.py --arm left`

### 4. Combined RViz + real hardware mode

```bash
roslaunch movo_servo_teleop_demo movo_servo_teleop_demo.launch \
  use_real_arms:=true \
  local_machine_ip:=192.168.131.101
```

Verify:

- RViz still moves because the fake bridge is still active
- `/movo_servo_teleop_demo/real_arm_status` appears
- `/right_arm/right_arm_driver/in/joint_velocity` and `/left_arm/left_arm_driver/in/joint_velocity` publish at 100 Hz

### 5. Start / stop semantics

With the combined launch running:

1. Press `A`
   Expect both drivers to receive `start`.
2. Move one arm with the sticks
   Expect RViz motion and matching real-arm motion.
3. Press `B`
   Expect zero velocity streaming, real-arm `stop`, and `estop=true` in `/movo_servo_teleop_demo/real_arm_status`.
4. Press `A` again
   Expect motion to re-enable.

### 6. Left / right isolation

1. Press `X` until the right arm is active.
2. Move the sticks.
   Only the right real-arm namespace should see non-zero commands.
3. Press `X` again.
4. Move the sticks.
   Only the left real-arm namespace should see non-zero commands.

Helpful check:

```bash
rostopic echo /movo_servo_teleop_demo/real_arm_status
```

### 7. Home semantics

1. Press `RB`
   Expect only the active arm to receive a custom joint-angle goal.
2. Double tap `LB`
   Expect both arms to receive their configured custom joint-angle goals.

Configured custom home joint angles in Kinova native degrees:

- Right: `268.94, 82.92, 190.84, 322.63, 190.75, 142.04, 180.00`
- Left: `91.06, 277.08, 169.16, 37.37, 169.25, 217.96, 180.00`

### 8. Stale-command zeroing

1. Move an arm.
2. Release the stick.
3. Watch the driver topic:

```bash
rostopic echo /right_arm/right_arm_driver/in/joint_velocity
```

Expected:

- commands return to zero
- bridge keeps publishing zeros at 100 Hz when commands go stale or motion is disabled

## Notes On Units

The real bridge explicitly converts units at the driver boundary:

- MoveIt Servo input/output path in this demo uses `speed_units`
- the demo treats Servo joint output as `rad/s`
- classic Kinova `kinova_msgs/JointVelocity` expects `deg/s`

That conversion is done in:

- [`real_kinova_command_bridge.py`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/scripts/real_kinova_command_bridge.py)

The fake RViz bridge remains radians-based because it integrates directly into `/joint_states`.

## Home Recovery Behavior

- The real bridge no longer adds a post-home command lockout. As soon as the custom home action finishes, new teleop commands can flow again.
- The real bridge now also clears the Kinova driver trajectory FIFO after custom home, because the stock joint-angle action leaves a queued hold that can make the physical arm lag behind RViz.
- The fake RViz bridge no longer teleports instantly to home by default. It blends to the configured home pose over `home_transition_duration_sec` so MoveIt Servo sees a smoother `/joint_states` change.
- Current defaults:
  - [`config/real_arms.yaml`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/config/real_arms.yaml) sets `bridge.home_lockout_sec: 0.0`
  - [`config/real_arms.yaml`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/config/real_arms.yaml) sets `bridge.post_home_clear_trajectories: true` and `bridge.post_home_clear_delay_sec: 0.2`
  - [`config/fake_kinova_bridge.yaml`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/config/fake_kinova_bridge.yaml) sets `home_transition_duration_sec: 0.75`

## Assumptions Still Worth Confirming

- The right/left serial-number mapping above continues to match the physical robot.
- Gripper teleop remains RViz-side only in this package; this change focused on real-arm joint velocity streaming plus start/stop/custom-home semantics.

## Files That Avoid Stale Host-IP Assumptions

These new real-arm files do **not** hardcode a machine-specific host IP:

- [`config/real_arms.yaml`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/config/real_arms.yaml)
- [`launch/movo_servo_teleop_demo.launch`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/launch/movo_servo_teleop_demo.launch)
- [`launch/real_kinova_dual_arms.launch`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/launch/real_kinova_dual_arms.launch)
- [`scripts/real_kinova_command_bridge.py`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/scripts/real_kinova_command_bridge.py)
- [`scripts/ping_real_arm.py`](/home/abu/Downloads/movo/movo_moveit_servo/movo_servo_teleop_demo/scripts/ping_real_arm.py)

The fixed arm IPs stay explicit because they describe the robot hardware, not the host machine:

- left: `192.168.131.20`
- right: `192.168.131.30`
