cmake_minimum_required(VERSION 3.0.2)
project(movo_servo_teleop_demo)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  kinova_msgs
  moveit_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/xbox_servo_mapper.py
  scripts/fake_kinova_command_bridge.py
  scripts/real_kinova_command_bridge.py
  scripts/ping_real_arm.py
  scripts/check_d455_moveit_perception.py
  scripts/octomap_live_refresh.py
  scripts/rosbridge_compat_adapter.py
  scripts/rosbridge_public_gateway.py
  scripts/rosbridge_private_gateway.py
  scripts/private_compute_stack.py
  scripts/real_arm_launch_error.py
  scripts/add_test_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/delayed_servo_server.sh
  scripts/extract_local_kinova_sdk.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
