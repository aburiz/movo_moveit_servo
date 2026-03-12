cmake_minimum_required(VERSION 3.0.2)
project(movo_servo_teleop_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/xbox_servo_mapper.py
  scripts/fake_kinova_command_bridge.py
  scripts/add_test_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/delayed_servo_server.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
