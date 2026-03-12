cmake_minimum_required(VERSION 3.0.2)
project(movo_servo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_to_twist_stamped.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
