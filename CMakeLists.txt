cmake_minimum_required(VERSION 3.0.2)
project(gripper_force_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dynamixel_force_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
