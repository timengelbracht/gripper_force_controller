cmake_minimum_required(VERSION 3.0.2)
project(gripper_force_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  dynamixel_sdk
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/dynamixel_force_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build and link C++ node
add_executable(gripper_force_controller_node
  src/gripper_force_controller_node.cpp
)

target_link_libraries(gripper_force_controller_node
  ${catkin_LIBRARIES}
)

# Install C++ executable
install(TARGETS gripper_force_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



