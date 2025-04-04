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
  scripts/dynamixel_force_controller_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build and link C++ node
add_executable(dynamixel_force_controller_state_publisher
  src/dynamixel_force_controller_state_publisher.cpp
)

target_link_libraries(dynamixel_force_controller_state_publisher
  ${catkin_LIBRARIES}
)

# Install C++ executable
install(TARGETS dynamixel_force_controller_state_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



