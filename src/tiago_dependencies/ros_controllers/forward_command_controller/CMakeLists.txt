cmake_minimum_required(VERSION 3.16)
project(forward_command_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_interface hardware_interface std_msgs realtime_tools)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS controller_interface hardware_interface std_msgs realtime_tools
  INCLUDE_DIRS include
  )

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
