cmake_minimum_required(VERSION 3.16)
project(simple_grasping_action)

find_package(catkin REQUIRED COMPONENTS)
catkin_python_setup()
catkin_package()

install(PROGRAMS
  scripts/simple_grasping_action
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
