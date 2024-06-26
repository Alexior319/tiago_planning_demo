cmake_minimum_required(VERSION 3.16)
project(controller_manager)

add_compile_options(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/controller_manager.cpp
  include/controller_manager/controller_manager.h
  include/controller_manager/controller_loader_interface.h
  include/controller_manager/controller_loader.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  add_rostest_gtest(controller_manager_hwi_switch_test
    test/hwi_switch_test.test
    test/hwi_switch_test.cpp
  )
  target_link_libraries(controller_manager_hwi_switch_test ${PROJECT_NAME} ${catkin_LIBRARIES})

endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/spawner scripts/unspawner scripts/controller_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_python_setup()
