cmake_minimum_required(VERSION 3.16)
project(temperature_sensor_controller)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  pal_hardware_interfaces realtime_tools hardware_interface pluginlib controller_interface sensor_msgs
  message_generation)

add_message_files(
  DIRECTORY msg
  FILES ActuatorTemperatureState.msg)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  )

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS realtime_tools hardware_interface pluginlib controller_interface sensor_msgs pal_hardware_interfaces
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/temperature_sensor_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES temperature_sensor_plugin.xml temperature_sensor_controller.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

