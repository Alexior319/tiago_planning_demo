cmake_minimum_required(VERSION 3.16)
project(pal_gripper_controller_configuration)

find_package(catkin REQUIRED)

catkin_package(
)

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS scripts/home_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

