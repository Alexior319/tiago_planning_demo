cmake_minimum_required(VERSION 3.16)
project(pal_navigation_cfg_tiago)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

