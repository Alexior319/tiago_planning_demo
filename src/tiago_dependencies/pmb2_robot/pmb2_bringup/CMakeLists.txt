cmake_minimum_required(VERSION 3.16)
project(pmb2_bringup)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
