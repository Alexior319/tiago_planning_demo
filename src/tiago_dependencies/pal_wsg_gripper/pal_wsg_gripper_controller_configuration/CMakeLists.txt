cmake_minimum_required(VERSION 3.16)
project(pal_wsg_gripper_controller_configuration)

find_package(catkin REQUIRED)

catkin_package(
)

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

