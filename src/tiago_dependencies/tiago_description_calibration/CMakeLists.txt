cmake_minimum_required(VERSION 3.16)
project(tiago_description_calibration)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
