cmake_minimum_required(VERSION 3.16)
project(pmb2_gazebo)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch Media models worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
