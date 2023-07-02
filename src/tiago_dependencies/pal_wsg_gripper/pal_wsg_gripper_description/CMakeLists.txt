cmake_minimum_required(VERSION 3.16)
project(pal_wsg_gripper_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir gazebo meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


