cmake_minimum_required(VERSION 3.16)
project(tiago_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "disable_collisions" EXCLUDE
        PATTERN "end_effectors" EXCLUDE
        PATTERN "scripts" EXCLUDE
)
