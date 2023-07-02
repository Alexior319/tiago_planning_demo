cmake_minimum_required(VERSION 3.16)
project(simple_models_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir urdf robots)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
