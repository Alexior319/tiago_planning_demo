cmake_minimum_required(VERSION 3.16)
project(custom_dual_ee_configuration)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(DIRECTORY config
	    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	    USE_SOURCE_PERMISSIONS
)
