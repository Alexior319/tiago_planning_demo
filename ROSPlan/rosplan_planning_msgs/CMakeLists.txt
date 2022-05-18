cmake_minimum_required(VERSION 2.8.3)
project(rosplan_planning_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  diagnostic_msgs
  roscpp
  rospy
  std_msgs
  genmsg
  rosplan_dispatch_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
# )

add_service_files(
  FILES
  PlanningService.srv
  PlanByString.srv
  PlanningServiceWithReturn.srv
  ProblemService.srv
  ParsingService.srv
  ParsingStringEsterel.srv
  ParsingStringSimple.srv
)

# add_action_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   diagnostic_msgs
   std_msgs
   rosplan_dispatch_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_generation message_runtime diagnostic_msgs roscpp rospy std_msgs genmsg rosplan_dispatch_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
