cmake_minimum_required(VERSION 3.15)

project(ROS_project)

add_subdirectory(src/ROSPlan/rosplan_knowledge_msgs)
add_subdirectory(src/ROSPlan/rosplan_dispatch_msgs)
add_subdirectory(src/ROSPlan/rosplan_planning_msgs)

add_subdirectory(src/planning_node)
add_subdirectory(src/tiago_demo_planning)
