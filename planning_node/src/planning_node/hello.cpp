//
// Created by xyz on 2022/3/22.
//
#include <sstream>
#include <queue>


#include <ros/ros.h>
#include <std_msgs/String.h>

#include <fmt/core.h>


#include <planning_node/planner.h>
#include <xyz_knowledge_base/parser.h>


using namespace planning_node;

int main(int argc, char** argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    planning_node::parser p;
    p.parseDomain(nh, "/planning");
//    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);


    planning_node::planner planner{p};
    auto plan = planner.planning(p.initial, p.goal);
    debug_info("Found plan(BFS): {}", plan);
//    auto dfsPlan = planner.planningDFS(p.initial, p.goal);
//    debug_info("Found plan(DFS): {}", dfsPlan);
//    auto backwardPlan = planner.backward(p.initial, p.goal);
//    debug_info("Found plan(backward): {}", backwardPlan);
    return 0;
}
