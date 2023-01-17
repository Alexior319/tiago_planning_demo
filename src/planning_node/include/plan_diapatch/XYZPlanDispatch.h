//
// Created by xyz on 22-5-17.
//

#ifndef PLANNING_NODE_XYZPLANDISPATCH_H
#define PLANNING_NODE_XYZPLANDISPATCH_H

#include <ros/ros.h>
#include <xyz_dispatch_msgs/DispatchService.h>
#include <xyz_dispatch_msgs/OnlineDispatchService.h>
#include <xyz_dispatch_msgs/CompletePlan.h>
#include <xyz_dispatch_msgs/ActionFeedback.h>
#include <std_srvs/Empty.h>

#include "common.h"

namespace planning_node {

    class XYZPlanDispatch {

    protected:
        ros::NodeHandle& _nh;
        xyz_dispatch_msgs::CompletePlan current_plan;
        bool plan_received{false};
        ros::ServiceServer dispatch_server, pause_dispatch_server, recover_dispatch_server;
        ros::ServiceServer online_dispatch_server;
        ros::ServiceClient online_planning_client;
        ros::Publisher dispatch_pub, feedback_pub;
        ros::Subscriber feedback_subscriber, plan_subscriber;

        std::string action_dispatch_topic;
        std::string action_feedback_topic;

        int current_action{0};
        bool dispatch_paused{false};

        std::map<int,bool> action_received;
        std::map<int,bool> action_completed;

    public:
        explicit XYZPlanDispatch(ros::NodeHandle& nh);

        bool dispatchPlanService(xyz_dispatch_msgs::DispatchService::Request& req,
                                 xyz_dispatch_msgs::DispatchService::Response& res);

        bool onlineDispatchPlanService(xyz_dispatch_msgs::DispatchService::Request& req,
                                 xyz_dispatch_msgs::DispatchService::Response& res);
        bool pauseDispatchService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        bool recoverDispatchService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        void planCallback(const xyz_dispatch_msgs::CompletePlan::ConstPtr& plan);
        void feedbackCallback(const xyz_dispatch_msgs::ActionFeedback::ConstPtr& msg);
    };
}


#endif //PLANNING_NODE_XYZPLANDISPATCH_H
