//
// Created by xyz on 22-5-17.
//

#ifndef PLANNING_NODE_XYZPLANDISPATCH_H
#define PLANNING_NODE_XYZPLANDISPATCH_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/DispatchService.h>
#include <rosplan_dispatch_msgs/CompletePlan.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include "common.h"

namespace planning_node {

    class XYZPlanDispatch {

    protected:
        ros::NodeHandle& _nh;
        rosplan_dispatch_msgs::CompletePlan current_plan;
        bool plan_received{false};
        ros::ServiceServer dispatch_server;
        ros::Publisher dispatch_pub, feedback_pub;
        ros::Subscriber feedback_subscriber, plan_subscriber;

        std::string action_dispatch_topic;
        std::string action_feedback_topic;

        int current_action;

        std::map<int,bool> action_received;
        std::map<int,bool> action_completed;

    public:
        explicit XYZPlanDispatch(ros::NodeHandle& nh);

        bool dispatchPlanService(rosplan_dispatch_msgs::DispatchService::Request& req,
                                 rosplan_dispatch_msgs::DispatchService::Response& res);

        void planCallback(const rosplan_dispatch_msgs::CompletePlan::ConstPtr& plan);
        void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
    };
}


#endif //PLANNING_NODE_XYZPLANDISPATCH_H
