//
// Created by xyz on 22-5-17.
//

#ifndef PLANNING_NODE_XYZACTIONINTERFACE_H
#define PLANNING_NODE_XYZACTIONINTERFACE_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>

namespace planning_node {
    class XYZActionInterface {
    protected:
        std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates;

        rosplan_knowledge_msgs::DomainFormula params;
        rosplan_knowledge_msgs::DomainOperator op;

        ros::Publisher action_feedback_pub;
        ros::ServiceClient update_knowledge_client;
        bool action_cancelled{false}, action_success{false};
    public:
        void runActionInterface();

        /* listen to and process action_dispatch topic */
        void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

        virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) = 0;
    };
}

#endif //PLANNING_NODE_XYZACTIONINTERFACE_H
