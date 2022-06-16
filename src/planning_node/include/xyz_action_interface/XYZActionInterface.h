//
// Created by xyz on 22-5-17.
//

#ifndef PLANNING_NODE_XYZACTIONINTERFACE_H
#define PLANNING_NODE_XYZACTIONINTERFACE_H

#include <ros/ros.h>
#include <xyz_dispatch_msgs/ActionDispatch.h>
#include <xyz_knowledge_msgs/DomainFormula.h>
#include <xyz_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <unordered_map>
#include <std_srvs/Empty.h>

namespace planning_node {
    class XYZActionInterface {
    protected:
        std::unordered_map<std::string, xyz_knowledge_msgs::DomainFormula> predicates;

        xyz_knowledge_msgs::DomainFormula params;
        xyz_knowledge_msgs::DomainOperator op;
        std::vector<bool> at_end_add_effects_results, at_end_del_effects_results;

        ros::Publisher action_feedback_pub;
        ros::ServiceClient update_knowledge_client;
        ros::Subscriber action_dispatch_sub;
        bool action_cancelled{false};
        bool action_success{false};

        std::string pdn, kb;
    public:
        XYZActionInterface() = default;

        void runActionInterface();

        /* listen to and process action_dispatch topic */
        void dispatchCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg);

        virtual bool concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) = 0;
        virtual bool isSensingAction() = 0;
    };
}

#endif //PLANNING_NODE_XYZACTIONINTERFACE_H
