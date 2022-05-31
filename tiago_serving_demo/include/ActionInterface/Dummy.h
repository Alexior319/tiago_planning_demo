//
// Created by xyz on 22-5-19.
//

#ifndef ROS_PROJECT_DUMMY_H
#define ROS_PROJECT_DUMMY_H

#include "xyz_action_interface/XYZActionInterface.h"

namespace tiago_demo {
    class DummyActionInterface : public planning_node::XYZActionInterface {
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;
        bool isSensingAction() override;
    };
}


#endif //ROS_PROJECT_DUMMY_H
