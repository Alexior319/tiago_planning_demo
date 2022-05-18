//
// Created by xyz on 22-5-18.
//

#ifndef ROS_PROJECT_GOTOWAYPOINT_H
#define ROS_PROJECT_GOTOWAYPOINT_H

#include <xyz_action_interface/XYZActionInterface.h>


namespace tiago_demo {
    class GotoWaypointAction : public planning_node::XYZActionInterface {


    public:
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;
    };
}

#endif //ROS_PROJECT_GOTOWAYPOINT_H
