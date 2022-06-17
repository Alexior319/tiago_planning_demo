//
// Created by xyz on 22-6-1.
//

#ifndef ROS_PROJECT_PICK_H
#define ROS_PROJECT_PICK_H

#include <xyz_action_interface/XYZActionInterface.h>

namespace tiago_serving_demo {
    class PickAction : public planning_node::XYZActionInterface {
    private:
        ros::ServiceClient set_model_state_client;
        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle& nh_;
        /// waypoints reference frame

    public:
        explicit PickAction(ros::NodeHandle& nh);

        bool concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;

        bool isSensingAction() override { return false; };
    };
}

#endif //ROS_PROJECT_PICK_H
