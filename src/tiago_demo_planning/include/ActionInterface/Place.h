//
// Created by xyz on 22-6-1.
//

#ifndef ROS_PROJECT_PICK_H
#define ROS_PROJECT_PICK_H

#include <xyz_action_interface/XYZActionInterface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tiago_demo_planning {
    class PlaceAction : public planning_node::XYZActionInterface {
    private:
        ros::ServiceClient set_model_state_client;
        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle& nh_;
        /// waypoints reference frame
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener{tfBuffer};

    public:
        explicit PlaceAction(ros::NodeHandle& nh);

        bool concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;

        bool isSensingAction() override { return false; };
    };
}

#endif //ROS_PROJECT_PICK_H
