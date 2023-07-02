//
// Created by xyz on 22-5-18.
//

#ifndef ROS_PROJECT_WATCHTABLE_H
#define ROS_PROJECT_WATCHTABLE_H

#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs//MoveBaseAction.h>

#include <xyz_action_interface/XYZActionInterface.h>


namespace tiago_demo {
    class WatchTableAction : public planning_node::XYZActionInterface {
    private:
        // to clear costmaps if move base gets stuck
        ros::ServiceClient clear_costmaps_client_;
        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle& nh_;
        /// waypoints reference frame
        std::string waypoint_frameid_;
        std::string wp_namespace_;


        bool wpIDtoPoseStamped(const std::string& wpID, geometry_msgs::PoseStamped& result);

    public:
        explicit WatchTableAction(ros::NodeHandle& nh);

        bool concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;

        bool isSensingAction() override { return true; };
    };
}

#endif //ROS_PROJECT_WATCHTABLE_H
