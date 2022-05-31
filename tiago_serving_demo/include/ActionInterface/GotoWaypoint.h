//
// Created by xyz on 22-5-18.
//

#ifndef ROS_PROJECT_GOTOWAYPOINT_H
#define ROS_PROJECT_GOTOWAYPOINT_H

#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs//MoveBaseAction.h>

#include <xyz_action_interface/XYZActionInterface.h>
//#include <xyz_action_interface/RPActionInterface.h>


namespace tiago_demo {
    class GotoWaypointAction : public planning_node::XYZActionInterface {
    private:
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
        // to clear costmaps if move base gets stuck
        ros::ServiceClient clear_costmaps_client_;
        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle& nh_;
        /// waypoints reference frame
        std::string waypoint_frameid_;
        std::string wp_namespace_;



        bool wpIDtoPoseStamped(const std::string& wpID, geometry_msgs::PoseStamped &result);


    public:
        explicit GotoWaypointAction(ros::NodeHandle& nh, const std::string& actionServer);

        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;
        bool isSensingAction() override;
    };
}

#endif //ROS_PROJECT_GOTOWAYPOINT_H
