//
// Created by xyz on 22-5-18.
//

#ifndef ROS_PROJECT_WATCHTABLE_H
#define ROS_PROJECT_WATCHTABLE_H

#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs//MoveBaseAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <xyz_action_interface/XYZActionInterface.h>


namespace tiago_demo {
    class LocateTwoObjsAction : public planning_node::XYZActionInterface {
    private:
        // to clear costmaps if move base gets stuck
        ros::ServiceClient clear_costmaps_client_;
        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle& nh_;
        /// waypoints reference frame
        std::string waypoint_frameid_;
        std::string wp_namespace_;

        void lookAround();

        bool exec(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        ros::ServiceServer my_service;

        ros::Publisher pub_head_topic;
        ros::Publisher pub_look_around_topic;
        trajectory_msgs::JointTrajectory head_goal;
        control_msgs::PointHeadActionGoal after_look_around_goal;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        ros::Subscriber aruco_sub, another_aruco_sub;
        void arucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void anotherArucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        bool arucoDetected = false;
        bool another_aruco_detected = false;

    public:
        explicit LocateTwoObjsAction(ros::NodeHandle& nh);

        bool concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) override;

        bool isSensingAction() override { return true; };
    };
}

#endif //ROS_PROJECT_WATCHTABLE_H
