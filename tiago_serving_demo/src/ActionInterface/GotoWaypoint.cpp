//
// Created by xyz on 22-5-18.
//

#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>

#include "ActionInterface/GotoWaypoint.h"



namespace tiago_demo {


    bool GotoWaypointAction::wpIDtoPoseStamped(const std::string& wpID, geometry_msgs::PoseStamped &result) {

        ros::NodeHandle nh;
        std::vector<double> wp;
        if(nh.hasParam(wp_namespace_ + "/" + wpID)) {
            if(nh.getParam(wp_namespace_ + "/" + wpID, wp)) {
                if(wp.size() == 3) {
                    result.header.frame_id = waypoint_frameid_;
                    result.pose.position.x = wp[0];
                    result.pose.position.y = wp[1];

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, wp[2]);
                    result.pose.orientation.x = q[0];
                    result.pose.orientation.y = q[1];
                    result.pose.orientation.z = q[2];
                    result.pose.orientation.w = q[3];

                    return true;
                }
                else {
                    ROS_ERROR("wp size must be equal to 3 : (x, y, and theta)");
                    return false;
                }
            }
        }
        else
            return false;

        return false;
    }

    bool GotoWaypointAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // get waypoint ID from action dispatch msg
        std::string wpID;
        bool found = false;
        // iterating over parameters (e.g. kenny, wp0, wp1)
        for(const auto& para : msg->parameters) {
            ROS_INFO("%s : %s", para.key.c_str(), para.value.c_str());
            // check their keys
            if("to" == para.key or "w1" == para.key) {
                // wp id found in msg params
                wpID = para.value;
                found = true;
            }
        }
        if(!found) {
            ROS_INFO("(%s) aborting action dispatch; [ GotoWaypoint ] action missing required parameter [ to ]", params.name.c_str());
            return false;
        }

        // get waypoint coordinates from its ID via query to parameter server
        geometry_msgs::PoseStamped pose;
        if(!wpIDtoPoseStamped(wpID, pose)) {
            ROS_ERROR("Waypoint not found in parameter server");
            return false;
        }

        ROS_INFO("(%s) waiting for move_base action server to start", params.name.c_str());
        action_client_.waitForServer();

        // dispatch MoveBase action
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = pose;
        action_client_.sendGoal(goal);

        // ros::ServiceClient look_around_client = nh_.serviceClient<std_srvs::Empty>("/look_around_interface/tiago_look_around");
        // std_srvs::Empty e;
        // look_around_client.call(e);
        ROS_INFO("(%s) waiting fot move_base to finish", params.name.c_str());
        bool finished_before_timeout = action_client_.waitForResult();
        if (finished_before_timeout) {

            actionlib::SimpleClientGoalState state = action_client_.getState();
            ROS_INFO("(%s) action finished: %s", params.name.c_str(), state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

                // publish feedback (achieved)
                return true;

            } else {

                // clear costmaps
                std_srvs::Empty emptySrv;
                clear_costmaps_client_.call(emptySrv);

                // publish feedback (failed)
                return false;
            }
        } else {
            // timed out (failed)
            action_client_.cancelAllGoals();
            ROS_INFO("(%s) action timed out", params.name.c_str());
            return false;
        }

        return false;
    }

    GotoWaypointAction::GotoWaypointAction(ros::NodeHandle& nh, const std::string& actionServer) : nh_(nh), action_client_(actionServer, true) {
        // get waypoints reference frame from param server
        nh_.param<std::string>("waypoint_frameid", waypoint_frameid_, "map");
        nh_.param<std::string>("wp_namespace", wp_namespace_, "/rosplan_demo_waypoints/wp");

        // setup a move base clear costmap client (to be able to send clear costmap requests later on)
        clear_costmaps_client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }

    bool GotoWaypointAction::isSensingAction() {
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_goto_waypoint");
    ros::NodeHandle nh("~");
    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/move_base"));

    tiago_demo::GotoWaypointAction rpmb(nh, actionserver);
    ROS_INFO("(%s): ready to receive.", ros::this_node::getName().c_str());


    rpmb.runActionInterface();
    return 0;
}