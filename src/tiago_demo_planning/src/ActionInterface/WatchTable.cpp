//
// Created by xyz on 22-5-31.
//

#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ActionInterface/WatchTable.h"
#include "xyz_knowledge_msgs/KnowledgeItem.h"
#include "xyz_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "xyz_knowledge_msgs/KnowledgeUpdateService.h"


namespace tiago_demo {

    bool WatchTableAction::concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        arucoDetected = false;
        lookAround();
        ROS_INFO("Is aruco detected?: %d", arucoDetected);


        xyz_knowledge_msgs::KnowledgeItem item;
        item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
        item.attribute_name = "ball-at";
        item.is_negative = !arucoDetected;
        item.values.clear();
        diagnostic_msgs::KeyValue kvb, kvwp;
        kvb = msg->parameters[1];
        kvwp = msg->parameters[2];
        item.values.emplace_back(kvb);
        item.values.emplace_back(kvwp);
        xyz_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;
        updatePredSrv.request.knowledge.push_back(item);
        updatePredSrv.request.update_type.push_back(
            xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE
        );
        if (!updatePredSrv.request.knowledge.empty()
            && !update_knowledge_client.call(updatePredSrv)) {
            ROS_INFO("XYZ: (%s) failed to update PDDL model in knowledge base", params.name.c_str());
        }
        return true;
    }

    WatchTableAction::WatchTableAction(ros::NodeHandle& nh) : nh_(nh) {
        pub_head_topic = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 5);
        pub_look_around_topic = nh_.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal",
                                                                                 1);
        head_goal.joint_names.emplace_back("head_1_joint");
        head_goal.joint_names.emplace_back("head_2_joint");
        aruco_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, &WatchTableAction::arucoCallback, this);
        my_service = nh_.advertiseService("do", &WatchTableAction::exec, this);
    }

    void WatchTableAction::lookAround() {
        ROS_INFO("Looking around");
        head_goal.points.resize(4);

        for (int i = 0; i < 4; ++i) {
            head_goal.points[i].positions.resize(2);
            head_goal.points[i].velocities.resize(2);
        }

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 2; ++j) {
                head_goal.points[i].velocities[j] = 0.0;
            }
        }
        // first goal
        head_goal.points[0].positions[0] = 0.0;
        head_goal.points[0].positions[1] = -0.7;
        head_goal.points[0].time_from_start = ros::Duration(2.0);

        // second goal
        head_goal.points[1].positions[0] = -1.2;
        head_goal.points[1].positions[1] = -0.7;
        head_goal.points[1].time_from_start = ros::Duration(6.0);

        // third goal
        head_goal.points[2].positions[0] = 1.2;
        head_goal.points[2].positions[1] = -0.7;
        head_goal.points[2].time_from_start = ros::Duration(14.0);

        // fouth goal
        head_goal.points[3].positions[0] = 0.0;
        head_goal.points[3].positions[1] = -0.7;
        head_goal.points[3].time_from_start = ros::Duration(18.0);

        head_goal.header.stamp = ros::Time::now();
        pub_head_topic.publish(head_goal);
        ros::Rate loop(10);
        for (int i = 0; i < 18 * 10; ++i) {
            loop.sleep();
            ros::spinOnce();
        }
        ROS_INFO("Looking around done.");
    }

    void WatchTableAction::arucoCallback(const geometry_msgs::PoseStamped_<std::allocator<void>>::ConstPtr &msg) {
        arucoDetected = true;
    }

    bool WatchTableAction::exec(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        auto msg = boost::make_shared<xyz_dispatch_msgs::ActionDispatch>();
        concreteCallback(msg);
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "action_watch_table");
    ros::NodeHandle nh("~");
    std::string actionserver;

    tiago_demo::WatchTableAction rpmb(nh);
    ROS_INFO("(%s): ready to receive.", ros::this_node::getName().c_str());


    rpmb.runActionInterface();
    return 0;
}