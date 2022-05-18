//
// Created by xyz on 22-5-18.
//

#include "ActionInterface/GotoWaypoint.h"



namespace tiago_demo {

    bool GotoWaypointAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_gotowaypoint");


}