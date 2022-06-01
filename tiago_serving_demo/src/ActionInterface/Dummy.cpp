//
// Created by xyz on 22-5-19.
//

#include <ActionInterface/Dummy.h>

bool tiago_demo::DummyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    ros::NodeHandle nh("~");
    // set action name
    std::string action_name;
    nh.getParam("action_name", action_name);
    ROS_INFO("(ActionInterface: %s): concreteCallback", action_name.c_str());
    return true;
}

bool tiago_demo::DummyActionInterface::isSensingAction() {
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummyaction_node");
    tiago_demo::DummyActionInterface dsi;

    ROS_INFO("(%s): ready to receive.", ros::this_node::getName().c_str());
    dsi.runActionInterface();
    return 0;
}