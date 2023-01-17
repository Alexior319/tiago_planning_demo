//
// Created by xyz on 22-6-1.
//

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

#include "ActionInterface/Pick.h"

namespace tiago_demo_planning {

    PickAction::PickAction(ros::NodeHandle& nh) : nh_(nh) {

    }

    bool
    PickAction::concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ros::ServiceClient getModelStateClient = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        gazebo_msgs::GetModelState srvMsg;
        srvMsg.request.model_name = msg->parameters[0].value;
        srvMsg.request.relative_entity_name = "map";

        getModelStateClient.call(srvMsg);

        ros::ServiceClient setModelStateClient = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        gazebo_msgs::SetModelState setMsg;
        setMsg.request.model_state.model_name = msg->parameters[1].value;
        setMsg.request.model_state.pose = srvMsg.response.pose;
        setMsg.request.model_state.pose.position.z = 1.2f;
        setMsg.request.model_state.reference_frame = "map";
        setMsg.request.model_state.twist = srvMsg.response.twist;
        setModelStateClient.call(setMsg);
        return true;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_pick");
    ros::NodeHandle nh("~");
    tiago_demo_planning::PickAction rpmb(nh);
    ROS_INFO("(%s): ready to receive.", ros::this_node::getName().c_str());


    rpmb.runActionInterface();
    return 0;
}