//
// Created by xyz on 22-5-17.
//

#include "plan_diapatch/XYZPlanDispatch.h"


namespace planning_node {

    XYZPlanDispatch::XYZPlanDispatch(ros::NodeHandle& nh) : _nh(nh), plan_received(false) {

        _nh.param("action_dispatch_topic", action_dispatch_topic, std::string("action_dispatch"));
        _nh.param("action_feedback_topic", action_feedback_topic, std::string("action_feedback"));
        dispatch_pub = _nh.advertise<rosplan_dispatch_msgs::ActionDispatch>(action_dispatch_topic, 1, true);
        feedback_pub = _nh.advertise<rosplan_dispatch_msgs::ActionFeedback>(action_feedback_topic, 1, true);

        dispatch_server = _nh.advertiseService("dispatch_plan", &planning_node::XYZPlanDispatch::dispatchPlanService,
                                               this);
        feedback_subscriber = _nh.subscribe(action_feedback_topic, 1000,
                                            &planning_node::XYZPlanDispatch::feedbackCallback, this);

        std::string plan_topic;
        _nh.param("plan_topic", plan_topic, std::string("complete_plan"));
        plan_subscriber = _nh.subscribe(plan_topic, 100, &planning_node::XYZPlanDispatch::planCallback, this);
        ros_info("({}): Ready to receive.", ros::this_node::getName());

    }

    bool XYZPlanDispatch::dispatchPlanService(rosplan_dispatch_msgs::DispatchService::Request& req,
                                              rosplan_dispatch_msgs::DispatchService::Response& res) {
        ros::Rate loop_rate(10);
        while (ros::ok() && current_plan.plan.size() > current_action) {

//            // loop while dispatch is paused
//            while (ros::ok() && dispatch_paused) {
//                ros::spinOnce();
//                loop_rate.sleep();
//            }
//
//            // cancel plan
//            if(plan_cancelled) {
//                break;
//            }

            // get next action
            rosplan_dispatch_msgs::ActionDispatch currentMessage = current_plan.plan[current_action];


            std::string params = "(";
            for (size_t i = 0; i < currentMessage.parameters.size(); ++i) {
                if (i > 0) params += ", ";
                params += currentMessage.parameters[i].value;
            }
            params += ")";
            // dispatch action
            ros_info("KCL: ({}) Dispatching action [{}, {}{}]", ros::this_node::getName(), currentMessage.action_id,
                     currentMessage.name,
                     params);

            dispatch_pub.publish(currentMessage);
            // publish feedback (action dispatched)
            rosplan_dispatch_msgs::ActionFeedback fb;
            fb.action_id = currentMessage.action_id;
            fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_DISPATCHED_TO_GOAL_STATE;
            feedback_pub.publish(fb);


            // wait for action to complete
            while (ros::ok() && !action_completed[current_action]) {
                ros::spinOnce();
                loop_rate.sleep();
            }

            // get ready for next action
            current_action++;
            action_received[current_action] = false;
            action_completed[current_action] = false;

            // finish dispatch and replan
//            if (replan_requested) return false;
        }

        ros_info("({}) Dispatch complete.", ros::this_node::getName());
        return true;
    }

    void XYZPlanDispatch::planCallback(const rosplan_dispatch_msgs::CompletePlan::ConstPtr& plan) {
        ros_info("XYZPlanDispatch: Plan received.");
        plan_received = true;
        current_plan = *plan;
    }

    void XYZPlanDispatch::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
        // create error if the action is unrecognised
        ros_info("({}) Feedback received [{}, {}]", ros::this_node::getName(), msg->action_id, msg->status);
        if (current_action != (unsigned int) msg->action_id)
            ros_info("({}) Unexpected action ID: {}; current action: {}", ros::this_node::getName(),
                      msg->action_id, current_action);

        // action enabled
        if (!action_received[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_ENABLED)
            action_received[msg->action_id] = true;

        // action completed (successfuly)
        if (!action_completed[msg->action_id] &&
            msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE)
            action_completed[msg->action_id] = true;

        // action completed (failed)
        if (!action_completed[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_FAILED) {
//            replan_requested = true;
            action_completed[msg->action_id] = true;
        }
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xyz_plan_dispatch");
    ros::NodeHandle nh("~");

    planning_node::XYZPlanDispatch pd(nh);
    ros::spin();
    return 0;
}