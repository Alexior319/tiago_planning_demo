//
// Created by xyz on 22-5-17.
//
#include <xyz_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <xyz_dispatch_msgs/ActionFeedback.h>
#include <xyz_knowledge_msgs/KnowledgeUpdateService.h>
#include <xyz_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include "xyz_action_interface/XYZActionInterface.h"

namespace planning_node {


    void XYZActionInterface::dispatchCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // check action name
        if (msg->name == "cancel_action") {
            action_cancelled = true;
            return;
        }
        ROS_INFO("(%s): received action (%s).", ros::this_node::getName().c_str(), msg->name.c_str());
        if (msg->name != params.name) return;
        ROS_INFO("KCL: (%s) action received", params.name.c_str());
        action_cancelled = false;

        // check parameters
        std::vector<bool> found(params.typed_parameters.size(), false);
        std::map<std::string, std::string> boundParameters;
        for (size_t j = 0; j < params.typed_parameters.size(); j++) {
            for (const auto& [paraName, paraValue]: msg->parameters) {
                if (params.typed_parameters[j].key == paraName) {
                    boundParameters[paraName] = paraValue;
                    found[j] = true;
                    break;
                }
            }
            if (!found[j]) {
                ROS_INFO("KCL: (%s) aborting action dispatch; malformed parameters, missing %s", params.name.c_str(),
                         params.typed_parameters[j].key.c_str());
                return;
            }
        }

        // send feedback (enabled)
        xyz_dispatch_msgs::ActionFeedback fb;
        fb.action_id = msg->action_id;
        fb.status = xyz_dispatch_msgs::ActionFeedback::ACTION_ENABLED;
        action_feedback_pub.publish(fb);

        // call concrete implementation
        action_success = concreteCallback(msg);
        ros::spinOnce();
        if (action_cancelled) {
            action_success = false;
            ROS_INFO("(%s) an old action that was cancelled is stopping now", params.name.c_str());
            return;
        }

        if (action_success) {

            ROS_INFO("KCL: (%s) action completed successfully", params.name.c_str());

            // update knowledge base
            xyz_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;

            if (isSensingAction()) {
//                for (size_t i = 0; i < op.at_end_add_effects.size(); ++i) {
//                    if (at_end_add_effects_results[i]) {
//                        xyz_knowledge_msgs::KnowledgeItem item;
//                        item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
//                        item.attribute_name = op.at_end_add_effects[i].name;
//                        item.values.clear();
//                        diagnostic_msgs::KeyValue pair;
//                        for (size_t j = 0; j < op.at_end_add_effects[i].typed_parameters.size(); j++) {
//                            pair.key = predicates[op.at_end_add_effects[i].name].typed_parameters[j].key;
//                            pair.value = boundParameters[op.at_end_add_effects[i].typed_parameters[j].key];
//                            item.values.push_back(pair);
//                        }
//                        updatePredSrv.request.knowledge.push_back(item);
//                        updatePredSrv.request.update_type.push_back(
//                                xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
//                    }
//                }
//                for (size_t i = 0; i < op.at_end_del_effects.size(); ++i) {
//                    if (at_end_del_effects_results[i]) {
//                        xyz_knowledge_msgs::KnowledgeItem item;
//                        item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
//                        item.attribute_name = op.at_end_del_effects[i].name;
//                        item.is_negative = true;
//                        item.values.clear();
//                        diagnostic_msgs::KeyValue pair;
//                        for (size_t j = 0; j < op.at_end_del_effects[i].typed_parameters.size(); j++) {
//                            pair.key = predicates[op.at_end_del_effects[i].name].typed_parameters[j].key;
//                            pair.value = boundParameters[op.at_end_del_effects[i].typed_parameters[j].key];
//                            item.values.push_back(pair);
//                        }
//                        updatePredSrv.request.knowledge.push_back(item);
//                        updatePredSrv.request.update_type.push_back(
//                                xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
//                    }
//                }
            } else {
                // simple END del effects
                for (auto& at_end_del_effect: op.at_end_del_effects) {
                    xyz_knowledge_msgs::KnowledgeItem item;
                    item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
                    item.attribute_name = at_end_del_effect.name;
                    item.is_negative = true;
                    item.values.clear();
                    diagnostic_msgs::KeyValue pair;
                    for (size_t j = 0; j < at_end_del_effect.typed_parameters.size(); j++) {
                        pair.key = predicates[at_end_del_effect.name].typed_parameters[j].key;
                        pair.value = boundParameters[at_end_del_effect.typed_parameters[j].key];
                        item.values.push_back(pair);
                    }
                    updatePredSrv.request.knowledge.push_back(item);
                    updatePredSrv.request.update_type.push_back(
                            xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                }

                // simple END add effects
                for (auto& at_end_add_effect: op.at_end_add_effects) {
                    xyz_knowledge_msgs::KnowledgeItem item;
                    item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
                    item.attribute_name = at_end_add_effect.name;
                    item.values.clear();
                    diagnostic_msgs::KeyValue pair;
                    for (size_t j = 0; j < at_end_add_effect.typed_parameters.size(); j++) {
                        pair.key = predicates[at_end_add_effect.name].typed_parameters[j].key;
                        pair.value = boundParameters[at_end_add_effect.typed_parameters[j].key];
                        item.values.push_back(pair);
                    }
                    updatePredSrv.request.knowledge.push_back(item);
                    updatePredSrv.request.update_type.push_back(
                            xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
                }
            }

            if (!updatePredSrv.request.knowledge.empty() && !update_knowledge_client.call(updatePredSrv))
                ROS_INFO("XYZ: (%s) failed to update PDDL model in knowledge base", params.name.c_str());

            // publish feedback (achieved)
            fb.status = xyz_dispatch_msgs::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE;
            fb.pause_dispatch = false;
            action_feedback_pub.publish(fb);
            ros::Duration(1.0).sleep();

//            if (isSensingAction()) {
//                ros::NodeHandle nh;
//                std::stringstream ss;
//                std_srvs::Empty emptySrv;
//
////                // pause dispatch
////                ss << pdn << "/pause_dispatch";
////                ros::service::waitForService(ss.str(), ros::Duration(2));
////                ros::ServiceClient srvClient1 = nh.serviceClient<std_srvs::Empty>(ss.str());
////                srvClient1.call(emptySrv);
//
//                // planning
//                ss.str("");
//                ss << kb << "/planning_server";
//                ros::service::waitForService(ss.str(), ros::Duration(2));
//                ros::ServiceClient srvClient2 = nh.serviceClient<std_srvs::Empty>(ss.str());
//                srvClient2.call(emptySrv);
//
//                // continue dispatch
//                ss.str("");
//                ss << pdn << "/continue_dispatch";
//                ros::service::waitForService(ss.str(), ros::Duration(2));
//                ros::ServiceClient srvClient3 = nh.serviceClient<std_srvs::Empty>(ss.str());
//                srvClient3.call(emptySrv);
//            }

        } else {

            // publish feedback (failed)
            fb.status = xyz_dispatch_msgs::ActionFeedback::ACTION_FAILED;
            action_feedback_pub.publish(fb);
        }
    }

    void XYZActionInterface::runActionInterface() {
        ros::NodeHandle nh("~");
        nh.getParam("action_name", params.name);
        nh.getParam("knowledge_base", kb);

        // fetch action params
        std::stringstream ss;
        ss << "/" << kb << "/domain/operator_details";
        ros::service::waitForService(ss.str(), ros::Duration(20));
        ros::ServiceClient client = nh.serviceClient<xyz_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
        xyz_knowledge_msgs::GetDomainOperatorDetailsService srv;
        srv.request.name = params.name;
        if (client.call(srv)) {
            params = srv.response.op.formula;
            op = srv.response.op;
        } else {
            ROS_ERROR("KCL: (XYZActionInterface) could not call Knowledge Base for operator details, %s",
                      params.name.c_str());
            return;
        }

        if (isSensingAction()) {
            at_end_add_effects_results.resize(op.at_end_add_effects.size(), false);
            at_end_del_effects_results.resize(op.at_end_del_effects.size(), false);
        }


        // collect predicates from operator description
        std::vector<std::string> predicateNames;

        // effects
        for (const auto& at_start_add_effect: op.at_start_add_effects)
            predicateNames.push_back(at_start_add_effect.name);


        for (const auto& at_start_del_effect: op.at_start_del_effects)
            predicateNames.push_back(at_start_del_effect.name);

        at_end_add_effects_results.resize(op.at_end_add_effects.size());
        for (const auto& at_end_add_effect: op.at_end_add_effects)
            predicateNames.push_back(at_end_add_effect.name);

        at_end_del_effects_results.resize(op.at_end_del_effects.size());
        for (const auto& at_end_del_effect: op.at_end_del_effects)
            predicateNames.push_back(at_end_del_effect.name);

        // simple conditions
        for (const auto& pit: op.at_start_simple_condition)
            predicateNames.push_back(pit.name);

        for (const auto& pit: op.over_all_simple_condition)
            predicateNames.push_back(pit.name);


        for (const auto& pit: op.at_end_simple_condition)
            predicateNames.push_back(pit.name);

        // negative conditions

        for (const auto& pit: op.at_start_neg_condition)
            predicateNames.push_back(pit.name);


        for (const auto& pit: op.over_all_neg_condition)
            predicateNames.push_back(pit.name);


        for (const auto& pit: op.at_end_neg_condition)
            predicateNames.push_back(pit.name);

        // fetch and store predicate details
        ss.str("");
        ss << "/" << kb << "/domain/predicate_details";
        ros::service::waitForService(ss.str(), ros::Duration(20));
        ros::ServiceClient predClient = nh.serviceClient<xyz_knowledge_msgs::GetDomainPredicateDetailsService>(
                ss.str());

        for (auto& predicateName: predicateNames) {
            if (predicates.find(predicateName) != predicates.end()) continue;
            if (predicateName == "=" || predicateName == ">" || predicateName == "<" || predicateName == ">=" ||
                predicateName == "<=")
                continue;
            xyz_knowledge_msgs::GetDomainPredicateDetailsService predSrv;
            predSrv.request.name = predicateName;
            if (predClient.call(predSrv)) {
                if (predSrv.response.is_sensed) {
                    ROS_ERROR("Sensed predicate is not supported.");
                } else {
                    predicates.insert(std::pair<std::string, xyz_knowledge_msgs::DomainFormula>(predicateName,
                                                                                                    predSrv.response.predicate));
                }
            } else {
                ROS_ERROR("(XYZActionInterface) could not call Knowledge Base for predicate details, %s",
                          params.name.c_str());
                return;
            }
        }

        // create the action feedback publisher
        std::string aft = "default_feedback_topic";
        nh.getParam("action_feedback_topic", aft);
        action_feedback_pub = nh.advertise<xyz_dispatch_msgs::ActionFeedback>(aft, 10, true);

        // knowledge interface
        ss.str("");
        ss << "/" << kb << "/update_array";
        update_knowledge_client = nh.serviceClient<xyz_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

        // listen for action dispatch
        std::string adt = "default_dispatch_topic";
        nh.getParam("action_dispatch_topic", adt);
        action_dispatch_sub = nh.subscribe(adt, 1000, &XYZActionInterface::dispatchCallback, this);

        nh.getParam("plan_dispatch_node", pdn);


        ROS_INFO("XYZ: (%s) ready to receive.", ros::this_node::getName().c_str());

        ros::spin();
    }


}