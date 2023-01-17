//
// Created by xyz on 2022/3/22.
//

#include <std_msgs/String.h>
#include "pddl_knowledge_base/pddl_planner.h"

namespace planning_node {
    pddl_planner::pddl_planner(ros::NodeHandle& nh, const unique_ptr<PDDLKnowledgeBase>& ptr) : kb_ptr(ptr), _nh(nh) {
        std::string plannerTopic = "planner_output";
        _nh.getParam("planner_topic", plannerTopic);

        plan_publisher = _nh.advertise<xyz_dispatch_msgs::CompletePlan>(plannerTopic, 1, true);
        planning_server = _nh.advertiseService("planning_server", &planning_node::pddl_planner::runPlanningServerDefault,
                                               this);
        online_server = _nh.advertiseService("online_planning", &planning_node::pddl_planner::onlinePlanningServer, this);

        ros_info("(XYZPlanner): Ready to receive.");
    }


    vector<ActionPtr>
    pddl_planner::planning(const planning_node::StatePtr& initial, const planning_node::StatePtr& goal) {
        queue<StatePtr> q;
        unordered_set<StatePtr> bfsSeen;
        q.push(initial);
        bfsSeen.emplace(initial);

        vector<ActionPtr> plan;

        int nodesNum = 0;
        bool planFound = false;
        while (!q.empty()) {
            if (planFound) break;
            for (int i = q.size(); i > 0; --i) {
                auto node = q.front();
                q.pop();

                if (node->contains(goal)) {
                    // Found plan
                    while (node->appliedAction) {
                        plan.emplace_back(node->appliedAction);
                        node = node->parentState;
                    }
                    reverse(plan.begin(), plan.end());
                    planFound = true;
                    break;
                }

                for (const auto& a: kb_ptr->allPossibleActions) {
                    if (node->canApply(a)) {
                        auto newState = node->apply(a);
                        ++nodesNum;
                        if (visited(bfsSeen, newState)) continue;

                        // run_info("From state: {}", node);
                        // run_info("Apply action: {}", a);
                        // run_info("To state: {}", newState);

                        q.emplace(newState);
                        bfsSeen.emplace(newState);
                    }
                }
            }
        }
        for (const auto& a : plan) {
            if (!a->conditionsShouldNotSeen.empty()) {
                sensing_actions.emplace(a->meta->name);
            }
        }
//        run_info("Expanded nodes: {}", nodesNum);
        return plan;
    }

    vector<ActionPtr>
    pddl_planner::backward(const planning_node::StatePtr& initial, const planning_node::StatePtr& goal) {
        queue<StatePtr> q;
        unordered_set<StatePtr> bfsSeen;
        q.push(goal);
        bfsSeen.emplace(goal);

        vector<ActionPtr> plan;

        int nodesNum = 0;

        bool planFound = false;
        while (!q.empty() && !planFound) {
            for (int i = q.size(); i > 0; --i) {
                auto node = q.front();
                q.pop();

                if (initial->contains(node)) {
                    // Found plan
                    while (node->appliedAction) {
                        plan.emplace_back(node->appliedAction);
                        node = node->parentState;
                    }
                    planFound = true;
                    break;
                }

                bool hasRevelant = false;
                for (const auto& a: kb_ptr->allPossibleActions) {
                    if (node->isRelevant(a)) {
                        auto newState = node->applyRelevant(a);
                        ++nodesNum;
                        if (visited(bfsSeen, newState)) continue;

                        hasRevelant = true;
#ifndef NDEBUG
//                        run_info("From state: {}", node);
//                        run_info("Apply action: {}", a);
//                        run_info("To state: {}", newState);
//                        run_info("Prev: {}", newState->apply(a));
//                        run_info("");
#endif
                        q.emplace(newState);
                        bfsSeen.emplace(newState);
                    }
                }

                if (!hasRevelant) {
                    run_info("End state: {}", node);
                }
            }
        }
        run_info("Expanded nodes: {}", nodesNum);
        return plan;
    }

    bool pddl_planner::visited(const unordered_set<StatePtr>& seen, const StatePtr& s) {
        return std::any_of(seen.begin(), seen.end(), [&s](const StatePtr& n) {
            return n->contains(s);
        });
    }

    vector<ActionPtr> pddl_planner::planningDFS(const StatePtr& initial, const StatePtr& goal) {
        unordered_set<StatePtr> seen;

        vector<ActionPtr> actionSeqs;

        function<void(const StatePtr&, const StatePtr&, vector<ActionPtr>&)> dfs = [&, this](const StatePtr& curr,
                                                                                             const StatePtr& target,
                                                                                             vector<ActionPtr>& seq) {
            if (curr->contains(target)) {
                if (actionSeqs.empty() || seq.size() < actionSeqs.size()) actionSeqs = seq;
                return;
            }
            for (const auto& a: this->kb_ptr->allPossibleActions) {
                if (curr->canApply(a)) {
                    auto newState = curr->apply(a);
                    bool valid = true;
                    if (visited(seen, newState)) {
                        continue;
                    }
//                    debug_info("Using action: {}", a);
//                    debug_info("state0: {}", curr);
//                    debug_info("state1: {}", newState);
                    seen.emplace(newState);
                    seq.push_back(a);
                    dfs(newState, target, seq);
                    seq.pop_back();
                }
            }
        };
        vector<shared_ptr<Action>> s;
        dfs(initial, goal, s);
        return actionSeqs;
    }

    bool pddl_planner::runPlanningServerDefault(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        run_info("Running planner server");
        xyz_dispatch_msgs::CompletePlan plan;

        auto state = make_shared<State>();
        for (const auto& fact : kb_ptr->getState()) {
            Predicate p;
            p.meta = kb_ptr->metaPredicates[fact.attribute_name];
            p.neg = fact.is_negative;
            for (const auto & [k, v] : fact.values) {
                p.parameters.push_back(v);
            }
            state->add(p);
        }
        if (state->contains(kb_ptr->goal_state)) {
            run_info("No need to plan.");
            return true;
        }

        auto planned_actions = planning(state, kb_ptr->goal_state);
        run_info("Got plan: {}", planned_actions);

        for (int action_id = 0; action_id < planned_actions.size(); ++action_id) {
            xyz_dispatch_msgs::ActionDispatch a;
            a.name = planned_actions[action_id]->meta->name;
            a.action_id = action_id;
            a.plan_id = plan_id;
            a.dispatch_time = 0.0f;
            a.duration = 0.0f;
            for (int i = 0; i < planned_actions[action_id]->paras.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.key = planned_actions[action_id]->meta->names[i];
                kv.value = planned_actions[action_id]->paras[i];
                a.parameters.emplace_back(kv);
            }
            plan.plan.emplace_back(a);
        }
        plan_publisher.publish(plan);
        ++plan_id;
        if (planned_actions.empty()) {
            ros_error("Problem unsolvable.");
        }
        return true;
    }

    bool pddl_planner::onlinePlanningServer(xyz_dispatch_msgs::OnlineDispatchService::Request &req,
                                            xyz_dispatch_msgs::OnlineDispatchService::Response &res) {
        if (action_idx >= got_plan.size() || (action_idx > 0 && sensing_actions.count(got_plan[action_idx-1].name))) {
            // re-plan
            got_plan.clear();
            action_idx = 0;

            auto state = make_shared<State>();
            for (const auto& fact : kb_ptr->getState()) {
                Predicate p;
                p.meta = kb_ptr->metaPredicates[fact.attribute_name];
                p.neg = fact.is_negative;
                for (const auto & [k, v] : fact.values) {
                    p.parameters.push_back(v);
                }
                state->add(p);
            }
            if (state->contains(kb_ptr->goal_state)) {
                run_info("No need to plan.");
            }

            auto plan = planning(state, kb_ptr->goal_state);
            run_info("Got plan: {}", plan);

            for (int action_id = 0; action_id < plan.size(); ++action_id) {
                xyz_dispatch_msgs::ActionDispatch a;
                a.name = plan[action_id]->meta->name;
                a.action_id = plan_id * 1000 + action_id;
                a.plan_id = plan_id;
                a.dispatch_time = 0.0f;
                a.duration = 0.0f;
                for (int i = 0; i < plan[action_id]->paras.size(); ++i) {
                    diagnostic_msgs::KeyValue kv;
                    kv.key = plan[action_id]->meta->names[i];
                    kv.value = plan[action_id]->paras[i];
                    a.parameters.emplace_back(kv);
                }
                got_plan.emplace_back(a);
            }
        }
        if (got_plan.empty()) {
            res.action.action_id = -1;
        } else {
            res.action = got_plan[action_idx++];
        }
        return true;
    }
}
