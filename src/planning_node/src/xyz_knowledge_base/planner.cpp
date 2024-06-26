//
// Created by xyz on 2022/3/22.
//

#include <std_msgs/String.h>
#include "xyz_knowledge_base/planner.h"

namespace planning_node {
    planner::planner(ros::NodeHandle& nh, const unique_ptr<KCL_rosplan::XYZKnowledgeBase>& ptr) : kb_ptr(ptr), _nh(nh) {
        std::string plannerTopic = "planner_output";
        _nh.getParam("planner_topic", plannerTopic);

        plan_publisher = _nh.advertise<xyz_dispatch_msgs::CompletePlan>(plannerTopic, 1, true);
        planning_server = _nh.advertiseService("planning_server", &planning_node::planner::runPlanningServerDefault,
                                               this);

        run_info("(XYZPlanner): Ready to receive.");
    }


    vector<ActionPtr>
    planner::planning(const planning_node::StatePtr& initial, const planning_node::StatePtr& goal) {
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

//                        run_info("From state: {}", node);
//                        run_info("Apply action: {}", a);
//                        run_info("To state: {}", newState);

                        q.emplace(newState);
                        bfsSeen.emplace(newState);
                    }
                }
            }
        }
//        run_info("Expanded nodes: {}", nodesNum);
        return plan;
    }

    vector<ActionPtr>
    planner::backward(const planning_node::StatePtr& initial, const planning_node::StatePtr& goal) {
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

    bool planner::visited(const unordered_set<StatePtr>& seen, const StatePtr& s) {
        return std::any_of(seen.begin(), seen.end(), [&s](const StatePtr& n) {
            return n->contains(s);
        });
    }

    vector<ActionPtr> planner::planningDFS(const StatePtr& initial, const StatePtr& goal) {
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

    bool planner::runPlanningServerDefault(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
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

        auto got_plan = planning(state, kb_ptr->goal_state);
        run_info("Got plan: {}", got_plan);

        for (int action_id = 0; action_id < got_plan.size(); ++action_id) {
            xyz_dispatch_msgs::ActionDispatch a;
            a.name = got_plan[action_id]->meta->name;
            a.action_id = action_id;
            a.plan_id = plan_id;
            a.dispatch_time = 0.0f;
            a.duration = 0.0f;
            for (int i = 0; i < got_plan[action_id]->paras.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.key = got_plan[action_id]->meta->names[i];
                kv.value = got_plan[action_id]->paras[i];
                a.parameters.emplace_back(kv);
            }
            plan.plan.emplace_back(a);
        }
        plan_publisher.publish(plan);
        ++plan_id;
        if (got_plan.empty()) {
            ros_error("Problem unsolvable.");
        }
        return true;
    }
}
