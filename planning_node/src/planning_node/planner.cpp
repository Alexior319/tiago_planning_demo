//
// Created by xyz on 2022/3/22.
//

#include "planning_node/planner.h"

namespace planning_node {

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
                    while(node->appliedAction) {
                        plan.emplace_back(node->appliedAction);
                        node = node->parentState;
                    }
                    planFound = true;
                    break;
                }

                for (const auto& a : _parser.allPossibleActions) {
                    if (node->canApply(a)) {
                        auto newState = node->apply(a);
                        ++nodesNum;
                        if (visited(bfsSeen, newState)) continue;

                        ros_info("From state: {}", node);
                        ros_info("Apply action: {}", a);
                        ros_info("To state: {}", newState);

                        q.emplace(newState);
                        bfsSeen.emplace(newState);
                    }
                }
            }
        }
        reverse(plan.begin(), plan.end());
        ros_info("Expanded nodes: {}", nodesNum);
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
                    while(node->appliedAction) {
                        plan.emplace_back(node->appliedAction);
                        node = node->parentState;
                    }
                    planFound = true;
                    break;
                }

                bool hasRevelant = false;
                for (const auto& a : _parser.allPossibleActions) {
                    if (node->isRelevant(a)) {
                        auto newState = node->applyRelevant(a);
                        ++nodesNum;
                        if (visited(bfsSeen, newState)) continue;

                        hasRevelant = true;
#ifndef NDEBUG
//                        ros_info("From state: {}", node);
//                        ros_info("Apply action: {}", a);
//                        ros_info("To state: {}", newState);
//                        ros_info("Prev: {}", newState->apply(a));
//                        ros_info("");
#endif
                        q.emplace(newState);
                        bfsSeen.emplace(newState);
                    }
                }

                if(!hasRevelant) {
                    ros_info("End state: {}", node);
                }
            }
        }
        ros_info("Expanded nodes: {}", nodesNum);
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

        function<void(const StatePtr&, const StatePtr&, vector<ActionPtr>&)> dfs = [&, this](const StatePtr& curr, const StatePtr& target, vector<ActionPtr>& seq) {
            if (curr->contains(target)) {
                if (actionSeqs.empty() || seq.size() < actionSeqs.size()) actionSeqs = seq;
                return;
            }
            for (const auto& a : this->_parser.allPossibleActions) {
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
        dfs(_parser.initial, _parser.goal, s);
        return actionSeqs;
    }
}
