//
// Created by xyz on 2022/3/22.
//

#ifndef PLANNING_NODE_PLANNER_H
#define PLANNING_NODE_PLANNER_H


#include <string>
#include <vector>
#include <string_view>
#include <queue>


#include <xyz_knowledge_base/data_types.h>
#include <xyz_knowledge_base/parser.h>
#include "common.h"

namespace planning_node {
    using namespace std;


    class planner {
    public:
        explicit planner(const parser& p) : _parser(p) {}

        vector<ActionPtr> backward(const StatePtr& initial, const StatePtr& goal);
        vector<ActionPtr> planning(const StatePtr& initial, const StatePtr& goal);
        vector<ActionPtr> planningDFS(const StatePtr& initial, const StatePtr& goal);

    private:
        const parser& _parser;
        bool visited(const unordered_set<StatePtr>& seen, const StatePtr& s);
    };
}



#endif //PLANNING_NODE_PLANNER_H
