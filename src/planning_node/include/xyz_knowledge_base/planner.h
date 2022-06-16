//
// Created by xyz on 2022/3/22.
//

#ifndef PLANNING_NODE_PLANNER_H
#define PLANNING_NODE_PLANNER_H


#include <string>
#include <vector>
#include <string_view>
#include <queue>
#include <memory>


#include <xyz_dispatch_msgs/CompletePlan.h>

#include <xyz_knowledge_base/data_types.h>
#include <xyz_knowledge_base/parser.h>
#include "XYZKnowledgeBase.h"
#include "common.h"

namespace planning_node {
    using namespace std;


    class planner {
    public:
        // explicit planner(const parser& parser) : _parser(parser) {}
        explicit planner(ros::NodeHandle& nh, const unique_ptr<KCL_rosplan::XYZKnowledgeBase>& ptr);
        vector<ActionPtr> backward(const StatePtr& initial, const StatePtr& goal);
        vector<ActionPtr> planning(const StatePtr& initial, const StatePtr& goal);
        vector<ActionPtr> planningDFS(const StatePtr& initial, const StatePtr& goal);

        bool runPlanningServerDefault(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    private:
        // const parser& _parser;
        const unique_ptr<KCL_rosplan::XYZKnowledgeBase>& kb_ptr;
        ros::NodeHandle& _nh;



        ros::Publisher plan_publisher;
        ros::ServiceServer planning_server;

        int plan_id = 0;

        static bool visited(const unordered_set<StatePtr>& seen, const StatePtr& s);
    };
}



#endif //PLANNING_NODE_PLANNER_H
