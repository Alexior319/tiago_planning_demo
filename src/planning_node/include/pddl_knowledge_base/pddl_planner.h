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
#include <xyz_dispatch_msgs/OnlineDispatchService.h>

#include <xyz_knowledge_base/data_types.h>
#include <xyz_knowledge_base/parser.h>
#include "pddl_knowledge_base/PDDLKnowledgeBase.h"
#include "common.h"

namespace planning_node {
    using namespace std;


    class pddl_planner {
    public:
        // explicit planner(const parser& parser) : _parser(parser) {}
        explicit pddl_planner(ros::NodeHandle& nh, const unique_ptr<PDDLKnowledgeBase>& ptr);
        vector<ActionPtr> backward(const StatePtr& initial, const StatePtr& goal);
        vector<ActionPtr> planning(const StatePtr& initial, const StatePtr& goal);
        vector<ActionPtr> planningDFS(const StatePtr& initial, const StatePtr& goal);

        bool runPlanningServerDefault(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool onlinePlanningServer(xyz_dispatch_msgs::OnlineDispatchService::Request& req, xyz_dispatch_msgs::OnlineDispatchService::Response& res);
        int total_planning_time = 0;
        ros::Publisher planning_time_pub;

    private:
        // const parser& _parser;
        const unique_ptr<PDDLKnowledgeBase>& kb_ptr;
        ros::NodeHandle& _nh;



        ros::Publisher plan_publisher;
        ros::Publisher partial_plan_pub;
        ros::ServiceServer planning_server;
        ros::ServiceServer online_server;
        std::vector<xyz_dispatch_msgs::ActionDispatch> got_plan;
        int action_idx = 0;
        unordered_set<std::string> sensing_actions;

        int plan_id = 0;

        static bool visited(const unordered_set<StatePtr>& seen, const StatePtr& s);
    };
}



#endif //PLANNING_NODE_PLANNER_H
