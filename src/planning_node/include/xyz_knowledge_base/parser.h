//
// Created by xyz on 2022/3/22.
//

#ifndef PLANNING_NODE_PARSER_H
#define PLANNING_NODE_PARSER_H

#include <string>
#include <memory>
#include <unordered_map>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <fmt/format.h>

#include <xyz_knowledge_base/data_types.h>
#include "common.h"

using namespace std;
namespace planning_node {
    // return initial state
    // goal state
    // predicates
    // types
    // actions



    class parser {
    private:
        // XmlRpc::XmlRpcValue planning_data;
        void backtrack(vector<string>& paras, const shared_ptr<MetaAction>& meta);
    public:
        // explicit parser(const ros::NodeHandle& nh, const string& key = "planning");
        unordered_map<Type, unordered_set<string>> allObjects;
        unordered_map<string, Type> objTypes;
        string data;
        shared_ptr<State> initial, goal;
        vector<string> types;
        vector<shared_ptr<Action>> allPossibleActions;
        unordered_map<string, shared_ptr<MetaAction>> metaActions; // actionName -> ptrMetaAction
        unordered_map<string, shared_ptr<MetaPredicate>> metaPredicates; // predName -> ptrMetaPredicate;

        void parseDomain(const XmlRpc::XmlRpcValue& domain_name);
        void parseDomain(const ros::NodeHandle& nh, const string& domain_name = "planning");

        void parseMetaAction(const XmlRpc::XmlRpcValue &v) noexcept;
        Predicate parsePredicate(const string& s, bool output_err = true) noexcept;
        Predicate parseStatePredicate(const string& s, bool output_err = true) noexcept;
        static pair<string, vector<string>> parsePredicatePattern(const string& s, bool output_err = true) noexcept;
        static bool startsWith(const string& s, const string& prefix) noexcept;

        void generatePossibleActions() noexcept;

        void parseMetaPredicates(const XmlRpc::XmlRpcValue& v) noexcept;

    };
}



#endif //PLANNING_NODE_PARSER_H
