//
// Created by xyz on 22-6-11.
//

#ifndef ROS_PROJECT_PDDLKNOWLEDGEBASE_H
#define ROS_PROJECT_PDDLKNOWLEDGEBASE_H

#include <ros/ros.h>

#include "PDDLParser.h"

#include "xyz_knowledge_base/KnowledgeBase.h"

#include "xyz_knowledge_msgs/KnowledgeUpdateService.h"
#include "xyz_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "xyz_knowledge_msgs/KnowledgeQueryService.h"
#include "xyz_knowledge_msgs/GetDomainNameService.h"
#include "xyz_knowledge_msgs/GetDomainTypeService.h"
#include "xyz_knowledge_msgs/GetDomainAttributeService.h"
#include "xyz_knowledge_msgs/GetDomainOperatorService.h"
#include "xyz_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "xyz_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "xyz_knowledge_msgs/DomainFormula.h"
#include "xyz_knowledge_msgs/GetAttributeService.h"
#include "xyz_knowledge_msgs/GetInstanceService.h"
#include "xyz_knowledge_msgs/GetMetricService.h"
#include "xyz_knowledge_msgs/KnowledgeItem.h"

#include "common.h"
#include "xyz_knowledge_base/data_types.h"

namespace planning_node {
    class PDDLKnowledgeBase : public KCL_rosplan::KnowledgeBase {
    private:
        PDDLParser parser;

        planning_node::StatePtr current_state, initial_state, goal_state;

        unordered_map<Type, unordered_set<string>> allObjects;
        unordered_map<string, Type> objTypes;
        vector<string> types;
        vector<shared_ptr<Action>> allPossibleActions;
        unordered_map<string, shared_ptr<MetaAction>> metaActions;       // actionName -> ptrMetaAction
        unordered_map<string, shared_ptr<MetaPredicate>> metaPredicates; // predName -> ptrMetaPredicate;

    public:
        explicit PDDLKnowledgeBase(ros::NodeHandle& n) : KnowledgeBase(n) {};

        ~PDDLKnowledgeBase() = default;

        /* parse domain and probelm files */
        void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        /* add the initial state to the knowledge base */
        void addInitialState() override;

        void addConstants() override;

        /* service methods for fetching the domain details */
        bool getDomainName(xyz_knowledge_msgs::GetDomainNameService::Request& req,
                           xyz_knowledge_msgs::GetDomainNameService::Response& res) override;

        bool getTypes(xyz_knowledge_msgs::GetDomainTypeService::Request& req,
                      xyz_knowledge_msgs::GetDomainTypeService::Response& res) override;

        bool getPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request& req,
                           xyz_knowledge_msgs::GetDomainAttributeService::Response& res) override;

        bool getFunctionPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request& req,
                                   xyz_knowledge_msgs::GetDomainAttributeService::Response& res) override;

        bool getOperators(xyz_knowledge_msgs::GetDomainOperatorService::Request& req,
                          xyz_knowledge_msgs::GetDomainOperatorService::Response& res) override;

        bool getOperatorDetails(xyz_knowledge_msgs::GetDomainOperatorDetailsService::Request& req,
                                xyz_knowledge_msgs::GetDomainOperatorDetailsService::Response& res) override;

        bool getPredicateDetails(xyz_knowledge_msgs::GetDomainPredicateDetailsService::Request& req,
                                 xyz_knowledge_msgs::GetDomainPredicateDetailsService::Response& res) override;
    };
}

namespace fmt {
    template<>
    struct formatter<Predicate> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const Predicate& predicate, FormatContext& ctx)
        -> decltype(ctx.out()) {
            std::stringstream ss;
            ss << predicate;
            return format_to(ctx.out(), "{}", ss.str());
        }
    };

    template<>
    struct formatter<Predicate*> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const Predicate* predicate, FormatContext& ctx)
        -> decltype(ctx.out()) {
            std::stringstream ss;
            ss << *predicate;
            return format_to(ctx.out(), "{}", ss.str());
        }
    };

    template<>
    struct formatter<Action> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const Action& action, FormatContext& ctx)
        -> decltype(ctx.out()) {
            std::stringstream ss;
            ss << action;
            return format_to(ctx.out(), "{}", ss.str());
        }
    };

    template<>
    struct formatter<LiteralState> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const LiteralState& state, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", to_string(state));
        }
    };
} // namespace fmt



#endif //ROS_PROJECT_PDDLKNOWLEDGEBASE_H
