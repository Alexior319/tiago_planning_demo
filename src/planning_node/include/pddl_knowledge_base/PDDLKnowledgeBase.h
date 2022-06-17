//
// Created by xyz on 22-6-11.
//

#ifndef ROS_PROJECT_PDDLKNOWLEDGEBASE_H
#define ROS_PROJECT_PDDLKNOWLEDGEBASE_H

#include <ros/ros.h>

#include "KB.h"
#include "xyz_knowledge_base/KnowledgeComparitor.h"
#include "PDDLDomainParser.h"
#include "PDDLProblemParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"

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

namespace planning_node {
    class PDDLKnowledgeBase : public KCL_rosplan::KnowledgeBase {
    private:
        /* parsing domain using VAL */
        PDDLDomainParser domain_parser;

        /* initial state from problem file using VAL */
        PDDLProblemParser problem_parser;
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


#endif //ROS_PROJECT_PDDLKNOWLEDGEBASE_H
