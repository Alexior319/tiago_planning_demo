//
// Created by xyz on 22-5-13.
//

#ifndef PLANNING_NODE_XYZKNOWLEDGEBASE_H
#define PLANNING_NODE_XYZKNOWLEDGEBASE_H

#include "KnowledgeBase.h"
#include "data_types.h"
#include "parser.h"
#include <unordered_map>

namespace KCL_rosplan {

using namespace planning_node;

class XYZKnowledgeBase : public KnowledgeBase {
  private:
    /* parsing domain using VAL */
    // PDDLDomainParser domain_parser;
    planning_node::parser domain_parser;
    std::string domain_name;
    ros::ServiceServer stateServer6;
  public:

    planning_node::StatePtr current_state, initial_state, goal_state;

    unordered_map<Type, unordered_set<string>> allObjects;
    unordered_map<string, Type> objTypes;
    vector<string> types;
    vector<shared_ptr<Action>> allPossibleActions;
    unordered_map<string, shared_ptr<MetaAction>> metaActions;       // actionName -> ptrMetaAction
    unordered_map<string, shared_ptr<MetaPredicate>> metaPredicates; // predName -> ptrMetaPredicate;

    explicit XYZKnowledgeBase(ros::NodeHandle &n) : KnowledgeBase(n){
        stateServer6 = n.advertiseService("state/current", &XYZKnowledgeBase::getCurrent, this);
    };
    ~XYZKnowledgeBase() = default;

    /* parse domain and probelm files */
    void parseDomain(const std::string &domain_file_path, const std::string &_domain_name) override;

    /* add the initial state to the knowledge base */
    void addInitialState() override;
    void addConstants() override{};

    bool getGoals(xyz_knowledge_msgs::GetAttributeService::Request &req, xyz_knowledge_msgs::GetAttributeService::Response &res) override;
    bool getCurrent(xyz_knowledge_msgs::GetAttributeService::Request &req, xyz_knowledge_msgs::GetAttributeService::Response &res);

    /* service methods for fetching the domain details */
    bool getDomainName(xyz_knowledge_msgs::GetDomainNameService::Request &req, xyz_knowledge_msgs::GetDomainNameService::Response &res) override;
    bool getTypes(xyz_knowledge_msgs::GetDomainTypeService::Request &req, xyz_knowledge_msgs::GetDomainTypeService::Response &res) override;
    bool getPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request &req, xyz_knowledge_msgs::GetDomainAttributeService::Response &res) override;
    bool getFunctionPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request &req, xyz_knowledge_msgs::GetDomainAttributeService::Response &res) override{};
    bool getOperators(xyz_knowledge_msgs::GetDomainOperatorService::Request &req, xyz_knowledge_msgs::GetDomainOperatorService::Response &res) override;
    bool getOperatorDetails(xyz_knowledge_msgs::GetDomainOperatorDetailsService::Request &req, xyz_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) override;
    bool getPredicateDetails(xyz_knowledge_msgs::GetDomainPredicateDetailsService::Request &req, xyz_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) override;

    const vector<xyz_knowledge_msgs::KnowledgeItem>& getState() const;
};
} // namespace KCL_rosplan

#endif // PLANNING_NODE_XYZKNOWLEDGEBASE_H
