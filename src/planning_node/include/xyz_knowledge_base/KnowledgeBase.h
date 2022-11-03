#ifndef KCL_knowledgebase
#define KCL_knowledgebase

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "std_srvs/Empty.h"

#include "xyz_knowledge_msgs/KnowledgeQueryService.h"
#include "xyz_knowledge_msgs/KnowledgeUpdateService.h"
#include "xyz_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "xyz_knowledge_msgs/SetNamedBool.h"

#include "xyz_knowledge_msgs/DomainFormula.h"
#include "xyz_knowledge_msgs/GetDomainAttributeService.h"
#include "xyz_knowledge_msgs/GetDomainNameService.h"
#include "xyz_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "xyz_knowledge_msgs/GetDomainOperatorService.h"
#include "xyz_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "xyz_knowledge_msgs/GetDomainTypeService.h"
#include "xyz_knowledge_msgs/StatusUpdate.h"

#include "xyz_knowledge_msgs/GetAttributeService.h"
#include "xyz_knowledge_msgs/GetInstanceService.h"
#include "xyz_knowledge_msgs/GetMetricService.h"
#include "xyz_knowledge_msgs/KnowledgeItem.h"

#include "KnowledgeComparitor.h"

namespace KCL_rosplan {

class KnowledgeBase {
  private:
    ros::ServiceServer domainServer1; // getDomainName
    ros::ServiceServer domainServer2; // getTypes
    ros::ServiceServer domainServer3; // getPredicates
    ros::ServiceServer domainServer4; // getFunctionPredicates
    ros::ServiceServer domainServer5; // getOperators
    ros::ServiceServer domainServer6; // getOperatorDetails
    ros::ServiceServer domainServer7; // getPredicateDetails

    // query knowledge
    ros::ServiceServer queryServer; // queryKnowledge
    ros::ServiceServer senseServer; // queryKnowledge

    // update knowledge
    ros::ServiceServer updateServer0; // clearKnowledge
    ros::ServiceServer updateServer1; // updateKnowledge
    ros::ServiceServer updateServer2; // updateKnowledgeArray
    ros::ServiceServer updateServer3; // updateKnowledgeConstraintsOneOf

    // fetch knowledge
    ros::ServiceServer stateServer1; // getInstances
    ros::ServiceServer stateServer2; // getPropositions
    ros::ServiceServer stateServer3; // getFunctions
    ros::ServiceServer stateServer4; // getTimedKnowledge
    ros::ServiceServer stateServer5; // getGoals
    ros::ServiceServer stateServer6; // getMetric

  protected:
    ros::Publisher status_pub;

    /* adding items to the knowledge base */
    void addKnowledge(xyz_knowledge_msgs::KnowledgeItem &msg);
    void addMissionGoal(xyz_knowledge_msgs::KnowledgeItem &msg);
    void addMissionMetric(xyz_knowledge_msgs::KnowledgeItem &msg);

    /* removing items from the knowledge base */
    void removeKnowledge(xyz_knowledge_msgs::KnowledgeItem &msg);
    virtual void removeFact(const xyz_knowledge_msgs::KnowledgeItem &msg);

    void removeMissionGoal(xyz_knowledge_msgs::KnowledgeItem &msg);
    void removeMissionMetric(xyz_knowledge_msgs::KnowledgeItem &msg);

    /* PDDL model (persistent state) */
    std::map<std::string, std::vector<std::string>> domain_constants;

    /* PDDL model (current state) */
    std::map<std::string, std::vector<std::string>> model_constants;
    std::map<std::string, std::vector<std::string>> model_instances;
    std::vector<xyz_knowledge_msgs::KnowledgeItem> model_facts;
    std::vector<xyz_knowledge_msgs::KnowledgeItem> model_functions;
    std::vector<xyz_knowledge_msgs::KnowledgeItem> model_goals;
    xyz_knowledge_msgs::KnowledgeItem model_metric;

    /* timed initial literals */
    std::vector<xyz_knowledge_msgs::KnowledgeItem> model_timed_initial_literals;

    /* conditional planning */
    std::vector<std::vector<xyz_knowledge_msgs::KnowledgeItem>> model_oneof_constraints;

    /* sensing information */
    std::map<std::string, bool> sensed_predicates;

    ros::NodeHandle _nh;

    double _kb_rate;

  public:
    explicit KnowledgeBase(ros::NodeHandle &n);
    virtual ~KnowledgeBase() = default;

    bool use_unknowns;

    /* parse domain and probelm files */
    virtual void parseDomain(const std::string &domain_file_path, const std::string &problem_file_path) = 0;

    /* add the initial state to the knowledge base */
    virtual void addInitialState() = 0;
    virtual void addConstants() = 0;

    /* service methods for querying the model */
    virtual bool queryKnowledge(xyz_knowledge_msgs::KnowledgeQueryService::Request &req, xyz_knowledge_msgs::KnowledgeQueryService::Response &res);

    /* service methods for fetching the current state */
    virtual bool getInstances(xyz_knowledge_msgs::GetInstanceService::Request &req, xyz_knowledge_msgs::GetInstanceService::Response &res);
    virtual bool getPropositions(xyz_knowledge_msgs::GetAttributeService::Request &req, xyz_knowledge_msgs::GetAttributeService::Response &res);
    virtual bool getFunctions(xyz_knowledge_msgs::GetAttributeService::Request &req, xyz_knowledge_msgs::GetAttributeService::Response &res);
    virtual bool getGoals(xyz_knowledge_msgs::GetAttributeService::Request &req, xyz_knowledge_msgs::GetAttributeService::Response &res);
    virtual bool getMetric(xyz_knowledge_msgs::GetMetricService::Request &req, xyz_knowledge_msgs::GetMetricService::Response &res);
    virtual bool getTimedKnowledge(xyz_knowledge_msgs::GetAttributeService::Request &req, xyz_knowledge_msgs::GetAttributeService::Response &res);

    /* service methods for adding and removing items to and from the current state */
    virtual bool updateKnowledgeArray(ros::ServiceEvent<xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request, xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response> &event);
    // bool updateKnowledgeArray(xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
    virtual bool updateKnowledge(xyz_knowledge_msgs::KnowledgeUpdateService::Request &req, xyz_knowledge_msgs::KnowledgeUpdateService::Response &res);
    virtual bool clearKnowledge(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /* service methods for fetching the domain details */
    virtual void getSubtypes(std::string &type, std::vector<std::string> &subtypes);
    virtual bool getDomainName(xyz_knowledge_msgs::GetDomainNameService::Request &req, xyz_knowledge_msgs::GetDomainNameService::Response &res) = 0;
    virtual bool getTypes(xyz_knowledge_msgs::GetDomainTypeService::Request &req, xyz_knowledge_msgs::GetDomainTypeService::Response &res) = 0;
    virtual bool getPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request &req, xyz_knowledge_msgs::GetDomainAttributeService::Response &res) = 0;
    virtual bool getFunctionPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request &req, xyz_knowledge_msgs::GetDomainAttributeService::Response &res) = 0;
    virtual bool getOperators(xyz_knowledge_msgs::GetDomainOperatorService::Request &req, xyz_knowledge_msgs::GetDomainOperatorService::Response &res) = 0;
    virtual bool getOperatorDetails(xyz_knowledge_msgs::GetDomainOperatorDetailsService::Request &req, xyz_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) = 0;
    virtual bool getPredicateDetails(xyz_knowledge_msgs::GetDomainPredicateDetailsService::Request &req, xyz_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) = 0;

    /* service methods for conditional planning */
    virtual bool updateKnowledgeConstraintsOneOf(xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);

    /* service methods for sensed predicates */
    bool setSensedPredicate(xyz_knowledge_msgs::SetNamedBool::Request &req, xyz_knowledge_msgs::SetNamedBool::Response &res);

    /* publish status */
    void publishStatusUpdate(ros::Time &time, std::string &caller_id);
    std::string toString(const xyz_knowledge_msgs::KnowledgeItem& item);

    /* main loop */
    void runKnowledgeBase();
};
} // namespace KCL_rosplan
#endif
