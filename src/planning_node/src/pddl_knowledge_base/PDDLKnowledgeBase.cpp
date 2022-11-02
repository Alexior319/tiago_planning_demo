//
// Created by xyz on 22-6-11.
//
#include "pddl_knowledge_base/PDDLKnowledgeBase.h"

namespace planning_node {
    /*-----------------*/
    /* fetching domain */
    /*-----------------*/

    /* get domain name */
    bool PDDLKnowledgeBase::getDomainName(xyz_knowledge_msgs::GetDomainNameService::Request& req,
                                          xyz_knowledge_msgs::GetDomainNameService::Response& res) {
        res.domain_name = parser.getDomain()._name;
        return true;
    }

    /* get domain types */
    bool PDDLKnowledgeBase::getTypes(xyz_knowledge_msgs::GetDomainTypeService::Request& req,
                                     xyz_knowledge_msgs::GetDomainTypeService::Response& res) {

        res.types = types;
        return true;
    }

    /* get domain predicates */
    bool PDDLKnowledgeBase::getPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request& req,
                                          xyz_knowledge_msgs::GetDomainAttributeService::Response& res) {

        return false;
    }

    /* get domain functions */
    bool PDDLKnowledgeBase::getFunctionPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request& req,
                                                  xyz_knowledge_msgs::GetDomainAttributeService::Response& res) {
        return false;

    }

    /* get domain operators */
    bool PDDLKnowledgeBase::getOperators(xyz_knowledge_msgs::GetDomainOperatorService::Request& req,
                                         xyz_knowledge_msgs::GetDomainOperatorService::Response& res) {

        return false;
    }

    /* get domain operator details */
    bool PDDLKnowledgeBase::getOperatorDetails(xyz_knowledge_msgs::GetDomainOperatorDetailsService::Request& req,
                                               xyz_knowledge_msgs::GetDomainOperatorDetailsService::Response& res) {
        return false;
    }

    /* get domain predicate details */
    bool PDDLKnowledgeBase::getPredicateDetails(xyz_knowledge_msgs::GetDomainPredicateDetailsService::Request& req,
                                                xyz_knowledge_msgs::GetDomainPredicateDetailsService::Response& res) {
        return true;
    }

    /*-------------------------------------*/
    /* add initial state to knowledge base */
    /*-------------------------------------*/

    /* get constants from the domain file */
    void PDDLKnowledgeBase::addConstants() {


    }

    /* get the initial state from the domain and problem files */
    void PDDLKnowledgeBase::addInitialState() {

    }

    void PDDLKnowledgeBase::parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) {
        parser.parseDomain(domain_file_path);
        parser.parseProblem(problem_file_path);
        ros_info(parser.output());

        for (const auto& t : *parser.getDomain()._types) {
            types.push_back(t);
        }


        for (const auto& pred : *parser.getDomain()._predicates) {
            auto predicate = std::make_shared<MetaPredicate>();
            predicate->name = pred->_name;
            for (const auto& para : *pred->_args) {
                predicate->names.push_back(para);
                predicate->types.push_back(pred->_types->at(para));
            }
            metaPredicates[pred->_name] = std::move(predicate);
        }

        for (const auto& action : *parser.getDomain()._actions) {
            auto metaaction = std::make_shared<MetaAction>();
            metaaction->name = action->_name;
            for (const auto& para : *action->_params) {
                metaaction->names.push_back(para);
                metaaction->types.push_back(action->_types->at(para));
            }
            for (const auto& precondition : *action->_precond) {
                auto& pred = precondition->first;
                auto& status = precondition->second;

                if (status == LiteralState::POSITIVE || status == LiteralState::NEGATIVE) {
                    metaaction->metaConditions.emplace_back(status == LiteralState::NEGATIVE, metaPredicates.at(pred->_name));
                    for (const auto& conditionPara : *pred->_args) {
                        metaaction->metaConditions.back().pos.push_back(find(metaaction->names.begin(), metaaction->names.end(), conditionPara) - metaaction->names.begin());
                    }
                } else if (status == LiteralState::UNKNOWN) {
                    metaaction->metaConditionsShouldNotSeen.emplace_back(false, metaPredicates.at(pred->_name));
                    for (const auto& conditionPara : *pred->_args) {
                        metaaction->metaConditionsShouldNotSeen.back().pos.push_back(find(metaaction->names.begin(), metaaction->names.end(), conditionPara) - metaaction->names.begin());
                    }
                } else {
                    ros_error("Unsupported action precondition: {}", *precondition);
                    ros::shutdown();
                }
            }
            for (const auto& effect : *action->_effects) {
                auto& pred = effect->first;
                auto& status = effect->second;
                if (status == LiteralState::UNKNOWN) {
                    ros_error("Unsupported action effect: {}", *effect);
                    ros::shutdown();
                }

                if (status == LiteralState::POSITIVE || status == LiteralState::NEGATIVE) {
                    metaaction->metaEffects.emplace_back(status == LiteralState::NEGATIVE, metaPredicates.at(pred->_name));
                    for (const auto& effectParaName : *pred->_args) {
                        metaaction->metaEffects.back().pos.push_back(find(metaaction->names.begin(), metaaction->names.end(), effectParaName) - metaaction->names.begin());
                    }
                } else if (status == LiteralState::KNOWN) {
                    metaaction->metaEffects.emplace_back(true, metaPredicates.at(pred->_name));
                    for (const auto& effectParaName : *pred->_args) {
                        metaaction->metaEffects.back().pos.push_back(find(metaaction->names.begin(), metaaction->names.end(), effectParaName) - metaaction->names.begin());
                    }
                    metaaction->metaEffects.emplace_back(false, metaPredicates.at(pred->_name));
                    for (const auto& effectParaName : *pred->_args) {
                        metaaction->metaEffects.back().pos.push_back(find(metaaction->names.begin(), metaaction->names.end(), effectParaName) - metaaction->names.begin());
                    }
                }
            }
            metaActions[action->_name] = std::move(metaaction);
        }

        for (const auto& object : *parser.getProblem()._objects) {
            allObjects[object.second].insert(object.first);
            objTypes[object.first] = object.second;
        }
        for (const auto& objtype : allObjects) {
            types.push_back(objtype.first);
        }

        for (const auto& initialPredicate: *parser.getProblem()._init) {
            const auto& pred = initialPredicate->first;
            const auto& state = initialPredicate->second;
            if (state == LiteralState::POSITIVE) {
                
            }
        }
    }
}

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosplan_knowledge_base");
    ros::NodeHandle n("~");

    // parameters
    std::string domainPath, problemPath, dataPath = "/tmp";
    bool useUnknowns;
    n.param("/rosplan/domain_path", domainPath, std::string("common/domain.pddl"));
    n.param("use_unknowns", useUnknowns, false);
    n.param("domain_path", domainPath, domainPath);
    n.param("problem_path", problemPath, problemPath);
    n.param("data_path", dataPath, dataPath);
    if (dataPath.back() != '/') dataPath.push_back('/');

    std::unique_ptr<KCL_rosplan::KnowledgeBase> kb = std::make_unique<planning_node::PDDLKnowledgeBase>(n);\
    // parse domain
    kb->parseDomain(domainPath, problemPath);
    kb->use_unknowns = useUnknowns;
    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    kb->runKnowledgeBase();

    return 0;
}