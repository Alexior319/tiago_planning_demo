//
// Created by xyz on 22-5-13.
//
#include <csignal>
#include <memory>

#include "common.h"
#include "xyz_knowledge_base/XYZKnowledgeBase.h"
#include <boost/filesystem.hpp>
#include <xmlrpcpp/XmlRpc.h>
#include "xyz_knowledge_base/planner.h"

namespace KCL_rosplan {

/*-----------------*/
/* fetching domain */
/*-----------------*/

/* get domain name */
    bool XYZKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request& req,
                                         rosplan_knowledge_msgs::GetDomainNameService::Response& res) {
        // if (!domain_parser.domain_parsed)
        //     return false;
        res.domain_name = domain_name;
        return true;
    }

/* get domain types */
    bool XYZKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request& req,
                                    rosplan_knowledge_msgs::GetDomainTypeService::Response& res) {
        res.types = types;
        return true;
    }

    bool XYZKnowledgeBase::getGoals(rosplan_knowledge_msgs::GetAttributeService::Request& req,
                                    rosplan_knowledge_msgs::GetAttributeService::Response& res) {
        for (const auto& g: goal_state->state) {
            if (!req.predicate_name.empty() && req.predicate_name != g.meta->name)
                continue;

            rosplan_knowledge_msgs::KnowledgeItem item;
            item.attribute_name = g.meta->name;
            item.is_negative = g.neg;
            for (int i = 0; i < g.parameters.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.key = g.meta->names[i];
                kv.value = g.parameters[i];
                item.values.push_back(kv);
            }
            res.attributes.emplace_back(item);
        }
        return true;
    }

/* get domain predicates */
    bool XYZKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request& req,
                                         rosplan_knowledge_msgs::GetDomainAttributeService::Response& res) {

        for (const auto& [name, namep]: metaPredicates) {
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = name;
            for (int i = 0; i < namep->types.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.value = namep->types[i];
                kv.key = namep->names[i];
                formula.typed_parameters.emplace_back(kv);
            }
            res.items.emplace_back(formula);
        }
        return true;
    }

/* get domain operators */
    bool XYZKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request& req,
                                        rosplan_knowledge_msgs::GetDomainOperatorService::Response& res) {

        for (const auto& [name, namep]: metaActions) {
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = name;
            for (int i = 0; i < namep->types.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.value = namep->types[i];
                kv.key = namep->names[i];
                formula.typed_parameters.emplace_back(kv);
            }
            res.operators.emplace_back(formula);
        }
        return true;
    }

/* get domain operator details */
    bool XYZKnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request& req,
                                              rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response& res) {
        if (!metaActions.count(req.name)) {
            ros_error("No operator named [ {} ] in knowledge base.", req.name);
            return false;
        }
        const auto& action = metaActions[req.name];
        // operator name and parameters
        res.op.formula.name = req.name;
        for (size_t i = 0; i < action->types.size(); ++i) {
            diagnostic_msgs::KeyValue kv;
            kv.key = action->names[i];
            kv.value = action->types[i];
            res.op.formula.typed_parameters.emplace_back(kv);
        }

        // conditions
        for (size_t i = 0; i < action->metaConditions.size(); ++i) {
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = action->metaConditions[i].meta->name;
            for (size_t j = 0; j < action->metaConditions[i].meta->types.size(); ++j) {
                diagnostic_msgs::KeyValue kv;
                kv.key = action->names[action->metaConditions[i].pos[j]];
                kv.value = action->metaConditions[i].meta->types[j];
                formula.typed_parameters.emplace_back(kv);
            }
            if (action->metaConditions[i].neg) {
                res.op.at_start_neg_condition.emplace_back(formula);
            } else {
                res.op.at_start_simple_condition.emplace_back(formula);
            }
        }
        // effects
        for (size_t i = 0; i < action->metaEffects.size(); ++i) {
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = action->metaEffects[i].meta->name;
            for (size_t j = 0; j < action->metaEffects[i].meta->types.size(); ++j) {
                diagnostic_msgs::KeyValue kv;
                kv.key = action->names[action->metaEffects[i].pos[j]];
                kv.value = action->metaEffects[i].meta->types[j];
                formula.typed_parameters.emplace_back(kv);
            }
            if (action->metaEffects[i].neg) {
                res.op.at_end_del_effects.emplace_back(formula);
            } else {
                res.op.at_end_add_effects.emplace_back(formula);
            }
        }
        return true;
    }

/* get domain predicate details */
    bool XYZKnowledgeBase::getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request& req,
                                               rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response& res) {
        if (!metaPredicates.count(req.name)) {
            ros_error("No predicate named [ {} ] in knowledge base.", req.name);
            return false;
        }

        const auto& pred = metaPredicates[req.name];
        res.predicate.name = req.name;
        for (size_t i = 0; i < pred->types.size(); ++i) {
            diagnostic_msgs::KeyValue kv;
            kv.key = pred->names[i];
            kv.value = pred->types[i];
            res.predicate.typed_parameters.emplace_back(kv);
        }
        return true;
    }

/*-------------------------------------*/
/* add initial state to knowledge base */
/*-------------------------------------*/

/* get constants from the domain file */
// void XYZKnowledgeBase::addConstants() {

// if (!domain_parser.domain_parsed)
//     return;

// VAL1_2::const_symbol_list *c = domain_parser.domain->constants;
// if (c) {
//     for (VAL1_2::const_symbol_list::const_iterator symbolListIterator = c->begin();
//          symbolListIterator != c->end(); symbolListIterator++) {
//         const VAL1_2::const_symbol *object = *symbolListIterator;
//         domain_constants[object->type->getName()].push_back(object->pddl_typed_symbol::getName());
//     }
// }
// }

/* get the initial state from the domain and problem files */
    void XYZKnowledgeBase::addInitialState() {
    }

    void XYZKnowledgeBase::parseDomain(const std::string& domain_file_path, const std::string& _domain_name) {

        this->domain_name = _domain_name;
        ros_info("Parsing domain file {} using domain name {}", domain_file_path, _domain_name);
        namespace fsys = boost::filesystem;
        if (!fsys::exists(domain_file_path) ||
            !(fsys::is_regular_file(domain_file_path) || fsys::is_symlink(domain_file_path))) {
            ros_warn("Could not open domain file {}", domain_file_path);
            return;
        } else {
            // load the YAML file using the external rosparam command
            std::string command = fmt::format("rosparam load {} {}", domain_file_path, _nh.getNamespace());
            int result = std::system(command.c_str());
            if (result != 0) {
                ros_warn("Could not load domain file {}", domain_file_path);
            }
        }

        // parse the domain file
        XmlRpc::XmlRpcValue domain_xml;
        string key = _nh.getNamespace() + _domain_name;
        if (!_nh.getParam(key, domain_xml)) {
            ros_error("Could not read domain from parameter server!");
            return;
        }
        domain_parser.parseDomain(domain_xml);

        this->objTypes = move(domain_parser.objTypes);
        this->allPossibleActions = move(domain_parser.allPossibleActions);
        this->types = move(domain_parser.types);
        this->current_state = move(domain_parser.initial);
        this->goal_state = move(domain_parser.goal);
        this->metaActions = move(domain_parser.metaActions);
        this->metaPredicates = move(domain_parser.metaPredicates);


        for (const auto& [name, type] : objTypes) {
            model_instances[type].push_back(name);
        }

        for (const auto& p : current_state->state) {
            rosplan_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            item.attribute_name = p.meta->name;
            item.is_negative = p.neg;
            for (int i = 0; i < p.parameters.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.key = p.meta->names[i];
                kv.value = p.parameters[i];
                item.values.push_back(kv);
            }
            model_facts.emplace_back(item);
        }



        for (const auto& p : goal_state->state) {
            rosplan_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            item.attribute_name = p.meta->name;
            item.is_negative = p.neg;
            for (int i = 0; i < p.parameters.size(); ++i) {
                diagnostic_msgs::KeyValue kv;
                kv.key = p.meta->names[i];
                kv.value = p.parameters[i];
                item.values.push_back(kv);
            }
            model_goals.emplace_back(item);
        }
    }

    bool XYZKnowledgeBase::getCurrent(rosplan_knowledge_msgs::GetAttributeService::Request& req,
                                      rosplan_knowledge_msgs::GetAttributeService::Response& res) {
        for (const auto& g: model_facts) {
            if (!req.predicate_name.empty() && req.predicate_name != g.attribute_name)
                continue;

            res.attributes.emplace_back(g);
        }
        return true;
    }

    const vector<rosplan_knowledge_msgs::KnowledgeItem>& XYZKnowledgeBase::getState() const {
        return model_facts;
    }

} // namespace KCL_rosplan

static const char *default_node_name = "xyz_knowledge_base";

void handle_term(int) {
    std::system(fmt::format("rosparam delete /{}", default_node_name).c_str());
    //    std::terminate();
    ros::shutdown();
    ::exit(0);
}

int main(int argc, char **argv) {
    signal(SIGTERM, handle_term);
    ros::init(argc, argv, default_node_name);
    ros::NodeHandle nh("~");

    std::string domainPath, domainName;
    nh.param("domain_path", domainPath, domainPath);
    nh.param("domain_name", domainName, domainName);
    std::string ext = domainPath.substr(domainPath.find_last_of('.'));
    if (ext != ".yml" && ext != ".yaml") {
        ros_error("Invalid domain filename extension: {}", ext);
        ros::shutdown();
    }
    ros_info("Starting a XYZ knowledge base.");
    auto xyzKB = std::make_unique<KCL_rosplan::XYZKnowledgeBase>(nh);
    xyzKB->parseDomain(domainPath, domainName);
    ros_info("(XYZKB): Ready to receive");

    planning_node::planner planner(nh, xyzKB);


    xyzKB->runKnowledgeBase();
    std::system(fmt::format("rosparam delete /{}", default_node_name).c_str());

    return 0;
}