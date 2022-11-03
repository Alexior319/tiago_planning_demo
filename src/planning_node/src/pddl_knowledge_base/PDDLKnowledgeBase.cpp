//
// Created by xyz on 22-6-11.
//
#include "pddl_knowledge_base/PDDLKnowledgeBase.h"
#include "pddl_knowledge_base/pddl_planner.h"

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
        for (const auto& [name, namep]: metaPredicates) {
            xyz_knowledge_msgs::DomainFormula formula;
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

    /* get domain functions */
    bool PDDLKnowledgeBase::getFunctionPredicates(xyz_knowledge_msgs::GetDomainAttributeService::Request& req,
                                                  xyz_knowledge_msgs::GetDomainAttributeService::Response& res) {
        ros_error("Unsupported operation: getFunctionPredicates");
        return false;

    }

    /* get domain operators */
    bool PDDLKnowledgeBase::getOperators(xyz_knowledge_msgs::GetDomainOperatorService::Request& req,
                                         xyz_knowledge_msgs::GetDomainOperatorService::Response& res) {
        for (const auto& [name, namep]: metaActions) {
            xyz_knowledge_msgs::DomainFormula formula;
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
    bool PDDLKnowledgeBase::getOperatorDetails(xyz_knowledge_msgs::GetDomainOperatorDetailsService::Request& req,
                                               xyz_knowledge_msgs::GetDomainOperatorDetailsService::Response& res) {
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
            xyz_knowledge_msgs::DomainFormula formula;
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
            xyz_knowledge_msgs::DomainFormula formula;
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
    bool PDDLKnowledgeBase::getPredicateDetails(xyz_knowledge_msgs::GetDomainPredicateDetailsService::Request& req,
                                                xyz_knowledge_msgs::GetDomainPredicateDetailsService::Response& res) {
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
    void PDDLKnowledgeBase::addConstants() {
        ros_error("Unsupported operation: addConstants()");
    }

    /* get the initial state from the domain and problem files */
    void PDDLKnowledgeBase::addInitialState() {
        ros_error("Unsupported operation: addInitialState()");
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
        ros_info("Meta predicates: {}", metaPredicates);

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
                        int idx = find(metaaction->names.begin(), metaaction->names.end(), conditionPara) - metaaction->names.begin();
                        if (idx == metaaction->names.size()) {
                            ros_error("Unknown parameter [{}] in condition {} of action {}", conditionPara, *pred, metaaction->name);
                        }
                        metaaction->metaConditions.back().pos.push_back(idx);
                    }
                } else if (status == LiteralState::UNKNOWN) {
                    metaaction->metaConditionsShouldNotSeen.emplace_back(false, metaPredicates.at(pred->_name));
                    for (const auto& conditionPara : *pred->_args) {
                        int idx = find(metaaction->names.begin(), metaaction->names.end(), conditionPara) - metaaction->names.begin();
                        if (idx == metaaction->names.size()) {
                            ros_error("Unknown parameter [{}] in condition {} of action {}", conditionPara, *pred, metaaction->name);
                        }
                        metaaction->metaConditionsShouldNotSeen.back().pos.push_back(idx);
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
                        int idx = find(metaaction->names.begin(), metaaction->names.end(), effectParaName) - metaaction->names.begin();
                        if (idx == metaaction->names.size()) {
                            ros_error("Unknown parameter [{}] in effect {} of action {}", effectParaName, *pred, metaaction->name);
                        }
                        metaaction->metaEffects.back().pos.push_back(idx);
                    }
                } else if (status == LiteralState::KNOWN) {
                    metaaction->metaEffects.emplace_back(true, metaPredicates.at(pred->_name));
                    for (const auto& effectParaName : *pred->_args) {
                        int idx = find(metaaction->names.begin(), metaaction->names.end(), effectParaName) - metaaction->names.begin();
                        if (idx == metaaction->names.size()) {
                            ros_error("Unknown parameter [{}] in effect {} of action {}", effectParaName, *pred, metaaction->name);
                        }
                        metaaction->metaEffects.back().pos.push_back(idx);
                    }
                    metaaction->metaEffects.emplace_back(false, metaPredicates.at(pred->_name));
                    for (const auto& effectParaName : *pred->_args) {
                        int idx = find(metaaction->names.begin(), metaaction->names.end(), effectParaName) - metaaction->names.begin();
                        if (idx == metaaction->names.size()) {
                            ros_error("Unknown parameter [{}] in effect {} of action {}", effectParaName, *pred, metaaction->name);
                        }
                        metaaction->metaEffects.back().pos.push_back(idx);
                    }
                }
            }
            metaActions[action->_name] = std::move(metaaction);
        }
        ros_info("Meta actions: {}", metaActions);

        for (const auto& object : *parser.getProblem()._objects) {
            allObjects[object.second].insert(object.first);
            objTypes[object.first] = object.second;
        }
        ros_info("allObjects: {}", allObjects);
        ros_info("types: {}", types);

        if (!current_state) current_state = make_shared<State>();
        else current_state->clear();
        for (const auto& initialPredicate: *parser.getProblem()._init) {
            const auto& pred = initialPredicate->first;
            const auto& state = initialPredicate->second;
            ros_info("Adding predicate: {}", *pred);
            if (state != LiteralState::POSITIVE && state != LiteralState::NEGATIVE) {
                ros_error("Unsupported initial state: {}", *initialPredicate);
                ros::shutdown();
            }
            if (!metaPredicates.count(pred->_name)) {
                ros_error("Undefined predicate: {}", *pred);
                ros::shutdown();
            }
            Predicate p;
            p.meta = metaPredicates.at(pred->_name);
            p.neg = state == LiteralState::NEGATIVE;
            for (const auto& para : *pred->_args) {
                p.parameters.push_back(para);
            }
            current_state->add(p);
        }
        ros_info("Initial state: {}", current_state);
        if (!goal_state) goal_state = make_shared<State>();
        else goal_state->clear();
        for (const auto& goalPredicate: *parser.getProblem()._goal) {
            const auto& pred = goalPredicate->first;
            const auto& state = goalPredicate->second;
            if (state != LiteralState::POSITIVE && state != LiteralState::NEGATIVE) {
                ros_error("Unsupported initial state: {}", *goalPredicate);
                ros::shutdown();
            }
            if (!metaPredicates.count(pred->_name)) {
                ros_error("Undefined predicate: {}", *pred);
                ros::shutdown();
            }
            Predicate p;
            p.meta = metaPredicates.at(pred->_name);
            p.neg = state == LiteralState::NEGATIVE;
            for (const auto& para : *pred->_args) {
                p.parameters.push_back(para);
            }
            goal_state->add(p);
        }
        ros_info("Goal state: {}", goal_state);

        for (const auto& [name, type] : objTypes) {
            model_instances[type].push_back(name);
        }

        for (const auto& p : current_state->state) {
            xyz_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
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
            xyz_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = xyz_knowledge_msgs::KnowledgeItem::FACT;
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

        generatePossibleActions();
    }

    bool PDDLKnowledgeBase::getCurrent(xyz_knowledge_msgs::GetAttributeService::Request& req,
                                       xyz_knowledge_msgs::GetAttributeService::Response& res) {
        for (const auto& g: model_facts) {
            if (!req.predicate_name.empty() && req.predicate_name != g.attribute_name)
                continue;

            res.attributes.emplace_back(g);
        }
        return true;
    }

    const vector<xyz_knowledge_msgs::KnowledgeItem>& PDDLKnowledgeBase::getState() const {
        return model_facts;
    }

    bool PDDLKnowledgeBase::getGoals(xyz_knowledge_msgs::GetAttributeService::Request& req,
                                     xyz_knowledge_msgs::GetAttributeService::Response& res) {
        for (const auto& g: goal_state->state) {
            if (!req.predicate_name.empty() && req.predicate_name != g.meta->name)
                continue;

            xyz_knowledge_msgs::KnowledgeItem item;
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


    void PDDLKnowledgeBase::generatePossibleActions() noexcept {
        vector<string> paras;

        function<void(vector<string>&, const shared_ptr<MetaAction>&)> backtrack = [&](vector<string>& paras, const shared_ptr<MetaAction>& meta) {
            if (paras.size() == meta->types.size()) {
                auto ptr = make_shared<Action>();
                ptr->meta = meta;
                ptr->paras.assign(paras.begin(), paras.end());
                for (const auto& mc : meta->metaConditions) {
                    Predicate pp;
                    pp.meta = mc.meta;
                    pp.neg = mc.neg;
                    for (const auto& idx : mc.pos) {
                        pp.parameters.emplace_back(ptr->paras[idx]);
                    }
                    ptr->conditions.emplace_back(pp);
                }
                for (const auto& mc : meta->metaConditionsShouldNotSeen) {
                    Predicate pp;
                    pp.meta = mc.meta;
                    pp.neg = mc.neg;
                    for (const auto& idx : mc.pos) {
                        pp.parameters.emplace_back(ptr->paras[idx]);
                    }
                    ptr->conditionsShouldNotSeen.emplace_back(pp);
                }
                for (const auto& mc : meta->metaEffects) {
                    Predicate pp;
                    pp.meta = mc.meta;
                    pp.neg = mc.neg;
                    for (const auto& idx : mc.pos) {
                        pp.parameters.emplace_back(ptr->paras[idx]);
                    }
                    ptr->effects.emplace_back(pp);
                }
                this->allPossibleActions.emplace_back(ptr);
                return;
            }
            for (const auto& obj : this->allObjects[meta->types[paras.size()]]) {
                // 遍历这种类型的object
                if (paras.end() != find(paras.begin(), paras.end(), obj)) continue;

                paras.emplace_back(obj);
                backtrack(paras, meta);
                paras.pop_back();
            }
        };

        for (const auto& [name, metaAction] : this->metaActions) {
//            ros_info("{}", name);
            backtrack(paras, metaAction);
        }
    }
}

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "pddl_knowledge_base");
    ros::NodeHandle n("~");

    // parameters
    std::string domainPath, problemPath;
    n.param("domain_path", domainPath, domainPath);
    n.param("problem_path", problemPath, problemPath);

    auto kb = std::make_unique<planning_node::PDDLKnowledgeBase>(n);
    // parse domain
    kb->parseDomain(domainPath, problemPath);
    planning_node::pddl_planner planner{n, kb};
    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    kb->runKnowledgeBase();

    return 0;
}