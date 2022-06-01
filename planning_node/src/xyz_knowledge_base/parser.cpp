//
// Created by xyz on 2022/3/22.
//

#include "xyz_knowledge_base/parser.h"
#include <fmt/format.h>

#include <iostream>

namespace planning_node {
    void parser::parseDomain(const XmlRpc::XmlRpcValue& planning_data) {
        if (planning_data.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("Invalid yaml data");
            return;
        }

//        run_info("{}", planning_data.toXml());

        string members[] = {"initial", "goal", "types", "predicates", "actions"};
        planning_data.getType();
        for (const auto& m: members) {
            if (!planning_data.hasMember(m)) {
                ROS_ERROR("Invalid yaml data: key %s does not exist.", m.c_str());
                return;
            }
        }
        const auto& initial_ = planning_data["initial"];
        const auto& goal_ = planning_data["goal"];
        const auto& types_ = planning_data["types"];
        const auto& predicates_ = planning_data["predicates"];
        const auto& actions_ = planning_data["actions"];
        if (initial_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            goal_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            types_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            predicates_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Invalid yaml data: types, initial and goal should be list "
                      "of predicates.");
        }
        if (actions_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Invalid yaml data: actions should be a list.");
        }
        // 所有对象类型
        for (int i = 0; i < types_.size(); ++i) {
            types.emplace_back(types_[i]);
        }
        // 所有谓词
        for (int i = 0; i < predicates_.size(); ++i) {
            parseMetaPredicates(predicates_[i]);
        }
        // 初始状态和目标状态
        initial = make_shared<State>();
        goal = make_shared<State>();
        for (int i = 0; i < initial_.size(); ++i) {
            initial->add(parseStatePredicate(initial_[i]));
        }
        for (int i = 0; i < goal_.size(); ++i) {
            goal->add(parseStatePredicate(goal_[i]));
        }

        for (int i = 0; i < actions_.size(); ++i) {
            parseMetaAction(actions_[i]);
        }

        generatePossibleActions();
//        run_info("Possible actions: {}", allPossibleActions);
    }

    void parser::parseDomain(const ros::NodeHandle& nh, const string& domain_name) {
        XmlRpc::XmlRpcValue planning_data;
        nh.getParam(domain_name, planning_data);
        parseDomain(planning_data);
    }
    Predicate parser::parseStatePredicate(const string& s, bool output_error) noexcept {
        Predicate p = parsePredicate(s, output_error);
        for (int i = 0; i < p.parameters.size(); ++i) {
            objTypes[p.parameters[i]] = p.meta->types[i];
            allObjects[p.meta->types[i]].emplace(p.parameters[i]);
        }
        return p;
    }

    void parser::parseMetaAction(const XmlRpc::XmlRpcValue& v) noexcept {
        if (v.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !v.hasMember("name") || !v.hasMember("conditions") ||
            !v.hasMember("effects")) {
            ros_error("The actions should possess name, conditions and effects "
                      "properties.");
        }
        auto a = make_shared<MetaAction>();
        a->name = string(v["name"]);
        this->metaActions[a->name] = a;


        vector<Type> action_types;
        vector<string> action_paras;
        unordered_map<string, int> paraIdx;
        a->types.reserve(v["parameters"].size());
        a->names.reserve(v["parameters"].size());
        for (int i = 0; i < v["parameters"].size(); ++i) {
            a->types.emplace_back(v["parameters"][i]["type"]);
            a->names.emplace_back(v["parameters"][i]["name"]);
            paraIdx[v["parameters"][i]["name"]] = i;
        }

        for (int i = 0; i < v["conditions"].size(); ++i) {
            string pattern = v["conditions"][i];
            if (startsWith(pattern, "not ")) {
                auto pp = parsePredicate(pattern.substr(4));
                a->metaConditionsShouldNotSeen.emplace_back(pp.neg, pp.meta);
                for (auto& para : pp.parameters) {
                    a->metaConditionsShouldNotSeen[a->metaConditionsShouldNotSeen.size() - 1].pos.emplace_back(paraIdx[para]);
                }
            } else {
                auto pp = parsePredicate(pattern);
                a->metaConditions.emplace_back(pp.neg, pp.meta);
                for (auto& para : pp.parameters) {
                    a->metaConditions[a->metaConditions.size() - 1].pos.emplace_back(paraIdx[para]);
                }
            }
        }
        for (int i = 0; i < v["effects"].size(); ++i) {
            auto pp = parsePredicate(v["effects"][i]);
            a->metaEffects.emplace_back(pp.neg, pp.meta);
            for (auto& para : pp.parameters) {
                a->metaEffects[a->metaEffects.size() - 1].pos.emplace_back(paraIdx[para]);
            }

        }
    }

    void parser::parseMetaPredicates(const XmlRpc::XmlRpcValue& v) noexcept {
        if (v.getType() != XmlRpc::XmlRpcValue::TypeStruct || !v.hasMember("name")) {
            ros_error("The predicates should possess [ name ] property.");
        }
        auto predicate = make_shared<MetaPredicate>();
        predicate->name = string(v["name"]);
        this->metaPredicates[predicate->name] = predicate;

        if (!v.hasMember("parameters")) return;

        vector<Type> pred_para_types;
        vector<string> pred_para_names;
        unordered_map<string, int> paraIdx;
        const auto& paras = v["parameters"];
        predicate->types.reserve(paras.size());
        predicate->names.reserve(paras.size());
        for (int i = 0; i < paras.size(); ++i) {
            predicate->types.emplace_back(paras[i]["type"]);
            predicate->names.emplace_back(paras[i]["name"]);
            paraIdx[paras[i]["name"]] = i;
        }
    }

    pair<string, vector<Type>>
    parser::parsePredicatePattern(const string& s, bool output_err) noexcept {
        string name;
        vector<string> paras;

        int idx = 0;
        string val;
        if (s[0] == '-') {
            ++idx;
            val.push_back('-');
        }

        enum State {
            ReadName = 0, ReadParam, ReadComma, Illegal, EndState
        };

        State state = ReadName;
        bool parsed = false;
        for (; idx < s.length(); ++idx) {
            switch (state) {
                case ReadName:
                    // 谓词名称支持字母和下划线和数字
                    if (isalpha(s[idx]) ||
                        ((s[idx] == '_' || isdigit(s[idx])) && !val.empty())) {
                        val.push_back(s[idx]);
                    } else if (s[idx] == '(') {
                        name = move(val);
                        val.clear();
                        state = ReadParam;
                    } else {
                        state = Illegal;
                    }
                    break;
                case ReadParam:
                    if (isspace(s[idx]) && !val.empty()) {
                        // 参数后面的空格
                        paras.emplace_back(move(val));
                        val.clear();
                        state = ReadComma;
                    } else if (isspace(s[idx]) && val.empty()) {
                        // 参数前面的空格，忽略
                    } else if (isalpha(s[idx]) ||
                               ((s[idx] == '_' || isdigit(s[idx])) && !val.empty())) {
                        // 参数名
                        val.push_back(s[idx]);
                    } else if (s[idx] == ',') {
                        if (val.empty()) {
                            state = Illegal;
                            break;
                        }
                        paras.emplace_back(move(val));
                        val.clear();
                        state = ReadParam;
                    } else if (s[idx] == ')') {
                        if (!val.empty())
                            paras.emplace_back(move(val));
                        val.clear();
                        parsed = true;
                        state = EndState;
                    } else {
                        // 其他字符
                        state = Illegal;
                    }
                    break;
                case ReadComma:
                    if (!isspace(s[idx]) && s[idx] != ',' && s[idx] != ')') {
                        state = Illegal;
                    } else if (s[idx] == ',') {
                        state = ReadParam;
                    } else if (s[idx] == ')') {
                        parsed = true;
                        state = EndState;
                    }
                    break;
                case Illegal:
                    parsed = false;
                    if (output_err) {
                        ros_error("'{}' Unsupported character '{}'.", s, s[idx - 1]);
                        ros_error("{}^", string(idx, ' '));
                    }
                    name.clear();
                    paras.clear();
                    // 跳出循环
                    idx = s.length();
                    break;
                case EndState:
                    parsed = true;
                    if (!isspace(s[idx]))
                        state = Illegal;
                    break;
            }
        }
        if (!parsed) {
            if (output_err) {
                ros_error("Failed to parse '{}'.", s);
            }
            return {"", {}};
        }
        return {move(name), move(paras)};
    }

    Predicate parser::parsePredicate(const string& s, bool output_err) noexcept {
        // parse name
        // parse parameters
        const auto &[name, paras] = parsePredicatePattern(s, output_err);
        Predicate p;
        if (name[0] == '-') {
            p.neg = true;
            p.meta = metaPredicates[name.substr(1)];
        } else {
            p.neg = false;
            p.meta = metaPredicates[name];
        }
        for (int i = 0; i < paras.size(); ++i) {
            p.parameters.emplace_back(paras[i]);
        }
        return p;
    }

    void parser::backtrack(vector<string>& paras, const shared_ptr<MetaAction>& meta) {
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
    }

    void parser::generatePossibleActions() noexcept {
        vector<string> paras;
        for (const auto& [name, metaAction] : this->metaActions) {
            backtrack(paras, metaAction);
        }
//        run_info("Generating done. {} possible actions.", this->allPossibleActions.size());
//        run_info("{}", this->allPossibleActions);
    }

    bool parser::startsWith(const string& s, const string& prefix) noexcept {
        auto prefixSize = prefix.size();
        if (prefixSize > s.size()) return false;

        for (size_t i = 0; i < prefixSize; ++i) {
            if (s[i] != prefix[i]) return false;
        }
        return true;
    }
} // namespace planning_node
