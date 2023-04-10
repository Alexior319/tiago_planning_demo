//
// Created by xyz on 22-5-11.
//
#include "xyz_knowledge_base/data_types.h"
#include "common.h"

namespace planning_node {
    MemPool<State> pool(204800);


    const string &MetaPredicate::data() noexcept {
        if (!repr.empty()) return repr;
        repr = name;
        repr.push_back('(');
        if (!types.empty()) repr.append(types[0]);

        for (int i = 1; i < types.size(); ++i) {
            repr.append(", ");
            repr.append(types[i]);
        }
        repr.push_back(')');
        return repr;
    }

    [[nodiscard]] string Predicate::data() const noexcept {
//            if (!repr.empty()) return repr;

        string repr;
        if (neg) repr.push_back('-');
        repr.append(meta->name);
        repr.push_back('(');
        if (!parameters.empty()) repr.append(parameters[0]);

        for (int i = 1; i < parameters.size(); ++i) {
            repr.append(", ");
            repr.append(parameters[i]);
        }
        repr.push_back(')');
        return repr;
    }

    const string &MetaAction::data() noexcept {
        if (!repr.empty()) return repr;
        repr.append(name);
        repr.push_back('(');
        if (!types.empty()) repr.append(types[0]);

        for (int i = 1; i < types.size(); ++i) {
            repr.append(", ");
            repr.append(types[i]);
        }
        repr.push_back(')');
        return repr;
    }

    string Action::data() const noexcept {
        string repr;
        repr.append(meta->name);
        repr.push_back('(');
        if (!paras.empty()) repr.append(paras[0]);

        for (int i = 1; i < paras.size(); ++i) {
            repr.append(", ");
            repr.append(paras[i]);
        }
        repr.push_back(')');
        return repr;
    }

    string State::data() noexcept {
        return fmt::format("{}", state);
    }

    StatePtr State::apply(const ActionPtr &action) noexcept {
//        if (!canApply(action)) {
//            return {nullptr};
//        }

//        auto newState = make_shared<State>(shared_from_this(), action);
        auto newState = shared_ptr<State>(new State(shared_from_this(), action));
//            for (const auto& eff : action->effects) {
//                if (newState->contains(-eff)) {
//                    newState->remove(-eff);
//                }
//                newState->add(eff);
//            }
        for (const auto &eff: action->effects) {
            if (newState->contains(-eff)) {
                newState->remove(-eff);
            }
        }
        for (const auto &eff: action->effects) {
            newState->add(eff);
        }
        return newState;
    }

    StatePtr State::applyRelevant(const ActionPtr &action) noexcept {
        if (!isRelevant(action)) return {nullptr};

        auto newState = make_shared<State>(shared_from_this(), action);
        // newS = (s - ADD(a)) \cup PRECOND(a)
        for (const auto &e: action->effects) {
//                if (e.neg) continue;
            newState->remove(e);
        }
        for (const auto &c: action->conditions) {
            newState->add(c);
        }

        return newState;
    }

    bool State::isRelevant(const ActionPtr &action) const noexcept {
        auto hasCommit = false;
        auto conflict = false;
        for (const auto &p: action->effects) {
            if (this->contains(p)) hasCommit = true;
//                if ((this->contains(-p)) || (!hasCommit && this->contains(p.meta->name))) conflict = true;
            if (this->contains(-p)) conflict = true;
        }
        return hasCommit && !conflict;
    }

    bool State::contains(const string &predicateName) const noexcept {
        return std::any_of(state.begin(), state.end(), [&predicateName](const Predicate &p) -> bool {
            return p.meta->name == predicateName;
        });
    }

    bool State::canApply(const ActionPtr &action) const noexcept {
        // run_info("Action {}", action);
        // run_info("Action pre: {}", this->state);
        // run_info("Action cond: {}", action->conditions);
        // run_info("Action cond not: {}", action->conditionsShouldNotSeen);
        // run_info("State: {}", state);
        for (const auto &c: action->conditions) {
            if (!this->contains(c)) return false;
        }
        for (const auto &c: action->conditionsShouldNotSeen) {
            if (this->contains(c) || this->contains(-c)) {
                return false;
            }
        }
        return true;
        // bool conditionSatisfied = all_of(action->conditions.begin(), action->conditions.end(), [this](const auto& p) {
        //     return this->contains(p);
        // });
        // bool conditionsShouldNotSeenSatisfied = all_of(action->conditionsShouldNotSeen.begin(), action->conditionsShouldNotSeen.end(), [this](const auto& p) {
        //     return !this->contains(p) && !this->contains(-p);
        // });
        // return conditionSatisfied && conditionsShouldNotSeenSatisfied;
    }

    bool State::contains(const State &s) const noexcept {
//        for (const auto& p : s.state) {
//            if (!contains(p)) return false;
//        }
        return std::all_of(s.state.begin(), s.state.end(), [this](const auto &p) -> bool {
            return this->contains(p);
        });
//        return true;
    }


    void *State::operator new(size_t size) {

//        cout << "alloc" << endl;
        return pool.allocate();
    }

    void State::operator delete(void *p, size_t size) {
        pool.deallocate(static_cast<State *>(p));
    }
}