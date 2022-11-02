//
// Created by xyz on 2022/3/22.
//

#ifndef PLANNING_NODE_DATA_TYPES_H
#define PLANNING_NODE_DATA_TYPES_H

#include <utility>
#include <vector>
#include <string>
#include <string_view>
#include <unordered_set>
#include <algorithm>
#include <memory>

#include <fmt/core.h>
#include <fmt/ranges.h>

namespace planning_node {
    using namespace std;

//    typedef string Type;
    using Type = string;
//    typedef vector<string> State;

    struct Object {
        Type type;
        string name;
    };

    struct MetaPredicate {
        string name;
        vector<Type> types;
        vector<string> names;
        string repr;

        MetaPredicate() = default;

        MetaPredicate(string& name, vector<Type>& types, vector<string>& names) : name(name), types(types), names(names) {}

        const string& data() noexcept;
    };


    using MetaPredicatePtr = std::shared_ptr<MetaPredicate>;

    struct Predicate {
        MetaPredicatePtr meta;
        vector<string> parameters;
        bool neg = false;

        bool operator==(const Predicate& p) const noexcept {
            if (meta != p.meta) return false;
            for (int i = 0; i < parameters.size(); ++i) {
                if (parameters[i] != p.parameters[i]) return false;
            }
            if (neg != p.neg || meta->name != p.meta->name) return false;
            return true;
        }

        Predicate() noexcept = default;

        Predicate(Predicate&& pred) noexcept: meta(move(pred.meta)), parameters(move(pred.parameters)),
                                              neg(pred.neg) {

        }

        Predicate(const Predicate& pred) = default;

        Predicate& operator=(Predicate&& p) noexcept {
            parameters = std::move(p.parameters);
            neg = p.neg;
            return *this;
        }

        Predicate& operator=(const Predicate& p) = default;

        Predicate operator-() const noexcept {
            auto ret = *this;
            ret.neg = !ret.neg;
            return ret;
        }

        void setParameters(vector<string>& p) noexcept {
            parameters = std::move(p);
        }

        void negate() noexcept {
            neg = !neg;
        }

        [[nodiscard]] string data() const noexcept;
    };

    using PredicatePtr = std::shared_ptr<Predicate>;

    struct PredicateIdx {
        bool neg = false; // 是否为负文字
        MetaPredicatePtr meta; // 元谓词（里面包括谓词名称和参数类型）
        vector<int> pos; // 这个谓词中各个参数对应的位置
        PredicateIdx(bool neg, MetaPredicatePtr meta) : neg(neg), meta(std::move(meta)) {}
    };


    struct PredicateHasher {
        size_t operator()(const Predicate& p) const noexcept {
            return std::hash<string>{}(p.data());
        }
    };


    struct MetaAction {
        string name; // 动作名称
        vector<Type> types; // 各参数类型
        vector<string> names; // 各参数名称
        vector<PredicateIdx> metaConditions;
        vector<PredicateIdx> metaConditionsShouldNotSeen;
        vector<PredicateIdx> metaEffects;
        string repr;


        MetaAction() = default;

        MetaAction(string name, const vector<Type>& types, const vector<string>& names) : name(std::move(name)), types(types), names(names) {}

        const string& data() noexcept;
    };

    using MetaActionPtr = std::shared_ptr<MetaAction>;


    struct Action {
        MetaActionPtr meta;
        vector<string> paras;
        vector<Predicate> conditions;
        vector<Predicate> conditionsShouldNotSeen;
        vector<Predicate> effects;

        Action() noexcept: meta(nullptr) {}

        Action(Action&& action) noexcept: conditions(std::move(action.conditions)),
                                          conditionsShouldNotSeen(std::move(action.conditionsShouldNotSeen)),
                                          effects(std::move(action.effects)),
                                          paras(std::move(action.paras)),
                                          meta(std::move(action.meta)) {}

        Action(Action const& a) {

        }

        Action& operator=(Action&& action) noexcept {
            if (this != std::addressof(action)) {
                conditions = std::move(action.conditions);
                conditionsShouldNotSeen = std::move(action.conditionsShouldNotSeen);
                effects = std::move(action.effects);
                paras = std::move(action.paras);
                meta = move(action.meta);
            }
            return *this;
        }

        [[nodiscard]] string data() const noexcept;
    };


    using ActionPtr = std::shared_ptr<Action>;


    struct State;
    using StatePtr = std::shared_ptr<State>;

    struct State : enable_shared_from_this<State> {
        unordered_set<Predicate, PredicateHasher> state;
        ActionPtr appliedAction;
        StatePtr parentState;

        State() = default;

        State(const StatePtr& parent, ActionPtr action) : state(parent->state), parentState(parent),
                                                                 appliedAction(std::move(action)) {}

        // s |= s'
        bool contains(const State& s) const noexcept;

        bool contains(const StatePtr& s) const noexcept {
            return contains(*s);
        }

        // p \in s
        bool contains(const Predicate& p) const noexcept {
            return state.count(p);
        }

        bool contains(const string& predicateName) const noexcept;

        // s |= action.conditions
        bool canApply(const ActionPtr& action) const noexcept;

        bool isRelevant(const ActionPtr& action) const noexcept;

        void add(const Predicate& p) noexcept {
            state.emplace(p);
        }

        void remove(const Predicate& p) noexcept {
            if (state.count(p)) state.erase(p);
        }

        StatePtr apply(const ActionPtr& action) noexcept;

        StatePtr applyRelevant(const ActionPtr& action) noexcept;

        string data() noexcept;
    };

}

namespace fmt {
    template<>
    struct formatter<planning_node::Action> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(planning_node::Action& a, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", a.data());
        }
    };

    template<>
    struct formatter<planning_node::MetaPredicatePtr> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const planning_node::MetaPredicatePtr& ptr, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", ptr->data());
        }
    };

    template<>
    struct formatter<planning_node::MetaActionPtr> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const planning_node::MetaActionPtr& ptr, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", ptr->data());
        }
    };

    template<>
    struct formatter<planning_node::ActionPtr> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const planning_node::ActionPtr& ptr, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", ptr->data());
        }
    };

    template<>
    struct formatter<planning_node::Predicate> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const planning_node::Predicate& a, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", a.data());
        }
    };

    template<>
    struct formatter<planning_node::State> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const planning_node::State& s, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", s.state);
        }
    };

    template<>
    struct formatter<planning_node::StatePtr> {
        constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
            return ctx.begin();
        }

        template<class FormatContext>
        auto format(const planning_node::StatePtr& s, FormatContext& ctx)
        -> decltype(ctx.out()) {
            return format_to(ctx.out(), "{}", *s);
        }
    };

} // namespace fmt


#endif //PLANNING_NODE_DATA_TYPES_H
