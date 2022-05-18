//
// Created by xyz on 2022/3/25.
//

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

#include <planning_node/parser.h>



TEST(parser, normal) {

    auto p_normal = planning_node::parser::parsePredicate("goto_waypoint(wp1,wp2)", false);
    ASSERT_EQ(p_normal.neg, false);
    ASSERT_EQ(p_normal.name, "goto_waypoint");
    ASSERT_EQ(p_normal.parameters.size(), 2);
    ASSERT_EQ(p_normal.parameters[0], "wp1");
    ASSERT_EQ(p_normal.parameters[1], "wp2");
    auto p_normal_neg = planning_node::parser::parsePredicate("-goto_waypoint(wp1,wp2)", false);
    ASSERT_EQ(p_normal_neg.neg, true);

    p_normal = planning_node::parser::parsePredicate("g12341235()", false);
    ASSERT_EQ(p_normal.name, "g12341235");
    ASSERT_EQ(p_normal.parameters.size(), 0);

    p_normal = planning_node::parser::parsePredicate("g12341235(   )", false);
    ASSERT_EQ(p_normal.name, "g12341235");
    ASSERT_EQ(p_normal.parameters.size(), 0);
}

TEST(parser, spaces) {
    auto p_normal = planning_node::parser::parsePredicate("goto_waypoint(   wp1   ,   wp2   )", false);
    ASSERT_EQ(p_normal.name, "goto_waypoint");
    ASSERT_EQ(p_normal.parameters.size(), 2);
    ASSERT_EQ(p_normal.parameters[0], "wp1");
    ASSERT_EQ(p_normal.parameters[1], "wp2");

    p_normal = planning_node::parser::parsePredicate("goto_waypoint(   wp1   ,   wp2   )   ", false);
    ASSERT_EQ(p_normal.name, "goto_waypoint");
    ASSERT_EQ(p_normal.parameters.size(), 2);
    ASSERT_EQ(p_normal.parameters[0], "wp1");
    ASSERT_EQ(p_normal.parameters[1], "wp2");

    auto p_empty = planning_node::parser::parsePredicate("goto wp(wp1, wp2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("goto_wp (wp1, wp2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);
}

TEST(parser, invalid) {
    auto p_normal = planning_node::parser::parsePredicate("goto_waypoint(   wp1   ,   wp2   )", false);
    ASSERT_EQ(p_normal.name, "goto_waypoint");
    ASSERT_EQ(p_normal.parameters.size(), 2);
    ASSERT_EQ(p_normal.parameters[0], "wp1");
    ASSERT_EQ(p_normal.parameters[1], "wp2");

    auto p_empty = planning_node::parser::parsePredicate("goto wp(wp1, wp2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("goto_wp (wp1, wp2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("goto_wp(w p1, w p2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("_goto_wp(w p1, w p2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("1233goto_wp(w p1, w p2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("goto_wp(wp1!@#$%^, wp2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("goto_wp(wp1,, wp2)", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);

    p_empty = planning_node::parser::parsePredicate("goto_wp(wp1, wp2", false);
    ASSERT_EQ(p_empty.name.empty(), true);
    ASSERT_EQ(p_empty.parameters.empty(), true);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "parser_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}