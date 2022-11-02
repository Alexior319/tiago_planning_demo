//
// Created by xyz on 22-6-11.
//

#ifndef ROS_PROJECT_PDDLDOMAINPARSER_H
#define ROS_PROJECT_PDDLDOMAINPARSER_H

#include <ros/ros.h>
#include "pddldriver.hh"

namespace planning_node {
    using namespace std;
    class PDDLParser {
    private:
        PDDLDriver driver;
        bool domain_parsed{false};
        bool problem_parsed{false};
    public:
        PDDLParser() = default;
        bool parseDomain(const string& domain_path);
        bool parseProblem(const string& problem_path);
        string output() const;
        const Domain& getDomain() const {
            return *driver.domain;
        }
        const Problem& getProblem() const {
            return *driver.problem;
        }
    };
}


#endif //ROS_PROJECT_PDDLDOMAINPARSER_H
