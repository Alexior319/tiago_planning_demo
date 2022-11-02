//
// Created by xyz on 22-6-11.
//

#include <fstream>

#include "pddl_knowledge_base/PDDLParser.h"


namespace planning_node {

    bool PDDLParser::parseDomain(const string& domain_path) {
        return domain_parsed = !driver.parse(domain_path);
    }

    bool PDDLParser::parseProblem(const string& problem_path) {
        return problem_parsed = !driver.parse(problem_path);
    }

    string PDDLParser::output() const {
        stringstream ss;
        if (domain_parsed && problem_parsed) {
            ss << *(driver.domain);
            ss << *(driver.problem);
        }
        return ss.str();
    }
}
