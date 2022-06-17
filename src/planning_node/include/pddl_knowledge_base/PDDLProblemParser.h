//
// Created by xyz on 22-6-11.
//

#ifndef ROS_PROJECT_PDDLPROBLEMPARSER_H
#define ROS_PROJECT_PDDLPROBLEMPARSER_H

#include <ros/ros.h>

#include "VALfiles/FlexLexer.h"
#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"
#include "VALVisitorProblem.h"

namespace planning_node {
    class PDDLProblemParser {
    private:

    public:
        /* VAL pointers */
        VAL1_2::analysis *val_analysis;
        VAL1_2::problem *problem;

        /* Problem information */
        bool problem_parsed;
        std::string problem_name;

        /* Problem parsing */
        VAL1_2::problem *parseProblem(const std::string ProblemPath);
    };
}


#endif //ROS_PROJECT_PDDLPROBLEMPARSER_H
