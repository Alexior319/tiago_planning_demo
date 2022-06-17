//
// Created by xyz on 22-6-11.
//

#ifndef ROS_PROJECT_PDDLDOMAINPARSER_H
#define ROS_PROJECT_PDDLDOMAINPARSER_H

#include <ros/ros.h>

#include "VALfiles/ptree.h"
#include "VALfiles/FlexLexer.h"

extern int yyparse();

extern int yydebug;

namespace planning_node {
    class PDDLDomainParser {
    private:

    public:

        /* VAL pointers */
        VAL1_2::analysis *val_analysis;
        VAL1_2::domain *domain;

        /* domain information */
        bool domain_parsed;
        std::string domain_name;

        /* domain parsing */
        VAL1_2::domain *parseDomain(const std::string domainPath);
    };
}


#endif //ROS_PROJECT_PDDLDOMAINPARSER_H
