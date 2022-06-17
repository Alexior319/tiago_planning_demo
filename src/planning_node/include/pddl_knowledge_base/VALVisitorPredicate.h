//
// Created by xyz on 22-6-11.
//

#ifndef ROS_PROJECT_VALVISITORPREDICATE_H
#define ROS_PROJECT_VALVISITORPREDICATE_H

#include <sstream>
#include <string>
#include <vector>

#include "VALfiles/ptree.h"
#include "VALfiles/VisitController.h"

#include "xyz_knowledge_msgs/DomainFormula.h"

namespace planning_node {

    class VALVisitorPredicate : public VAL1_2::VisitController {
    private:

    public:

        /* message */
        xyz_knowledge_msgs::DomainFormula msg;

        /* visitor methods */
        virtual void visit_pred_decl(VAL1_2::pred_decl *);
    };

} // close namespace

#endif //ROS_PROJECT_VALVISITORPREDICATE_H
