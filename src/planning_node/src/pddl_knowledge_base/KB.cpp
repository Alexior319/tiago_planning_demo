//
// Created by xyz on 22-6-13.
//


#include <pddl_knowledge_base/KB.h>

namespace KCL_rosplan {

    /**
     * Main loop of the knowledge base. Calls ros::spinOnce() and monitors timed-initial-literals.
     */
    void KnowledgeBase::runKnowledgeBase() {

        // loop
        ros::Rate loopRate(_kb_rate);
        while (ros::ok()) {
            if (running) {
                // update TILs
                std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator tit = model_timed_initial_literals.begin();
                for (; tit != model_timed_initial_literals.end();) {
                    if (tit->initial_time <= ros::Time::now()) {
                        if (tit->is_negative) {
                            tit->is_negative = false;
                            removeKnowledge(*tit);
                        } else {
                            addKnowledge(*tit);
                        }
                        model_timed_initial_literals.erase(tit);
                    } else {
                        tit++;
                    }
                }
            }
            // services
            loopRate.sleep();
            ros::spinOnce();
        }
    }

    /*-----------------*/
    /* knowledge query */
    /*-----------------*/

    bool KnowledgeBase::queryKnowledge(xyz_knowledge_msgs::KnowledgeQueryService::Request& req, xyz_knowledge_msgs::KnowledgeQueryService::Response& res) {

        res.all_true = true;
        std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator iit;
        for (iit = req.knowledge.begin(); iit != req.knowledge.end(); iit++) {

            bool present = false;
            switch (iit->knowledge_type) {

                case xyz_knowledge_msgs::KnowledgeItem::INSTANCE:
                {
                    // check if instance exists
                    std::vector<std::string>::iterator sit;
                    sit = find(model_instances[iit->instance_type].begin(), model_instances[iit->instance_type].end(), iit->instance_name);
                    present = (sit != model_instances[iit->instance_type].end());

                    // check if instance exists as a constant
                    if (!present) {
                        sit = find(domain_constants[iit->instance_type].begin(), domain_constants[iit->instance_type].end(), iit->instance_name);
                        present = (sit != domain_constants[iit->instance_type].end());
                    }
                }
                    break;

                case xyz_knowledge_msgs::KnowledgeItem::FUNCTION:
                {
                    // check if function exists and has the correct value
                    std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
                    for (pit = model_functions.begin(); pit != model_functions.end(); pit++) {
                        if (KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
                            present = true;
                            break;
                        }
                    }
                }
                    break;

                case xyz_knowledge_msgs::KnowledgeItem::FACT:
                {
                    // check if fact is true
                    std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
                    for (pit = model_facts.begin(); pit != model_facts.end(); pit++) {
                        if (KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
                            present = true;
                            break;
                        }
                    }
                }
                    break;

                case xyz_knowledge_msgs::KnowledgeItem::INEQUALITY:
                {
                    // evaluate inequality
                    present = KnowledgeComparitor::inequalityTrue(*iit, model_functions);
                }
                    break;
            }

            if (!present) {
                res.all_true = false;
                res.results.push_back(false);
                res.false_knowledge.push_back(*iit);
            } else {
                res.results.push_back(true);
            }
        }

        return true;
    }

    /*--------------------*/
    /* adding constraints */
    /*--------------------*/

    bool KnowledgeBase::updateKnowledgeConstraintsOneOf(xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request& req, xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response& res) {

        int count = 0;
        for (int i = 0;i < req.knowledge.size();i++) {
            // check if fact is true
            std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
            for (pit = model_facts.begin(); pit != model_facts.end(); pit++) {
                if (KnowledgeComparitor::containsKnowledge(req.knowledge[i], *pit)) {
                    count++;
                }
            }
        }
        if (count > 1)    ROS_ERROR("KCL: (%s) Warning: more than one Knowledge Item is true in new OneOf constraint!", ros::this_node::getName().c_str());
        model_oneof_constraints.push_back(req.knowledge);
        res.success = true;
        return true;
    }

    /*------------------*/
    /* knowledge update */
    /*------------------*/

    bool KnowledgeBase::updateKnowledge(xyz_knowledge_msgs::KnowledgeUpdateService::Request& req, xyz_knowledge_msgs::KnowledgeUpdateService::Response& res) {

        ros::Time time = ros::Time::now();

        switch (req.update_type) {

            case xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE:

                // check initial time of knowledge
                if (req.knowledge.initial_time > time) {
                    // add to TILs for later
                    model_timed_initial_literals.push_back(req.knowledge);
                } else {
                    // update time and add to state now
                    if (req.knowledge.initial_time.toSec() == 0) req.knowledge.initial_time = time;
                    addKnowledge(req.knowledge);
                }
                break;

            case xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL:
                addMissionGoal(req.knowledge);
                break;

            case xyz_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE:
                removeKnowledge(req.knowledge);
                break;

            case xyz_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL:
                removeMissionGoal(req.knowledge);
                break;

            case xyz_knowledge_msgs::KnowledgeUpdateService::Request::ADD_METRIC:
                addMissionMetric(req.knowledge);
                break;

            case xyz_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_METRIC:
                removeMissionMetric(req.knowledge);
                break;
        }

        res.success = true;
        return true;
    }

    //bool KnowledgeBase::updateKnowledgeArray(xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res) {
    bool KnowledgeBase::updateKnowledgeArray(ros::ServiceEvent<xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request, xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response>& event) {

        xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Request req = event.getRequest();
        xyz_knowledge_msgs::KnowledgeUpdateServiceArray::Response& res = event.getResponse(); // Note this is a reference! It acts as in a normal service method, where the response is passed as a reference

        res.success = true;

        xyz_knowledge_msgs::KnowledgeUpdateService srv;
        for (int i = 0; i < req.update_type.size() && i < req.knowledge.size(); i++) {

            srv.request.update_type = req.update_type[i];
            srv.request.knowledge = req.knowledge[i];

            updateKnowledge(srv.request, srv.response);
            res.success = res.success && srv.response.success;
        }

        xyz_knowledge_msgs::StatusUpdate update_msg;
        update_msg.last_update_time = ros::Time::now();
        update_msg.last_update_client = event.getConnectionHeader()["callerid"];
        status_pub.publish(update_msg);

        return true;
    }

    /*----------------*/
    /* removing items */
    /*----------------*/

    /**
     * remove everything
     */
    bool KnowledgeBase::clearKnowledge(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

        xyz_knowledge_msgs::KnowledgeItem empty;

        ROS_INFO("KCL: (%s) Removing whole model", ros::this_node::getName().c_str());

        // model
        model_timed_initial_literals.clear();
        model_constants.clear();
        model_instances.clear();
        model_facts.clear();
        model_functions.clear();
        model_goals.clear();
        model_metric = empty;
    }

    /**
     * service method for removing the state
     */
    void KnowledgeBase::removeKnowledge(xyz_knowledge_msgs::KnowledgeItem& msg) {

        // check if knowledge is timed, and remove from timed list
        ros::Time time = ros::Time::now();
        if (msg.initial_time > time) {
            std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
            for (pit = model_timed_initial_literals.begin(); pit != model_timed_initial_literals.end(); ) {
                if (msg.initial_time == pit->initial_time && KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                    ROS_INFO("KCL: (%s) Removing timed attribute (%s)", msg.attribute_name.c_str(), ros::this_node::getName().c_str());
                    model_timed_initial_literals.erase(pit);
                    return;
                } else {
                    pit++;
                }
            }

        }

        // otherwise remove from correct part of the state
        switch (msg.knowledge_type) {

            case xyz_knowledge_msgs::KnowledgeItem::INSTANCE:
            {
                // search for instance
                std::vector<std::string>::iterator iit;
                for (iit = model_instances[msg.instance_type].begin(); iit != model_instances[msg.instance_type].end(); ) {

                    std::string name = *iit;
                    if (name.compare(msg.instance_name) == 0 || msg.instance_name.compare("") == 0) {
                        // remove instance from knowledge base
                        ROS_INFO("KCL: (%s) Removing instance (%s, %s)",
                                 ros::this_node::getName().c_str(),
                                 msg.instance_type.c_str(),
                                 (msg.instance_name.compare("") == 0) ? "ALL" : msg.instance_name.c_str());
                        iit = model_instances[msg.instance_type].erase(iit);
                        if (iit != model_instances[msg.instance_type].begin()) iit--;

                        // remove affected domain attributes
                        std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
                        for (pit = model_facts.begin(); pit != model_facts.end(); ) {
                            if (KnowledgeComparitor::containsInstance(*pit, name)) {
                                ROS_INFO("KCL: (%s) Removing domain attribute (%s)", ros::this_node::getName().c_str(), pit->attribute_name.c_str());
                                pit = model_facts.erase(pit);
                            } else {
                                pit++;
                            }
                        }
                        for (pit = model_functions.begin(); pit != model_functions.end(); ) {
                            if (KnowledgeComparitor::containsInstance(*pit, name)) {
                                ROS_INFO("KCL: (%s) Removing domain attribute (%s)", ros::this_node::getName().c_str(), pit->attribute_name.c_str());
                                pit = model_functions.erase(pit);
                            } else {
                                pit++;
                            }
                        }

                    } else {
                        iit++;
                    }
                }
            }
                break;

            case xyz_knowledge_msgs::KnowledgeItem::FUNCTION:
            {
                // remove domain attribute (function) from knowledge base
                std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
                for (pit = model_functions.begin(); pit != model_functions.end(); ) {
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        ROS_INFO("KCL: (%s) Removing domain attribute (%s)", ros::this_node::getName().c_str(), msg.attribute_name.c_str());
                        pit = model_functions.erase(pit);
                    } else {
                        pit++;
                    }
                }
            }
                break;

            case xyz_knowledge_msgs::KnowledgeItem::FACT:
            {
                removeFact(msg);
            }
                break;

        }
    }

    void KnowledgeBase::removeFact(const xyz_knowledge_msgs::KnowledgeItem& msg) {
        // remove domain attribute (predicate) from knowledge base
        std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
        for (pit = model_facts.begin(); pit != model_facts.end(); ) {
            if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                ROS_INFO("KCL: (%s) Removing Fact (%s,%i)", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), msg.is_negative);
                pit = model_facts.erase(pit);
            } else {
                pit++;
            }
        }
    }

    /**
     * remove mission goal
     */
    void KnowledgeBase::removeMissionGoal(xyz_knowledge_msgs::KnowledgeItem& msg) {

        std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator git;
        for (git = model_goals.begin(); git != model_goals.end(); ) {
            if (KnowledgeComparitor::containsKnowledge(msg, *git)) {
                ROS_INFO("KCL: (%s) Removing goal (%s)", ros::this_node::getName().c_str(), git->attribute_name.c_str());
                git = model_goals.erase(git);
            } else {
                git++;
            }
        }
    }

    /**
     * remove mission metric
     */
    void KnowledgeBase::removeMissionMetric(xyz_knowledge_msgs::KnowledgeItem& msg) {
        xyz_knowledge_msgs::KnowledgeItem empty;
        ROS_INFO("KCL: (%s) Removing metric", ros::this_node::getName().c_str());
        model_metric = empty;
    }

    /*--------------*/
    /* adding items */
    /*--------------*/

    /*
     * add an instance, fact, or function to the knowledge base
     */
    void KnowledgeBase::addKnowledge(xyz_knowledge_msgs::KnowledgeItem& msg) {

        switch (msg.knowledge_type) {

            case xyz_knowledge_msgs::KnowledgeItem::INSTANCE:
            {
                // check if instance is already in knowledge base
                std::vector<std::string>::iterator iit;
                iit = find(domain_constants[msg.instance_type].begin(), domain_constants[msg.instance_type].end(), msg.instance_name);
                if (iit == domain_constants[msg.instance_type].end()) {

                    iit = find(model_instances[msg.instance_type].begin(), model_instances[msg.instance_type].end(), msg.instance_name);

                    // add instance
                    if (iit == model_instances[msg.instance_type].end()) {
                        ROS_INFO("KCL: (%s) Adding instance (%s, %s)", ros::this_node::getName().c_str(), msg.instance_type.c_str(), msg.instance_name.c_str());
                        model_instances[msg.instance_type].push_back(msg.instance_name);
                    }
                } else {
                    ROS_WARN("KCL: (%s) instance (%s, %s) already exists as a constant.", ros::this_node::getName().c_str(), msg.instance_type.c_str(), msg.instance_name.c_str());
                }
            }
                break;

            case xyz_knowledge_msgs::KnowledgeItem::FACT:
            {
                // create parameter string for ROS_INFO messages
                std::string param_str;
                for (size_t i = 0; i < msg.values.size(); ++i) {
                    param_str += " " + msg.values[i].value;
                }

                // check if fact exists already
                std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
                for (pit = model_facts.begin(); pit != model_facts.end(); pit++) {
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        ROS_WARN("KCL: (%s) fact (%s%s) already exists", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str());
                        return;
                    }
                    msg.is_negative = 1 - msg.is_negative;
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        ROS_INFO("KCL: (%s) Setting fact (%s%s) is_negative=%i", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str(), (1 - msg.is_negative));
                        pit->is_negative = 1 - pit->is_negative;
                        return;
                    }
                    msg.is_negative = 1 - msg.is_negative; // Reset it to the original value
                }

                // add fact
                ROS_INFO("KCL: (%s) Adding fact (%s%s, %i)", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str(), msg.is_negative);
                model_facts.push_back(msg);

            }
                break;

            case xyz_knowledge_msgs::KnowledgeItem::FUNCTION:
            {
                // create parameter string for ROS_INFO messages
                std::string param_str;
                for (size_t i = 0; i < msg.values.size(); ++i) {
                    param_str += " " + msg.values[i].value;
                }

                // check if function already exists
                std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
                for (pit = model_functions.begin(); pit != model_functions.end(); pit++) {
                    if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                        ROS_INFO("KCL: (%s) Updating function (= (%s%s) %f)", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str(), msg.function_value);
                        pit->function_value = msg.function_value;
                        return;
                    }
                }
                ROS_INFO("KCL: (%s) Adding function (= (%s%s) %f)", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str(), msg.function_value);
                model_functions.push_back(msg);
            }
                break;

        }
    }

    /*
     * add mission goal to knowledge base
     */
    void KnowledgeBase::addMissionGoal(xyz_knowledge_msgs::KnowledgeItem& msg) {

        // create parameter string for ROS_INFO messages
        std::string param_str;
        for (size_t i = 0; i < msg.values.size(); ++i) {
            param_str += " " + msg.values[i].value;
        }

        // check to make sure goal is not already added
        std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator pit;
        for (pit = model_goals.begin(); pit != model_goals.end(); pit++) {
            if (KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                ROS_WARN("KCL: (%s) Goal (%s%s) already posted", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str());
                return;
            }
        }

        // add goal
        ROS_INFO("KCL: (%s) Adding mission goal (%s%s)", ros::this_node::getName().c_str(), msg.attribute_name.c_str(), param_str.c_str());
        model_goals.push_back(msg);
    }

    /*
    * add mission metric to knowledge base
    */
    void KnowledgeBase::addMissionMetric(xyz_knowledge_msgs::KnowledgeItem& msg) {
        ROS_INFO("KCL: (%s) Adding mission metric", ros::this_node::getName().c_str());
        model_metric = msg;
    }

    /*----------------*/
    /* fetching items */
    /*----------------*/

    bool KnowledgeBase::getInstances(xyz_knowledge_msgs::GetInstanceService::Request& req, xyz_knowledge_msgs::GetInstanceService::Response& res) {

        // fetch the instances of the correct type
        if ("" == req.type_name) {
            std::map<std::string, std::vector<std::string> >::iterator iit;
            // objects
            for (iit = model_instances.begin(); iit != model_instances.end(); iit++) {
                for (size_t j = 0; j < iit->second.size(); j++)
                    res.instances.push_back(iit->second[j]);
            }
            // constants
            for (iit = domain_constants.begin(); iit != domain_constants.end(); iit++) {
                for (size_t j = 0; j < iit->second.size(); j++)
                    res.instances.push_back(iit->second[j]);
            }
        } else {
            std::map<std::string, std::vector<std::string> >::iterator iit;
            // objects
            iit = model_instances.find(req.type_name);
            if (iit != model_instances.end()) {
                for (size_t j = 0; j < iit->second.size(); j++)
                    res.instances.push_back(iit->second[j]);
            }
            // constants
            iit = domain_constants.find(req.type_name);
            if (iit != domain_constants.end()) {
                for (size_t j = 0; j < iit->second.size(); j++)
                    res.instances.push_back(iit->second[j]);
            }
        }

        return true;
    }

    bool KnowledgeBase::getPropositions(xyz_knowledge_msgs::GetAttributeService::Request& req, xyz_knowledge_msgs::GetAttributeService::Response& res) {

        // fetch the knowledgeItems of the correct attribute
        for (size_t i = 0; i < model_facts.size(); i++) {
            if (0 == req.predicate_name.compare(model_facts[i].attribute_name) || "" == req.predicate_name) {
                res.attributes.push_back(model_facts[i]);
            }
        }

        return true;
    }

    bool KnowledgeBase::getFunctions(xyz_knowledge_msgs::GetAttributeService::Request& req, xyz_knowledge_msgs::GetAttributeService::Response& res) {

        // ...or fetch the knowledgeItems of the correct function
        for (size_t i = 0; i < model_functions.size(); i++) {
            if (0 == req.predicate_name.compare(model_functions[i].attribute_name) || "" == req.predicate_name) {
                res.attributes.push_back(model_functions[i]);
            }
        }

        return true;
    }


    bool KnowledgeBase::getTimedKnowledge(xyz_knowledge_msgs::GetAttributeService::Request& req, xyz_knowledge_msgs::GetAttributeService::Response& res) {

        // all TILs and TIFs
        std::vector<xyz_knowledge_msgs::KnowledgeItem>::iterator tit = model_timed_initial_literals.begin();
        for (; tit != model_timed_initial_literals.end(); tit++) {
            if (0 == req.predicate_name.compare(tit->attribute_name) || "" == req.predicate_name) {
                res.attributes.push_back(*tit);
            }
        }

        return true;
    }

    bool KnowledgeBase::getGoals(xyz_knowledge_msgs::GetAttributeService::Request& req, xyz_knowledge_msgs::GetAttributeService::Response& res) {

        for (size_t i = 0; i < model_goals.size(); i++)
            res.attributes.push_back(model_goals[i]);
        return true;
    }

    bool KnowledgeBase::getMetric(xyz_knowledge_msgs::GetMetricService::Request& req, xyz_knowledge_msgs::GetMetricService::Response& res) {
        res.metric = model_metric;
        return true;
    }

    bool KnowledgeBase::setSensedPredicate(xyz_knowledge_msgs::SetNamedBool::Request& req, xyz_knowledge_msgs::SetNamedBool::Response& res) {
        sensed_predicates[req.name] = req.value;
        res.success = true;
        return true;
    }

    KnowledgeBase::KnowledgeBase(ros::NodeHandle& n) {
        _nh = n;

        // Start all the servers
        // fetch domain info
        domainServer1 = _nh.advertiseService("domain/name", &KCL_rosplan::KnowledgeBase::getDomainName, this);
        domainServer2 = _nh.advertiseService("domain/types", &KCL_rosplan::KnowledgeBase::getTypes, this);
        domainServer3 = _nh.advertiseService("domain/predicates", &KCL_rosplan::KnowledgeBase::getPredicates, this);
        domainServer4 = _nh.advertiseService("domain/functions", &KCL_rosplan::KnowledgeBase::getFunctionPredicates, this);
        domainServer5 = _nh.advertiseService("domain/operators", &KCL_rosplan::KnowledgeBase::getOperators, this);
        domainServer6 = _nh.advertiseService("domain/operator_details", &KCL_rosplan::KnowledgeBase::getOperatorDetails, this);
        domainServer7 = _nh.advertiseService("domain/predicate_details", &KCL_rosplan::KnowledgeBase::getPredicateDetails, this);

        // query knowledge
        queryServer = _nh.advertiseService("query_state", &KCL_rosplan::KnowledgeBase::queryKnowledge, this);

        // update knowledge
        updateServer0 = _nh.advertiseService("clear", &KCL_rosplan::KnowledgeBase::clearKnowledge, this);
        updateServer1 = _nh.advertiseService("update", &KCL_rosplan::KnowledgeBase::updateKnowledge, this);
        updateServer2 = _nh.advertiseService("update_array", &KCL_rosplan::KnowledgeBase::updateKnowledgeArray, this);
        updateServer3 = _nh.advertiseService("update_constraints_oneof", &KCL_rosplan::KnowledgeBase::updateKnowledgeConstraintsOneOf, this);

        // fetch knowledge
        stateServer1 = _nh.advertiseService("state/instances", &KCL_rosplan::KnowledgeBase::getInstances, this);
        stateServer2 = _nh.advertiseService("state/propositions", &KCL_rosplan::KnowledgeBase::getPropositions, this);
        stateServer3 = _nh.advertiseService("state/functions", &KCL_rosplan::KnowledgeBase::getFunctions, this);
        stateServer4 = _nh.advertiseService("state/timed_knowledge", &KCL_rosplan::KnowledgeBase::getTimedKnowledge, this);
        stateServer5 = _nh.advertiseService("state/goals", &KCL_rosplan::KnowledgeBase::getGoals, this);
        stateServer6 = _nh.advertiseService("state/metric", &KCL_rosplan::KnowledgeBase::getMetric, this);

        // set sensed predicates
        senseServer = _nh.advertiseService("update_sensed_predicates", &KCL_rosplan::KnowledgeBase::setSensedPredicate, this);

        // status publishers
        status_pub = _nh.advertise<xyz_knowledge_msgs::StatusUpdate>("status/update", 100);

        _nh.param("kb_rate", _kb_rate, 100.0);
    }

    void KnowledgeBase::setup(KB kb_type, const std::string& domain_path, const std::string& problem_path, const std::string& data_path) {
        this->kb_type = kb_type;
        this->domainPath = domain_path;
        this->problemPath = problem_path;
        this->dataPath = data_path;
    }


} // close namespace


