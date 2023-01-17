//
// Created by xyz on 22-5-31.
//

#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ActionInterface/WatchTable.h"


namespace tiago_demo {

    bool WatchTableAction::concreteCallback(const xyz_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        std::unordered_map<std::string, std::string> boundParameters;
        for (auto& typed_parameter: params.typed_parameters) {
            for (const auto& [paraName, paraValue]: msg->parameters) {
                if (typed_parameter.key == paraName) {
                    boundParameters[paraName] = paraValue;
                    break;
                }
            }
        }

        std::string current_table = "table1";
        for (const auto& para: msg->parameters) {
            // watch_table(r, t, wp, obj)
            // eff: on_table(obj, t) -on_table(obj, t)
            if (para.key == "t" && para.value == current_table) {
                // 在table1上
                for (int i = 0; i < op.at_end_add_effects.size(); ++i) {
                    if (op.at_end_add_effects[i].name == "on_table") {
                        if (boundParameters[op.at_end_add_effects[i].typed_parameters[1].key] == current_table) {
                            at_end_add_effects_results[i] = true;
                        } else {
                            at_end_add_effects_results[i] = false;
                        }
                    }
                }
                for (int i = 0; i < op.at_end_del_effects.size(); ++i) {
                    if (op.at_end_del_effects[i].name == "on_table") {
                        if (boundParameters[op.at_end_del_effects[i].typed_parameters[1].key] != current_table) {
                            at_end_del_effects_results[i] = true;
                        } else {
                            at_end_del_effects_results[i] = false;
                        }
                    }
                }
            } else if (para.key == "t" && para.value != current_table) {
                // 在table1上
                for (int i = 0; i < op.at_end_del_effects.size(); ++i) {
                    if (op.at_end_del_effects[i].name == "on_table") {
                        for (const auto& p: op.at_end_del_effects[i].typed_parameters) {
                            if (p.key == para.key && boundParameters[p.key] != current_table) {
                                at_end_del_effects_results[i] = true;
                            } else {
                                at_end_del_effects_results[i] = false;
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

    WatchTableAction::WatchTableAction(ros::NodeHandle& nh) : nh_(nh) {
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "action_watch_table");
    ros::NodeHandle nh("~");
    std::string actionserver;

    tiago_demo::WatchTableAction rpmb(nh);
    ROS_INFO("(%s): ready to receive.", ros::this_node::getName().c_str());


    rpmb.runActionInterface();
    return 0;
}