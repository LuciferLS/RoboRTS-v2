#ifndef ROBORTS_GOAL_FACTORY_H
#define ROBORTS_GOAL_FACTORY_H

#include <chrono>

#include <ros/ros.h>

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/simple_decision_tree.h"
#include "example_behavior/shoot_behavior.h"
#include "example_behavior/reload_behavior.h"

namespace goal_factory{

enum class states{
    SEARCH,
    SHOOT,
    RELOAD,
    ESCAPE,
    TOBUFF,
    CHASE,
    STOP,
};

class Goal_Factory{
public:
Goal_Factory(ros::NodeHandle nh){
    std::string config_name;
    nh.param<std::string>("decision_config_name",config_name,"decision");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/"+config_name+".prototxt";
    auto blackboard = new roborts_decision::Blackboard(full_path);
    auto chassis_executor = new roborts_decision::ChassisExecutor;
    chase_behavior          = new roborts_decision::ChaseBehavior(chassis_executor, blackboard, full_path);
    search_behavior         = new roborts_decision::SearchBehavior(chassis_executor, blackboard, full_path);
    escape_behavior         = new roborts_decision::EscapeBehavior(chassis_executor, blackboard, full_path);
    shoot_behavior          = new roborts_decision::ShootBehavior(chassis_executor, blackboard, full_path);
    reload_behavior         = new roborts_decision::ReloadBehavior(chassis_executor, blackboard, full_path);
}

~Goal_Factory();
private:
states current_state = states::STOP;
std::chrono::milliseconds execution_duration;
roborts_decision::ChaseBehavior        *chase_behavior;
roborts_decision::SearchBehavior       *search_behavior;
roborts_decision::EscapeBehavior       *escape_behavior;
roborts_decision::ShootBehavior        *shoot_behavior);
roborts_decision::ReloadBehavior       *reload_behavior;
}
}
#endif