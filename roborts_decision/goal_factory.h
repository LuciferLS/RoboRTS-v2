#ifndef ROBORTS_GOAL_FACTORY_H
#define ROBORTS_GOAL_FACTORY_H

#include <chrono>
#include <string>

#include <ros/ros.h>
#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/simple_decision_tree.h"
#include "example_behavior/shoot_behavior.h"
#include "example_behavior/reload_behavior.h"
#include "example_behavior/to_buff_zone_behavior.h"

#include "roborts_sim/Countdown.h"

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
Goal_Factory(ros::NodeHandle* &nh):nh(nh){
    std::string config_name;
    nh->param<std::string>("decision_config_name",config_name,"decision");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/"+config_name+".prototxt";
    blackboard = new roborts_decision::Blackboard(full_path);
    auto chassis_executor = new roborts_decision::ChassisExecutor;
    chase_behavior          = new roborts_decision::ChaseBehavior(chassis_executor, *blackboard, full_path);
    search_behavior         = new roborts_decision::SearchBehavior(chassis_executor, *blackboard, full_path);
    escape_behavior         = new roborts_decision::EscapeBehavior(chassis_executor, *blackboard, full_path);
    shoot_behavior          = new roborts_decision::ShootBehavior(chassis_executor, *blackboard, full_path);
    reload_behavior         = new roborts_decision::ReloadBehavior(chassis_executor, *blackboard, full_path);
    tobuff_behavior         = new roborts_decision::ToBuffZoneBehavior(chassis_executor, *blackboard, full_path);
}

void run(){
    std::thread count_down_thread(&goal_factory::count_down, this);
    count_down_thread.detach();

    std::thread damage_thread(&goal_factory::damage_detect, this);
    damage_thread.detach();

    while(current_state == states::STOP){}
    std::chrono::steady_clock::time_point *game_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point start_time = *game_time;
    int escape_time = 0;

    while(current_state != states::STOP){
        ros::spinOnce();
        
        if(std::chrono::steady_clock::now() >= *game_time + execution_duration || blackboard->get_remain_time() < remain_time){
            if(blackboard->get_remain_time() < remain_time){
                remain_time = blackboard->get_remain_time();
                *game_time = start_time + std::chrono::seconds(300 - remain_time);
            }
            else{
                *game_time = *game_time + execution_duration;
            }
            update();
            switch(current_state){
                case states::SEARCH:
                    if(enemy_detected){
                        search_behavior->Cancel();
                        chase_behavior->run();
                        current_state = states::CHASE;
                    }
                    break;
                case states::CHASE:
                    if((damage && damage_armor == 1 && ammo > 0) ||
                     (blackboard->GetDistance(blackboard->GetRobotMapPose(), *enemy_pose) <= 5)){
                        chase_behavior->Cancel();
                        shoot_behavior->run();
                        current_state = states::SHOOT;
                    }
                    else if(damage && 
                    (damage_armor == 2 || damage_armor == 3 || damage_armor == 4)){
                        chase_behavior->Cancel();
                        escape_behavior->run();
                        current_state = states::ESCAPE;
                    }
                    break;
                case states::SHOOT:
                    if(damage && 
                    (damage_armor == 2 || damage_armor == 3 || damage_armor == 4)){
                        shoot_behavior->Cancel();
                        escape_behavior->run();
                        current_state = states::ESCAPE;
                    }
                    else if(ammo == 0){
                        shoot_behavior->Cancel();
                        reload_behavior->run();
                        current_state = states::RELOAD;
                    }
                    else if(!enemy_detected){
                        shoot_behavior->Cancel();
                        chase_behavior->run();
                        current_state = states::CHASE;
                    }
                    break;
                case states::RELOAD:
                    if(blackboard->is_reloading() == 0)
                        break;
                    if(ammo == 50){
                        reload_behavior->Cancel();
                        search_behavior->run();
                        current_state = states::SEARCH;
                    }
                    else if(damage && ammo == 0){
                        reload_behavior->Cancel();
                        tobuff_behavior->run();
                        current_state = states::TOBUFF;
                    }
                    break;
                case states::ESCAPE:
                    if(escape_time == 0 || escape_time == 1)
                        escape_time++;
                    else{
                        escapetime++;
                        if(ammo != 0){
                            escape_behavior->Cancel();
                            search_behavior->run();
                            escape_time = 0;
                            current_state = states::SEARCH;
                        }
                        else if(ammo == 0){
                            escape_behavior->Cancel();
                            reload_behavior->run();
                            escape_time = 0;
                            current_state = states::RELOAD;
                        }
                    }
                    break;
                case states::TOBUFF:
                    if(damage){
                        tobuff_behavior->Cancel();
                        escape_behavior->run();
                        current_state = states::ESCAPE;
                    }
                    else if(has_buff){
                        tobuff_behavior->Cancel();
                        reload_behavior->run();
                        current_state = states::RELOAD;
                    }
                    break;
            }
        }
    }
}

void count_down(){
    ros::Subscriber game_timer = n.subscribe("countdown", 1000, &goal_factory::count_down_callback, this);
    ros::spin();
}

void count_down_callback(const roborts_sim::Countdown& msg){
    if(msg->gameState.compare("Game starts!") == 0){
        current_state = states::SEARCH;
        search_behavior->run();
    }
    if(msg->gameState.compare("Countdown ends!") == 0){
        current_state = states::STOP;
    }
}

void update(){
    enemy_detected = blackboard->is_enemy_dected();
    enemy_pose = &blackboard->GetEnemy();
    if(damage == true){
        if(blackboard->get_damage_timepoint() - *damage_timepoint < damage_duration){
            damage = blackboard->get_damage();
            damage_armor = blackboard->get_damage_armor();
            damage_timepoint = &blackboard->get_damage_timepoint();
        }
        else{
            damage = false;
            blackboard->un_damaged();     
        }
    }
    else{
        damage = blackboard->get_damage();
        damage_armor = blackboard->get_damage_armor();
        damage_timepoint = &blackboard->get_damage_timepoint();
    }
    buff = blackboard->get_buff();
    hp = blackboard->get_hp();
    reload_status = blackboard->get_reload_status();
    if(reload_status == 1){
        ammo = 50;
    }
}

/*void damage_detect(){
    ros::Subscriber damage_subscriber = n.subscribe("robot_damage", 1000, &goal_factory::damage_detect_callback, this);
    ros::spin();
}

void damage_detect_callback(const roborts_msgs::RobotDamage& msg){
    damage = true;
    damage_armor = msg.armor_id;
}*/
~Goal_Factory();
private:
ros::NodeHandle *nh;
roborts_decision::Blackboard *blackboard;

states current_state = states::STOP;
std::chrono::seconds execution_duration(0.5);
roborts_decision::ChaseBehavior        *chase_behavior;
roborts_decision::SearchBehavior       *search_behavior;
roborts_decision::EscapeBehavior       *escape_behavior;
roborts_decision::ShootBehavior        *shoot_behavior;
roborts_decision::ReloadBehavior       *reload_behavior;
roborts_decision::ToBuffZoneBehavior   *tobuff_behavior;
int ammo = 50;
int hp = 2000;
bool damage;
int damage_armor;
bool enemy_detected = false;
geometry_msgs::PoseStamped *enemy_pose;
bool has_buff;
int reload_status = 0;
std::chrono::seconds damage_duration(2);
std::chrono::steady_clock::time_point &damage_timepoint;
int remain_time;
}
}
#endif