#ifndef _OVERSEER_H_
#define _OVERSEER_H_

#include "ros/ros.h"
#include "global_state_machine/GlobalStateMachine.h"
#include "state_machine_msgs/SendState.h"
#include "state_machine_msgs/GetState.h"
#include "global_state_machine/GetGlobalState.h"
#include "global_state_machine/SetGlobalState.h"

class Overseer {
private:
    GlobalStateMachine gsm;
    std::map<std::string, ros::Timer> critical_map;
    std::map<std::string, int> state_map;
    ros::ServiceServer state_receiver, state_provider, set_global_state, get_global_state;
    ros::Publisher notification;
    ros::NodeHandle handle;
        
    bool receiveState(state_machine_msgs::SendState::Request  &req, state_machine_msgs::SendState::Response &res);
    bool provideState(state_machine_msgs::GetState::Request  &req, state_machine_msgs::GetState::Response &res);
    bool setGlobalState(global_state_machine::SetGlobalState::Request  &req, global_state_machine::SetGlobalState::Response &res);
    bool getGlobalState(global_state_machine::GetGlobalState::Request  &req, global_state_machine::GetGlobalState::Response &res);
    void criticalViolation(const ros::TimerEvent& event);
public:
    Overseer();
    bool prepare();
    void run();
    void shutdown();
};

#endif
