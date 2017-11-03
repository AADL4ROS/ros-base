#ifndef _OVERSEER_H_
#define _OVERSEER_H_

#include "ros/ros.h"
#include "ros_base/GlobalStateMachine.h"
#include "state_machine_msgs/SendState.h"
#include "state_machine_msgs/GetState.h"
#include "state_machine_msgs/GetGlobalState.h"
#include "state_machine_msgs/SetGlobalState.h"

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
    bool setGlobalState(state_machine_msgs::SetGlobalState::Request  &req, state_machine_msgs::SetGlobalState::Response &res);
    bool getGlobalState(state_machine_msgs::GetGlobalState::Request  &req, state_machine_msgs::GetGlobalState::Response &res);
    void criticalViolation(const ros::TimerEvent& event);
public:
    Overseer();
    bool prepare();
    void run();
    void shutdown();
};

#endif
