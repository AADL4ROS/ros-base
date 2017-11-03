#ifndef _SM_INTERFACE_H_
#define _SM_INTERFACE_H_

#include "ros_base/GlobalStateMachine.h"
#include "ros/ros.h"

class SMInterface {
private:
    ros::NodeHandle handle;
    ros::ServiceClient get_state, set_state;
    GlobalStateMachine *gsm;
    
    bool (SMInterface::*realSet)(unsigned int);
    unsigned int (SMInterface::*realGet)();  
    bool setGlobalStateByMemory(unsigned int state);
    unsigned int getGlobalStateFromMemory();
    bool setGlobalStateByROS(unsigned int state);
    unsigned int getGlobalStateFromROS();
public:
    SMInterface();
    bool setGlobalState(unsigned int state);
    unsigned int getGlobalState();
};

#endif
