#ifndef _ROS_NODE_H
#define _ROS_NODE_H

#include "ros/ros.h"
#include "StateMachine/StateMachine.h"
#include "state_machine_msgs/SendState.h"
#include <signal.h>

sig_atomic_t volatile g_request_shutdown = 0;

namespace ros_base {

enum Errors {
    NO_ERROR,
    PARAM_ERROR,
    SUB_FAILED,
    PUB_FAILED,
    INVALID_MESSAGE
};

class ROSNode : public StateMachine {
private:
    sig_atomic_t volatile g_error;
    ros::AsyncSpinner spinner;
    ros::ServiceClient stateService;
    state_machine_msgs::SendState lastState;
    
    enum States {
        ST_INIT,
        ST_RUNNING,
        ST_ERROR,
        ST_CLOSING,
        ST_MAX_STATES
    };
    
    STATE_DECLARE(ROSNode, Init,    NoEventData)
    STATE_DECLARE(ROSNode, Running, NoEventData)
    STATE_DECLARE(ROSNode, Error,   NoEventData)
    STATE_DECLARE(ROSNode, Closing, NoEventData)
        
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&Init)
        STATE_MAP_ENTRY(&Running)
        STATE_MAP_ENTRY(&Error)
        STATE_MAP_ENTRY(&Closing) 
    END_STATE_MAP
    
    bool initialize();
    bool notifyState();
public:
    ROSNode();
    void start();
protected:
    ros::NodeHandle handle;
    
    void setName(std::string name);
    void faultDetected(Errors e);
    bool noError();
    bool virtual prepare() = 0;
    void virtual errorHandling() = 0;
    void virtual tearDown() = 0;
};
};

#endif
