#ifndef _ROS_NODE_H
#define _ROS_NODE_H

#include "ros/ros.h"
#include "ros_base/life_cycle.h"
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
    
    class ROSNode : public LifeCycle {
    private:
        sig_atomic_t volatile g_error;
        ros::AsyncSpinner spinner;
        ros::ServiceClient stateService;
        state_machine_msgs::SendState lastState;
        double frequency;
        bool critical;
                        
        bool initialize();
        void notifyState();
        void Init();
        void Running();
        void Error();
        void Closing();
    public:
        ROSNode(double frequency = 1, bool critical = false);
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
