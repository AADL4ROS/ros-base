#include "ros/ros.h"
#include "std_msgs/String.h"
#include "StateMachine/StateMachine.h"
#include <signal.h>

#define NODE_NAME "reference"

sig_atomic_t volatile g_request_shutdown = 0;

enum Errors {
    NO_ERROR,
    PARAM_ERROR,
    SUB_FAILED,
    PUB_FAILED,
    INVALID_MESSAGE
};

class ErrorData : public EventData {
private:
    int error;
public:
    int getError() const { return error; };
    ErrorData(int e) { error = e; };
};

class ROSNode : public StateMachine {
private:
    ros::NodeHandle handle;
    ros::AsyncSpinner spinner;
    ros::Publisher pub1, pub2;
    ros::Subscriber sub;
    ros::Timer timer;
    struct {
        int a;
        int b;
    } parameters;
    
    struct {
        int k;
    } state;
    
    sig_atomic_t volatile g_error;
    
    enum States {
        ST_INIT,
        ST_RUNNING,
        ST_ERROR,
        ST_CLOSING,
        ST_MAX_STATES
    };
            
    void subCallback(const std_msgs::String::ConstPtr&);
    void pubCallback(const ros::TimerEvent&);
    
    STATE_DECLARE(ROSNode, Init,    NoEventData)
    STATE_DECLARE(ROSNode, Running, NoEventData)
    STATE_DECLARE(ROSNode, Error,   ErrorData)
    STATE_DECLARE(ROSNode, Closing, NoEventData)
        
    BEGIN_STATE_MAP
        STATE_MAP_ENTRY(&Init)
        STATE_MAP_ENTRY(&Running)
        STATE_MAP_ENTRY(&Error)
        STATE_MAP_ENTRY(&Closing) 
    END_STATE_MAP

public:
    ROSNode();
    void start();
protected:
    void faultDetected(Errors );
    bool prepare();
    void errorHandling(int id);
};

void ROSNode::start() {
    BEGIN_TRANSITION_MAP                          // - Current State -
        TRANSITION_MAP_ENTRY (ST_INIT)            // ST_INIT
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_RUNNING
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_ERROR
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_CLOSING
    END_TRANSITION_MAP(NULL)
}

void ROSNode::faultDetected(Errors e) {
    g_error = e;
}


ROSNode::ROSNode() : StateMachine(ST_MAX_STATES), spinner(0), handle("~") {
    g_error = NO_ERROR;
}

bool ROSNode::prepare() {
    ROS_INFO("Preparing");
    if(!handle.getParam("param_a", parameters.a) &&
       !handle.getParam("/param_b", parameters.b)) {
        g_error = PARAM_ERROR;
        return false;
    }
    
    state.k = 0;
    
    pub1 = handle.advertise<std_msgs::String>("publisher1", 10);
    pub2 = handle.advertise<std_msgs::String>("/publisher2", 10);
      
    sub = handle.subscribe<std_msgs::String>("/subscriber1", 10, &ROSNode::subCallback, this);
    
    timer = handle.createTimer(ros::Duration(0.5), &ROSNode::pubCallback, this);
    
    spinner.start();
    
    return true;
}

void ROSNode::errorHandling(int id) {
    ROS_INFO("Error %d occured", id);
    if(id == INVALID_MESSAGE) {
        state.k = 0;
        g_error = NO_ERROR;
    }    
    return;
}

void ROSNode::subCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("%s %d", msg->data.c_str(), state.k);
    state.k++;
    if(state.k > 4) {
        ROS_INFO("error");
        g_error = INVALID_MESSAGE;
        return;
    }
}

void ROSNode::pubCallback(const ros::TimerEvent& t) {
    std_msgs::String msg;
    msg.data = "Something something a message";
    pub1.publish(msg);
}

STATE_DEFINE(ROSNode, Init, NoEventData) {
    if(prepare()) {
        InternalEvent(ST_RUNNING);
    }
    else {
        InternalEvent(ST_ERROR, new ErrorData(g_error));
    }
}

STATE_DEFINE(ROSNode, Running, NoEventData) {
    usleep(1000);
    if(g_error != NO_ERROR)
        InternalEvent(ST_ERROR, new ErrorData(g_error));
    else if(g_request_shutdown)
        InternalEvent(ST_CLOSING);
    else
        InternalEvent(ST_RUNNING);
}

STATE_DEFINE(ROSNode, Error, ErrorData) {
    ROS_INFO("in error state");
    errorHandling(data->getError());
    if(g_error != NO_ERROR)
        InternalEvent(ST_CLOSING);
    else
        InternalEvent(ST_RUNNING);
}

STATE_DEFINE(ROSNode, Closing, NoEventData) {
    ROS_INFO("Shutting down");
    ros::shutdown();
}

void nodeSigintHandler(int sig) {
    g_request_shutdown = 1;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeSigintHandler);
    ROSNode node;
    node.start();
    return 0;
}
