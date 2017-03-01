#include "ros_base/ROSNode.h"

namespace ros_base {

ROSNode::ROSNode() : StateMachine(ST_MAX_STATES), spinner(0), handle("~") {
    g_error = NO_ERROR;
    lastState.request.state = ST_MAX_STATES;
}

void ROSNode::faultDetected(Errors e) {
    g_error = e;
}

bool ROSNode::initialize() {
    spinner.start();
    stateService = handle.serviceClient<state_machine_msgs::SendState>("/state_notifier");
    return true;
}

bool ROSNode::noError() {
    if(g_error == NO_ERROR)
        return true;
    else
        return false;
}

void ROSNode::start() {
    BEGIN_TRANSITION_MAP                          // - Current State -
        TRANSITION_MAP_ENTRY (ST_INIT)            // ST_INIT
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_RUNNING
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_ERROR
        TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_CLOSING
    END_TRANSITION_MAP(NULL)
}

void ROSNode::errorHandling() {
    switch(g_error) {
        case PARAM_ERROR:
        case PUB_FAILED:
        case SUB_FAILED:
            ROS_ERROR("Initialization error, shutting down");
            break;
        default:
            ROS_ERROR("Unidentified error, shutting down");
    }
    InternalEvent(ST_CLOSING);
}

bool ROSNode::notifyState() {
    if(GetCurrentState() != lastState.request.state) {
        lastState.request.state = GetCurrentState();
        stateService.call(lastState);
        return lastState.response.ok;
    }
    return true;    
}

void ROSNode::setName(std::string node_name) {
    lastState.request.node = node_name;
}

STATE_DEFINE(ROSNode, Init, NoEventData) {
    if(initialize()) {
        notifyState();
        if(prepare())
            InternalEvent(ST_RUNNING);
        else
            InternalEvent(ST_ERROR);
    }
}

STATE_DEFINE(ROSNode, Running, NoEventData) {
    notifyState();
    usleep(1000);
    if(!noError())
        InternalEvent(ST_ERROR);
    else if(g_request_shutdown)
        InternalEvent(ST_CLOSING);
    else
        InternalEvent(ST_RUNNING);
}

STATE_DEFINE(ROSNode, Error, NoEventData) {
    notifyState();
    errorHandling();
    if(!noError())
        InternalEvent(ST_CLOSING);
    else
        InternalEvent(ST_RUNNING);
}

STATE_DEFINE(ROSNode, Closing, NoEventData) {
    notifyState();
    tearDown();
    ros::shutdown();
}
};
