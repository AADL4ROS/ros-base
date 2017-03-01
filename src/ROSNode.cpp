#include "ros_base/ROSNode.h"

ROSNode::ROSNode() : StateMachine(ST_MAX_STATES), spinner(0), handle("~") {
    g_error = NO_ERROR;
}

void ROSNode::faultDetected(Errors e) {
    g_error = e;
}

bool ROSNode::stateServiceCallback(state_machine_msgs::GetState::Request &req, state_machine_msgs::GetState::Response &res) {
    res.state = (int) GetCurrentState();
    switch (res.state) {
        case ST_INIT:
            res.name = "ST_INIT";
            break;
        case ST_RUNNING:
            res.name = "ST_RUNNING";
            break;
        case ST_CLOSING:
            res.name = "ST_CLOSING";
            break;
        case ST_ERROR:
            res.name = "ST_ERROR";
            break;
    }
    return true;
}

bool ROSNode::initialize() {
    spinner.start();
    stateService = handle.advertiseService("get_state", &ROSNode::stateServiceCallback, this);
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

STATE_DEFINE(ROSNode, Init, NoEventData) {
    if(initialize() && prepare())
        InternalEvent(ST_RUNNING);
    else
        InternalEvent(ST_ERROR);
}

STATE_DEFINE(ROSNode, Running, NoEventData) {
    usleep(1000);
    if(!noError())
        InternalEvent(ST_ERROR);
    else if(g_request_shutdown)
        InternalEvent(ST_CLOSING);
    else
        InternalEvent(ST_RUNNING);
}

STATE_DEFINE(ROSNode, Error, NoEventData) {
    errorHandling();
    if(!noError())
        InternalEvent(ST_CLOSING);
    else
        InternalEvent(ST_RUNNING);
}

STATE_DEFINE(ROSNode, Closing, NoEventData) {
    ros::shutdown();
}
