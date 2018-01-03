#include "ros_base/ROSNode.h"

enum class life_cycle::States {
    ST_INIT,
    ST_RUNNING,
    ST_ERROR,
    ST_CLOSING
};

namespace ros_base {

ROSNode::ROSNode(double frequency, bool critical) : life_cycle::LifeCycle(life_cycle::States::ST_INIT), spinner(0), handle("~") {
    g_error = NO_ERROR;
//     lastState.request.state = ST_MAX_STATES;
    this->frequency = frequency;
    this->critical = critical;
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
    SelectNextState(life_cycle::States::ST_CLOSING);
}

void ROSNode::notifyState() {
//     if(critical || GetCurrentState() != lastState.request.state) {
//         lastState.request.state = GetCurrentState();
//         stateService.call(lastState);
//     }
}

void ROSNode::setName(std::string node_name) {
    lastState.request.node = node_name;
}

void ROSNode::Init() {
    if(initialize()) {
        notifyState();
        if(prepare())
            SelectNextState(life_cycle::States::ST_RUNNING);
        else
            SelectNextState(life_cycle::States::ST_ERROR);
    }
}

void ROSNode::Running() {
    notifyState();
    usleep(1/frequency * 1000);
    if(!noError())
        SelectNextState(life_cycle::States::ST_ERROR);
    else if(g_request_shutdown)
        SelectNextState(life_cycle::States::ST_CLOSING);
    else
        SelectNextState(life_cycle::States::ST_RUNNING);
}

void ROSNode::Error() {
    notifyState();
    errorHandling();
    if(!noError())
        SelectNextState(life_cycle::States::ST_CLOSING);
    else
        SelectNextState(life_cycle::States::ST_RUNNING);
}

void ROSNode::Closing() {
    notifyState();
    tearDown();
    ros::shutdown();
}
};
