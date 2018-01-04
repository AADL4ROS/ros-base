#include "ros_base/ROSNode.h"

namespace ros_base {

enum class States {
    ST_INIT,
    ST_RUNNING,
    ST_ERROR,
    ST_CLOSING
};

ROSNode::ROSNode(double frequency, bool critical) : LifeCycle(States::ST_INIT), spinner(0), handle("~") {
    g_error = NO_ERROR;
//     lastState.request.state = ST_MAX_STATES;
    this->frequency = frequency;
    this->critical = critical;
    AddStateAction(States::ST_INIT, &ROSNode::Init);
    AddStateAction(States::ST_RUNNING, &ROSNode::Init);
    AddStateAction(States::ST_ERROR, &Init);
    AddStateAction(States::ST_CLOSING, &Init);
}

void ROSNode::start() {
    Start();
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
    SelectNextState(States::ST_CLOSING);
}

void ROSNode::notifyState() {
//     if(critical || GetCurrentState() != lastState.request.state) {
//         lastState.request.state = GetCurrentState();
//         stateService.call(lastState);
//     }
    std::cout<<"switching state"<<std::endl;
}

void ROSNode::setName(std::string node_name) {
    lastState.request.node = node_name;
}

void ROSNode::Init() {
    if(initialize()) {
        notifyState();
        if(prepare())
            SelectNextState(States::ST_RUNNING);
        else
            SelectNextState(States::ST_ERROR);
    }
}

void ROSNode::Running() {
    notifyState();
    usleep(1/frequency * 1000);
    if(!noError())
        SelectNextState(States::ST_ERROR);
    else if(g_request_shutdown)
        SelectNextState(States::ST_CLOSING);
    else
        SelectNextState(States::ST_RUNNING);
}

void ROSNode::Error() {
    notifyState();
    errorHandling();
    if(!noError())
        SelectNextState(States::ST_CLOSING);
    else
        SelectNextState(States::ST_RUNNING);
}

void ROSNode::Closing() {
    notifyState();
    tearDown();
    ros::shutdown();
}
};
