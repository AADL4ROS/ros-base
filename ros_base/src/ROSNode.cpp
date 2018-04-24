#include "ros_base/ROSNode.h"

namespace ros_base {

enum class States : int {
    ST_INIT = 0,
    ST_RUNNING,
    ST_ERROR,
    ST_CLOSING
};

ROSNode::ROSNode(double frequency, bool critical) : LifeCycle(States::ST_INIT), spinner(0), handle("~") {
    g_error = NO_ERROR;
    lastState.request.state = -1;
    this->frequency = frequency;
    this->critical = critical;
    AddStateAction(States::ST_INIT, std::bind(&ROSNode::Init, this));
    AddStateAction(States::ST_RUNNING, std::bind(&ROSNode::Running, this));
    AddStateAction(States::ST_ERROR, std::bind(&ROSNode::Error, this));
    AddStateAction(States::ST_CLOSING, std::bind(&ROSNode::Closing, this));
    //TODO think about a wildcard?
    std::vector<std::pair<States, States>> transition_list = {
        {States::ST_INIT, States::ST_RUNNING},
        {States::ST_INIT, States::ST_ERROR},
        {States::ST_INIT, States::ST_CLOSING},
        {States::ST_RUNNING, States::ST_RUNNING},
        {States::ST_RUNNING, States::ST_ERROR},
        {States::ST_RUNNING, States::ST_CLOSING},
        {States::ST_ERROR, States::ST_INIT},
        {States::ST_ERROR, States::ST_RUNNING},
        {States::ST_ERROR, States::ST_CLOSING}
    };
    SetTransitionList(transition_list);
}

void ROSNode::start() {
    Start();
}

void ROSNode::faultDetected(Errors e) {
    g_error = e;
}

bool ROSNode::initialize() {
    if(critical) {
        ros::service::waitForService("/state_notifier");
    }
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
    int current_state = (int) GetCurrentState();
    if(critical || current_state != lastState.request.state) {
        lastState.request.state = current_state;
        stateService.call(lastState);
    }
}

void ROSNode::setName(std::string node_name) {
    lastState.request.node = node_name;
}

void ROSNode::Init() {
    //TODO closing?
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
