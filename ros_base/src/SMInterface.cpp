#include "ros_base/SMInterface.h"
#include "state_machine_msgs/GetGlobalState.h"
#include "state_machine_msgs/SetGlobalState.h"

SMInterface::SMInterface() {
    try {
        gsm = new GlobalStateMachine();
        realGet = &SMInterface::getGlobalStateFromMemory;
        realSet = &SMInterface::setGlobalStateByMemory;
    } catch (std::exception) {
        get_state = handle.serviceClient<state_machine_msgs::GetGlobalState>("/get_global_state");
        set_state = handle.serviceClient<state_machine_msgs::SetGlobalState>("/set_global_state");
        realGet = &SMInterface::getGlobalStateFromROS;
        realSet = &SMInterface::setGlobalStateByROS;
    }    
}

unsigned int SMInterface::getGlobalStateFromMemory() { return gsm->getState(); }

bool SMInterface::setGlobalStateByMemory(unsigned int state) { return gsm->setState(state); }

unsigned int SMInterface::getGlobalStateFromROS() {
    state_machine_msgs::GetGlobalState srv;
    if(get_state.call(srv))
        return srv.response.state;
    else
        return 666;
}

bool SMInterface::setGlobalStateByROS(unsigned int state) {
    state_machine_msgs::SetGlobalState srv;
    srv.request.state = state;
    return set_state.call(srv);
}

unsigned int SMInterface::getGlobalState() { return (*this.*realGet)(); }

bool SMInterface::setGlobalState(unsigned int state) { return (*this.*realSet)(state); }
