#include "ros_base/SMInterface.h"
#include "state_machine_msgs/GetGlobalState.h"
#include "state_machine_msgs/SetGlobalState.h"

SMInterface::SMInterface() {
    while (true) {
        if (GlobalStateMachine::Exists()) {
            gsm = new GlobalStateMachine(load_gsm);
            realGet = &SMInterface::getGlobalStateFromMemory;
            realSet = &SMInterface::setGlobalStateByMemory;
            return;
        } else if (ros::service::exists("/get_global_state", true)) {
            gsm = nullptr;
            get_state = handle.serviceClient<state_machine_msgs::GetGlobalState>("/get_global_state");
            set_state = handle.serviceClient<state_machine_msgs::SetGlobalState>("/set_global_state");
            realGet = &SMInterface::getGlobalStateFromROS;
            realSet = &SMInterface::setGlobalStateByROS;
            return;
        }
        ros::Duration(1.0).sleep();
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
