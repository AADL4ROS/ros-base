#include "global_state_machine/SMInterface.h"
#include "global_state_machine/GetGlobalState.h"
#include "global_state_machine/SetGlobalState.h"

SMInterface::SMInterface() {
    while (true) {
        if (GlobalStateMachine::Exists()) {
            gsm = new GlobalStateMachine(load_gsm);
            realGet = &SMInterface::getGlobalStateFromMemory;
            realSet = &SMInterface::setGlobalStateByMemory;
            return;
        } else if (ros::service::exists("/get_global_state", true)) {
            gsm = nullptr;
            get_state = handle.serviceClient<global_state_machine::GetGlobalState>("/get_global_state");
            set_state = handle.serviceClient<global_state_machine::SetGlobalState>("/set_global_state");
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
    global_state_machine::GetGlobalState srv;
    if(get_state.call(srv))
        return srv.response.state;
    else
        return 666;
}

bool SMInterface::setGlobalStateByROS(unsigned int state) {
    global_state_machine::SetGlobalState srv;
    srv.request.state = state;
    return set_state.call(srv);
}

unsigned int SMInterface::getGlobalState() { return (*this.*realGet)(); }

bool SMInterface::setGlobalState(unsigned int state) { return (*this.*realSet)(state); }
