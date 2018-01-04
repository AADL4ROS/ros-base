#include "ros_base/life_cycle.h"
#include <utility>
#include <algorithm>

namespace ros_base {
    States LifeCycle::GetCurrentState() {
        return current_state_;
    }
    
    void LifeCycle::AddStateAction(States state, StateFunction function) {
    }
    
    void LifeCycle::SetStateActions(std::map<States, StateFunction> state_actions) {
        
    }
    
    void LifeCycle::AddTransition(States source_state, States destination_state) {
    }

    void LifeCycle::SetTransitionList(std::vector<std::pair<States, States> > transition_list) {
        
    }
    
    LifeCycle::LifeCycle(States inital_state) {
        next_state_ = inital_state;
        valid_transition_ = true;
    }

    void LifeCycle::Start() {
        while(valid_transition_) {
            valid_transition_ = false;
            current_state_ = next_state_;
            auto iter = state_actions_.find(next_state_);
            (*iter->second)();
        }
    }
    
    bool LifeCycle::SelectNextState(States next_state) {
        auto transition = std::make_pair(current_state_, next_state);
        if(std::find(transition_list_.begin(), transition_list_.end(), transition) != transition_list_.end()) {
            next_state_ = next_state;
            valid_transition_ = true;
        } else {
            valid_transition_ = false;
        }
        return valid_transition_;
    }

} // namespace ros_base
