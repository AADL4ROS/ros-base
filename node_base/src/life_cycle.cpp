#include "node_base/life_cycle.h"
#include <utility>
#include <algorithm>

#include <iostream>

namespace node_base {
    States LifeCycle::GetCurrentState() {
        return current_state_;
    }
    
    void LifeCycle::AddStateAction(States state, std::function<void()> function) {
        state_actions_.emplace(state, function);
    }
    
    void LifeCycle::SetStateActions(std::map<States, std::function<void()>> state_actions) {
        state_actions_ = state_actions;        
    }
    
    void LifeCycle::AddTransition(States source_state, States destination_state) {
        transition_list_.push_back(std::make_pair(source_state, destination_state));
    }

    void LifeCycle::SetTransitionList(std::vector<std::pair<States, States>> transition_list) {
        transition_list_ = transition_list;
    }
    
    LifeCycle::LifeCycle(States inital_state) {
        next_state_ = inital_state;
        valid_transition_ = true;
    }

    void LifeCycle::Start() {
        if(state_actions_.empty() || transition_list_.empty()) {
            std::cout<<"Incomplete initialization, shutting down"<<std::endl;
            return;
        }
        
        while(valid_transition_) {
            valid_transition_ = false;
            current_state_ = next_state_;
            auto iter = state_actions_.find(current_state_);
            iter->second();
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

} // namespace node_base
