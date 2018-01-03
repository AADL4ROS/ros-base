#include "life_cycle/life_cycle.h"
#include <utility>
#include <algorithm>

namespace life_cycle {
    LifeCycle::LifeCycle(States inital_state) {
        next_state_ = inital_state;
        valid_transition_ = true;
    }

    void LifeCycle::Start() {
        while(valid_transition_) {
            valid_transition_ = false;
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

} // namespace life_cycle
