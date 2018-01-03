#ifndef _LIFE_CYCLE_H_
#define _LIFE_CYCLE_H_

#include <vector>
#include <map>

namespace life_cycle {

typedef void (*StateFunction)(void);
enum class States;

class LifeCycle {
public:
    LifeCycle(States inital_state);
    void Start();
protected:
    bool SelectNextState(States next_state);
    
    std::map<States, StateFunction> state_actions_;
    std::vector<std::pair<States, States>> transition_list_;
private:
    void NoValidTransition();
    
    bool valid_transition_;
    States current_state_;
    States next_state_;
};

} // namespace life_cycle

#endif // _LIFE_CYCLE_H_
