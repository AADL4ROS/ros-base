#ifndef _LIFE_CYCLE_H_
#define _LIFE_CYCLE_H_

#include <vector>
#include <map>
#include <functional>

namespace ros_base {

enum class States : int;

class LifeCycle {
public:
    LifeCycle(States inital_state);
    void Start();
protected:
    bool SelectNextState(States next_state);
    States GetCurrentState();
    void AddTransition(States source_state, States destination_state);
    void SetTransitionList(std::vector<std::pair<States, States>> transition_list);
    void AddStateAction(States state, std::function<void()> function);
    void SetStateActions(std::map<States, std::function<void()>> state_actions);
private:
    void NoValidTransition();
    
    bool valid_transition_;
    States current_state_;
    States next_state_;
    std::map<States, std::function<void()>> state_actions_;
    std::vector<std::pair<States, States>> transition_list_;
};

} // namespace ros_base

#endif // _LIFE_CYCLE_H_
