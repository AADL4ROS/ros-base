#ifndef _LIFE_CYCLE_H_
#define _LIFE_CYCLE_H_

#include <vector>

namespace life_cycle {
enum States {
    ST_INITIAL = 0,
    ST_FINAL,
    ST_INVALID
};

class LifeCycle {
public:
    LifeCycle(unsigned int inital_state);
    void Start();
private:
    bool LifeCycleEngine();
    void NoValidTransition();
    std::vector<States, States> transition_map_;
    States current_state_;
    States intial_state_;
};

} // namespace life_cycle

#endif // _LIFE_CYCLE_H_
