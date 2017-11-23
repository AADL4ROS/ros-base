#include "ros_base/GlobalStateMachine.h"

GlobalStateMachine::GlobalStateMachine(unsigned int init) {
    using namespace boost::interprocess;
    
    shared_memory_object::remove("GlobalStateMachine");
    segment = managed_shared_memory(create_only, "GlobalStateMachine", 65536, 0, permissions(0666));
        
    ShmemAllocator alloc_inst(segment.get_segment_manager());
    transitions = segment.construct<StateMap>("transitions")(std::less<MapKey>(), alloc_inst);
    currentState = segment.construct<unsigned int>("currentState")(init);
    //HALT: 0, SAFE: 1, MANUAL: 2, ASSISTED: 3, AUTO: 4
    *transitions = {
        {std::make_pair(0, 1), 1},
        {std::make_pair(1, 0), 0},
        {std::make_pair(1, 2), 2},
        {std::make_pair(1, 3), 3},
        {std::make_pair(1, 4), 4},
        {std::make_pair(2, 0), 0},
        {std::make_pair(2, 1), 1},
        {std::make_pair(2, 3), 3},
        {std::make_pair(3, 0), 0},
        {std::make_pair(3, 1), 1},
        {std::make_pair(3, 2), 2},
        {std::make_pair(4, 0), 0},
        {std::make_pair(4, 1), 1}
    };
}

GlobalStateMachine::GlobalStateMachine() {
    using namespace boost::interprocess;
    segment = managed_shared_memory(open_only, "GlobalStateMachine");

    std::pair<StateMap *, managed_shared_memory::size_type> tr_;
    tr_ = segment.find<StateMap>("transitions");
    transitions = tr_.first;

    std::pair<unsigned int *, managed_shared_memory::size_type> cs_;
    cs_ = segment.find<unsigned int>("currentState");
    currentState = cs_.first;
    
}

bool GlobalStateMachine::setState(unsigned int state) {
    std::pair<unsigned int, unsigned int>  key = std::make_pair(*currentState, state);
    auto it = transitions->find(key);
    if(it != transitions->end()) {
        *currentState = it->second;
        return true;
    } else {
        return false;
    }
    return true;
}

unsigned int GlobalStateMachine::getState() { return *currentState; }

void GlobalStateMachine::cleanUp() {
    boost::interprocess::shared_memory_object::remove("GlobalStateMachine");
}
