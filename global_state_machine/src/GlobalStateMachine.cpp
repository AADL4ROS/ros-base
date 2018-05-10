#include "global_state_machine/GlobalStateMachine.h"

bool GlobalStateMachine::Exists() {
    using namespace boost::interprocess;
    try {
        managed_shared_memory segment(open_only, kSegmentName);
        return segment.check_sanity();
    } 
    catch (const std::exception &ex) {
    }
    return false;
}

GlobalStateMachine::GlobalStateMachine(create_gsm_t) {
    using namespace boost::interprocess;
    
    shared_memory_object::remove(kSegmentName);
    segment = managed_shared_memory(create_only, kSegmentName, 65536, 0, permissions(0666));
        
    ShmemAllocator alloc_inst(segment.get_segment_manager());
    transitions = segment.construct<StateMap>(kTableName)(std::less<MapKey>(), alloc_inst);
    currentState = segment.construct<unsigned int>(kStateName)(0);
    initialized = segment.construct<bool>(kFlagName)(false);
}

GlobalStateMachine::GlobalStateMachine(load_gsm_t) {
    using namespace boost::interprocess;
    segment = managed_shared_memory(open_only, kSegmentName);

    std::pair<StateMap *, managed_shared_memory::size_type> tr_;
    tr_ = segment.find<StateMap>(kTableName);
    transitions = tr_.first;

    std::pair<unsigned int *, managed_shared_memory::size_type> cs_;
    cs_ = segment.find<unsigned int>(kStateName);
    currentState = cs_.first;
    
    std::pair<bool *, managed_shared_memory::size_type> fl_;
    fl_ = segment.find<bool>(kFlagName);
    initialized = fl_.first;
}

bool GlobalStateMachine::SetGlobalStateMachine(unsigned int init, 
                                               std::map<std::pair<unsigned int, unsigned int>, unsigned int> transitions) {
    for(auto it = transitions.begin(); it != transitions.end(); ++it) {
        this->transitions->insert(std::make_pair(it->first, it->second));
    }
    *currentState = init;
    *initialized = true;
}

bool GlobalStateMachine::setState(unsigned int state) {
    if(!*initialized) {
        throw std::exception();
    }
    std::pair<unsigned int, unsigned int>  key = std::make_pair(*currentState, state);
    auto it = transitions->find(key);
    if(it != transitions->end()) {
        *currentState = it->second;
        return true;
    } else {
        return false;
    }
}

unsigned int GlobalStateMachine::getState() {
    if(!*initialized) {
        throw std::exception();
    }
    return *currentState;
    
}

void GlobalStateMachine::cleanUp() {
    boost::interprocess::shared_memory_object::remove(kSegmentName);
}
