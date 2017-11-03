#ifndef _GLOBAL_STATE_MACHINE_H_
#define _GLOBAL_STATE_MACHINE_H_

#include <map>
#include "boost/interprocess/managed_shared_memory.hpp"
#include "boost/interprocess/containers/map.hpp"
#include "boost/interprocess/allocators/allocator.hpp"

typedef std::pair<unsigned int, unsigned int> MapKey;
typedef unsigned int MapValue;
typedef std::pair<const MapKey, MapValue> MapPair;
typedef boost::interprocess::allocator<MapPair, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
typedef boost::interprocess::map<MapKey, MapValue, std::less<MapKey>, ShmemAllocator> StateMap;

class GlobalStateMachine {
private:
    unsigned int *currentState;
    StateMap *transitions;
    boost::interprocess::managed_shared_memory segment;
public:
    GlobalStateMachine(unsigned int init);
    GlobalStateMachine();
    void cleanUp();
    unsigned int getState();
    bool setState(unsigned int state);
};

#endif
