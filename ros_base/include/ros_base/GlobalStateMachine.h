#ifndef _GLOBAL_STATE_MACHINE_H_
#define _GLOBAL_STATE_MACHINE_H_

#include <map>
#include "boost/interprocess/managed_shared_memory.hpp"
#include "boost/interprocess/containers/map.hpp"
#include "boost/interprocess/allocators/allocator.hpp"

typedef std::pair<unsigned int, unsigned int> MapKey;
typedef std::pair<const MapKey, unsigned int> MapPair;
typedef boost::interprocess::allocator<MapPair, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
typedef boost::interprocess::map<MapKey, unsigned int, std::less<MapKey>, ShmemAllocator> StateMap;

struct create_gsm_t { };
struct load_gsm_t { };

static const create_gsm_t create_gsm;
static const load_gsm_t load_gsm;

class GlobalStateMachine {
private:
    unsigned int *currentState;
    StateMap *transitions;
    bool *initialized;
    boost::interprocess::managed_shared_memory segment;
        
    constexpr static const char *kSegmentName = "GlobalStateMachine";
    constexpr static const char *kTableName = "transitions";
    constexpr static const char *kStateName = "currentState";
    constexpr static const char *kFlagName = "initialized";
public:
    bool SetGlobalStateMachine(unsigned int init, std::map<std::pair<unsigned int, unsigned int>, unsigned int> transitions);
    GlobalStateMachine(create_gsm_t);
    GlobalStateMachine(load_gsm_t);
    void cleanUp();
    unsigned int getState();
    bool setState(unsigned int state);
    
    static bool Exists();
};

#endif
