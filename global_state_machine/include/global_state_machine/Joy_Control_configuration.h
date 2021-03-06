#ifndef _JOY_CONTROL_CONFIGURATION_H
#define _JOY_CONTROL_CONFIGURATION_H
/**
 * Auto-generated Internal State
 */
#include "node_base/Configuration.h"
#include "node_base/SMInterface.h"

struct Variables: node_base::VariablesBase {
    SMInterface smi;
};

struct Parameters: node_base::ParametersBase {
    SMInterface smi;
    struct button_mapping_t { 
        int halt;
        int safe;
        int manual;
        int assisted;
        int autonomous;
        int enabler;
    } button_mapping;
    struct axis_mapping_t { 
        int forward;
        int rotate;
    } axis_mapping;
};

typedef std::shared_ptr < const Parameters > Parameters_ptr;
typedef std::shared_ptr < Variables > Variables_ptr;

class InternalState: node_base::InternalStateBase {
public:
    
    Variables_ptr vars() {
        return std::static_pointer_cast < Variables > (_vars);
    };
    
    Parameters_ptr params() const {
        return std::static_pointer_cast < const Parameters > (_params);
    };
    
    void initialize (node_base::ParametersBase * p = NULL) {
        _vars = std::make_shared < Variables > ();
        _params = std::make_shared < const Parameters > (*static_cast < Parameters * > (p));
    }
};
#endif
