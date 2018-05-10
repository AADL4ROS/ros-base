#ifndef _NODE_CONFIGURATION_H
#define _NODE_CONFIGURATION_H

#include "node_base/Configuration.h"
#include "node_base/tf_interface.h"

struct Parameters : node_base::ParametersBase {
    int increment;
};

struct Variables : node_base::VariablesBase {
    int incrementer;
    
    Variables() {
        incrementer = 10;
    }
};

typedef std::shared_ptr<const Parameters> Parameters_ptr;
typedef std::shared_ptr<Variables> Variables_ptr;

class InternalState : node_base::InternalStateBase {
public:
    Variables_ptr vars() {
        return std::static_pointer_cast<Variables>(_vars);
    };
    
    Parameters_ptr params() const {
            return std::static_pointer_cast<const Parameters>(_params);
    };
        
    void initialize(node_base::ParametersBase * p) {
        _params = std::make_shared<const Parameters>(* static_cast<Parameters *>(p));
        _vars = std::make_shared<Variables>();
    }
};

#endif
