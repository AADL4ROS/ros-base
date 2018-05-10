#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

namespace node_base {
    struct ParametersBase {
    };

    struct VariablesBase {
    };

    class InternalStateBase {
    protected:
        std::shared_ptr<const ParametersBase> _params;
        std::shared_ptr<VariablesBase> _vars;
    public:
        void virtual initialize(ParametersBase *p) = 0;
    };
}

#endif
