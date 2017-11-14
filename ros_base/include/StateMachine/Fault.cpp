#include "Fault.h"

//----------------------------------------------------------------------------
// FaultHandler
//----------------------------------------------------------------------------
void FaultHandler(const char* file, unsigned short line) {
    // If you hit this line, it means one of the ASSERT macros failed.
    // Trap fault here
    while(1);
}
