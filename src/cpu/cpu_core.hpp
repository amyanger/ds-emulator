#pragma once

#include "ds/common.hpp"

namespace ds {

// Abstract CPU core. Both ARM9 and ARM7 implement this.
// In Phase 0 the implementations are empty stubs; Phase 1 fills them in.
class CpuCore {
public:
    virtual ~CpuCore() = default;

    // Advance this core up to (but not past) the given ARM9 cycle target.
    // ARM7's implementation internally converts to ARM7 cycles (half rate).
    virtual void run_until(Cycle arm9_target) = 0;

    virtual void reset() = 0;
};

}  // namespace ds
