#pragma once

#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7 : public CpuCore {
public:
    // Takes an ARM9-cycle target; internally converts to ARM7 cycles (half rate).
    void run_until(Cycle arm9_target) override;
    void reset() override;

private:
    Cycle arm7_cycles_ = 0;
};

}  // namespace ds
