#pragma once

#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm9 : public CpuCore {
public:
    void run_until(Cycle arm9_target) override;
    void reset() override;

private:
    Cycle cycles_ = 0;  // ARM9 cycles executed
};

}  // namespace ds
