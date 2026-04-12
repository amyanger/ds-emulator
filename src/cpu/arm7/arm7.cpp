#include "cpu/arm7/arm7.hpp"

namespace ds {

void Arm7::run_until(Cycle arm9_target) {
    // ARM7 runs at half the ARM9 clock rate. Phase 0 no-op.
    const Cycle arm7_target = arm9_target / 2;
    if (arm7_target > arm7_cycles_) {
        arm7_cycles_ = arm7_target;
    }
}

void Arm7::reset() {
    arm7_cycles_ = 0;
}

}  // namespace ds
