#include "cpu/arm9/arm9.hpp"

namespace ds {

void Arm9::run_until(Cycle arm9_target) {
    // Phase 0 no-op: pretend we executed up to the target.
    if (arm9_target > cycles_) {
        cycles_ = arm9_target;
    }
}

void Arm9::reset() {
    cycles_ = 0;
}

}  // namespace ds
