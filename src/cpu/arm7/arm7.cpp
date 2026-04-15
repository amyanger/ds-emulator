#include "cpu/arm7/arm7.hpp"

#include <cassert>

namespace ds {

void Arm7::reset() {
    // attach_bus() must run before reset(). Today reset() does not touch
    // bus_, but later slices will (e.g. cart-header reads during direct
    // boot), so enforce the construction-order contract loudly here.
    assert(bus_ != nullptr && "Arm7::reset called before attach_bus");
    state_.reset();
}

void Arm7::run_until(Cycle arm9_target) {
    const Cycle arm7_target = arm9_target / 2;
    while (state_.cycles < arm7_target) {
        if (state_.cpsr & (1u << 5)) {
            step_thumb();
        } else {
            step_arm();
        }
    }
}

} // namespace ds
