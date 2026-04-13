#include "cpu/arm7/arm7.hpp"

#include "bus/arm7_bus.hpp"

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
        step_arm();
    }
}

}  // namespace ds
