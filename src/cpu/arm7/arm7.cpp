#include "cpu/arm7/arm7.hpp"

#include "bus/arm7_bus.hpp"

namespace ds {

void Arm7::reset() {
    state_.reset();
}

void Arm7::run_until(Cycle arm9_target) {
    const Cycle arm7_target = arm9_target / 2;
    // Task 4 wires up a real fetch/decode loop that calls step_arm().
    // For now, just advance the cycle counter to the target so scheduler
    // consumers see a well-behaved CPU that does nothing.
    if (arm7_target > state_.cycles) {
        state_.cycles = arm7_target;
    }
    // Reference bus_ so the compiler does not warn about an unused member
    // until Task 4 actually uses it. This line is removed in Task 4.
    (void)bus_;
}

// Defined in arm7_decode.cpp — Task 4.
// void Arm7::step_arm() { ... }

}  // namespace ds
