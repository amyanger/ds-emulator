#include "cpu/arm9/arm9.hpp"

#include <cassert>

namespace ds {

void Arm9::reset() {
    // attach_bus() must run before reset(). Today reset() does not touch bus_,
    // but later slices will (e.g. cart-header reads during direct boot), so
    // enforce the construction-order contract loudly here, like ARM7.
    assert(bus_ != nullptr && "Arm9::reset called before attach_bus");
    state_.reset();
    // The pipeline invariant is "no pending IRQ at boot". NDS pushes the real
    // initial level (false) after reset via update_arm9_irq_signals, but reset
    // the field here too so the CPU is coherent even if instantiated without
    // an NDS wrapper (tests may do this).
    irq_line_ = false;
}

void Arm9::run_until(Cycle arm9_target) {
    // Full rate: `now` is in ARM9 cycles, so state_.cycles tracks the target
    // 1:1. NO /2 here (that is the ARM7-only divergence).
    while (state_.cycles < arm9_target) {
        // ARM samples nIRQ at instruction boundaries only. Sampling here,
        // before step_*, keeps the interpreter's pipeline model clean.
        // Level-sensitive — if the handler does not ack IF the line stays
        // asserted and fires again next boundary.
        if (irq_line_ && !(state_.cpsr & (1u << 7))) {
            // TODO(commit 14): vector to arm9_enter_irq here. The exception
            // entry function does not exist until commit 14, so this slice
            // must not call it — and must not break/return (that would hang
            // the scheduler). Fall through to step_arm; the IRQ becomes
            // properly vectored once commit 14 wires arm9_enter_irq.
        }
        step_arm();
    }
}

void Arm9::step_one_instruction() {
    // Sample the IRQ line at the instruction boundary, mirroring run_until.
    if (irq_line_ && !(state_.cpsr & (1u << 7))) {
        // TODO(commit 14): vector to arm9_enter_irq here (see run_until).
    }
    // CPSR.T is never set this slice (Thumb lands in 3n), so always ARM.
    step_arm();
}

void Arm9::step_thumb() {
    // ARM9 Thumb state is not implemented until slice 3n; nothing in 3l sets
    // CPSR.T, so this path is unreachable.
    assert(false && "ARM9 Thumb state not implemented until slice 3n");
}

} // namespace ds
