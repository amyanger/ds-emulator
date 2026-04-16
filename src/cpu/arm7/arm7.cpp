#include "cpu/arm7/arm7.hpp"

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_exception.hpp"

#include <cassert>

namespace ds {

void Arm7::reset() {
    // attach_bus() must run before reset(). Today reset() does not touch
    // bus_, but later slices will (e.g. cart-header reads during direct
    // boot), so enforce the construction-order contract loudly here.
    assert(bus_ != nullptr && "Arm7::reset called before attach_bus");
    state_.reset();
    // The pipeline invariant is "no pending IRQ at boot". NDS will push the
    // real initial level (false) after reset via update_arm7_irq_line, but
    // reset the field here too so the CPU is coherent even if instantiated
    // without an NDS wrapper (tests may do this).
    irq_line_ = false;
}

void Arm7::run_until(Cycle arm9_target) {
    const Cycle arm7_target = arm9_target / 2;
    while (state_.cycles < arm7_target) {
        // ARMv4T samples nIRQ at instruction boundaries only. Sampling here,
        // before step_*, keeps the interpreter's pipeline model clean: the
        // step methods never see an in-flight IRQ. Level-sensitive — if the
        // handler does not ack IF the line stays asserted and fires again
        // next boundary (which is what real hardware does and why handlers
        // always start with a write-1-clear of the IF bit they service).
        if (irq_line_ && !(state_.cpsr & (1u << 7))) {
            arm7_enter_irq(state_, *bus_);
            continue;
        }
        if (state_.cpsr & (1u << 5)) {
            step_thumb();
        } else {
            step_arm();
        }
    }
}

void Arm7::raise_prefetch_abort(u32 instr_addr) {
    // ARMv4 spec: R14_abt = instr_addr + 4, vector 0x0C, mode Abort, I=1.
    // Handler returns via SUBS PC, R14, #4 to re-execute the faulting fetch.
    enter_exception(state_, ExceptionKind::PrefetchAbort, instr_addr + 4u);
}

void Arm7::raise_data_abort(u32 instr_addr) {
    // ARMv4 spec: R14_abt = instr_addr + 8, vector 0x10, mode Abort, I=1.
    // Handler returns via SUBS PC, R14, #8 to re-execute the faulting load/store.
    // (The +8 difference from prefetch abort is documented GBATEK quirk.)
    enter_exception(state_, ExceptionKind::DataAbort, instr_addr + 8u);
}

void Arm7::raise_fiq() {
    // ARMv4 spec: R14_fiq = state.pc + 4 (post-advance ARM-style — same formula
    // as IRQ), vector 0x1C, mode FIQ, I=1 and F=1. Handler returns via
    // SUBS PC, R14, #4 to resume at the next instruction.
    enter_exception(state_, ExceptionKind::Fiq, state_.pc + 4u);
}

} // namespace ds
