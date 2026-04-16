#include "cpu/arm7/arm7_exception.hpp"

#include "bus/arm7_bus.hpp"

#include <cassert>

namespace ds {

namespace {
// Direct-boot IRQ handler pointer lives at the last word of ARM7 WRAM.
// Games set this up during init so the BIOS thunk at 0x00000018 has
// somewhere to jump to. Until a real ARM7 BIOS ROM lands, we synthesize
// the thunk by doing the indirection ourselves on IRQ entry.
constexpr u32 kArm7IrqVectorIndirection = 0x0380FFFCu;
} // namespace

u32 enter_exception(Arm7State& state, ExceptionKind kind, u32 return_addr) {
    // ARMv4 exception entry sequence (GBATEK §"Actions performed by CPU when
    // entering an exception"):
    //   R14_<new> = return_addr
    //   SPSR_<new> = old CPSR
    //   CPSR.M = new_mode; CPSR.T = 0; CPSR.I = 1; CPSR.F set only on Reset + FIQ.
    //   PC = vector
    const u32 old_cpsr = state.cpsr;
    const Mode new_mode = exception_mode(kind);
    const u32 vector = exception_vector(kind);
    const bool set_f = exception_sets_fiq_mask(kind);

    state.switch_mode(new_mode); // stores old bank, loads new bank, updates CPSR.M
    state.r[14] = return_addr;   // write into the NEW bank (switch_mode already done)

    u32* spsr = state.spsr_slot();
    assert(spsr != nullptr && "enter_exception: target mode must have SPSR");
    *spsr = old_cpsr;

    state.cpsr &= ~(1u << 5); // T = 0 (always enter in ARM state)
    state.cpsr |= (1u << 7);  // I = 1 (mask IRQ)
    if (set_f)
        state.cpsr |= (1u << 6); // F = 1 for Reset + FIQ only

    state.pc = vector;

    // Coarse cycle cost. Real hardware: 2 for IRQ/FIQ, 3 for SWI/UNDEF/abort.
    // Phase 6 will tighten this.
    return 3;
}

void arm7_enter_irq(Arm7State& state, Arm7Bus& bus) {
    // GBATEK: IRQ saves "return address in ARM-style format" regardless of
    // whether the CPU was in ARM or Thumb state. In our "state.pc = next
    // instruction to fetch" convention, that is state.pc + 4 in both cases
    // (see slice-3d design §5.3 — this matches melonDS's uniform formula).
    const u32 return_addr = state.pc + 4u;
    const u32 cycles = enter_exception(state, ExceptionKind::Irq, return_addr);

    // enter_exception just set PC = 0x18. Real ARM7 BIOS at 0x18 is a tiny
    // thunk that loads a function pointer from [0x0380FFFC] and branches.
    // We direct-boot — no BIOS ROM — so synthesize the thunk's effect here.
    // Once a real BIOS backing lands, this read and write disappear.
    state.pc = bus.read32(kArm7IrqVectorIndirection);

    state.cycles += cycles;
}

} // namespace ds
