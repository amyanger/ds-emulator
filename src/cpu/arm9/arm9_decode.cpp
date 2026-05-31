// arm9_decode.cpp — top-level ARMv5TE-subset (ARMv4T families) fetch /
// decode / dispatch loop for the ARM9. dispatch_arm() is a small switch on
// bits 27..25 that delegates to a family handler (DP, branch, LDR/STR,
// etc.). Each handler lives in its own translation unit; see
// arm9_decode_internal.hpp for the contract. Mirrors the ARM7 decoder.
//
// Cycle cost model: every executed (non-skipped) instruction costs 1 ARM9
// cycle this slice. Real per-instruction cycle counts are deferred.

#include "bus/arm9_bus.hpp"
#include "cpu/arm9/arm9.hpp"
#include "cpu/arm9/arm9_decode_internal.hpp"
#include "cpu/common/arm_alu.hpp"

#include <cassert>

namespace ds {

namespace {

// Route bits[27:25] == 0b110/0b111 — either a coprocessor access (CDP/MRC/
// MCR/LDC/STC) or a SWI. The SWI-vs-coproc recognizer is permanent dispatch
// logic, so it lives here next to the bits[27:25] switch (mirroring ARM7's
// arm7_decode.cpp). Exception/SWI vectoring is deferred to commit 14: this
// slice recognizes the split and warns, but takes neither vector.
u32 dispatch_coproc_or_swi(Arm9& cpu, u32 instr, u32 instr_addr) {
    (void) cpu;
    if (((instr >> 24) & 0xFu) == 0xFu) {
        DS_LOG_WARN(
            "arm9: SWI #0x%06X stub — exception entry unimplemented (commit 14) addr=0x%08X",
            instr & 0x00FFFFFFu,
            instr_addr);
    } else {
        DS_LOG_WARN("arm9: coproc/undefined stub — UND entry unimplemented (commit 14) "
                    "instr=0x%08X addr=0x%08X",
                    instr,
                    instr_addr);
    }
    return 1;
}

u32 dispatch_arm(Arm9& cpu, u32 instr, u32 instr_addr) {
    Arm9State& state = cpu.state();
    Arm9Bus& bus = cpu.bus();
    const u32 cond = instr >> 28;
    if (!eval_condition(cond, state.cpsr)) {
        return 1; // condition-skipped instructions still consume 1 cycle
    }

    const u32 bits_27_25 = (instr >> 25) & 0x7u;
    switch (bits_27_25) {
    case 0b000:
        return dispatch_000_space(state, bus, instr, instr_addr);
    case 0b001:
        return dispatch_dp(state, instr, instr_addr);
    case 0b010:
    case 0b011:
        return dispatch_single_data_transfer(state, bus, instr, instr_addr);
    case 0b100:
        return dispatch_block(state, bus, instr, instr_addr);
    case 0b101:
        return dispatch_branch(state, instr);
    case 0b110:
    case 0b111:
        return dispatch_coproc_or_swi(cpu, instr, instr_addr);
    default:
        return 1; // unreachable
    }
}

} // namespace

void Arm9::step_arm() {
    assert((state_.pc & 0x3u) == 0 && "Arm9::step_arm: ARM pc must be 4-aligned");
    assert((state_.cpsr & (1u << 5)) == 0 && "Arm9::step_arm requires CPSR.T=0");

    const u32 instr_addr = state_.pc;
    const u32 instr = bus_->read32(instr_addr);

    // R15 during execute reads as instruction_addr + 8 (3-stage pipeline).
    // Advance pc BEFORE dispatch so branch handlers can stomp it freely
    // without us overwriting their target afterwards.
    state_.r[15] = instr_addr + 8;
    state_.pc = instr_addr + 4;

    const u32 cycles_consumed = dispatch_arm(*this, instr, instr_addr);
    assert(cycles_consumed > 0 && "Arm9::step_arm: dispatch must consume >= 1 cycle");
    state_.cycles += cycles_consumed;
}

} // namespace ds
