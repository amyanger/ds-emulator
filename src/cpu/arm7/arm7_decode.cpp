// arm7_decode.cpp — top-level ARMv4T fetch / decode / dispatch loop.
// dispatch_arm() is a small switch on bits 27..25 that delegates to a
// family handler (DP, branch, LDR/STR). Each handler lives in its own
// translation unit; see arm7_decode_internal.hpp for the contract.
//
// Cycle cost model: every executed (non-skipped) instruction costs 1
// ARM7 cycle, same as slice 3a. Real per-instruction cycle counts are
// deferred to a later slice.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_exception.hpp"
#include "cpu/arm7/bios/bios7_hle.hpp"

#include <cassert>

namespace ds {

namespace {

// Route bits[27:25] = 0b110/0b111 — either a coprocessor access (CDP/MRC/MCR/
// LDC/STC) or a SWI. ARMv4T ARM7TDMI has no coprocessor interface, so any
// coproc encoding is legitimately undefined and must trap to the UND vector.
// SWI (bits[27:24] == 0b1111) enters Supervisor mode with return_addr =
// instr_addr + 4, then hands off to the BIOS-HLE dispatcher.
u32 dispatch_coproc_or_swi(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    const u32 bits_27_24 = (instr >> 24) & 0xFu;
    if (bits_27_24 == 0xFu) {
        // ARM-state SWI: bits[23:0] = comment field / SWI number. Entry is a
        // fixed 3 cycles (matches enter_exception's coarse model); the HLE
        // stub returns 0 today, so we hard-code 3 rather than summing the
        // two return values to keep the cost contract explicit.
        const u32 swi_number = instr & 0x00FFFFFFu;
        enter_exception(state, ExceptionKind::Swi, instr_addr + 4);
        arm7_bios_hle_dispatch_swi(state, bus, swi_number);
        return 3;
    }
    // Coprocessor instructions (CDP / MRC / MCR / LDC / STC): enter UND mode
    // with return address = instr_addr + 4, so `MOVS PC, R14` in the handler
    // resumes at the instruction FOLLOWING the faulting one.
    return enter_exception(state, ExceptionKind::Undef, instr_addr + 4);
}

u32 dispatch_arm(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
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
        return dispatch_coproc_or_swi(state, bus, instr, instr_addr);
    default:
        return 1; // unreachable
    }
}

} // namespace

void Arm7::step_arm() {
    assert((state_.pc & 0x3u) == 0 && "Arm7::step_arm: ARM pc must be 4-aligned");
    assert((state_.cpsr & (1u << 5)) == 0 && "Arm7::step_arm requires CPSR.T=0");

    const u32 instr_addr = state_.pc;
    const u32 instr = bus_->read32(instr_addr);

    // R15 during execute reads as instruction_addr + 8 (3-stage pipeline).
    // Advance pc BEFORE dispatch so branch handlers can stomp it freely
    // without us overwriting their target afterwards.
    state_.r[15] = instr_addr + 8;
    state_.pc = instr_addr + 4;

    const u32 cycles_consumed = dispatch_arm(state_, *bus_, instr, instr_addr);
    assert(cycles_consumed > 0 && "Arm7::step_arm: dispatch must consume >= 1 cycle");
    state_.cycles += cycles_consumed;
}

} // namespace ds
