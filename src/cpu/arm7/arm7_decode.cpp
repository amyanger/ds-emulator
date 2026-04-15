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

#include <cassert>

namespace ds {

namespace {

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
        // Coproc / SWI — deferred to slice 3d.
        DS_LOG_WARN("arm7: coproc/SWI form 0x%08X at 0x%08X", instr, instr_addr);
        return 1;
    default:
        return 1; // unreachable
    }
}

} // namespace

void Arm7::step_arm() {
    assert((state_.pc & 0x3u) == 0 && "Arm7::step_arm: ARM pc must be 4-aligned");
    assert((state_.cpsr & (1u << 5)) == 0 && "Arm7::step_arm: Thumb mode not implemented");

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
