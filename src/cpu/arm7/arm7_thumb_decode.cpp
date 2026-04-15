// arm7_thumb_decode.cpp — top-level ARMv4T Thumb fetch / decode / dispatch.
// Slice 3c Phase B commit 7: dispatch_thumb materializes two PC values
// at the top — pc_read (instr_addr + 4) and pc_literal ((instr_addr + 4)
// & ~2) — and forwards both to every bucket handler. Bucket handlers are
// still warn stubs; per-format decode lands in Phase C/D.
//
// PC conventions (see §4.5 of the slice 3c spec):
//   pc_read    = instr_addr + 4       — what R15 reads as during execute
//   pc_literal = (instr_addr + 4) & ~2 — word-aligned, for THUMB.6 / .12 /
//                                        THUMB.5 BX R15
//
// Cycle cost model: 1 ARM7 cycle per Thumb instruction (coarse).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_thumb_internal.hpp"

#include <cassert>

namespace ds {

// dispatch_thumb_shift_or_addsub — THUMB.1 shift imm + THUMB.2 add/sub.
// Defined in arm7_thumb_dp.cpp.

// dispatch_thumb_imm_dp — THUMB.3 MOV/CMP/ADD/SUB imm8.
// Defined in arm7_thumb_dp.cpp.

u32 dispatch_thumb_010_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal) {
    // Fan-out: bit12 separates THUMB.4/5/6 (bit12=0) from THUMB.7/8 (bit12=1).
    if ((instr & 0xFC00u) == 0x4000u) {
        // THUMB.4: 010000 op4 Rs3 Rd3
        return dispatch_thumb_alu(state, bus, instr, instr_addr, pc_read, pc_literal);
    }
    if ((instr & 0xFC00u) == 0x4400u) {
        // THUMB.5: 010001 op2 Hd Hs Rs3 Rd3
        return dispatch_thumb_hireg_bx(state, bus, instr, instr_addr, pc_read, pc_literal);
    }
    if ((instr & 0xF800u) == 0x4800u) {
        // THUMB.6: 01001 Rd3 imm8 — LDR PC-rel
        return dispatch_thumb_ldr_pc(state, bus, instr, instr_addr, pc_read, pc_literal);
    }
    // THUMB.7: 0101 xx 0 Ro Rb Rd — LDR/STR/LDRB/STRB reg offset (bit 9 = 0)
    if ((instr & 0xF200u) == 0x5000u) {
        return dispatch_thumb_ldst_reg(state, bus, instr, instr_addr, pc_read, pc_literal);
    }
    // THUMB.8: 0101 xx 1 Ro Rb Rd — halfword/signed reg offset (bit 9 = 1)
    return dispatch_thumb_ldst_halfword_reg(state, bus, instr, instr_addr, pc_read, pc_literal);
}

u32 dispatch_thumb_ldst_imm_wb(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal) {
    // Entire 011 bucket is THUMB.9 — LDR/STR/LDRB/STRB imm offset.
    return dispatch_thumb_ldst_imm(state, bus, instr, instr_addr, pc_read, pc_literal);
}

u32 dispatch_thumb_100_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal) {
    if ((instr & 0xF000u) == 0x9000u) {
        // THUMB.11: 1001 op1 Rd3 imm8 — LDR/STR SP-rel
        return dispatch_thumb_ldst_sp(state, bus, instr, instr_addr, pc_read, pc_literal);
    }
    // THUMB.10: 1000 op1 imm5 Rb3 Rd3 — STRH/LDRH imm offset
    return dispatch_thumb_ldst_halfword_imm(state, bus, instr, instr_addr, pc_read, pc_literal);
}

u32 dispatch_thumb_101_space(
    Arm7State&, Arm7Bus&, u16 instr, u32 instr_addr, u32 /*pc_read*/, u32 /*pc_literal*/) {
    DS_LOG_WARN("arm7/thumb: 101_space stub instr=0x%04X at 0x%08X", instr, instr_addr);
    return 1;
}

u32 dispatch_thumb_110_space(
    Arm7State&, Arm7Bus&, u16 instr, u32 instr_addr, u32 /*pc_read*/, u32 /*pc_literal*/) {
    DS_LOG_WARN("arm7/thumb: 110_space stub instr=0x%04X at 0x%08X", instr, instr_addr);
    return 1;
}

u32 dispatch_thumb_111_space(
    Arm7State&, Arm7Bus&, u16 instr, u32 instr_addr, u32 /*pc_read*/, u32 /*pc_literal*/) {
    DS_LOG_WARN("arm7/thumb: 111_space stub instr=0x%04X at 0x%08X", instr, instr_addr);
    return 1;
}

u32 dispatch_thumb(Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr) {
    // Thumb PC during execute = instr_addr + 4 (2-stage prefetch).
    // step_thumb already sets state.r[15] = instr_addr + 4 before calling
    // us, so state.r[15] == pc_read at this point. We materialize both
    // values here once and thread them to bucket handlers so that:
    //   (a) literal-pool formats get the word-aligned version without
    //       re-deriving it from state.r[15] (which would be error-prone),
    //   (b) no format handler needs to know the raw instr_addr.
    const u32 pc_read = instr_addr + 4;
    const u32 pc_literal = (instr_addr + 4) & ~2u;

    const u32 bits_15_13 = (instr >> 13) & 0x7u;
    switch (bits_15_13) {
    case 0b000:
        return dispatch_thumb_shift_or_addsub(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b001:
        return dispatch_thumb_imm_dp(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b010:
        return dispatch_thumb_010_space(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b011:
        return dispatch_thumb_ldst_imm_wb(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b100:
        return dispatch_thumb_100_space(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b101:
        return dispatch_thumb_101_space(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b110:
        return dispatch_thumb_110_space(state, bus, instr, instr_addr, pc_read, pc_literal);
    case 0b111:
        return dispatch_thumb_111_space(state, bus, instr, instr_addr, pc_read, pc_literal);
    default:
        return 1; // unreachable — bits_15_13 is a 3-bit value
    }
}

void Arm7::step_thumb() {
    assert((state_.pc & 0x1u) == 0 && "Arm7::step_thumb: Thumb pc must be halfword-aligned");
    assert((state_.cpsr & (1u << 5)) != 0 && "Arm7::step_thumb requires CPSR.T=1");

    const u32 instr_addr = state_.pc;
    const u16 instr = bus_->read16(instr_addr);

    // R15 during execute reads as instruction_addr + 4 (Thumb 2-stage
    // prefetch). Advance pc BEFORE dispatch so branch handlers can
    // stomp it freely without us overwriting their target afterwards.
    // Branch-target contract: handlers that redirect execution must
    // write state_.pc (the next-fetch cursor). state_.r[15] is
    // recomputed from instr_addr at the top of the next step, so
    // writing only r[15] in a handler silently loses the branch.
    state_.r[15] = instr_addr + 4;
    state_.pc = instr_addr + 2;

    const u32 cycles_consumed = dispatch_thumb(state_, *bus_, instr, instr_addr);
    assert(cycles_consumed > 0 && "Arm7::step_thumb: dispatch must consume >= 1 cycle");
    state_.cycles += cycles_consumed;
}

} // namespace ds
