// arm7_thumb_branch.cpp — Thumb conditional/unconditional branch and BL
// (two-halfword) formats. Slice 3c capstone (commit 18):
//
//   THUMB.16: 1101 cond4 imm8             — Bcond (cond 0xE UNDEF, 0xF → SWI)
//   THUMB.17: 11011111  imm8              — SWI (slice 3c warn stub only)
//   THUMB.18: 11100 imm11                 — B unconditional
//   THUMB.19: 11110 imm11_hi              — BL first halfword (sets LR)
//             11111 imm11_lo              — BL second halfword (branches via LR)
//             11101 imm11_lo              — BLX second halfword (ARMv5 UNDEF)
//
// Branch targets MUST be written via state.pc, NOT via write_rd. step_thumb
// pre-clobbers state.r[15] = instr_addr + 4 at the top of every step, so
// writing only r[15] here would silently lose the branch. Writing state.pc
// is the supported redirect contract (see arm7_thumb_decode.cpp:step_thumb).
// It also avoids write_rd's word-align-down which would drop bit 1 of a
// halfword-aligned Thumb target.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/arm7_thumb_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_thumb_bcond_swi(Arm7State& state,
                             Arm7Bus& /*bus*/,
                             u16 instr,
                             u32 instr_addr,
                             u32 /*pc_read*/,
                             u32 /*pc_literal*/) {
    const u32 cond = (instr >> 8) & 0xFu;
    const u32 imm8 = instr & 0xFFu;

    if (cond == 0xFu) {
        // THUMB.17 SWI — slice 3c warn stub. Mode switch / SPSR save / vector
        // jump land in slice 3d. CPSR, R14_svc, SPSR_svc, and PC must all
        // remain unchanged modulo the PC += 2 step_thumb already performed.
        DS_LOG_WARN("arm7/thumb17: SWI imm=0x%02X (slice 3c stub) at 0x%08X", imm8, instr_addr);
        return 1;
    }
    if (cond == 0xEu) {
        // cond 0xE in THUMB.16 is UNDEF (the "always" code is reserved here).
        DS_LOG_WARN("arm7/thumb16: cond=0xE UNDEF at 0x%08X", instr_addr);
        return 1;
    }

    if (!eval_condition(cond, state.cpsr)) {
        return 1; // not taken — state.pc already advanced to instr_addr + 2
    }

    // Sign-extend imm8 to 32 bits, then multiply by 2 for a halfword offset.
    const i32 off = static_cast<i32>(static_cast<i8>(static_cast<u8>(imm8))) * 2;
    state.pc = instr_addr + 4u + static_cast<u32>(off);
    return 1;
}

u32 dispatch_thumb_b_uncond(Arm7State& state,
                            Arm7Bus& /*bus*/,
                            u16 instr,
                            u32 instr_addr,
                            u32 /*pc_read*/,
                            u32 /*pc_literal*/) {
    const u32 imm11 = instr & 0x7FFu;
    // Sign-extend 11 bits via left-then-arithmetic-right. Result is a signed
    // halfword count; <<1 turns it into a byte offset.
    const i32 sext = static_cast<i32>(imm11 << 21) >> 21;
    const i32 off = sext << 1;
    state.pc = instr_addr + 4u + static_cast<u32>(off);
    return 1;
}

u32 dispatch_thumb_bl_prefix(Arm7State& state,
                             Arm7Bus& /*bus*/,
                             u16 instr,
                             u32 /*instr_addr*/,
                             u32 pc_read,
                             u32 /*pc_literal*/) {
    // First halfword: LR = (instr_addr + 4) + (sext(imm11_hi) << 12).
    // pc_read == instr_addr + 4.
    const u32 imm11 = instr & 0x7FFu;
    const i32 sext = static_cast<i32>(imm11 << 21) >> 21;
    const i32 off = sext << 12;
    state.r[14] = pc_read + static_cast<u32>(off);
    // Do NOT touch state.pc — it's already instr_addr + 2, which is the
    // correct "sequential next halfword" for the following BL suffix.
    return 1;
}

u32 dispatch_thumb_bl_suffix(Arm7State& state,
                             Arm7Bus& /*bus*/,
                             u16 instr,
                             u32 instr_addr,
                             u32 /*pc_read*/,
                             u32 /*pc_literal*/) {
    // bits[12:11] discriminate BL (11111) from BLX (11101).
    const u32 form = (instr >> 11) & 0x3u;
    if (form == 0b01u) {
        // 11101 imm11_lo — BLX label, ARMv5 only. UNDEF on ARMv4.
        DS_LOG_WARN(
            "arm7/thumb19: BLX (second halfword 11101) is ARMv5 only, UNDEF on ARMv4 at 0x%08X",
            instr_addr);
        return 1;
    }

    // 11111 imm11_lo — BL second halfword.
    const u32 imm11_lo = instr & 0x7FFu;
    const u32 target = state.r[14] + (imm11_lo << 1);
    // Return address is the instruction AFTER the BL suffix; the |1 sets the
    // Thumb bit so a later `BX LR` stays in Thumb state.
    state.r[14] = (instr_addr + 2u) | 1u;
    state.pc = target;
    return 1;
}

} // namespace ds
