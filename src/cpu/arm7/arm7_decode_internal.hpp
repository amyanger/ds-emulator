#pragma once

// Private header shared between the ARM7 decoder translation units
// (arm7_decode.cpp, arm7_dp.cpp, arm7_branch.cpp, arm7_loadstore.cpp).
// NOT part of any public include path. Only files under src/cpu/arm7/
// may include it.

#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7Bus;

// Single register-file writeback point used by every family handler.
// Writing to R15 also stomps state.pc so the next fetch lands on the
// target; callers relying on the pipeline model (state.r[15] ==
// instr_addr + 8) must read R15 BEFORE calling this.
inline void write_rd(Arm7State& s, u32 rd, u32 value) {
    s.r[rd] = value;
    if (rd == 15) {
        s.pc = value & ~0x3u;
    }
}

// Read Rm in register-shift DP form. In reg-shift form, the ARM reads Rm
// one pipeline stage later than in imm-shift form, so Rm == 15 returns
// PC + 12 instead of PC + 8. Also used for Rn reads in reg-shift form.
inline u32 read_rm_pc12(const Arm7State& s, u32 rm) {
    return (rm == 15) ? (s.r[15] + 4) : s.r[rm];
}

// Read Rs (the register holding the shift amount) in register-shift DP
// form. Rs == 15 is UNPREDICTABLE on real ARMv4T hardware; we log a warn
// and return PC + 12 for determinism. Games do not use this form.
inline u32 read_rs_for_reg_shift(const Arm7State& s, u32 rs) {
    if (rs == 15) {
        DS_LOG_WARN("arm7: reg-shift DP with Rs == 15 (unpredictable) at PC 0x%08X", s.pc);
    }
    return (rs == 15) ? (s.r[15] + 4) : s.r[rs];
}

// LDR-style misaligned word rotate: ARMv4T rotates the aligned word read
// right by (addr & 3) * 8 so that the requested byte lands in bits[7:0].
// Used by LDR word load (arm7_loadstore.cpp) and by SWP word (commit 16).
inline u32 rotate_read_word(u32 raw, u32 addr) {
    const u32 rot = (addr & 0x3u) * 8u;
    return (rot == 0) ? raw : ((raw >> rot) | (raw << (32u - rot)));
}

// Family dispatch helpers. Each returns the number of ARM7 cycles the
// instruction consumed. In slice 3b1 every path returns 1.
u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr);
u32 dispatch_branch(Arm7State& state, u32 instr);
u32 dispatch_single_data_transfer(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Fan-out for the bits[27:25] == 000 encoding neighborhood. Checks BX,
// multiply, halfword / signed data transfer, and PSR transfer (MRS / MSR)
// in that order; falls through to dispatch_dp() for any operand-form DP
// instruction that didn't match a recognizer. The bits[27:25] == 001
// (imm-form DP) slot is routed directly to dispatch_dp() from
// arm7_decode.cpp and does not pass through here.
u32 dispatch_000_space(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Block data transfer (LDM / STM). Called from dispatch_arm for the
// bits[27:25] == 100 primary slot. Scaffolded in commit 3 as a warn
// stub; actual LDM/STM semantics land in commits 4-15.
u32 dispatch_block(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Single data swap (SWP / SWPB). Called from dispatch_000_space when
// the (instr & 0x0FB00FF0) == 0x01000090 recognizer matches. SWP is
// routed before the multiply/halfword recognizers so the bit pattern
// cannot be mis-claimed.
u32 dispatch_swap(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Multiply family dispatcher. Called from dispatch_000_space after the
// multiply recognizer matches ((instr & 0x0F0000F0) == 0x00000090).
u32 dispatch_multiply(Arm7State& state, u32 instr, u32 instr_addr);

// PSR transfer (MRS / MSR) dispatcher. Called from dispatch_000_space for
// MRS and MSR-reg, and from dispatch_dp for MSR-imm. Handles both CPSR and
// SPSR forms; the caller has already confirmed the encoding shape.
u32 dispatch_psr_transfer(Arm7State& state, u32 instr, u32 instr_addr);

// Halfword / signed data transfer dispatch entry point.
// Called from dispatch_000_space after the halfword recognizer matches.
// Handles LDRH, STRH, LDRSB, LDRSH. Warns on LDRD/STRD/SWP slots and
// on malformed encodings per GBATEK §"ARM Memory: Halfword, Doubleword,
// and Signed Data Transfer".
u32 dispatch_halfword(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

} // namespace ds
