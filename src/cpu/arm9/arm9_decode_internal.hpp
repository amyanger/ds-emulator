#pragma once

// Private header shared between the ARM9 decoder translation units
// (arm9_decode.cpp, arm9_decode_000.cpp, arm9_decode_stubs.cpp, and the
// family executors landing in later commits). NOT part of any public
// include path. Only files under src/cpu/arm9/ may include it.

#include "cpu/arm9/arm9_state.hpp"
#include "cpu/common/arm_alu.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm9Bus;

// Single register-file writeback point used by every family handler.
// Writing to R15 also stomps state.pc so the next fetch lands on the
// target; callers relying on the pipeline model (state.r[15] ==
// instr_addr + 8) must read R15 BEFORE calling this.
inline void write_rd(Arm9State& s, u32 rd, u32 value) {
    s.r[rd] = value;
    if (rd == 15) {
        s.pc = value & ~0x3u;
    }
}

// Read Rm in register-shift DP form. In reg-shift form, the ARM reads Rm
// one pipeline stage later than in imm-shift form, so Rm == 15 returns
// PC + 12 instead of PC + 8. Also used for Rn reads in reg-shift form.
inline u32 read_rm_pc12(const Arm9State& s, u32 rm) {
    return (rm == 15) ? (s.r[15] + 4) : s.r[rm];
}

// Read Rs (the register holding the shift amount) in register-shift DP
// form. Rs == 15 is UNPREDICTABLE on real hardware; we log a warn and
// return PC + 12 for determinism. Games do not use this form.
inline u32 read_rs_for_reg_shift(const Arm9State& s, u32 rs) {
    if (rs == 15) {
        DS_LOG_WARN("arm9: reg-shift DP with Rs == 15 (unpredictable) at PC 0x%08X", s.pc);
    }
    return (rs == 15) ? (s.r[15] + 4) : s.r[rs];
}

// LDR-style misaligned word rotate: rotates the aligned word read right by
// (addr & 3) * 8 so that the requested byte lands in bits[7:0]. Used by LDR
// word load and by SWP word.
inline u32 rotate_read_word(u32 raw, u32 addr) {
    const u32 rot = (addr & 0x3u) * 8u;
    return (rot == 0) ? raw : ((raw >> rot) | (raw << (32u - rot)));
}

// Fan-out for the bits[27:25] == 000 encoding neighborhood. Checks BX,
// SWP, multiply, halfword / signed data transfer, and the MRS/MSR-reg PSR
// forms in that order; falls through to dispatch_dp() for any operand-form
// DP instruction that didn't match a recognizer. Real recognizer fan-out
// in commit 5; the leaves it routes to are commit-5 stubs.
u32 dispatch_000_space(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr);

// Data-processing (DP) dispatcher.
u32 dispatch_dp(Arm9State& state, u32 instr, u32 instr_addr);

// Shared ARMv4T-subset data-processing executor. Callers pre-materialize all
// operands (including any PC pipeline offset on rn_value) so the same body
// serves ARM-state dispatch_dp and the Thumb DP handlers (slice 3n). The DP
// family has no ARMv5 divergence, so this is a faithful peer of ARM7's
// execute_dp_op.
u32 execute_dp_op(
    Arm9State& state, DpOp op, u32 rn_value, u32 operand2, bool shifter_carry, bool s_flag, u32 rd);

// Branch / branch-and-link (B, BL) dispatcher. No ARMv5 divergence; faithful
// port of ARM7's dispatch_branch (arm9_branch.cpp). BLX-imm is deferred to 3m.
u32 dispatch_branch(Arm9State& state, u32 instr);

// Single data transfer (LDR / STR word/byte) dispatcher.
// Warn-stub in commit 5; real executor lands in commit 8.
u32 dispatch_single_data_transfer(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr);

// Shared BX executor: CPSR.T <- rm_value[0], PC <- rm_value aligned for the
// target state. Warn-stub in commit 5; real executor lands in commit 8.
u32 execute_bx(Arm9State& state, u32 rm_value);

// Halfword / signed data transfer (LDRH / STRH / LDRSB / LDRSH) dispatcher.
// Warn-stub in commit 5; real executor lands in commit 9.
u32 dispatch_halfword(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr);

// Block data transfer (LDM / STM) dispatcher.
// Warn-stub in commit 5; real executor lands in commit 10.
u32 dispatch_block(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr);

// Multiply family dispatcher.
// Warn-stub in commit 5; real executor lands in commit 11.
u32 dispatch_multiply(Arm9State& state, u32 instr, u32 instr_addr);

// PSR transfer (MRS / MSR) dispatcher.
// Warn-stub in commit 5; real executor lands in commit 12.
u32 dispatch_psr_transfer(Arm9State& state, u32 instr, u32 instr_addr);

// Single data swap (SWP / SWPB) dispatcher.
// Warn-stub in commit 5; real executor lands in commit 13.
u32 dispatch_swap(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr);

} // namespace ds
