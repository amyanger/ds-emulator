// arm9_branch.cpp — ARM9 branch dispatch. Handles B and BL.
//
// B/BL is bit-identical between ARMv4T and ARMv5TE (slice 3l design §2.1,
// §5.7): same encoding, same (PC+8)-relative signed word offset, same LR
// value for BL, no flag or mode effects. The lone v5 addition sharing this
// primary slot is BLX(imm) at cond==1111, deferred to slice 3m; it never
// reaches here because eval_condition() in arm9_decode.cpp treats cond==1111
// as "never" (the v4 NV slot this slice models, §4.6). BX — the register
// branch in the bits[27:25]==000 space — lands with LDR/STR in commit 8.
// Faithful port of arm7_branch.cpp onto Arm9State.

#include "cpu/arm9/arm9_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_branch(Arm9State& state, u32 instr) {
    // B / BL encoding: cond 101 L offset24
    // offset24 is a 24-bit signed word displacement from (pc + 8).
    const u32 offset24 = instr & 0x00FF'FFFFu;
    // Sign-extend 24 bits → 32 bits, then shift left by 2 for a byte offset.
    const i32 signed_off = static_cast<i32>(offset24 << 8) >> 8; // arith shift
    const u32 byte_off = static_cast<u32>(signed_off) << 2;
    const u32 target = state.r[15] + byte_off; // r[15] == instr_addr + 8

    const bool link = ((instr >> 24) & 1u) != 0;
    if (link) {
        // LR = address of the instruction after the BL. step_arm set
        // state.pc = instr_addr + 4 before dispatch, which is exactly that.
        state.r[14] = state.pc;
    }
    write_rd(state, 15, target);
    return 1;
}

} // namespace ds
