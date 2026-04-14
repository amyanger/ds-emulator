// arm7_branch.cpp — ARMv4T branch dispatch. Handles B and BL.
// BX lives in DP encoding space and is caught inside arm7_dp.cpp.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_branch(Arm7State& state, u32 instr) {
    // B / BL encoding: cond 101 L offset24
    // offset24 is a 24-bit signed word displacement from (pc + 8).
    const u32  offset24 = instr & 0x00FF'FFFFu;
    // Sign-extend 24 bits → 32 bits, then shift left by 2 for a byte offset.
    const i32  signed_off = static_cast<i32>(offset24 << 8) >> 8;  // arith shift
    const u32  byte_off   = static_cast<u32>(signed_off) << 2;
    const u32  target     = state.r[15] + byte_off;  // r[15] == instr_addr + 8

    const bool link = ((instr >> 24) & 1u) != 0;
    if (link) {
        // LR = address of the instruction after the BL.
        // state.pc was set to instr_addr + 4 by step_arm BEFORE dispatch,
        // which is exactly the address we want.
        state.r[14] = state.pc;
    }
    write_rd(state, 15, target);
    return 1;
}

}  // namespace ds
