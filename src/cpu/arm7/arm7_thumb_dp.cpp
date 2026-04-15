// arm7_thumb_dp.cpp — Thumb data-processing format handlers.
// Slice 3c commit 8: THUMB.3 MOV/CMP/ADD/SUB imm8.
//
// THUMB.3 encoding: 001 op2 Rd3 imm8
//   op (bits[12:11]): 00=MOV, 01=CMP, 10=ADD, 11=SUB
//   Rd: bits[10:8] (R0-R7 only)
//   imm8: bits[7:0] (0..255, zero-extended)
//
// Flag rules (GBATEK):
//   MOV: N, Z only. C and V unchanged.
//   CMP: full NZCV, no register writeback.
//   ADD: full NZCV.
//   SUB: full NZCV.
//
// MOV's NZ-only behavior falls out naturally from execute_dp_op: MOV is
// a logical-form op, so execute_dp_op sets NZ + shifter_carry. By passing
// shifter_carry = current_c, C is preserved. V is never touched for
// logical-form ops.

#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_thumb_internal.hpp"

namespace ds {

u32 dispatch_thumb_imm_dp(Arm7State& state,
                          Arm7Bus& /*bus*/,
                          u16 instr,
                          u32 instr_addr,
                          u32 /*pc_read*/,
                          u32 /*pc_literal*/) {
    const u32 op = (instr >> 11) & 0x3u;
    const u32 rd = (instr >> 8) & 0x7u;
    const u32 imm8 = instr & 0xFFu;

    static constexpr DpOp op_table[4] = {DpOp::MOV, DpOp::CMP, DpOp::ADD, DpOp::SUB};

    const u32 rn_value = state.r[rd];
    const bool current_c = (state.cpsr & (1u << 29)) != 0;

    return execute_dp_op(state, op_table[op], rn_value, imm8, current_c, true, rd, instr_addr);
}

} // namespace ds
