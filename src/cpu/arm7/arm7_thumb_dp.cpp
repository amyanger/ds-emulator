// arm7_thumb_dp.cpp — Thumb data-processing format handlers.
//
// THUMB.1 (shift imm): barrel_shift_imm + execute_dp_op(MOV). The
//   barrel shifter already handles the ARMv4T zero-amount quirks:
//   LSL #0 = no shift, C unchanged; LSR/ASR #0 = shift by 32.
//   Flags: NZC updated (V unchanged). C unchanged for LSL #0.
//
// THUMB.2 (add/sub): ADD/SUB with reg or imm3 operand. Full NZCV
//   always, even for the ADD Rd,Rs,#0 (MOV pseudo).
//
// THUMB.3 (imm8 DP): execute_dp_op with shifter_carry = current_c
//   so MOV preserves C/V (logical-form path sets NZ + shifter_carry).

#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_thumb_internal.hpp"

namespace ds {

// ---- THUMB.1 + THUMB.2 bucket (bits[15:13] == 000) ----

u32 dispatch_thumb_shift_or_addsub(Arm7State& state,
                                   Arm7Bus& /*bus*/,
                                   u16 instr,
                                   u32 instr_addr,
                                   u32 /*pc_read*/,
                                   u32 /*pc_literal*/) {
    const u32 op = (instr >> 11) & 0x3u;

    if (op == 3) {
        // THUMB.2: 00011 I op Rn3/imm3 Rs3 Rd3
        // I (bit 10): 0=register, 1=3-bit immediate
        // op (bit 9): 0=ADD, 1=SUB
        const bool i_bit = ((instr >> 10) & 1u) != 0;
        const bool sub = ((instr >> 9) & 1u) != 0;
        const u32 rn_or_imm = (instr >> 6) & 0x7u;
        const u32 rs = (instr >> 3) & 0x7u;
        const u32 rd = instr & 0x7u;

        const u32 operand2 = i_bit ? rn_or_imm : state.r[rn_or_imm];
        const bool current_c = (state.cpsr & (1u << 29)) != 0;

        return execute_dp_op(state,
                             sub ? DpOp::SUB : DpOp::ADD,
                             state.r[rs],
                             operand2,
                             current_c,
                             true,
                             rd,
                             instr_addr);
    }

    // THUMB.1: 000 op2 off5 Rs3 Rd3
    // op: 00=LSL, 01=LSR, 10=ASR
    const u32 offset = (instr >> 6) & 0x1Fu;
    const u32 rs = (instr >> 3) & 0x7u;
    const u32 rd = instr & 0x7u;

    const bool current_c = (state.cpsr & (1u << 29)) != 0;
    const auto sr = barrel_shift_imm(state.r[rs], static_cast<ShiftType>(op), offset, current_c);

    // Rd = shifted result, flags NZC (V unchanged).
    // execute_dp_op(MOV) with s_flag=true sets NZ + shifter_carry. V untouched.
    return execute_dp_op(state, DpOp::MOV, 0, sr.value, sr.carry, true, rd, instr_addr);
}

// ---- THUMB.3 (bits[15:13] == 001) ----

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
