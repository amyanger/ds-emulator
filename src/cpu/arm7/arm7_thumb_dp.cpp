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

// ---- THUMB.4 (bits[15:10] == 010000) ----
// ALU register/register: Rd = Rd <op> Rs.  Sixteen ops.

u32 dispatch_thumb_alu(Arm7State& state,
                       Arm7Bus& /*bus*/,
                       u16 instr,
                       u32 instr_addr,
                       u32 /*pc_read*/,
                       u32 /*pc_literal*/) {
    const u32 op = (instr >> 6) & 0xFu;
    const u32 rs = (instr >> 3) & 0x7u;
    const u32 rd = instr & 0x7u;

    const u32 rd_value = state.r[rd];
    const u32 rs_value = state.r[rs];
    const bool current_c = (state.cpsr & (1u << 29)) != 0;

    // Shift ops (LSL/LSR/ASR/ROR) use the register-shift barrel path.
    // The amount is Rs[7:0]. If amount == 0, C is unchanged.
    auto do_shift = [&](ShiftType st) -> u32 {
        const u32 amount = rs_value & 0xFFu;
        auto sr = barrel_shift_reg(rd_value, st, amount);
        const bool carry = (amount == 0) ? current_c : sr.carry;
        return execute_dp_op(state, DpOp::MOV, 0, sr.value, carry, true, rd, instr_addr);
    };

    switch (op) {
    case 0x0: // AND — NZ only
        return execute_dp_op(state, DpOp::AND, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0x1: // EOR — NZ only
        return execute_dp_op(state, DpOp::EOR, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0x2: // LSL reg — NZC (C unchanged if amount==0)
        return do_shift(ShiftType::Lsl);
    case 0x3: // LSR reg
        return do_shift(ShiftType::Lsr);
    case 0x4: // ASR reg
        return do_shift(ShiftType::Asr);
    case 0x5: // ADC — NZCV
        return execute_dp_op(state, DpOp::ADC, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0x6: // SBC — NZCV
        return execute_dp_op(state, DpOp::SBC, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0x7: // ROR reg — NZC
        return do_shift(ShiftType::Ror);
    case 0x8: // TST — NZ only, no writeback
        return execute_dp_op(state, DpOp::TST, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0x9: // NEG = RSB Rd, Rs, #0 — NZCV
        return execute_dp_op(state, DpOp::RSB, rs_value, 0, current_c, true, rd, instr_addr);
    case 0xA: // CMP — NZCV, no writeback
        return execute_dp_op(state, DpOp::CMP, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0xB: // CMN — NZCV, no writeback
        return execute_dp_op(state, DpOp::CMN, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0xC: // ORR — NZ only
        return execute_dp_op(state, DpOp::ORR, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0xD: { // MUL — ARMv4: NZ set, C destroyed, V unchanged
        const u32 result = rd_value * rs_value;
        write_rd(state, rd, result);
        state.cpsr = set_nz(state.cpsr, result);
        // ARMv4: C is implementation-defined (destroyed). We invert it
        // so the destroy is observable in tests. ARM9 (ARMv5) will flip
        // this to "C unchanged."
        state.cpsr = set_c(state.cpsr, !current_c);
        return 1;
    }
    case 0xE: // BIC — NZ only
        return execute_dp_op(state, DpOp::BIC, rd_value, rs_value, current_c, true, rd, instr_addr);
    case 0xF: // MVN — NZ only
        return execute_dp_op(state, DpOp::MVN, rd_value, rs_value, current_c, true, rd, instr_addr);
    default:
        return 1; // unreachable — op is 4 bits
    }
}

// ---- THUMB.5 (bits[15:10] == 010001) ----
// Hi-register ADD/CMP/MOV and BX.  Operates on the full R0-R15 range.

u32 dispatch_thumb_hireg_bx(
    Arm7State& state, Arm7Bus& /*bus*/, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal) {
    const u32 op = (instr >> 8) & 0x3u;
    const u32 hd = (instr >> 7) & 0x1u;
    const u32 hs = (instr >> 6) & 0x1u;
    const u32 rs_full = (hs << 3) | ((instr >> 3) & 0x7u);
    const u32 rd_full = (hd << 3) | (instr & 0x7u);

    // PC-as-source: ADD/CMP/MOV read R15 as $+4 (no alignment).
    // BX R15 uses ($+4) & ~2 (word-aligned for ARM-state target).
    const u32 pc_for_op = (op == 3) ? pc_literal : pc_read;
    const u32 rs_value = (rs_full == 15) ? pc_for_op : state.r[rs_full];
    const u32 rd_value = (rd_full == 15) ? pc_for_op : state.r[rd_full];

    switch (op) {
    case 0: { // ADD — no flag update
        if (hd == 0 && hs == 0) {
            DS_LOG_WARN("arm7/thumb5: ADD with Hd=0 Hs=0 (UNPREDICTABLE) "
                        "at 0x%08X",
                        instr_addr);
        }
        const u32 result = rd_value + rs_value;
        write_rd(state, rd_full, result);
        return 1;
    }
    case 1: { // CMP — full NZCV, no writeback
        if (hd == 0 && hs == 0) {
            DS_LOG_WARN("arm7/thumb5: CMP with Hd=0 Hs=0 (UNPREDICTABLE) "
                        "at 0x%08X",
                        instr_addr);
        }
        const bool current_c = (state.cpsr & (1u << 29)) != 0;
        execute_dp_op(state, DpOp::CMP, rd_value, rs_value, current_c, true, rd_full, instr_addr);
        return 1;
    }
    case 2: { // MOV — no flag update
        if (hd == 0 && hs == 0) {
            DS_LOG_WARN("arm7/thumb5: MOV with Hd=0 Hs=0 (UNPREDICTABLE) "
                        "at 0x%08X",
                        instr_addr);
        }
        write_rd(state, rd_full, rs_value);
        return 1;
    }
    case 3: { // BX (Hd=0) or BLX (Hd=1, ARMv5 only)
        if (hd != 0) {
            DS_LOG_WARN("arm7/thumb5: BLX (Hd=1) is ARMv5 only, stub on "
                        "ARMv4 at 0x%08X",
                        instr_addr);
            return 1;
        }
        return execute_bx(state, rs_value);
    }
    default:
        return 1; // unreachable
    }
}

// ---- THUMB.12 (bits[15:12] == 1010) ----
// ADD Rd, PC, #imm8<<2  (op=0)  — load-effective-address from literal-pool PC
// ADD Rd, SP, #imm8<<2  (op=1)  — load-effective-address from SP
// CPSR is not modified.
//
// PC source MUST be `pc_literal` ((instr_addr + 4) & ~2), NOT `state.r[15]`.
// state.r[15] equals instr_addr + 4 (== pc_read), which is unaligned when
// instr_addr % 4 == 2 — using it would silently break every literal-pool
// fetch from a Thumb instruction that landed on a non-word-aligned address.

u32 dispatch_thumb_add_pc_sp(Arm7State& state,
                             Arm7Bus& /*bus*/,
                             u16 instr,
                             u32 /*instr_addr*/,
                             u32 /*pc_read*/,
                             u32 pc_literal) {
    const u32 op = (instr >> 11) & 1u;
    const u32 rd = (instr >> 8) & 0x7u;
    const u32 offset = (static_cast<u32>(instr) & 0xFFu) << 2;
    const u32 source = (op == 0u) ? pc_literal : state.r[13];
    write_rd(state, rd, source + offset);
    return 1;
}

// ---- THUMB.13 (bits[15:8] == 10110000) ----
// ADD SP, #imm7<<2  (S=0)  — SP += offset
// SUB SP, #imm7<<2  (S=1)  — SP -= offset
// CPSR is not modified. Rd is implicitly R13 — no Rd field in the encoding.

u32 dispatch_thumb_add_sp_signed(Arm7State& state,
                                 Arm7Bus& /*bus*/,
                                 u16 instr,
                                 u32 /*instr_addr*/,
                                 u32 /*pc_read*/,
                                 u32 /*pc_literal*/) {
    const u32 s_bit = (instr >> 7) & 1u;
    const u32 offset = (static_cast<u32>(instr) & 0x7Fu) << 2;
    const u32 sp = state.r[13];
    write_rd(state, 13, (s_bit != 0u) ? (sp - offset) : (sp + offset));
    return 1;
}

} // namespace ds
