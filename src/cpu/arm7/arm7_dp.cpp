// arm7_dp.cpp — ARMv4T data-processing dispatch. This file owns
// dispatch_dp(), which handles the whole bits[27:26] == 00 encoding
// space. That space also contains MUL/MLA, halfword LDR/STR, MRS/MSR,
// and BX. Slice 3b1 recognizes BX at the top of this file; the rest
// remain deferred and fall through to a warn path.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_alu.hpp"

namespace ds {

namespace {

// Data-processing opcode numbers (bits 24..21 of the instruction).
enum class DpOp : u8 {
    AND = 0x0, EOR = 0x1, SUB = 0x2, RSB = 0x3,
    ADD = 0x4, ADC = 0x5, SBC = 0x6, RSC = 0x7,
    TST = 0x8, TEQ = 0x9, CMP = 0xA, CMN = 0xB,
    ORR = 0xC, MOV = 0xD, BIC = 0xE, MVN = 0xF,
};

}  // namespace

u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr) {
    // BX lives in DP encoding space: cond 0001 0010 1111 1111 1111 0001 Rm.
    // Mask: 0x0FFFFFF0, match: 0x012FFF10 (with the condition field masked).
    if ((instr & 0x0FFF'FFF0u) == 0x012F'FF10u) {
        const u32  rm      = instr & 0xFu;
        const u32  rm_val  = state.r[rm];          // rm==15 reads instr_addr+8
        const bool thumb   = (rm_val & 0x1u) != 0;
        const u32  target  = thumb ? (rm_val & ~0x1u) : (rm_val & ~0x3u);
        if (thumb) {
            state.cpsr |= (1u << 5);               // set T flag
        }
        write_rd(state, 15, target);
        (void)instr_addr;
        return 1;
    }

    // PSR transfer: three encoding variants, all with bits[27:26]==00,
    // bits[24:23]==10, bit[20]==0. Bit 22 (P) picks CPSR vs SPSR.
    // Bit 21 = 0 → MRS; bit 21 = 1 → MSR. Bit 25 picks register vs
    // immediate form for MSR.
    //
    //   MRS:          xxxx 00010 P 00 1111 Rd   000000000000
    //   MSR reg form: xxxx 00010 P 10 mask 1111 00000000 Rm
    //   MSR imm form: xxxx 00110 P 10 mask 1111 rot imm8
    //
    // Masks: 0x0FB00000 matches bits[27:20] with bit 22 left free for P.
    //   MRS          → 0x01000000
    //   MSR reg      → 0x01200000
    //   MSR imm      → 0x03200000
    if ((instr & 0x0FB00000u) == 0x01000000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);  // MRS
    }
    if ((instr & 0x0FB00000u) == 0x01200000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);  // MSR reg form
    }
    if ((instr & 0x0FB00000u) == 0x03200000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);  // MSR imm form
    }

    // Multiply family: bits[27:24] == 0000, bits[7:4] == 1001. Bit 23
    // distinguishes short (MUL/MLA) vs long (UMULL/UMLAL/SMULL/SMLAL);
    // bit 22 is signed/unsigned for the long form. Peeled off before
    // any DP operand decoding so the reg-shift form (bit4==1 with
    // bit7==0) doesn't false-match. The halfword/swap encodings at
    // bits[7:4] ∈ {1011,1101,1111} naturally don't collide with 1001.
    if ((instr & 0x0F0000F0u) == 0x00000090u) {
        return dispatch_multiply(state, instr, instr_addr);
    }

    // Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;

    ShifterResult op2;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
    } else if (reg_shift) {
        const u32  rm         = instr & 0xFu;
        const u32  rs         = (instr >> 8) & 0xFu;
        const u32  shift_type = (instr >> 5) & 0x3u;
        const u32  amount     = read_rs_for_reg_shift(state, rs) & 0xFFu;
        const u32  rm_value   = read_rm_pc12(state, rm);
        // barrel_shift_reg returns meaningless carry when amount == 0; the caller
        // is responsible for preserving CPSR.C in that case.
        op2 = barrel_shift_reg(rm_value, static_cast<ShiftType>(shift_type), amount);
        if (amount == 0) {
            op2.carry = (state.cpsr & (1u << 29)) != 0;
        }
    } else {
        // Immediate-shifted register operand2. Rm==15 naturally reads
        // state.r[15] == instr_addr + 8.
        const u32  rm         = instr & 0xFu;
        const u32  shift_type = (instr >> 5) & 0x3u;
        const u32  shift_amt  = (instr >> 7) & 0x1Fu;
        const bool c_in       = (state.cpsr & (1u << 29)) != 0;
        op2 = barrel_shift_imm(
            state.r[rm],
            static_cast<ShiftType>(shift_type),
            shift_amt,
            c_in);
    }

    const u32  opcode = (instr >> 21) & 0xFu;
    const bool s_flag = ((instr >> 20) & 1u) != 0;
    const u32  rn     = (instr >> 16) & 0xFu;
    const u32  rd     = (instr >> 12) & 0xFu;

    // In reg-shift form, Rn == 15 also reads PC + 12 (same pipeline-stage
    // offset as Rm). In imm-shift and imm-form it reads PC + 8 as usual.
    const u32 rn_val = reg_shift ? read_rm_pc12(state, rn) : state.r[rn];

    u32  result       = 0;
    bool logical_form = true;
    bool writeback    = true;
    AddResult ar { 0, false, false };

    switch (static_cast<DpOp>(opcode)) {
        case DpOp::AND:
            result = rn_val & op2.value;
            break;
        case DpOp::EOR:
            result = rn_val ^ op2.value;
            break;
        case DpOp::SUB:
            ar = sbc(rn_val, op2.value, true);
            result = ar.value;
            logical_form = false;
            break;
        case DpOp::RSB:
            ar = sbc(op2.value, rn_val, true);
            result = ar.value;
            logical_form = false;
            break;
        case DpOp::ADD:
            ar = adc(rn_val, op2.value, false);
            result = ar.value;
            logical_form = false;
            break;
        case DpOp::ADC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            ar = adc(rn_val, op2.value, c);
            result = ar.value;
            logical_form = false;
            break;
        }
        case DpOp::SBC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            ar = sbc(rn_val, op2.value, c);
            result = ar.value;
            logical_form = false;
            break;
        }
        case DpOp::RSC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            ar = sbc(op2.value, rn_val, c);
            result = ar.value;
            logical_form = false;
            break;
        }
        case DpOp::ORR:
            result = rn_val | op2.value;
            break;
        case DpOp::MOV:
            result = op2.value;
            break;
        case DpOp::BIC:
            result = rn_val & ~op2.value;
            break;
        case DpOp::MVN:
            result = ~op2.value;
            break;
        case DpOp::TST:
            result = rn_val & op2.value;
            writeback = false;
            break;
        case DpOp::TEQ:
            result = rn_val ^ op2.value;
            writeback = false;
            break;
        case DpOp::CMP:
            ar = sbc(rn_val, op2.value, true);
            result = ar.value;
            logical_form = false;
            writeback = false;
            break;
        case DpOp::CMN:
            ar = adc(rn_val, op2.value, false);
            result = ar.value;
            logical_form = false;
            writeback = false;
            break;
    }

    if (writeback) {
        write_rd(state, rd, result);
    }

    if (s_flag) {
        if (rd == 15 && writeback) {
            DS_LOG_WARN("arm7: S-flag set with Rd=R15 at 0x%08X", instr_addr);
        } else {
            state.cpsr = set_nz(state.cpsr, result);
            if (logical_form) {
                state.cpsr = set_c(state.cpsr, op2.carry);
            } else {
                state.cpsr = set_c(state.cpsr, ar.carry);
                state.cpsr = set_v(state.cpsr, ar.overflow);
            }
        }
    }

    return 1;
}

}  // namespace ds
