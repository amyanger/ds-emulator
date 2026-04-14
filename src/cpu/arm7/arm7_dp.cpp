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
    // Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;
    if (reg_shift) {
        // Register-shifted-register operand2 — deferred to slice 3b2.
        // Also catches MUL/MLA and halfword LDR/STR, which all live in
        // this encoding space and are unimplemented in slice 3b1.
        DS_LOG_WARN("arm7: register-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        return 1;
    }

    // Compute operand2 first so we can early-out on the unimplemented
    // immediate-shift form before paying for a register-file lookup.
    ShifterResult op2;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
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

    const u32 rn_val = state.r[rn];

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
