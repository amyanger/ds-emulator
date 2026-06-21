// arm9_dp.cpp — ARM9 data-processing operand logic. Handles the imm-shift
// and reg-shift operand forms (bits[27:25]==000 fall-through from
// dispatch_000_space) plus the imm-form (bits[27:25]==001 routed directly
// from arm9_decode.cpp). The family-sibling recognizers (BX, multiply,
// halfword, PSR transfer) that share the bits[27:25]==000 primary slot live
// in arm9_decode_000.cpp.
//
// The data-processing family is bit-identical between ARMv4T and ARMv5TE:
// the barrel-shifter carry edge cases, the SPSR exception-return on `S` with
// Rd==15, and the flag rules are unchanged (slice 3l design §5.7). This is a
// faithful port of arm7_dp.cpp onto Arm9State; the v5 divergences all live in
// other families (LDR/LDM PC interworking, multiply C-preservation, the live
// Q flag in the PSR path). Only the shared pure ALU helpers are reused —
// everything else is ARM9-owned by design (§4.1).

#include "cpu/arm9/arm9_decode_internal.hpp"
#include "cpu/common/arm_alu.hpp"

namespace ds {

u32 execute_dp_op(Arm9State& state,
                  DpOp op,
                  u32 rn_value,
                  u32 operand2,
                  bool shifter_carry,
                  bool s_flag,
                  u32 rd) {
    u32 result = 0;
    bool logical_form = true;
    bool writeback = true;
    AddResult ar{0, false, false};

    switch (op) {
    case DpOp::AND:
        result = rn_value & operand2;
        break;
    case DpOp::EOR:
        result = rn_value ^ operand2;
        break;
    case DpOp::SUB:
        ar = sbc(rn_value, operand2, true);
        result = ar.value;
        logical_form = false;
        break;
    case DpOp::RSB:
        ar = sbc(operand2, rn_value, true);
        result = ar.value;
        logical_form = false;
        break;
    case DpOp::ADD:
        ar = adc(rn_value, operand2, false);
        result = ar.value;
        logical_form = false;
        break;
    case DpOp::ADC: {
        const bool c = (state.cpsr & (1u << 29)) != 0;
        ar = adc(rn_value, operand2, c);
        result = ar.value;
        logical_form = false;
        break;
    }
    case DpOp::SBC: {
        const bool c = (state.cpsr & (1u << 29)) != 0;
        ar = sbc(rn_value, operand2, c);
        result = ar.value;
        logical_form = false;
        break;
    }
    case DpOp::RSC: {
        const bool c = (state.cpsr & (1u << 29)) != 0;
        ar = sbc(operand2, rn_value, c);
        result = ar.value;
        logical_form = false;
        break;
    }
    case DpOp::ORR:
        result = rn_value | operand2;
        break;
    case DpOp::MOV:
        result = operand2;
        break;
    case DpOp::BIC:
        result = rn_value & ~operand2;
        break;
    case DpOp::MVN:
        result = ~operand2;
        break;
    case DpOp::TST:
        result = rn_value & operand2;
        writeback = false;
        break;
    case DpOp::TEQ:
        result = rn_value ^ operand2;
        writeback = false;
        break;
    case DpOp::CMP:
        ar = sbc(rn_value, operand2, true);
        result = ar.value;
        logical_form = false;
        writeback = false;
        break;
    case DpOp::CMN:
        ar = adc(rn_value, operand2, false);
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
            // Exception-return form (MOVS/SUBS PC, ...): the current mode's
            // SPSR copies into CPSR. Mode bits change, so R13/R14 must re-bank
            // BEFORE we finalize CPSR. MOVS PC, R14 returns from SWI/UNDEF;
            // SUBS PC, R14, #4 returns from IRQ/FIQ/PrefetchAbort. User /
            // System have no SPSR — behavior is UNPREDICTABLE, so we leave
            // CPSR untouched (matches melonDS).
            //
            // PC alignment depends on the *new* T bit (from SPSR), so we
            // re-align state.pc here after the CPSR copy. In the User/System
            // fallback we keep write_rd's ARM-aligned value, which is correct
            // because no mode change happens and we remain in ARM state.
            u32* spsr = state.spsr_slot();
            if (spsr != nullptr) {
                const u32 new_cpsr = *spsr;
                const Mode new_mode = static_cast<Mode>(new_cpsr & 0x1Fu);
                state.switch_mode(new_mode);
                state.cpsr = new_cpsr;
                const bool thumb = (new_cpsr & (1u << 5)) != 0;
                state.pc = thumb ? (result & ~0x1u) : (result & ~0x3u);
            }
        } else {
            state.cpsr = set_nz(state.cpsr, result);
            if (logical_form) {
                state.cpsr = set_c(state.cpsr, shifter_carry);
            } else {
                state.cpsr = set_c(state.cpsr, ar.carry);
                state.cpsr = set_v(state.cpsr, ar.overflow);
            }
        }
    }

    return 1;
}

u32 dispatch_dp(Arm9State& state, u32 instr, u32 instr_addr) {
    // MSR imm-form lives in the bits[27:25]==001 slot and is routed directly
    // here from arm9_decode.cpp. The 000-slot PSR forms (MRS, MSR reg) are
    // handled by dispatch_000_space.
    if ((instr & 0x0FB0F000u) == 0x0320F000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);
    }

    // Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;

    ShifterResult op2;
    if (i_bit) {
        const u32 imm8 = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in = (state.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
    } else if (reg_shift) {
        const u32 rm = instr & 0xFu;
        const u32 rs = (instr >> 8) & 0xFu;
        const u32 shift_type = (instr >> 5) & 0x3u;
        const u32 amount = read_rs_for_reg_shift(state, rs) & 0xFFu;
        const u32 rm_value = read_rm_pc12(state, rm);
        // barrel_shift_reg returns meaningless carry when amount == 0; the
        // caller is responsible for preserving CPSR.C in that case.
        op2 = barrel_shift_reg(rm_value, static_cast<ShiftType>(shift_type), amount);
        if (amount == 0) {
            op2.carry = (state.cpsr & (1u << 29)) != 0;
        }
    } else {
        // Immediate-shifted register operand2. Rm==15 naturally reads
        // state.r[15] == instr_addr + 8.
        const u32 rm = instr & 0xFu;
        const u32 shift_type = (instr >> 5) & 0x3u;
        const u32 shift_amt = (instr >> 7) & 0x1Fu;
        const bool c_in = (state.cpsr & (1u << 29)) != 0;
        op2 = barrel_shift_imm(state.r[rm], static_cast<ShiftType>(shift_type), shift_amt, c_in);
    }

    const u32 opcode = (instr >> 21) & 0xFu;
    const bool s_flag = ((instr >> 20) & 1u) != 0;
    const u32 rn = (instr >> 16) & 0xFu;
    const u32 rd = (instr >> 12) & 0xFu;

    // In reg-shift form, Rn == 15 also reads PC + 12 (same pipeline-stage
    // offset as Rm). In imm-shift and imm-form it reads PC + 8 as usual.
    const u32 rn_val = reg_shift ? read_rm_pc12(state, rn) : state.r[rn];

    return execute_dp_op(
        state, static_cast<DpOp>(opcode), rn_val, op2.value, op2.carry, s_flag, rd);
}

} // namespace ds
