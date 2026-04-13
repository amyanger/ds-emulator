// arm7_decode.cpp implements Arm7::step_arm() and the ARM instruction
// decoder dispatch. In slice 3a only data-processing (bits 27..26 == 00)
// is supported; everything else logs a warning and no-ops.
//
// Cycle cost model: every executed (non-skipped) instruction is 1 ARM7
// cycle. Skipped (condition-failed) instructions also cost 1 cycle.
// This is deliberately simplified — slice 3b revisits cycle counts when
// we model memory access and branch pipeline flush. Per-instruction
// cycle counts are returned from `dispatch_arm` so slice 3b can refine
// them without touching `step_arm`'s control flow.
//
// NOTE: in ARMv4T, the "bit 4 == 1 with I == 0" encoding space inside
// data-processing actually overlaps with MUL/MLA, MRS/MSR, and halfword
// loads/stores. Slice 3a lumps all of those into the same "unimplemented"
// bucket because they are all deferred to slice 3b. When 3b lands, this
// decoder's dispatch tree has to be refined to break them apart before
// implementing register-shifted data-processing for real.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_alu.hpp"

#include <cassert>

namespace ds {

namespace {

// Data-processing opcode numbers (bits 24..21 of the instruction).
enum class DpOp : u8 {
    AND = 0x0, EOR = 0x1, SUB = 0x2, RSB = 0x3,
    ADD = 0x4, ADC = 0x5, SBC = 0x6, RSC = 0x7,
    TST = 0x8, TEQ = 0x9, CMP = 0xA, CMN = 0xB,
    ORR = 0xC, MOV = 0xD, BIC = 0xE, MVN = 0xF,
};

// Single register-file writeback point. The post-switch block in
// dispatch_arm calls this when writeback == true; flag-only opcodes
// (TST/TEQ/CMP/CMN) skip it. R15 writes are masked to ARM alignment
// and also stomp state.pc so the next fetch lands on the target.
void write_rd(Arm7State& s, u32 rd, u32 value) {
    s.r[rd] = value;
    if (rd == 15) {
        s.pc = value & ~0x3u;
    }
}

// Returns the number of ARM7 cycles this instruction consumed. Slice 3a
// returns 1 from every path; slice 3b refines with per-instruction costs.
// Single-exit cycle accounting in `step_arm` means a forgotten increment
// in a future opcode case becomes an assert hit instead of an infinite
// loop in `run_until`.
u32 dispatch_arm(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    (void)bus;  // slice 3b's load/store dispatch will use this.

    const u32 cond = instr >> 28;
    if (!eval_condition(cond, state.cpsr)) {
        return 1;  // condition-skipped instructions still consume 1 cycle
    }

    const u32 bits_27_26 = (instr >> 26) & 0x3u;
    if (bits_27_26 != 0) {
        // Non-data-processing form — branches, loads/stores, coproc, SWI.
        // Deferred to slices 3b/3d.
        DS_LOG_WARN("arm7: unimplemented ARM form 0x%08X at 0x%08X", instr, instr_addr);
        return 1;
    }

    // Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;
    if (reg_shift) {
        // Register-shifted-register operand2 — deferred to slice 3b. Also
        // catches MUL/MLA and halfword LDR/STR, which all live in this
        // encoding space and are unimplemented in slice 3a.
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
        // state.r[15] == instr_addr + 8 (Task 4's pipeline model).
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

    // Per-opcode products consumed by the flag-update block below.
    u32  result       = 0;
    bool logical_form = true;   // true → C from shifter (op2.carry), V unchanged
    bool writeback    = true;   // false for TST/TEQ/CMP/CMN
    AddResult ar { 0, false, false };  // only meaningful when !logical_form

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
        // Compare-only opcodes: compute result, update flags, no writeback.
        // Real ARMv4T only encodes these with S=1; S=0 here would be MRS/MSR
        // which is deferred to slice 3b. Slice 3a unconditionally treats
        // them as flag-setting.
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
            // Slice 3a stub: PC was already written by write_rd() above.
            // NZCV are left entirely untouched (whatever the previous op
            // set stays set). Slice 3d's real path copies SPSR->CPSR,
            // which is what a game-visible exception return would see.
            // Until then, a test relying on post-MOVS-PC flag state is
            // an emulator bug.
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

}  // namespace

void Arm7::step_arm() {
    assert((state_.pc & 0x3u) == 0 && "Arm7::step_arm: ARM pc must be 4-aligned");

    const u32 instr_addr = state_.pc;
    const u32 instr      = bus_->read32(instr_addr);

    // R15 during execute reads as instruction_addr + 8 (3-stage pipeline).
    // Advance pc BEFORE dispatch so branch handlers can stomp it freely
    // without us overwriting their target afterwards.
    state_.r[15] = instr_addr + 8;
    state_.pc    = instr_addr + 4;

    const u32 cycles_consumed = dispatch_arm(state_, *bus_, instr, instr_addr);
    assert(cycles_consumed > 0 && "Arm7::step_arm: dispatch must consume >= 1 cycle");
    state_.cycles += cycles_consumed;
}

}  // namespace ds
