// arm7_decode.cpp implements Arm7::step_arm() and the ARM instruction
// decoder dispatch. In slice 3a only data-processing (bits 27..26 == 00)
// is supported; everything else logs a warning and no-ops.
//
// Cycle cost model: every executed (non-skipped) instruction is 1 ARM7
// cycle. Skipped (condition-failed) instructions also cost 1 cycle.
// This is deliberately simplified — slice 3b revisits cycle counts when
// we model memory access and branch pipeline flush.
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

namespace ds {

namespace {

// Data-processing opcode numbers (bits 24..21 of the instruction).
enum class DpOp : u8 {
    AND = 0x0, EOR = 0x1, SUB = 0x2, RSB = 0x3,
    ADD = 0x4, ADC = 0x5, SBC = 0x6, RSC = 0x7,
    TST = 0x8, TEQ = 0x9, CMP = 0xA, CMN = 0xB,
    ORR = 0xC, MOV = 0xD, BIC = 0xE, MVN = 0xF,
};

// Slice 3a: only MOV-immediate is fully implemented here. Task 5 adds
// ADD/SUB, Task 6 adds the remaining opcodes and the S flag, Task 7 adds
// shifted-register operand2.
void exec_dp_mov(Arm7State& s, u32 rd, u32 operand2) {
    s.r[rd] = operand2;
    if (rd == 15) {
        s.pc = operand2 & ~0x3u;
    }
}

}  // namespace

void Arm7::step_arm() {
    const u32 instr_addr = state_.pc;
    const u32 instr      = bus_->read32(instr_addr);

    // R15 during execute reads as instruction_addr + 8 (3-stage pipeline).
    // Advance pc BEFORE dispatch so branch handlers can stomp it freely
    // without us overwriting their target afterwards.
    state_.r[15] = instr_addr + 8;
    state_.pc    = instr_addr + 4;

    const u32 cond = instr >> 28;
    if (!eval_condition(cond, state_.cpsr)) {
        state_.cycles += 1;
        return;
    }

    const u32 bits_27_26 = (instr >> 26) & 0x3u;
    if (bits_27_26 != 0) {
        // Non-data-processing form — branches, loads/stores, coproc, SWI.
        // Deferred to slices 3b/3d. Warn and no-op.
        DS_LOG_WARN("arm7: unimplemented ARM form 0x%08X at 0x%08X", instr, instr_addr);
        state_.cycles += 1;
        return;
    }

    // Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;
    if (reg_shift) {
        // Register-shifted-register operand2 — deferred to slice 3b.
        // Also catches MUL/MLA and halfword LDR/STR, which all live in
        // this encoding space and are unimplemented in slice 3a.
        DS_LOG_WARN("arm7: register-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        state_.cycles += 1;
        return;
    }

    // Decode fields common to all data-processing forms.
    const u32 opcode = (instr >> 21) & 0xFu;
    const u32 s_flag = (instr >> 20) & 1u;
    const u32 rn     = (instr >> 16) & 0xFu;
    const u32 rd     = (instr >> 12) & 0xFu;
    (void)rn;
    (void)s_flag;  // Task 6 wires this in.

    // Compute operand2.
    ShifterResult op2;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state_.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
    } else {
        // Immediate-shift form. Task 7 wires this in. Slice 3a only
        // supports rotated-immediate operand2 right now.
        DS_LOG_WARN("arm7: immediate-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        state_.cycles += 1;
        return;
    }

    switch (static_cast<DpOp>(opcode)) {
        case DpOp::MOV:
            exec_dp_mov(state_, rd, op2.value);
            break;
        default:
            DS_LOG_WARN("arm7: unimplemented dp opcode 0x%X at 0x%08X",
                        opcode, instr_addr);
            break;
    }

    state_.cycles += 1;
}

}  // namespace ds
