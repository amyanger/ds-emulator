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

// Task 5 completes the non-compare data-processing executors
// (AND/EOR/SUB/RSB/ADD/ADC/SBC/RSC/ORR/BIC/MOV/MVN). Task 6 adds the
// S-flag path and the four compare-only opcodes (TST/TEQ/CMP/CMN).
// Task 7 wires the immediate-shifted register operand2.
//
// `write_rd` is the single writeback point so a future Task 6 can swap
// in flag-setting without touching every opcode branch.
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
        // Immediate-shift form. Task 7 wires this in. Slice 3a only
        // supports rotated-immediate operand2 right now.
        DS_LOG_WARN("arm7: immediate-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        return 1;
    }

    const u32 opcode = (instr >> 21) & 0xFu;
    const u32 s_flag = (instr >> 20) & 1u;
    const u32 rn     = (instr >> 16) & 0xFu;
    const u32 rd     = (instr >> 12) & 0xFu;
    (void)s_flag;  // Task 6 wires this in.

    const u32 rn_val = state.r[rn];
    switch (static_cast<DpOp>(opcode)) {
        case DpOp::AND: write_rd(state, rd, rn_val & op2.value); break;
        case DpOp::EOR: write_rd(state, rd, rn_val ^ op2.value); break;
        case DpOp::SUB: write_rd(state, rd, rn_val - op2.value); break;
        case DpOp::RSB: write_rd(state, rd, op2.value - rn_val); break;
        case DpOp::ADD: write_rd(state, rd, rn_val + op2.value); break;
        case DpOp::ADC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            write_rd(state, rd, rn_val + op2.value + (c ? 1u : 0u));
            break;
        }
        case DpOp::SBC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            write_rd(state, rd, rn_val - op2.value - (c ? 0u : 1u));
            break;
        }
        case DpOp::RSC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            write_rd(state, rd, op2.value - rn_val - (c ? 0u : 1u));
            break;
        }
        case DpOp::ORR: write_rd(state, rd, rn_val | op2.value); break;
        case DpOp::MOV: write_rd(state, rd, op2.value);          break;
        case DpOp::BIC: write_rd(state, rd, rn_val & ~op2.value); break;
        case DpOp::MVN: write_rd(state, rd, ~op2.value);          break;
        // TST/TEQ/CMP/CMN land in Task 6 with S-flag handling; they
        // only exist in their flag-setting form so there is nothing to
        // do here until the flag path is in place.
        case DpOp::TST:
        case DpOp::TEQ:
        case DpOp::CMP:
        case DpOp::CMN:
            DS_LOG_WARN("arm7: TST/TEQ/CMP/CMN at 0x%08X - Task 6",
                        instr_addr);
            break;
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
