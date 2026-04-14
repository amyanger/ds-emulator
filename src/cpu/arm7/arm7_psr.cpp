// arm7_psr.cpp — ARMv4T PSR transfer (MRS and MSR).
// Handles the four encoding forms:
//   MRS Rd, CPSR    bits[27:23]==00010, bit[22]=0, bits[21:20]==00
//   MRS Rd, SPSR    bits[27:23]==00010, bit[22]=1, bits[21:20]==00
//   MSR PSR, Rm     bits[27:23]==00010, bits[21:20]==10, register form
//   MSR PSR, #imm   bits[27:23]==00110, bits[21:20]==10, immediate form
// Byte mask lives at bits[19:16] and chooses which of the four CPSR
// byte-fields (f, s, x, c) get written.
//
// Slice 3b2 Task 9 implements the MRS half; MSR lands across Tasks
// 10..13.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "ds/common.hpp"

namespace ds {

namespace {

// Valid ARMv4T mode-bit patterns. Used by Task 13's MSR CPSR mode-change
// path; declared here so Task 9 and Task 13 can share it. Marked
// [[maybe_unused]] for the duration of Task 9 since MRS doesn't call it.
[[maybe_unused]] bool is_valid_mode(Mode m) {
    switch (m) {
        case Mode::User: case Mode::Fiq: case Mode::Irq:
        case Mode::Supervisor: case Mode::Abort:
        case Mode::Undefined: case Mode::System:
            return true;
    }
    return false;
}

}  // namespace

u32 dispatch_psr_transfer(Arm7State& state, u32 instr, u32 /*instr_addr*/) {
    const bool is_msr   = ((instr >> 21) & 1u) != 0;
    const bool use_spsr = ((instr >> 22) & 1u) != 0;

    // --- MRS path ---
    if (!is_msr) {
        const u32 rd = (instr >> 12) & 0xFu;
        u32 value;
        if (use_spsr) {
            const u32* slot = state.spsr_slot();
            if (slot == nullptr) {
                DS_LOG_WARN("arm7: MRS SPSR in User/System mode (unpredictable) at 0x%08X",
                            state.pc);
                value = 0;
            } else {
                value = *slot;
            }
        } else {
            value = state.cpsr;
        }
        write_rd(state, rd, value);
        return 1;
    }

    // --- MSR path: compute source value ---
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    u32 source;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state.cpsr & (1u << 29)) != 0;
        source = rotated_imm(imm8, rotate, c_in).value;
    } else {
        source = state.r[instr & 0xFu];
    }

    // Build byte mask from bits[19:16] (the "_fsxc" field).
    u32 byte_mask = 0;
    if (instr & (1u << 19)) byte_mask |= 0xFF000000u;  // f (flags byte)
    if (instr & (1u << 18)) byte_mask |= 0x00FF0000u;  // s (reserved on v4T)
    if (instr & (1u << 17)) byte_mask |= 0x0000FF00u;  // x (reserved on v4T)
    if (instr & (1u << 16)) byte_mask |= 0x000000FFu;  // c (control byte)

    // SPSR writes land in Task 12 — for now, warn and no-op.
    if (use_spsr) {
        DS_LOG_WARN("arm7: MSR SPSR not yet implemented, instr 0x%08X at 0x%08X",
                    instr, state.pc);
        return 1;
    }

    // CPSR merge: write only the masked bytes from `source`, leave the
    // rest unchanged. Mode-bit change handling (and the switch_mode() call)
    // lands in Task 13. For Task 10, we do the raw merge; the Task 10
    // tests deliberately stay in a single mode so the merge matches what
    // Task 13 will produce.
    state.cpsr = (state.cpsr & ~byte_mask) | (source & byte_mask);
    return 1;
}

}  // namespace ds
