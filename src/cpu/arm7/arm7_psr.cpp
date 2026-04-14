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
// path; declared here so Task 9 and Task 13 can share it.
bool is_valid_mode(Mode m) {
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

    // SPSR write: route through spsr_slot() which returns the current
    // mode's banked SPSR slot, or nullptr for User/System (which share
    // the user bank and have no SPSR storage — writing SPSR there is
    // UNPREDICTABLE on real hardware).
    if (use_spsr) {
        u32* slot = state.spsr_slot();
        if (slot == nullptr) {
            DS_LOG_WARN("arm7: MSR SPSR in User/System mode (unpredictable) at 0x%08X",
                        state.pc);
            return 1;
        }
        *slot = (*slot & ~byte_mask) | (source & byte_mask);
        return 1;
    }

    // --- MSR CPSR ---
    // Two ordering subtleties:
    //   1. The T-bit change warn fires on any byte-mask pattern that
    //      writes bit 5 to a different value. Games should flip T via
    //      BX, not MSR; warn but honor the write.
    //   2. If the control byte is being written and the new mode differs
    //      from the current mode, we call switch_mode() *first* to save/
    //      load register banks, then overwrite CPSR with the final merged
    //      value (which has the correct mode bits by construction).
    const u32 new_cpsr = (state.cpsr & ~byte_mask) | (source & byte_mask);

    if ((byte_mask & 0x000000FFu) != 0) {
        const u32 new_t = (new_cpsr >> 5) & 1u;
        const u32 cur_t = (state.cpsr >> 5) & 1u;
        if (new_t != cur_t) {
            DS_LOG_WARN("arm7: MSR CPSR T-bit change (use BX) at 0x%08X", state.pc);
        }

        const Mode new_mode = static_cast<Mode>(new_cpsr & 0x1Fu);
        const Mode cur_mode = state.current_mode();
        if (new_mode != cur_mode) {
            if (!is_valid_mode(new_mode)) {
                DS_LOG_WARN("arm7: MSR CPSR illegal mode 0x%02X (unpredictable) at 0x%08X",
                            static_cast<u32>(new_mode), state.pc);
                // Skip the bank swap but still honor the byte write below,
                // so the mode bits reflect the game's request. Debug
                // visibility over strict fidelity.
            } else {
                state.switch_mode(new_mode);
            }
        }
    }

    state.cpsr = new_cpsr;
    return 1;
}

}  // namespace ds
