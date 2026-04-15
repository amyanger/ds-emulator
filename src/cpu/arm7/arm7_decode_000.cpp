// arm7_decode_000.cpp — recognizer fan-out for the bits[27:25] == 000
// encoding slot. Owns BX, multiply, halfword/signed transfer, and the
// MRS/MSR-reg PSR forms. Falls through to dispatch_dp() for plain
// register-form data-processing instructions.

#include "cpu/arm7/arm7_decode_internal.hpp"

namespace ds {

u32 dispatch_000_space(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    if ((instr & 0x0FFF'FFF0u) == 0x012F'FF10u) {
        const u32 rm = instr & 0xFu;
        const u32 rm_val = state.r[rm];
        const bool thumb = (rm_val & 0x1u) != 0;
        const u32 target = thumb ? (rm_val & ~0x1u) : (rm_val & ~0x3u);
        if (thumb) {
            state.cpsr |= (1u << 5);
        }
        write_rd(state, 15, target);
        return 1;
    }

    // SWP / SWPB (spec §5.3). Must precede the multiply and halfword
    // recognizers. Mask 0x0FB00FF0 pins bits[27:23]=00010, bits[21:20]=00,
    // and bits[11:4]=00001001. Bit 22 (B) is left free.
    if ((instr & 0x0FB00FF0u) == 0x01000090u) {
        return dispatch_swap(state, bus, instr, instr_addr);
    }

    if ((instr & 0x0F0000F0u) == 0x00000090u) {
        return dispatch_multiply(state, instr, instr_addr);
    }

    if ((instr & (1u << 4)) != 0 && (instr & (1u << 7)) != 0) {
        return dispatch_halfword(state, bus, instr, instr_addr);
    }

    // MRS and MSR-reg pin SBO/SBZ fields that halfword transfers do NOT, so
    // each recognizer must match the exact SBO/SBZ shape — bits[27:20] alone
    // would false-match STRH with negative offset.
    //   MRS     → bits[19:16]=1111, bits[11:0]=0    (0x0FBF0FFFu)
    //   MSR reg → bits[15:12]=1111, bits[11:4]=0    (0x0FB0FFF0u)
    if ((instr & 0x0FBF0FFFu) == 0x010F0000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);
    }
    if ((instr & 0x0FB0FFF0u) == 0x0120F000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);
    }

    return dispatch_dp(state, instr, instr_addr);
}

} // namespace ds
