// arm7_thumb_ls.cpp — Thumb load/store format handlers.
//
// THUMB.6: LDR PC-rel (literal pool). Word-aligned address from
//   pc_literal + (imm8 << 2). No rotate-on-misalign since the address
//   is always word-aligned.
//
// THUMB.11: LDR/STR SP-rel. Address = SP + (imm8 << 2). Rotate-on-
//   misalign applies if SP is misaligned (games shouldn't do this but
//   the CPU doesn't enforce alignment).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_thumb_internal.hpp"

namespace ds {

// THUMB.6: 01001 Rd3 imm8
// LDR Rd, [PC, #imm8*4] — word load from literal pool.
// PC value is pc_literal = (instr_addr + 4) & ~2 (force-aligned).
u32 dispatch_thumb_ldr_pc(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 /*pc_read*/, u32 pc_literal) {
    const u32 rd = (instr >> 8) & 0x7u;
    const u32 imm8 = instr & 0xFFu;
    const u32 addr = pc_literal + (imm8 << 2);
    // Address is always word-aligned (pc_literal is word-aligned, offset is *4).
    // Use execute_single_data_transfer_core: is_load=true, is_byte=false.
    return execute_single_data_transfer_core(state, bus, addr, rd, true, false, instr_addr);
}

// THUMB.11: 1001 op1 Rd3 imm8
// op=0: STR Rd, [SP, #imm8*4]
// op=1: LDR Rd, [SP, #imm8*4]
// Rotate-on-misalign applies for LDR if SP is misaligned.
u32 dispatch_thumb_ldst_sp(Arm7State& state,
                           Arm7Bus& bus,
                           u16 instr,
                           u32 instr_addr,
                           u32 /*pc_read*/,
                           u32 /*pc_literal*/) {
    const bool is_load = ((instr >> 11) & 1u) != 0;
    const u32 rd = (instr >> 8) & 0x7u;
    const u32 imm8 = instr & 0xFFu;
    const u32 addr = state.r[13] + (imm8 << 2);
    return execute_single_data_transfer_core(state, bus, addr, rd, is_load, false, instr_addr);
}

} // namespace ds
