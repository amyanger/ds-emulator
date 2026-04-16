#pragma once

// arm7_thumb_internal.hpp — private declarations shared between the
// ARM7 Thumb decoder translation units.

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// Top-level Thumb instruction fan-out. Dispatches on bits[15:13] of
// `instr` into one of eight bucket handlers. Returns ARM7 cycles
// consumed (1 per instruction in slice 3c — coarse cost model).
u32 dispatch_thumb(Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr);

// --- Per-family Thumb dispatchers (called from dispatch_thumb) ----
// Each receives two pre-materialized PC values:
//   pc_read    = instr_addr + 4  (R15 during Thumb execute)
//   pc_literal = (instr_addr + 4) & ~2  (word-aligned, for literal-pool formats)
// Format handlers that need the literal-pool PC use pc_literal;
// everything else reads state.r[15] directly (== pc_read).

u32 dispatch_thumb_shift_or_addsub(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_imm_dp(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_010_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.4 ALU register/register — 16 ops (AND..MVN).
// Defined in arm7_thumb_dp.cpp.
u32 dispatch_thumb_alu(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.5 hi-register operations (ADD/CMP/MOV) and BX.
// Defined in arm7_thumb_dp.cpp.
u32 dispatch_thumb_hireg_bx(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.6 LDR PC-relative (literal pool load).
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldr_pc(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.7 LDR/STR/LDRB/STRB with register offset.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_reg(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.8 STRH/LDSB/LDRH/LDSH with register offset.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_halfword_reg(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.9 LDR/STR/LDRB/STRB with immediate offset.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_imm(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.10 STRH/LDRH with immediate offset.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_halfword_imm(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.11 LDR/STR SP-relative.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_sp(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.14 PUSH/POP (± LR/PC). Delegate to execute_block_transfer.
// Defined in arm7_thumb_block.cpp.
u32 dispatch_thumb_push(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_pop(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.15 LDMIA/STMIA (low-reg multi-load/store with auto-increment).
// Defined in arm7_thumb_block.cpp.
u32 dispatch_thumb_ldmia_stmia(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.12 ADD Rd, PC/SP, #imm8<<2 (literal-pool / SP load-effective-address).
// Defined in arm7_thumb_dp.cpp.
u32 dispatch_thumb_add_pc_sp(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.13 ADD/SUB SP, #imm7<<2 (signed SP adjust).
// Defined in arm7_thumb_dp.cpp.
u32 dispatch_thumb_add_sp_signed(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

u32 dispatch_thumb_ldst_imm_wb(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_100_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_101_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_110_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);
u32 dispatch_thumb_111_space(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.16 Bcond + THUMB.17 SWI warn stub (both live in the 1101xxxx space).
// Defined in arm7_thumb_branch.cpp.
u32 dispatch_thumb_bcond_swi(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.18 B unconditional (11100 imm11).
// Defined in arm7_thumb_branch.cpp.
u32 dispatch_thumb_b_uncond(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.19 BL first halfword (11110 imm11_hi) — sets LR.
// Defined in arm7_thumb_branch.cpp.
u32 dispatch_thumb_bl_prefix(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.19 BL / BLX second halfword. BL (11111) branches via LR; BLX (11101)
// is ARMv5 only and warns on ARMv4. Defined in arm7_thumb_branch.cpp.
u32 dispatch_thumb_bl_suffix(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

} // namespace ds
