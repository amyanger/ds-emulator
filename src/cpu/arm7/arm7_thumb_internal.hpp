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

// THUMB.9 LDR/STR/LDRB/STRB with immediate offset.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_imm(
    Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr, u32 pc_read, u32 pc_literal);

// THUMB.11 LDR/STR SP-relative.
// Defined in arm7_thumb_ls.cpp.
u32 dispatch_thumb_ldst_sp(
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

} // namespace ds
