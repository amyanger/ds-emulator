// arm7_thumb_block.cpp — Thumb block-transfer format handlers.
//
// THUMB.14 encoding: 1011 op1 10 R Rlist8
//   op=0 PUSH: STMDB SP! {Rlist, LR?}   block-core flags p=1 u=0 s=0 w=1 l=0
//   op=1 POP:  LDMIA SP! {Rlist, PC?}   block-core flags p=0 u=1 s=0 w=1 l=1
//   R=1 adds LR (PUSH) or PC (POP) at the highest address.
//
// ARMv4 POP {PC} stays in Thumb state regardless of the loaded word's
// bit 0: write_rd masks the low two bits and does not touch CPSR.T, so
// no special-casing is needed here. ARMv5 interworking lands in ARM9.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_thumb_internal.hpp"

namespace ds {

u32 dispatch_thumb_push(Arm7State& state,
                        Arm7Bus& bus,
                        u16 instr,
                        u32 instr_addr,
                        u32 /*pc_read*/,
                        u32 /*pc_literal*/) {
    const u32 r_bit = (instr >> 8) & 1u;
    const u32 reg_list = (instr & 0xFFu) | (r_bit ? (1u << 14) : 0u);
    return execute_block_transfer(state,
                                  bus,
                                  static_cast<u32>(instr),
                                  /*rn=*/13,
                                  reg_list,
                                  /*p_bit=*/true,
                                  /*u_bit=*/false,
                                  /*s_bit=*/false,
                                  /*w_bit=*/true,
                                  /*l_bit=*/false,
                                  instr_addr);
}

u32 dispatch_thumb_pop(Arm7State& state,
                       Arm7Bus& bus,
                       u16 instr,
                       u32 instr_addr,
                       u32 /*pc_read*/,
                       u32 /*pc_literal*/) {
    const u32 r_bit = (instr >> 8) & 1u;
    const u32 reg_list = (instr & 0xFFu) | (r_bit ? (1u << 15) : 0u);
    return execute_block_transfer(state,
                                  bus,
                                  static_cast<u32>(instr),
                                  /*rn=*/13,
                                  reg_list,
                                  /*p_bit=*/false,
                                  /*u_bit=*/true,
                                  /*s_bit=*/false,
                                  /*w_bit=*/true,
                                  /*l_bit=*/true,
                                  instr_addr);
}

// THUMB.15 encoding: 1100 op1 Rb3 Rlist8 — LDMIA/STMIA Rb!, {Rlist}
// Low regs only (no synthetic high bit, unlike PUSH/POP). Rb-in-list and
// empty-Rlist semantics are inherited from execute_block_transfer.
u32 dispatch_thumb_ldmia_stmia(Arm7State& state,
                               Arm7Bus& bus,
                               u16 instr,
                               u32 instr_addr,
                               u32 /*pc_read*/,
                               u32 /*pc_literal*/) {
    const u32 l_bit = (instr >> 11) & 1u;
    const u32 rb = (instr >> 8) & 0x7u;
    const u32 reg_list = instr & 0xFFu;
    return execute_block_transfer(state,
                                  bus,
                                  static_cast<u32>(instr),
                                  /*rn=*/rb,
                                  reg_list,
                                  /*p_bit=*/false,
                                  /*u_bit=*/true,
                                  /*s_bit=*/false,
                                  /*w_bit=*/true,
                                  /*l_bit=*/l_bit != 0u,
                                  instr_addr);
}

} // namespace ds
