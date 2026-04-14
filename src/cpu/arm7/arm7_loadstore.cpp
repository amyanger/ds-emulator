// arm7_loadstore.cpp — ARMv4T single-data-transfer dispatch.
// Slice 3b1 handles LDR/STR/LDRB/STRB in all standard addressing modes:
// immediate and immediate-shifted register offsets, pre/post index,
// up/down, writeback. Halfword and signed forms stay deferred to 3b2.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_single_data_transfer(Arm7State& state, Arm7Bus& bus,
                                  u32 instr, u32 instr_addr) {
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool p_bit = ((instr >> 24) & 1u) != 0;
    const bool u_bit = ((instr >> 23) & 1u) != 0;
    const bool b_bit = ((instr >> 22) & 1u) != 0;
    const bool w_bit = ((instr >> 21) & 1u) != 0;
    const bool l_bit = ((instr >> 20) & 1u) != 0;
    const u32  rn    = (instr >> 16) & 0xFu;
    const u32  rd    = (instr >> 12) & 0xFu;

    // Offset.
    u32 offset;
    if (i_bit) {
        // Register offset, always immediate-shifted for LDR/STR.
        const u32  rm         = instr & 0xFu;
        const u32  shift_type = (instr >> 5) & 0x3u;
        const u32  shift_amt  = (instr >> 7) & 0x1Fu;
        const bool c_in       = (state.cpsr & (1u << 29)) != 0;
        // Shifter carry output is discarded for LDR/STR.
        offset = barrel_shift_imm(
            state.r[rm],
            static_cast<ShiftType>(shift_type),
            shift_amt,
            c_in).value;
    } else {
        offset = instr & 0xFFFu;
    }

    // Addresses.
    const u32 base        = state.r[rn];          // rn==15 reads instr_addr+8
    const u32 signed_off  = u_bit ? offset : (0u - offset);
    const u32 pre_addr    = base + signed_off;    // 32-bit wrap
    const u32 access_addr = p_bit ? pre_addr : base;
    const u32 wb_addr     = pre_addr;             // same whether P=1 or P=0

    // Access.
    u32 loaded = 0;
    if (l_bit) {
        if (b_bit) {
            loaded = bus.read8(access_addr);       // zero-extended
        } else {
            const u32 raw = bus.read32(access_addr & ~0x3u);
            const u32 rot = (access_addr & 0x3u) * 8u;
            loaded = (rot == 0) ? raw : ((raw >> rot) | (raw << (32u - rot)));
        }
    } else {
        const u32 stored = (rd == 15) ? (instr_addr + 12u) : state.r[rd];
        if (b_bit) {
            bus.write8(access_addr, static_cast<u8>(stored & 0xFFu));
        } else {
            bus.write32(access_addr & ~0x3u, stored);
        }
    }

    // Writeback: post-index always writes back; pre-index writes back
    // only if W=1. On LDR, the loaded value wins over writeback when
    // Rn == Rd (ARMv4T defines this as the rule for that "unpredictable"
    // case).
    const bool do_wb = (!p_bit) || w_bit;
    if (do_wb && !(l_bit && rn == rd)) {
        write_rd(state, rn, wb_addr);
    }
    if (l_bit) {
        write_rd(state, rd, loaded);
    }

    return 1;
}

}  // namespace ds
