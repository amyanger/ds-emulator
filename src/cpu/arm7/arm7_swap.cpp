// arm7_swap.cpp
//
// ARMv4T Single Data Swap (SWP / SWPB). Atomic read-then-write to a
// single memory word or byte via a pair of bus accesses. Historically
// the instruction asserts a LOCK signal so no other bus master can
// interleave between the two halves, but the DS ARM7 bus has no
// contending masters during a CPU instruction — DMA and cart DMA
// are scheduled as separate events and never interleaved mid-op.
// Correctness therefore only requires performing read32 then write32
// (or read8/write8) back-to-back, with no scheduler tick between
// them. Matches melonDS.
//
// Dispatched from arm7_decode_000.cpp's bits[27:25] == 000 fan-out,
// tested before the multiply/halfword recognizers per spec §5.3.
// Word SWP reuses the LDR-style `rotate_read_word` helper hoisted
// to arm7_decode_internal.hpp in slice 3b4 commit 2.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_swap(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    // TODO(cycles): SWP is documented as 1S + 2N + 1I on ARM7TDMI.
    // We return 1 as a placeholder until the cycle-accuracy slice.
    const bool byte_op = ((instr >> 22) & 1u) != 0;
    const u32 rn = (instr >> 16) & 0xFu;
    const u32 rd = (instr >> 12) & 0xFu;
    const u32 rm = instr & 0xFu;

    // Spec §5.10 / §4.4 SWP step 2: R15 is UNPREDICTABLE on all three
    // operands. Real code never emits this; we warn deterministically
    // and fall through so the instruction still executes (using the
    // pipeline-offset value for R15, which is `instr_addr + 8`).
    if (rn == 15 || rd == 15 || rm == 15) {
        DS_LOG_WARN("arm7: SWP%s with R15 operand (unpredictable) 0x%08X at 0x%08X",
                    byte_op ? "B" : "",
                    instr,
                    instr_addr);
    }

    // LOAD-BEARING LATCH: capture Rm BEFORE the memory read. This
    // covers two distinct corner cases that a naive inline
    // `state.r[rm]` on the write path would break:
    //   * Rd == Rm : the destination register is overwritten by the
    //     memory read before we get to the write; without the latch
    //     the write would store the loaded value instead of the
    //     original Rm.
    //   * Rm == Rn : once we force-align the base for the write
    //     (word SWP only), it doesn't matter here — but if the loaded
    //     value were written back to r[rm]/r[rn] before the write,
    //     the bus would see the loaded value as the store operand.
    //     melonDS calls this out explicitly and we match the
    //     behavior.
    const u32 rn_addr = state.r[rn];
    const u32 rm_value = state.r[rm];

    if (byte_op) {
        // SWPB — byte swap. No alignment handling, no rotation. The
        // raw address is used for both read and write. The read zero-
        // extends to u32 so the Rd write installs the byte in bits
        // [7:0] and clears the upper 24 bits.
        const u32 loaded = static_cast<u32>(bus.read8(rn_addr));
        bus.write8(rn_addr, static_cast<u8>(rm_value & 0xFFu));
        state.r[rd] = loaded;
    } else {
        // SWP (word) — read force-aligned then LDR-style rotated by
        // (addr & 3) * 8 so the byte at Rn lands in bits [7:0] of
        // Rd (§5.9). The write uses the force-aligned address and
        // does NOT rotate — the full Rm value is stored as-is.
        const u32 raw = bus.read32(rn_addr & ~0x3u);
        const u32 loaded = rotate_read_word(raw, rn_addr);
        bus.write32(rn_addr & ~0x3u, rm_value);
        state.r[rd] = loaded;
    }
    return 1;
}

} // namespace ds
