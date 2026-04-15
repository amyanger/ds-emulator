// arm7_block.cpp
//
// ARMv4T block data transfer instructions (LDM / STM). Dispatched from
// dispatch_arm() for the bits[27:25] == 100 primary slot.
//
// Commit 5 of slice 3b4 lit up the very first real execution path
// through dispatch_block: plain `STM IA` with W=0, S=0, Rn not in list,
// and R15 not in list. Commit 6 generalized that to the mirror LDM IA
// base case, commit 7 added writeback (W=1) for the Rn-not-in-list IA
// path, and commit 8 drops the IA-only restriction so all four
// addressing modes (IA, IB, DA, DB) execute through the same
// forward-walk engine. Commit 9 implements the ARMv4 STM Rn-in-list
// rule (§5.5): when Rn is in the register list, the value stored at
// the i == Rn slot depends on whether Rn is the lowest-numbered set
// bit. If it is, the original Rn is stored (writeback, if requested,
// lands at the end as usual). If it is not, the writeback value is
// applied BEFORE the transfer loop so the loop's read of `state.r[Rn]`
// naturally picks up the new base. Commit 10 adds the mirror LDM
// Rn-in-list rule: on ARMv4, writeback is suppressed entirely when Rn
// is set in the list, because the loaded value lands in state.r[Rn]
// during the transfer loop and wins over wb_value. The §4.3
// normalization in compute_block_addressing means the transfer loop
// itself is mode-agnostic — only the start_addr and wb_value change.
// The remaining constraints (no S-bit, non-empty list, R15 not in
// list) stay in place and are wired up in their dedicated follow-up
// commits.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_block_internal.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

#include <bit>

namespace ds {

namespace arm7_block_detail {

u32 reg_list_count(u32 reg_list) {
    return static_cast<u32>(std::popcount(reg_list & 0xFFFFu));
}

BlockAddressing compute_block_addressing(u32 rn_value, u32 reg_list, bool p_bit, bool u_bit) {
    const u32 n = reg_list_count(reg_list);
    const u32 bytes = 4u * n;

    BlockAddressing out{};
    if (u_bit) {
        // Increasing walk: IA starts at Rn, IB starts at Rn + 4.
        out.start_addr = rn_value + (p_bit ? 4u : 0u);
        out.wb_value = rn_value + bytes;
    } else {
        // Decreasing walk normalized to a forward scan: the lowest
        // address touched is Rn - 4*n for DB, Rn - 4*n + 4 for DA.
        // Writeback value is always Rn - 4*n in decreasing modes.
        out.start_addr = rn_value - bytes + (p_bit ? 0u : 4u);
        out.wb_value = rn_value - bytes;
    }
    return out;
}

} // namespace arm7_block_detail

u32 dispatch_block(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    // TODO(cycles): LDM/STM have variable cycle counts (nS+1N+1I /
    // (n-1)S+2N). Returning 1 as a placeholder until the cycle-accuracy
    // slice.
    const bool p_bit = ((instr >> 24) & 1u) != 0;
    const bool u_bit = ((instr >> 23) & 1u) != 0;
    const bool s_bit = ((instr >> 22) & 1u) != 0;
    const bool w_bit = ((instr >> 21) & 1u) != 0;
    const bool l_bit = ((instr >> 20) & 1u) != 0;
    const u32 rn = (instr >> 16) & 0xFu;
    const u32 reg_list = instr & 0xFFFFu;

    // Supported today: all four addressing modes (any {P, U}), free W,
    // no S-bit, non-empty list, R15 not in list, and both families
    // tolerate Rn in the register list (STM via the §5.5 old/new-base
    // split below, LDM via end-of-instruction writeback suppression).
    // S-bit forms, R15-in-list, and the empty-list quirk still fall
    // through to the warn stub and land in later slice 3b4 commits.
    const bool rn_in_list = (reg_list & (1u << rn)) != 0u;
    const bool r15_in_list = (reg_list & (1u << 15)) != 0u;
    const bool is_supported = !s_bit && reg_list != 0u && !r15_in_list;

    if (!is_supported) {
        DS_LOG_WARN("arm7: LDM/STM path not yet implemented 0x%08X at 0x%08X", instr, instr_addr);
        return 1;
    }

    const auto addressing =
        arm7_block_detail::compute_block_addressing(state.r[rn], reg_list, p_bit, u_bit);

    // STM Rn-in-list ARMv4 rule (§5.5, spec Appendix B). Applied once
    // before the transfer loop: if Rn is set in the list and is NOT the
    // lowest-numbered set bit, preload Rn with the post-writeback value
    // so the `i == rn` iteration stores the NEW base. If Rn is the
    // lowest, leave it alone — the loop will store the original base on
    // the first iteration, and the end-of-instruction writeback (if
    // W=1) lands the new base as usual. The flag `stm_writeback_early`
    // suppresses the duplicate writeback at the end.
    //
    // This logic runs regardless of W. With W=0 and Rn not lowest the
    // STM encoding is UNPREDICTABLE on real hardware; we pick the
    // deterministic interpretation that matches the "store new base"
    // rule rather than introducing a W-gated second path.
    bool stm_writeback_early = false;
    if (!l_bit && rn_in_list) {
        const u32 lower_mask = (1u << rn) - 1u;
        const bool rn_is_lowest = (reg_list & lower_mask) == 0u;
        if (!rn_is_lowest) {
            state.r[rn] = addressing.wb_value;
            stm_writeback_early = true;
        }
    }

    u32 addr = addressing.start_addr;
    if (l_bit) {
        // LDM — load each listed register from [start_addr + 4*k]. Per
        // GBATEK / spec §4.4 LDM step 4 the effective address is
        // force-aligned to a word boundary and the read is NOT rotated
        // (LDM differs from LDR here). The walk is always low-register-
        // to-low-address; §4.3 normalized the decreasing modes to this
        // same forward scan before the loop starts.
        for (u32 i = 0; i < 16; ++i) {
            if ((reg_list & (1u << i)) != 0u) {
                state.r[i] = bus.read32(addr & ~0x3u);
                addr += 4u;
            }
        }
    } else {
        // STM — store each listed register to [start_addr + 4*k],
        // force-aligned per spec §4.4 STM step 5. R15-in-list is gated
        // out above, so no PC+12 special case is needed yet. Decreasing
        // modes share this loop because §4.3 already produced the
        // lowest-touched address in start_addr.
        for (u32 i = 0; i < 16; ++i) {
            if ((reg_list & (1u << i)) != 0u) {
                bus.write32(addr & ~0x3u, state.r[i]);
                addr += 4u;
            }
        }
    }

    // Writeback (spec §4.4 LDM/STM step 7). Two suppression paths:
    //   1. STM Rn-in-list early path already applied wb_value before
    //      the loop (§5.5, "Rn in list and NOT lowest") — skip so we
    //      don't write it twice.
    //   2. LDM Rn-in-list on ARMv4 (§5.5): the loaded value wins, so
    //      writeback is suppressed entirely. The transfer loop has
    //      already written the loaded word into state.r[rn]; running
    //      the writeback here would clobber it with wb_value.
    const bool ldm_rn_in_list_suppresses_wb = l_bit && rn_in_list;
    if (w_bit && !stm_writeback_early && !ldm_rn_in_list_suppresses_wb) {
        state.r[rn] = addressing.wb_value;
    }
    return 1;
}

} // namespace ds
