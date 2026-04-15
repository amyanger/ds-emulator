// arm7_block.cpp
//
// ARMv4T block data transfer instructions (LDM / STM). Dispatched from
// dispatch_arm() for the bits[27:25] == 100 primary slot.
//
// Commit 5 of slice 3b4 lit up the very first real execution path
// through dispatch_block: plain `STM IA` with W=0, S=0, Rn not in list,
// and R15 not in list. Commit 6 generalizes that gate into a single
// "IA base case" predicate that covers the mirror LDM IA form — same
// restrictions, same forward-walk addressing, just loads instead of
// stores. Commit 7 drops the W=0 restriction for the Rn-not-in-list
// case: when writeback is set, Rn is updated to wb_value after the
// transfer completes. The Rn-in-list writeback rules from spec §5.5
// remain gated out until commits 9 (STM) and 10 (LDM). Every other
// encoding still falls through to the warn stub until its dedicated
// commit wires it up.

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

    // IA base case implemented across commits 5–7: post-increment
    // (P=0), up (U=1), no S-bit, non-empty list, Rn not in list, R15
    // not in list. The L bit selects load vs store. W is free now that
    // commit 7 wires up the (Rn-not-in-list) writeback path; the §5.5
    // Rn-in-list writeback rules are still out of scope (commits 9 and
    // 10). Every other encoding stays on the warn stub until its
    // dedicated follow-up commit wires it up.
    const bool is_base_case_ia = !s_bit && !p_bit && u_bit && reg_list != 0u &&
                                 ((reg_list & (1u << rn)) == 0u) && ((reg_list & (1u << 15)) == 0u);

    if (!is_base_case_ia) {
        DS_LOG_WARN("arm7: LDM/STM path not yet implemented 0x%08X at 0x%08X", instr, instr_addr);
        return 1;
    }

    const auto addressing =
        arm7_block_detail::compute_block_addressing(state.r[rn], reg_list, p_bit, u_bit);

    u32 addr = addressing.start_addr;
    if (l_bit) {
        // LDM IA — load each listed register from [start_addr + 4*k].
        // Per GBATEK / spec §4.4 LDM step 4, the effective address is
        // force-aligned to a word boundary and the read is NOT rotated
        // (LDM differs from LDR here).
        for (u32 i = 0; i < 16; ++i) {
            if ((reg_list & (1u << i)) != 0u) {
                state.r[i] = bus.read32(addr & ~0x3u);
                addr += 4u;
            }
        }
    } else {
        // STM IA — store each listed register to [start_addr + 4*k],
        // force-aligned per spec §4.4 STM step 5. R15-in-list is gated
        // out above, so no PC+12 special case is needed yet.
        for (u32 i = 0; i < 16; ++i) {
            if ((reg_list & (1u << i)) != 0u) {
                bus.write32(addr & ~0x3u, state.r[i]);
                addr += 4u;
            }
        }
    }

    // Writeback (spec §4.4 LDM/STM step 7). Rn-in-list is gated out of
    // is_base_case_ia, so the simple rule applies unconditionally here:
    // Rn receives wb_value from the addressing helper.
    if (w_bit) {
        state.r[rn] = addressing.wb_value;
    }
    return 1;
}

} // namespace ds
