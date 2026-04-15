// arm7_block.cpp
//
// ARMv4T block data transfer instructions (LDM / STM). Dispatched from
// dispatch_arm() for the bits[27:25] == 100 primary slot.
//
// Commit 5 of slice 3b4 lit up the very first real execution path
// through dispatch_block: plain `STM IA` with W=0, S=0, Rn not in list,
// and R15 not in list. Commit 6 generalized that to the mirror LDM IA
// base case, commit 7 added writeback (W=1) for the Rn-not-in-list IA
// path, and commit 8 now drops the IA-only restriction so all four
// addressing modes (IA, IB, DA, DB) execute through the same
// forward-walk engine. The §4.3 normalization in compute_block_addressing
// means the transfer loop itself is mode-agnostic — only the start_addr
// and wb_value change. The remaining constraints (no S-bit, non-empty
// list, Rn not in list, R15 not in list) stay in place and are wired up
// in their dedicated follow-up commits.

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

    // Base case after commit 8: all four addressing modes (any {P, U}),
    // either L value, free W, Rn not in list, R15 not in list, no S-bit,
    // non-empty list. The §5.5 Rn-in-list writeback rules (commits 9/10),
    // R15-in-list forms (commits 11/12), S=1 user-bank transfer (commit
    // 13), LDM S=1 with R15 (commit 14), and the empty-list quirk
    // (commit 15) still fall through to the warn stub below.
    const bool is_base_case_any_mode = !s_bit && reg_list != 0u &&
                                       ((reg_list & (1u << rn)) == 0u) &&
                                       ((reg_list & (1u << 15)) == 0u);

    if (!is_base_case_any_mode) {
        DS_LOG_WARN("arm7: LDM/STM path not yet implemented 0x%08X at 0x%08X", instr, instr_addr);
        return 1;
    }

    const auto addressing =
        arm7_block_detail::compute_block_addressing(state.r[rn], reg_list, p_bit, u_bit);

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

    // Writeback (spec §4.4 LDM/STM step 7). Rn-in-list is gated out of
    // is_base_case_any_mode, so the simple rule applies unconditionally
    // here: Rn receives wb_value from the addressing helper, which is
    // Rn ± 4*n depending on U.
    if (w_bit) {
        state.r[rn] = addressing.wb_value;
    }
    return 1;
}

} // namespace ds
