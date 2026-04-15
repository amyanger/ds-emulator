// arm7_block.cpp
//
// ARMv4T block data transfer instructions (LDM / STM). Dispatched from
// dispatch_arm() for the bits[27:25] == 100 primary slot.
//
// Commit 5 of slice 3b4 lights up the very first real execution path
// through dispatch_block: plain `STM IA` with W=0, S=0, L=0, Rn not in
// list, and R15 not in list. Every other encoding still falls through
// to the warn stub until its dedicated commit (6–15) enables it.

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

    // Commit 5 implements only the simplest STM IA encoding: W=0, S=0,
    // L=0, post-increment (P=0), up (U=1), Rn not in list, R15 not in
    // list, non-empty list. Every other path stays on the warn stub
    // until its dedicated follow-up commit wires it up.
    const bool is_base_case_stm_ia = !l_bit && !s_bit && !w_bit && !p_bit && u_bit &&
                                     reg_list != 0u && ((reg_list & (1u << rn)) == 0u) &&
                                     ((reg_list & (1u << 15)) == 0u);

    if (!is_base_case_stm_ia) {
        DS_LOG_WARN("arm7: LDM/STM path not yet implemented 0x%08X at 0x%08X", instr, instr_addr);
        return 1;
    }

    const auto addressing =
        arm7_block_detail::compute_block_addressing(state.r[rn], reg_list, p_bit, u_bit);
    (void) addressing.wb_value; // writeback lands in commit 7

    u32 addr = addressing.start_addr;
    for (u32 i = 0; i < 16; ++i) {
        if ((reg_list & (1u << i)) != 0u) {
            bus.write32(addr & ~0x3u, state.r[i]);
            addr += 4u;
        }
    }
    return 1;
}

} // namespace ds
