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
// during the transfer loop and wins over wb_value. Commit 11 adds
// LDM with R15 in the list (S=0 form): the transfer loop captures the
// loaded word into a deferred `loaded_pc` slot instead of writing
// state.r[15] directly, and after the loop the word-aligned target
// is committed via `write_rd`. CPSR.T is never modified — that is an
// ARMv5 interworking behavior and is explicitly out of scope here.
// Commit 12 adds the mirror STM form: when R15 is in the list, the
// stored word is `instr_addr + 12` rather than `state.r[15]` (which
// step_arm already advanced to `instr_addr + 8`), matching the ARM7
// TRM PC-ahead-by-12 convention for STM. Commit 13 adds S=1 without
// R15 in the list: the transfer loop runs against the User bank via
// switch_mode(User), with a matching switch_mode(original_mode) once
// the loop completes. The base address is computed BEFORE the swap
// so Rn is read in the current mode, and the end-of-instruction
// writeback lands back in the current mode after the swap-back.
// S=1 + W=1 is UNPREDICTABLE on real hardware — we warn and still
// perform writeback (melonDS parity). The §4.3 normalization in
// compute_block_addressing means the transfer loop itself is
// mode-agnostic — only the start_addr and wb_value change. The
// remaining constraints (LDM S=1 with R15, empty list) stay in
// place and are wired up in their dedicated follow-up commits.

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
    // non-empty list, Rn in list (both families), R15 in list (both
    // families), and S=1 without R15 in the list (user-bank transfer,
    // commit 13). LDM S=1 WITH R15 in the list (the exception-return
    // form) and the empty-list quirk still fall through to the warn
    // stub.
    const bool rn_in_list = (reg_list & (1u << rn)) != 0u;
    const bool r15_in_list = (reg_list & (1u << 15)) != 0u;
    const bool ldm_s_with_r15_unsupported = l_bit && s_bit && r15_in_list;
    const bool is_supported = reg_list != 0u && !ldm_s_with_r15_unsupported;

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

    // S=1 user-bank transfer (spec §5.6 cases 1 and 2, commit 13).
    // When S=1 and we are NOT in the LDM-R15 exception-return form
    // (that is commit 14's scope and is gated out above), every
    // listed register read/written during the transfer loop refers
    // to the user-bank copy. We implement this by swapping to User
    // mode around the loop via switch_mode, which saves the current
    // mode's R8..R14 into its bank and loads the User bank into r[].
    // The base address was computed above from the CURRENT mode's
    // Rn (pre-swap), which is correct per spec §4.4 step 2.
    //
    // S=1 + W=1 is UNPREDICTABLE on real hardware. We warn and still
    // perform the end-of-instruction writeback (matches melonDS and
    // keeps the path deterministic). Real code does not use this
    // encoding.
    const bool s_user_bank_transfer = s_bit;
    const Mode original_mode = state.current_mode();
    if (s_user_bank_transfer) {
        if (w_bit) {
            DS_LOG_WARN("arm7: LDM/STM with S=1 and W=1 is UNPREDICTABLE 0x%08X at 0x%08X",
                        instr,
                        instr_addr);
        }
        state.switch_mode(Mode::User);
    }

    u32 addr = addressing.start_addr;
    u32 loaded_pc = 0u;
    bool r15_loaded = false;
    if (l_bit) {
        // LDM — load each listed register from [start_addr + 4*k]. Per
        // GBATEK / spec §4.4 LDM step 4 the effective address is
        // force-aligned to a word boundary and the read is NOT rotated
        // (LDM differs from LDR here). The walk is always low-register-
        // to-low-address; §4.3 normalized the decreasing modes to this
        // same forward scan before the loop starts.
        //
        // R15 is NOT written directly inside the loop: the loaded word
        // is captured in loaded_pc and committed below, after any
        // SPSR restore the S=1 exception-return form will eventually
        // need (commit 14). Deferring here keeps the S=0 and S=1 paths
        // sharing the same transfer engine.
        for (u32 i = 0; i < 16; ++i) {
            if ((reg_list & (1u << i)) != 0u) {
                const u32 loaded = bus.read32(addr & ~0x3u);
                if (i == 15) {
                    loaded_pc = loaded;
                    r15_loaded = true;
                } else {
                    state.r[i] = loaded;
                }
                addr += 4u;
            }
        }
    } else {
        // STM — store each listed register to [start_addr + 4*k],
        // force-aligned per spec §4.4 STM step 5. Decreasing modes
        // share this loop because §4.3 already produced the
        // lowest-touched address in start_addr.
        //
        // When R15 is in the list the stored word is `instr_addr + 12`,
        // NOT `state.r[15]`. Step_arm has already advanced `state.r[15]`
        // to `instr_addr + 8` by the time dispatch runs, so reading it
        // directly would give the wrong PC-ahead value (by one word)
        // per §5.7 / ARM7TDMI TRM.
        for (u32 i = 0; i < 16; ++i) {
            if ((reg_list & (1u << i)) != 0u) {
                const u32 stored = (i == 15) ? (instr_addr + 12u) : state.r[i];
                bus.write32(addr & ~0x3u, stored);
                addr += 4u;
            }
        }
    }

    // Swap back to the original mode before the R15 write and the
    // end-of-instruction writeback. Spec §4.4 step 5 / §5.6: the
    // user-bank transfer is strictly scoped to the transfer loop;
    // Rn writeback and the R15 commit (for the LDM S=0 form that
    // lives in commit 11) happen in the caller's original mode so
    // any banked Rn is the current-mode register, not User.
    if (s_user_bank_transfer) {
        state.switch_mode(original_mode);
    }

    // R15 write for LDM with R15 in the list (spec §4.4 LDM step 6,
    // §5.7). ARMv4: word-align the loaded value and write it into
    // both state.r[15] and state.pc; CPSR.T is NEVER touched by LDM
    // on ARMv4 (that's an ARMv5-only interworking behavior). The
    // S=1 exception-return form — which would restore CPSR from
    // SPSR around this write — is still gated to the warn stub in
    // this commit (commit 14). `write_rd` stores the given value
    // in r[15] and always masks pc, so passing the pre-aligned
    // value puts the same bits in both fields.
    if (r15_loaded) {
        write_rd(state, 15, loaded_pc & ~0x3u);
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
