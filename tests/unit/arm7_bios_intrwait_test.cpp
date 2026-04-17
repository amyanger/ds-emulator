// Handler contract for IntrWait (SWI 0x04) and VBlankIntrWait (SWI 0x05):
//
//   - IME is forced to 1 on entry.
//   - Condition already met (shadow & mask != 0) consumes and returns.
//   - Condition not met halts via HALTCNT and rewinds R14_svc by the
//     caller-instruction size (4 for ARM, 2 for Thumb), so the dispatcher's
//     implicit MOVS PC, R14 lands PC on the SWI itself for re-execute.
//   - R0 = 1 discards old matching bits from the shadow before checking.
//   - R0 = 0 returns immediately if an old flag was already set.
//   - VBlankIntrWait pre-loads R0=1, R1=1 and tail-calls IntrWait.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_halt.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kShadow = 0x0380FFF8u;

// Place the CPU into the post-exception-entry state that the BIOS
// dispatcher sees at the top of arm7_bios_hle_dispatch_swi for an ARM-
// state SWI at `swi_addr`: SVC mode, R14_svc = swi_addr + 4, SPSR_svc =
// caller's CPSR (User with I=0, F=0, T=0). Returns the caller CPSR so
// tests can check SPSR survives the handler.
static u32 enter_arm_swi_state(NDS& nds, u32 swi_addr) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = 0x00000013u; // SVC, I=0, F=0, T=0
    state.r[14] = swi_addr + 4u;
    const u32 caller_cpsr = 0x00000010u; // User, I=0, F=0, T=0
    *state.spsr_slot() = caller_cpsr;
    return caller_cpsr;
}

// Same as enter_arm_swi_state but with SPSR.T = 1 (Thumb caller) and
// R14_svc = swi_addr + 2.
static u32 enter_thumb_swi_state(NDS& nds, u32 swi_addr) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = 0x00000013u; // SVC itself is always ARM on entry
    state.r[14] = swi_addr + 2u;
    const u32 caller_cpsr = 0x00000030u; // User, T=1
    *state.spsr_slot() = caller_cpsr;
    return caller_cpsr;
}

} // namespace

// IntrWait with shadow already holding the requested bit: consume the
// bit, leave IME=1, do not halt, do not rewind. This is the "condition
// already met on entry" path — common when the game pre-armed the wait
// after an earlier IRQ had already landed.
static void intrwait_condition_met_consumes_and_returns() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.irq7().write_ime(0);
    nds.arm7_bus().write32(kShadow, 0x1u); // VBlank bit pre-set
    state.r[0] = 0u;                       // R0=0 = return on old flag
    state.r[1] = 0x1u;                     // mask = VBlank

    const u32 r14_before = 0x02000100u + 4u;
    enter_arm_swi_state(nds, 0x02000100u);

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.irq7().read_ime() == 1u);
    REQUIRE(nds.arm7_bus().read32(kShadow) == 0u); // consumed
    REQUIRE(nds.cpu7().is_halted() == false);
    REQUIRE(state.r[14] == r14_before); // no rewind
}

// IntrWait with empty shadow and R0=1 (discard old): halts, rewinds
// R14_svc by 4 (ARM), forces IME=1. Shadow remains empty. The discard
// step had nothing to discard in this case; the "not-matched → halt"
// path is what fires.
static void intrwait_no_match_halts_and_rewinds_arm() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.irq7().write_ime(0);
    nds.arm7_bus().write32(kShadow, 0u);
    state.r[0] = 1u;
    state.r[1] = 0x1u;

    constexpr u32 swi_addr = 0x02000100u;
    enter_arm_swi_state(nds, swi_addr);

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.irq7().read_ime() == 1u);
    REQUIRE(nds.cpu7().is_halted() == true);
    REQUIRE(state.r[14] == swi_addr); // rewound by 4
}

// IntrWait from Thumb caller: rewind is 2 bytes, not 4. SPSR.T = 1
// tells the handler which caller-instruction size to use.
static void intrwait_no_match_halts_and_rewinds_thumb() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0u);
    state.r[0] = 1u;
    state.r[1] = 0x1u;

    constexpr u32 swi_addr = 0x02000200u;
    enter_thumb_swi_state(nds, swi_addr);

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.cpu7().is_halted() == true);
    REQUIRE(state.r[14] == swi_addr); // rewound by 2 (Thumb)
}

// R0 = 1 with the target bit already set in the shadow: the handler
// clears the stale bit via the discard step, then the shadow check
// reads zero and halts. Contrast with the R0 = 0 case below where the
// stale bit would be returned immediately.
static void intrwait_discard_flag_clears_then_halts() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0x1u); // stale bit
    state.r[0] = 1u;                       // discard old
    state.r[1] = 0x1u;
    enter_arm_swi_state(nds, 0x02000100u);

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.arm7_bus().read32(kShadow) == 0u); // discarded
    REQUIRE(nds.cpu7().is_halted() == true);
}

// R0 = 0 with the target bit already set: return immediately with the
// bit consumed (matches the "condition met" path but via the R0=0
// branch — GBATEK: "0=Return immediately if an old flag was already
// set"). Other shadow bits must survive.
static void intrwait_r0_zero_consumes_only_matching_bits() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0x5u); // bits 0 and 2 set
    state.r[0] = 0u;
    state.r[1] = 0x1u; // wait on bit 0
    enter_arm_swi_state(nds, 0x02000100u);

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.arm7_bus().read32(kShadow) == 0x4u); // bit 0 cleared, bit 2 kept
    REQUIRE(nds.cpu7().is_halted() == false);
}

// R0 = 0 with an unmatching bit set: handler must NOT consume the
// unmatching bit and must halt, because the R0=0 branch only returns
// when the *requested* mask already matches.
static void intrwait_r0_zero_unmatching_bit_halts() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0x2u); // bit 1 only
    state.r[0] = 0u;
    state.r[1] = 0x1u; // wait on bit 0
    enter_arm_swi_state(nds, 0x02000100u);

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.cpu7().is_halted() == true);
    REQUIRE(nds.arm7_bus().read32(kShadow) == 0x2u); // unmatching bit preserved
}

// VBlankIntrWait hardwires R0 = R1 = 1 then tail-calls IntrWait. With
// empty shadow, the net effect is identical to IntrWait(1, 1) — halt
// and rewind — plus the two register writes.
static void vblank_intrwait_sets_r0_r1_and_tail_calls() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0u);
    state.r[0] = 0xDEADBEEFu; // sentinel — must be overwritten to 1
    state.r[1] = 0xCAFEBABEu; // sentinel — must be overwritten to 1

    constexpr u32 swi_addr = 0x02000300u;
    enter_arm_swi_state(nds, swi_addr);

    bios7_vblank_intr_wait(state, nds.arm7_bus());

    REQUIRE(state.r[0] == 1u);
    REQUIRE(state.r[1] == 1u);
    REQUIRE(nds.cpu7().is_halted() == true);
    REQUIRE(state.r[14] == swi_addr); // IntrWait did its rewind
}

// VBlankIntrWait always waits for a NEW VBlank — the R0=1 it hard-wires
// causes the discard step to clear any stale bit 0 before the shadow
// check. So even with bit 0 pre-set, the handler must halt (the stale
// bit gets discarded) rather than returning immediately. Other shadow
// bits must survive the discard.
static void vblank_intrwait_discards_stale_bit_and_halts() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0x5u); // bit 0 stale + bit 2 unrelated
    enter_arm_swi_state(nds, 0x02000400u);

    bios7_vblank_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.arm7_bus().read32(kShadow) == 0x4u); // bit 0 discarded, bit 2 kept
    REQUIRE(nds.cpu7().is_halted() == true);
}

// Re-execution after halt-wake must NOT re-run the discard step, even
// with R0 still set to 1. The HLE's rewind-PC-and-re-enter model means
// the second call would otherwise clear the very bit the IRQ handler
// set, deadlocking the wait. First invocation halts + rewinds; between
// calls the IRQ handler sets the shadow bit; second invocation (same
// R0=1, R1=mask) must skip discard, consume the bit, and return.
static void intrwait_re_entry_after_halt_does_not_re_discard() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0u);
    state.r[0] = 1u;
    state.r[1] = 0x1u;

    constexpr u32 swi_addr = 0x02000500u;
    enter_arm_swi_state(nds, swi_addr);
    bios7_intr_wait(state, nds.arm7_bus());
    REQUIRE(nds.cpu7().is_halted() == true);
    REQUIRE(state.r[14] == swi_addr);
    REQUIRE(state.intr_wait_pending == true);

    // Simulate what the game's IRQ handler does between halt and the
    // SWI's re-execution on wake: set the shadow bit AND re-stage R0=1
    // (handlers often reuse R0 as scratch for the ack-write).
    nds.arm7_bus().write32(kShadow, 0x1u);
    nds.cpu7().set_halted(false);
    state.r[0] = 1u;
    state.r[1] = 0x1u;
    state.r[14] = swi_addr + 4u; // caller's return addr re-established by SVC entry

    bios7_intr_wait(state, nds.arm7_bus());
    REQUIRE(nds.cpu7().is_halted() == false);
    REQUIRE(nds.arm7_bus().read32(kShadow) == 0u);
    REQUIRE(state.r[14] == swi_addr + 4u); // no second rewind
    REQUIRE(state.intr_wait_pending == false);
}

// A fresh wait that immediately hits its condition must leave the
// pending flag clear — guards against the flag leaking across
// successive unrelated waits.
static void intrwait_consume_clears_pending_flag() {
    NDS nds;
    auto& state = nds.cpu7().state();
    nds.arm7_bus().write32(kShadow, 0x2u);
    state.r[0] = 0u;
    state.r[1] = 0x2u;
    enter_arm_swi_state(nds, 0x02000600u);
    state.intr_wait_pending = true; // simulated stale from a prior call

    bios7_intr_wait(state, nds.arm7_bus());

    REQUIRE(nds.cpu7().is_halted() == false);
    REQUIRE(state.intr_wait_pending == false);
}

int main() {
    intrwait_condition_met_consumes_and_returns();
    intrwait_no_match_halts_and_rewinds_arm();
    intrwait_no_match_halts_and_rewinds_thumb();
    intrwait_discard_flag_clears_then_halts();
    intrwait_r0_zero_consumes_only_matching_bits();
    intrwait_r0_zero_unmatching_bit_halts();
    vblank_intrwait_sets_r0_r1_and_tail_calls();
    vblank_intrwait_discards_stale_bit_and_halts();
    intrwait_re_entry_after_halt_does_not_re_discard();
    intrwait_consume_clears_pending_flag();
    std::puts("arm7_bios_intrwait_test: all 10 cases passed");
    return 0;
}
