// arm7_block_test.cpp — ARMv4T block data transfer (LDM / STM) tests.
// Grown incrementally across slice 3b4: commit 3 proved the warn-stub
// dispatch wiring, commit 4 verified the addressing-mode helpers in
// isolation, commit 5 promoted the STMIA scaffold test into the first
// real-execution test (simplest STM IA base case), commit 6 added the
// mirror LDM IA base case, commit 7 wired up IA writeback, and commit
// 8 drops the IA-only restriction so all four addressing modes (IA,
// IB, DA, DB) run through the same forward-walk engine. The test_all_
// addressing_modes parametrized helper covers the 16-case matrix of
// {mode} × {LDM,STM} × {W=0,W=1}. The four IA-specific regression
// tests from commits 5–7 are kept as concrete anchors. The Rn-in-list
// writeback rules from spec §5.5 are still gated out (commits 9/10).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_block_internal.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Preload a single instruction word at `pc`, set PC and R15 for the ARM
// pipeline model, and run exactly one ARM7 cycle. Mirrors the helper used
// by every other arm7 family test.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

// Snapshot R0..R14 so we can assert the register file is unchanged by the
// stub. R15 is excluded because the pipeline model updates it on every
// step (instr_addr + 8 before dispatch, instr_addr + 4 after).
struct RegSnapshot {
    u32 r[15];
};

static RegSnapshot snapshot_regs(const Arm7State& s) {
    RegSnapshot out{};
    for (u32 i = 0; i < 15; ++i) {
        out.r[i] = s.r[i];
    }
    return out;
}

static void require_regs_unchanged(const RegSnapshot& before, const Arm7State& after) {
    for (u32 i = 0; i < 15; ++i) {
        REQUIRE(before.r[i] == after.r[i]);
    }
}

} // namespace

// LDMIA R0, {R1, R2, R3}  — 0xE890000E
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=0, L=1,
// Rn=0, reg_list=0x000E. Commit 6 base case: simplest LDM form with
// no writeback, no S-bit, no Rn-in-list, no R15 — three GPRs loaded
// low→high from the base address. Mirror of the STM IA base case.
static void test_ldm_ia_base_case() {
    NDS nds;

    // Base address lives in ARM7 WRAM (same region every other arm7
    // test uses). Pick a distinct page from kBase (where the instruction
    // word lives) so the instruction fetch and the LDM reads don't
    // alias. Seed three distinct sentinel words so we can prove each
    // register got the right one.
    constexpr u32 kLdmBase = 0x0380'0200u;
    nds.arm7_bus().write32(kLdmBase + 0u, 0xAAAA'AAAAu);
    nds.arm7_bus().write32(kLdmBase + 4u, 0xBBBB'BBBBu);
    nds.arm7_bus().write32(kLdmBase + 8u, 0xCCCC'CCCCu);

    nds.cpu7().state().r[0] = kLdmBase;
    nds.cpu7().state().r[1] = 0u;
    nds.cpu7().state().r[2] = 0u;
    nds.cpu7().state().r[3] = 0u;

    // Snapshot R4..R14 so we can assert the non-listed GPRs are
    // untouched. R0..R3 participate in the transfer (R0 as base, R1-R3
    // as destinations) so they're checked individually below.
    u32 r_before[11];
    for (u32 i = 0; i < 11; ++i) {
        r_before[i] = nds.cpu7().state().r[4 + i];
    }

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE890'000Eu);

    // Three words loaded sequentially from kLdmBase, low→high.
    REQUIRE(nds.cpu7().state().r[1] == 0xAAAA'AAAAu);
    REQUIRE(nds.cpu7().state().r[2] == 0xBBBB'BBBBu);
    REQUIRE(nds.cpu7().state().r[3] == 0xCCCC'CCCCu);

    // W=0, so R0 (the base) is unchanged. R4..R14 untouched.
    REQUIRE(nds.cpu7().state().r[0] == kLdmBase);
    for (u32 i = 0; i < 11; ++i) {
        REQUIRE(nds.cpu7().state().r[4 + i] == r_before[i]);
    }

    // Standard pipeline bookkeeping: PC advanced by 4, one ARM7 cycle.
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// STMIA R0, {R1, R2, R3}  — 0xE880000E
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=0, L=0,
// Rn=0, reg_list=0x000E. This is the commit-5 base case: simplest STM
// form with no writeback, no S-bit, no Rn-in-list, no R15 — three GPRs
// walked low→high from the base address.
static void test_stm_ia_base_case() {
    NDS nds;

    // Base address lives in ARM7 WRAM (same region every other arm7
    // test uses). Pick a distinct page from kBase (where the instruction
    // word lives) to keep memory reads/writes non-overlapping.
    constexpr u32 kStmBase = 0x0380'0200u;
    nds.cpu7().state().r[0] = kStmBase;
    nds.cpu7().state().r[1] = 0x1111'1111u;
    nds.cpu7().state().r[2] = 0x2222'2222u;
    nds.cpu7().state().r[3] = 0x3333'3333u;

    const RegSnapshot before = snapshot_regs(nds.cpu7().state());
    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE880'000Eu);

    // Three words stored sequentially starting at kStmBase, low→high.
    REQUIRE(nds.arm7_bus().read32(kStmBase + 0u) == 0x1111'1111u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 4u) == 0x2222'2222u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 8u) == 0x3333'3333u);

    // W=0, so R0 (the base) is unchanged. R1..R3 are source registers
    // and stores don't modify them either. Everything else untouched.
    require_regs_unchanged(before, nds.cpu7().state());

    // Standard pipeline bookkeeping: PC advanced by 4, one ARM7 cycle.
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// STMIA R0!, {R1, R2, R3}  — 0xE8A0000E
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=0,
// Rn=0, reg_list=0x000E. Commit 7 adds writeback on top of the commit-5
// STM IA base case. Rn is not in the list so §5.5's Rn-in-list rules
// don't apply; the helper's wb_value = Rn + 4*n is written unconditionally.
static void test_stm_ia_writeback() {
    NDS nds;

    constexpr u32 kStmBase = 0x0380'0200u;
    nds.cpu7().state().r[0] = kStmBase;
    nds.cpu7().state().r[1] = 0x1111'1111u;
    nds.cpu7().state().r[2] = 0x2222'2222u;
    nds.cpu7().state().r[3] = 0x3333'3333u;

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8A0'000Eu);

    // Same three-word store as the W=0 case: walked low→high from the
    // pre-writeback base address.
    REQUIRE(nds.arm7_bus().read32(kStmBase + 0u) == 0x1111'1111u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 4u) == 0x2222'2222u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 8u) == 0x3333'3333u);

    // Writeback: Rn = pre-value + 4*3 = kStmBase + 0xC.
    REQUIRE(nds.cpu7().state().r[0] == kStmBase + 0x0Cu);

    // Source registers (R1..R3) untouched by the store; higher GPRs
    // untouched as well. We don't use require_regs_unchanged here
    // because R0 intentionally changed.
    REQUIRE(nds.cpu7().state().r[1] == 0x1111'1111u);
    REQUIRE(nds.cpu7().state().r[2] == 0x2222'2222u);
    REQUIRE(nds.cpu7().state().r[3] == 0x3333'3333u);

    // Standard pipeline bookkeeping: PC advanced by 4, one ARM7 cycle.
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// LDMIA R0!, {R1, R2, R3}  — 0xE8B0000E
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=1,
// Rn=0, reg_list=0x000E. Commit 7 adds writeback on top of the commit-6
// LDM IA base case. Rn is not in the list, so even though ARMv4 loads
// the new Rn value on the same cycle as the list, the simple rule
// applies: writeback lands unconditionally at the end.
static void test_ldm_ia_writeback() {
    NDS nds;

    constexpr u32 kLdmBase = 0x0380'0200u;
    nds.arm7_bus().write32(kLdmBase + 0u, 0xAAAA'AAAAu);
    nds.arm7_bus().write32(kLdmBase + 4u, 0xBBBB'BBBBu);
    nds.arm7_bus().write32(kLdmBase + 8u, 0xCCCC'CCCCu);

    nds.cpu7().state().r[0] = kLdmBase;
    nds.cpu7().state().r[1] = 0u;
    nds.cpu7().state().r[2] = 0u;
    nds.cpu7().state().r[3] = 0u;

    // Snapshot R4..R14 so we can assert the non-listed GPRs are
    // untouched. R0..R3 are individually checked below.
    u32 r_before[11];
    for (u32 i = 0; i < 11; ++i) {
        r_before[i] = nds.cpu7().state().r[4 + i];
    }

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8B0'000Eu);

    // Three words loaded sequentially from kLdmBase, low→high.
    REQUIRE(nds.cpu7().state().r[1] == 0xAAAA'AAAAu);
    REQUIRE(nds.cpu7().state().r[2] == 0xBBBB'BBBBu);
    REQUIRE(nds.cpu7().state().r[3] == 0xCCCC'CCCCu);

    // Writeback: Rn = pre-value + 4*3 = kLdmBase + 0xC.
    REQUIRE(nds.cpu7().state().r[0] == kLdmBase + 0x0Cu);
    for (u32 i = 0; i < 11; ++i) {
        REQUIRE(nds.cpu7().state().r[4 + i] == r_before[i]);
    }

    // Standard pipeline bookkeeping: PC advanced by 4, one ARM7 cycle.
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Commit-8 parametrized coverage: one helper, one call per
// {mode, load_store, writeback} combination. Rn is R0 seeded at a
// mid-range ARM7-WRAM address (0x0380'0400) so all four modes stay
// well inside the 64 KB mirror regardless of direction. reg_list is
// {R1, R2, R3} (encoded as 0x000E) so the helper exercises a 3-word
// transfer — the same shape as the commits 5–7 regression tests.
//
// For every call we compute the expected start/last/wb addresses
// directly from the §5.4 table (not from the helper under test) to
// keep this an end-to-end check of dispatch_block rather than a
// circular verification of compute_block_addressing.
static void run_block_mode_case(bool p_bit, bool u_bit, bool l_bit, bool w_bit) {
    NDS nds;

    constexpr u32 kRnBase = 0x0380'0400u;
    constexpr u32 kRegList = 0x000Eu; // {R1, R2, R3}
    constexpr u32 kN = 3u;
    constexpr u32 kBytes = 4u * kN;

    // Expected first-transfer address and writeback value, derived from
    // spec §5.4 directly rather than from compute_block_addressing.
    u32 expected_first;
    u32 expected_wb;
    if (u_bit) {
        // IA: first = Rn,     wb = Rn + 4*n
        // IB: first = Rn + 4, wb = Rn + 4*n
        expected_first = kRnBase + (p_bit ? 4u : 0u);
        expected_wb = kRnBase + kBytes;
    } else {
        // DA: first = Rn - 4*(n-1), wb = Rn - 4*n
        // DB: first = Rn - 4*n,     wb = Rn - 4*n
        expected_first = p_bit ? (kRnBase - kBytes) : (kRnBase - 4u * (kN - 1u));
        expected_wb = kRnBase - kBytes;
    }

    // Build the instruction on the fly. cond=AL, bits[27:25]=100,
    // Rn=R0, reg_list=0x000E.
    const u32 instr = 0xE800'0000u | (static_cast<u32>(p_bit) << 24) |
                      (static_cast<u32>(u_bit) << 23) | (static_cast<u32>(w_bit) << 21) |
                      (static_cast<u32>(l_bit) << 20) | (0u << 16) // Rn = R0
                      | kRegList;

    // Sentinels:
    //  - For LDM, seed memory at expected_first + {0,4,8} so the three
    //    registers pick up AAAA/BBBB/CCCC in order.
    //  - For STM, seed R1..R3 with 1111/2222/3333 so we can assert the
    //    stored words land at expected_first + {0,4,8}.
    constexpr u32 kSentMemA = 0xAAAA'AAAAu;
    constexpr u32 kSentMemB = 0xBBBB'BBBBu;
    constexpr u32 kSentMemC = 0xCCCC'CCCCu;
    constexpr u32 kSentRegA = 0x1111'1111u;
    constexpr u32 kSentRegB = 0x2222'2222u;
    constexpr u32 kSentRegC = 0x3333'3333u;

    nds.cpu7().state().r[0] = kRnBase;

    if (l_bit) {
        // LDM: zero the target registers, preload memory.
        nds.cpu7().state().r[1] = 0u;
        nds.cpu7().state().r[2] = 0u;
        nds.cpu7().state().r[3] = 0u;
        nds.arm7_bus().write32(expected_first + 0u, kSentMemA);
        nds.arm7_bus().write32(expected_first + 4u, kSentMemB);
        nds.arm7_bus().write32(expected_first + 8u, kSentMemC);
    } else {
        // STM: seed the source registers, zero the target memory so a
        // missed write is visible.
        nds.cpu7().state().r[1] = kSentRegA;
        nds.cpu7().state().r[2] = kSentRegB;
        nds.cpu7().state().r[3] = kSentRegC;
        nds.arm7_bus().write32(expected_first + 0u, 0u);
        nds.arm7_bus().write32(expected_first + 4u, 0u);
        nds.arm7_bus().write32(expected_first + 8u, 0u);
    }

    // Snapshot R4..R14 to prove the non-listed GPRs are untouched.
    u32 r_before[11];
    for (u32 i = 0; i < 11; ++i) {
        r_before[i] = nds.cpu7().state().r[4 + i];
    }

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, instr);

    if (l_bit) {
        // LDM: R1/R2/R3 pick up the in-order memory sentinels. R4..R14
        // untouched.
        REQUIRE(nds.cpu7().state().r[1] == kSentMemA);
        REQUIRE(nds.cpu7().state().r[2] == kSentMemB);
        REQUIRE(nds.cpu7().state().r[3] == kSentMemC);
    } else {
        // STM: expected_first + {0,4,8} hold R1/R2/R3.
        REQUIRE(nds.arm7_bus().read32(expected_first + 0u) == kSentRegA);
        REQUIRE(nds.arm7_bus().read32(expected_first + 4u) == kSentRegB);
        REQUIRE(nds.arm7_bus().read32(expected_first + 8u) == kSentRegC);
        // Source registers untouched by the store.
        REQUIRE(nds.cpu7().state().r[1] == kSentRegA);
        REQUIRE(nds.cpu7().state().r[2] == kSentRegB);
        REQUIRE(nds.cpu7().state().r[3] == kSentRegC);
    }

    // Writeback: Rn = wb_value when W=1, unchanged when W=0.
    if (w_bit) {
        REQUIRE(nds.cpu7().state().r[0] == expected_wb);
    } else {
        REQUIRE(nds.cpu7().state().r[0] == kRnBase);
    }

    for (u32 i = 0; i < 11; ++i) {
        REQUIRE(nds.cpu7().state().r[4 + i] == r_before[i]);
    }

    // Standard pipeline bookkeeping: PC advanced by 4, one ARM7 cycle.
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Matrix coverage: {IA, IB, DA, DB} × {LDM, STM} × {W=0, W=1} — 16
// sub-cases. The helper above does the per-case assertions; this test
// exists to enumerate every leaf of the matrix so the commit-8 gate
// change is exercised end-to-end for all four addressing modes.
static void test_all_addressing_modes() {
    // IA — P=0, U=1
    run_block_mode_case(/*p=*/false, /*u=*/true, /*l=*/false, /*w=*/false);
    run_block_mode_case(/*p=*/false, /*u=*/true, /*l=*/false, /*w=*/true);
    run_block_mode_case(/*p=*/false, /*u=*/true, /*l=*/true, /*w=*/false);
    run_block_mode_case(/*p=*/false, /*u=*/true, /*l=*/true, /*w=*/true);

    // IB — P=1, U=1
    run_block_mode_case(/*p=*/true, /*u=*/true, /*l=*/false, /*w=*/false);
    run_block_mode_case(/*p=*/true, /*u=*/true, /*l=*/false, /*w=*/true);
    run_block_mode_case(/*p=*/true, /*u=*/true, /*l=*/true, /*w=*/false);
    run_block_mode_case(/*p=*/true, /*u=*/true, /*l=*/true, /*w=*/true);

    // DA — P=0, U=0
    run_block_mode_case(/*p=*/false, /*u=*/false, /*l=*/false, /*w=*/false);
    run_block_mode_case(/*p=*/false, /*u=*/false, /*l=*/false, /*w=*/true);
    run_block_mode_case(/*p=*/false, /*u=*/false, /*l=*/true, /*w=*/false);
    run_block_mode_case(/*p=*/false, /*u=*/false, /*l=*/true, /*w=*/true);

    // DB — P=1, U=0
    run_block_mode_case(/*p=*/true, /*u=*/false, /*l=*/false, /*w=*/false);
    run_block_mode_case(/*p=*/true, /*u=*/false, /*l=*/false, /*w=*/true);
    run_block_mode_case(/*p=*/true, /*u=*/false, /*l=*/true, /*w=*/false);
    run_block_mode_case(/*p=*/true, /*u=*/false, /*l=*/true, /*w=*/true);
}

// Spec §6.1 item 13: normalization smoke test. STMDB R0!, {R1,R2,R3,R4}
// with R0 = 0x0380'1000 must store R1 at 0x0380'0FF0 (NOT at the base)
// and land R4 at 0x0380'0FFC, with writeback of 0x0380'0FF0. The point
// is to prove the §4.3 forward-walk normalization computed the lowest
// address up front — a naive "subtract-then-step" implementation would
// put the first store at R0 itself.
//
// Base lives in the same ARM7 WRAM mirror every other test uses. The
// spec example uses main-RAM 0x2000'1000 but that region isn't where
// the rest of the arm7_block suite exercises the bus; the normalization
// semantics we're pinning are bus-region-independent.
static void test_stmdb_normalization_smoke() {
    NDS nds;

    constexpr u32 kDbBase = 0x0380'1000u;  // Rn
    constexpr u32 kFirstWr = 0x0380'0FF0u; // Rn - 4*n, lowest addr (n=4)
    // reg_list = 0x001E = {R1, R2, R3, R4} — baked into instr below.
    constexpr u32 kSentR1 = 0x1111'1111u;
    constexpr u32 kSentR2 = 0x2222'2222u;
    constexpr u32 kSentR3 = 0x3333'3333u;
    constexpr u32 kSentR4 = 0x4444'4444u;

    nds.cpu7().state().r[0] = kDbBase;
    nds.cpu7().state().r[1] = kSentR1;
    nds.cpu7().state().r[2] = kSentR2;
    nds.cpu7().state().r[3] = kSentR3;
    nds.cpu7().state().r[4] = kSentR4;

    // Clear the destination range so a missed write is visible, and
    // also clear the "naive" mis-targets at kDbBase..+12 so we can
    // prove nothing landed there.
    for (u32 off = 0; off < 16u; off += 4u) {
        nds.arm7_bus().write32(kFirstWr + off, 0u);
        nds.arm7_bus().write32(kDbBase + off, 0u);
    }

    // Encoding: STMDB R0!, {R1,R2,R3,R4}
    //   cond=AL, bits[27:25]=100, P=1, U=0, S=0, W=1, L=0,
    //   Rn=0, reg_list=0x001E → 0xE920001E.
    const u32 instr = 0xE920'001Eu;

    run_one(nds, kBase, instr);

    // First store at kFirstWr, last at kFirstWr + 12. The register-to-
    // address mapping is still low-register-to-low-address: R1 goes to
    // the lowest address, R4 to the highest.
    REQUIRE(nds.arm7_bus().read32(kFirstWr + 0u) == kSentR1);
    REQUIRE(nds.arm7_bus().read32(kFirstWr + 4u) == kSentR2);
    REQUIRE(nds.arm7_bus().read32(kFirstWr + 8u) == kSentR3);
    REQUIRE(nds.arm7_bus().read32(kFirstWr + 12u) == kSentR4);

    // Nothing should have landed at the base or above — this is the
    // anti-regression pin for the §4.3 normalization.
    REQUIRE(nds.arm7_bus().read32(kDbBase + 0u) == 0u);
    REQUIRE(nds.arm7_bus().read32(kDbBase + 4u) == 0u);
    REQUIRE(nds.arm7_bus().read32(kDbBase + 8u) == 0u);
    REQUIRE(nds.arm7_bus().read32(kDbBase + 12u) == 0u);

    // Writeback: Rn = Rn - 4*n = kFirstWr.
    REQUIRE(nds.cpu7().state().r[0] == kFirstWr);
}

// Commit 9: STM with Rn in the register list, Rn is the LOWEST-
// numbered set bit. ARMv4 rule (§5.5, GBATEK "Strange Effects on
// Invalid Rlist's"): store the ORIGINAL Rn value at the i == Rn slot,
// then apply writeback (if W=1) at the end. The writeback rule falls
// out of §4.3's "compute wb_value up front" normalization — the loop
// reads state.r[Rn] unchanged on the first iteration because we did
// NOT apply the early writeback.
//
// Encoding: STMIA R0!, {R0, R1, R2}
//   cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=0,
//   Rn=0, reg_list = 0x0007 → 0xE8A0'0007.
static void test_stm_rn_in_list_lowest() {
    NDS nds;

    constexpr u32 kStmBase = 0x0380'0200u;
    nds.cpu7().state().r[0] = kStmBase;
    nds.cpu7().state().r[1] = 0x1111'1111u;
    nds.cpu7().state().r[2] = 0x2222'2222u;

    // Clear the target range so any stale store is visible.
    nds.arm7_bus().write32(kStmBase + 0u, 0u);
    nds.arm7_bus().write32(kStmBase + 4u, 0u);
    nds.arm7_bus().write32(kStmBase + 8u, 0u);

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8A0'0007u);

    // Memory at kStmBase holds the ORIGINAL R0 (=kStmBase), not the
    // post-writeback value. R1/R2 walk in at +4 and +8 low→high.
    REQUIRE(nds.arm7_bus().read32(kStmBase + 0u) == kStmBase);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 4u) == 0x1111'1111u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 8u) == 0x2222'2222u);

    // End-of-instruction writeback still applies because the STM-early
    // path was not taken (Rn was lowest). Rn = kStmBase + 4*3.
    REQUIRE(nds.cpu7().state().r[0] == kStmBase + 0x0Cu);
    REQUIRE(nds.cpu7().state().r[1] == 0x1111'1111u);
    REQUIRE(nds.cpu7().state().r[2] == 0x2222'2222u);

    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Commit 9: STM with Rn in the register list, Rn is NOT the lowest-
// numbered set bit. ARMv4 rule (§5.5): apply writeback BEFORE the
// transfer loop so the i == Rn iteration stores the NEW (post-
// writeback) Rn value. The end-of-instruction writeback is suppressed
// to avoid writing wb_value twice.
//
// Encoding: STMIA R2!, {R1, R2, R3}
//   cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=0,
//   Rn=2, reg_list = 0x000E → 0xE8A2'000E.
//
// With R1 = 0x1111'1111, R2 = kStmBase, R3 = 0x3333'3333, we expect:
//   memory[kStmBase + 0] == 0x1111'1111        (R1 original)
//   memory[kStmBase + 4] == kStmBase + 0xC     (R2 NEW, post-writeback)
//   memory[kStmBase + 8] == 0x3333'3333        (R3 original)
//   R2 final == kStmBase + 0xC                  (writeback already applied)
static void test_stm_rn_in_list_not_lowest() {
    NDS nds;

    constexpr u32 kStmBase = 0x0380'0200u;
    nds.cpu7().state().r[1] = 0x1111'1111u;
    nds.cpu7().state().r[2] = kStmBase;
    nds.cpu7().state().r[3] = 0x3333'3333u;

    nds.arm7_bus().write32(kStmBase + 0u, 0u);
    nds.arm7_bus().write32(kStmBase + 4u, 0u);
    nds.arm7_bus().write32(kStmBase + 8u, 0u);

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8A2'000Eu);

    // §5.5 key assertion: R2's slot holds the POST-writeback base,
    // not the original kStmBase. This is the whole point of the
    // "apply writeback early" dance in dispatch_block.
    REQUIRE(nds.arm7_bus().read32(kStmBase + 0u) == 0x1111'1111u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 4u) == kStmBase + 0x0Cu);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 8u) == 0x3333'3333u);

    // R2 holds the new base after the instruction. R1 and R3 were
    // only sources and are unchanged.
    REQUIRE(nds.cpu7().state().r[1] == 0x1111'1111u);
    REQUIRE(nds.cpu7().state().r[2] == kStmBase + 0x0Cu);
    REQUIRE(nds.cpu7().state().r[3] == 0x3333'3333u);

    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Commit 10: LDM with Rn in the register list, W=1. ARMv4 rule
// (§5.5 and GBATEK "Strange Effects on Invalid Rlist's"): writeback
// is suppressed entirely. The transfer loop writes the loaded word
// into state.r[Rn], and the end-of-instruction writeback step must
// leave it alone so wb_value does not clobber the loaded value.
//
// Encoding: LDMIA R0!, {R0, R1, R2}
//   cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=1,
//   Rn=0, reg_list = 0x0007 → 0xE8B0'0007.
//
// Memory layout:
//   [kLdmBase + 0] = 0x5A5A'5A5Au   — this becomes the new R0
//   [kLdmBase + 4] = 0xBBBB'BBBBu   — destined for R1
//   [kLdmBase + 8] = 0xCCCC'CCCCu   — destined for R2
//
// Post-instruction expectations:
//   R0 == 0x5A5A'5A5A (loaded value, NOT kLdmBase + 12 — writeback
//                      suppressed per ARMv4 rule).
//   R1 == 0xBBBB'BBBB
//   R2 == 0xCCCC'CCCC
static void test_ldm_rn_in_list_suppresses_writeback() {
    NDS nds;

    constexpr u32 kLdmBase = 0x0380'0200u;
    constexpr u32 kLoadedR0 = 0x5A5A'5A5Au;

    nds.arm7_bus().write32(kLdmBase + 0u, kLoadedR0);
    nds.arm7_bus().write32(kLdmBase + 4u, 0xBBBB'BBBBu);
    nds.arm7_bus().write32(kLdmBase + 8u, 0xCCCC'CCCCu);

    nds.cpu7().state().r[0] = kLdmBase;
    nds.cpu7().state().r[1] = 0u;
    nds.cpu7().state().r[2] = 0u;

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8B0'0007u);

    // The loaded value wins: R0 is NOT kLdmBase + 0xC (which would be
    // the naive writeback result). It is the word at memory[kLdmBase].
    REQUIRE(nds.cpu7().state().r[0] == kLoadedR0);
    REQUIRE(nds.cpu7().state().r[1] == 0xBBBB'BBBBu);
    REQUIRE(nds.cpu7().state().r[2] == 0xCCCC'CCCCu);

    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Commit 11: LDM with R15 in the register list, S=0. ARMv4 rule
// (§5.7 and §4.4 LDM step 6): the loaded word is word-aligned
// (`& ~0x3u`) and written to both state.r[15] and state.pc. CPSR.T
// is NEVER modified by LDM on ARMv4 — that is an ARMv5 interworking
// behavior and is explicitly out of scope.
//
// Encoding: LDMIA R0, {R1, PC}
//   cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=0, L=1,
//   Rn=0, reg_list = 0x8002 → 0xE890'8002.
//
// Memory layout:
//   [kLdmBase + 0] = 0xBBBB'BBBBu      — destined for R1
//   [kLdmBase + 4] = kBranchTargetRaw  — destined for PC (see below)
//
// Post-instruction expectations:
//   R1        == 0xBBBB'BBBB
//   R15       == kBranchTargetAligned (the `& ~0x3` of the loaded word)
//   state.pc  == kBranchTargetAligned
//   CPSR      unchanged from the initial 0xD3 (System mode, F+I, ARM)
static void test_ldm_r15_in_list_s0() {
    NDS nds;

    constexpr u32 kLdmBase = 0x0380'0200u;
    // Deliberately pick an unaligned low bit on the loaded PC value to
    // prove the word-align mask (& ~0x3u) is applied. ARMv5 would
    // interpret bit 0 as "switch to Thumb"; ARMv4 must silently drop
    // the bit and keep CPSR.T clear.
    constexpr u32 kBranchTargetRaw = 0x0380'1003u;
    constexpr u32 kBranchTargetAligned = 0x0380'1000u;

    nds.arm7_bus().write32(kLdmBase + 0u, 0xBBBB'BBBBu);
    nds.arm7_bus().write32(kLdmBase + 4u, kBranchTargetRaw);

    nds.cpu7().state().r[0] = kLdmBase;
    nds.cpu7().state().r[1] = 0u;

    // Capture CPSR before — the ARMv4 rule says LDM must not touch
    // bit 5 (Thumb). Save the full word so we can assert no other
    // side-effect either.
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE890'8002u);

    // R1 picked up the first list slot.
    REQUIRE(nds.cpu7().state().r[1] == 0xBBBB'BBBBu);

    // R15 and state.pc both hold the word-aligned loaded value. The
    // next fetch will land at kBranchTargetAligned, NOT at kBase + 4
    // (the normal sequential-next pc).
    REQUIRE(nds.cpu7().state().r[15] == kBranchTargetAligned);
    REQUIRE(nds.cpu7().state().pc == kBranchTargetAligned);

    // CPSR is completely unchanged — key ARMv4 invariant. Bit 5
    // (Thumb) stays clear even though the loaded word had bit 0 set.
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);

    // W=0, Rn=R0 not in list, one cycle consumed.
    REQUIRE(nds.cpu7().state().r[0] == kLdmBase);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Commit 12: STM with R15 in the register list. ARMv4 rule
// (§5.7): the stored word is `instruction_addr + 12` — NOT
// `state.r[15]`. The ARM7TDMI step_arm loop has already advanced
// `state.r[15]` to `instruction_addr + 8` (the PC-ahead-by-8 value
// the pipeline exposes to the execute stage), so reading r[15]
// directly would produce an off-by-4 word for STM.
//
// Encoding: STMIA R0!, {R1, PC}
//   cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=0,
//   Rn=0, reg_list = 0x8002 → 0xE8A0'8002.
//
// The instruction word lives at `kBase`, so the expected stored PC
// is `kBase + 12`. Seed R15 to a distinct sentinel value to prove
// the code is NOT reading r[15] — if it were, the stored word would
// be `kBase + 8` (the step_arm value).
static void test_stm_r15_in_list() {
    NDS nds;

    constexpr u32 kStmBase = 0x0380'0200u;
    nds.cpu7().state().r[0] = kStmBase;
    nds.cpu7().state().r[1] = 0x1111'1111u;

    nds.arm7_bus().write32(kStmBase + 0u, 0u);
    nds.arm7_bus().write32(kStmBase + 4u, 0u);

    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8A0'8002u);

    // R1 at the lowest address, PC at +4. The PC slot holds
    // kBase + 12 per the ARMv4 STM-R15 rule.
    REQUIRE(nds.arm7_bus().read32(kStmBase + 0u) == 0x1111'1111u);
    REQUIRE(nds.arm7_bus().read32(kStmBase + 4u) == kBase + 12u);

    // Writeback: Rn = kStmBase + 4*2. R1 unchanged.
    REQUIRE(nds.cpu7().state().r[0] == kStmBase + 0x08u);
    REQUIRE(nds.cpu7().state().r[1] == 0x1111'1111u);

    // Standard pipeline bookkeeping: STM did not branch, PC advanced
    // to kBase + 4, one ARM7 cycle consumed.
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
}

// Direct-call tests for the addressing-mode normalization helpers in
// arm7_block_internal.hpp. These cover every row of the §5.4 table and
// the §5.8 empty-list case without standing up an NDS harness. Commit 5
// will wire compute_block_addressing into dispatch_block; for now we
// only verify that the helper math is correct in isolation.
static void test_block_addressing_helpers() {
    using ds::arm7_block_detail::compute_block_addressing;
    using ds::arm7_block_detail::reg_list_count;

    // --- Empty register list (n == 0) ---
    //
    // Spec §5.8: the helper produces wb_value == Rn because the loop is
    // effectively a no-op. start_addr == Rn for every mode except IB,
    // which adds +4 up front per §4.3.
    {
        const u32 rn = 0x0200'1000u;

        // IA  (P=0, U=1)
        auto a = compute_block_addressing(rn, 0x0000u, /*p=*/false, /*u=*/true);
        REQUIRE(a.start_addr == rn);
        REQUIRE(a.wb_value == rn);

        // IB  (P=1, U=1)
        a = compute_block_addressing(rn, 0x0000u, /*p=*/true, /*u=*/true);
        REQUIRE(a.start_addr == rn + 4u);
        REQUIRE(a.wb_value == rn);

        // DA  (P=0, U=0). With n=0 the formula degenerates to
        // start = Rn - 0 + 4 = Rn + 4. The empty-list path is a warned
        // no-op (spec §5.8), so the exact start address is never
        // observed by the transfer loop — we only pin the value the
        // helper produces so future refactors stay deterministic.
        a = compute_block_addressing(rn, 0x0000u, /*p=*/false, /*u=*/false);
        REQUIRE(a.start_addr == rn + 4u);
        REQUIRE(a.wb_value == rn);

        // DB  (P=1, U=0). start = Rn - 0 + 0 = Rn.
        a = compute_block_addressing(rn, 0x0000u, /*p=*/true, /*u=*/false);
        REQUIRE(a.start_addr == rn);
        REQUIRE(a.wb_value == rn);
    }

    // --- Single register (n == 1), Rn = 0x2000, reg_list = {R1} ---
    // Values taken directly from the §5.4 addressing-mode table.
    {
        const u32 rn = 0x0000'2000u;
        const u32 list = 0x0002u;

        // IA: start = Rn, wb = Rn + 4
        auto a = compute_block_addressing(rn, list, /*p=*/false, /*u=*/true);
        REQUIRE(a.start_addr == 0x0000'2000u);
        REQUIRE(a.wb_value == 0x0000'2004u);

        // IB: start = Rn + 4, wb = Rn + 4
        a = compute_block_addressing(rn, list, /*p=*/true, /*u=*/true);
        REQUIRE(a.start_addr == 0x0000'2004u);
        REQUIRE(a.wb_value == 0x0000'2004u);

        // DA: start = Rn - 4*(n-1) = Rn, wb = Rn - 4*n = Rn - 4
        a = compute_block_addressing(rn, list, /*p=*/false, /*u=*/false);
        REQUIRE(a.start_addr == 0x0000'2000u);
        REQUIRE(a.wb_value == 0x0000'1FFCu);

        // DB: start = Rn - 4*n = Rn - 4, wb = Rn - 4*n = Rn - 4
        a = compute_block_addressing(rn, list, /*p=*/true, /*u=*/false);
        REQUIRE(a.start_addr == 0x0000'1FFCu);
        REQUIRE(a.wb_value == 0x0000'1FFCu);
    }

    // --- Three registers (n == 3), Rn = 0x2000, reg_list = {R1,R2,R3} ---
    {
        const u32 rn = 0x0000'2000u;
        const u32 list = 0x000Eu;

        // IA: start = Rn, wb = Rn + 12
        auto a = compute_block_addressing(rn, list, /*p=*/false, /*u=*/true);
        REQUIRE(a.start_addr == 0x0000'2000u);
        REQUIRE(a.wb_value == 0x0000'200Cu);

        // IB: start = Rn + 4, wb = Rn + 12
        a = compute_block_addressing(rn, list, /*p=*/true, /*u=*/true);
        REQUIRE(a.start_addr == 0x0000'2004u);
        REQUIRE(a.wb_value == 0x0000'200Cu);

        // DA: start = Rn - 4*(n-1) = Rn - 8, wb = Rn - 12
        a = compute_block_addressing(rn, list, /*p=*/false, /*u=*/false);
        REQUIRE(a.start_addr == 0x0000'1FF8u);
        REQUIRE(a.wb_value == 0x0000'1FF4u);

        // DB: start = Rn - 12, wb = Rn - 12
        a = compute_block_addressing(rn, list, /*p=*/true, /*u=*/false);
        REQUIRE(a.start_addr == 0x0000'1FF4u);
        REQUIRE(a.wb_value == 0x0000'1FF4u);
    }

    // --- All 16 registers (n == 16), Rn = 0x2000, reg_list = 0xFFFF ---
    {
        const u32 rn = 0x0000'2000u;
        const u32 list = 0xFFFFu;

        // IA: start = Rn, wb = Rn + 64
        auto a = compute_block_addressing(rn, list, /*p=*/false, /*u=*/true);
        REQUIRE(a.start_addr == 0x0000'2000u);
        REQUIRE(a.wb_value == 0x0000'2040u);

        // DB: start = Rn - 64, wb = Rn - 64
        a = compute_block_addressing(rn, list, /*p=*/true, /*u=*/false);
        REQUIRE(a.start_addr == 0x0000'1FC0u);
        REQUIRE(a.wb_value == 0x0000'1FC0u);
    }

    // --- Wraparound: Rn near 2^32, single register ---
    // All arithmetic is u32-wrapping; these cases verify the helper does
    // not assume non-overflowing inputs.
    {
        const u32 rn = 0xFFFF'FFFCu;
        const u32 list = 0x0002u;

        // IA: start = Rn, wb = Rn + 4 = 0 (wraps)
        auto a = compute_block_addressing(rn, list, /*p=*/false, /*u=*/true);
        REQUIRE(a.start_addr == 0xFFFF'FFFCu);
        REQUIRE(a.wb_value == 0x0000'0000u);

        // IB: start = Rn + 4 = 0, wb = Rn + 4 = 0
        a = compute_block_addressing(rn, list, /*p=*/true, /*u=*/true);
        REQUIRE(a.start_addr == 0x0000'0000u);
        REQUIRE(a.wb_value == 0x0000'0000u);
    }

    // --- popcount smoke test ---
    REQUIRE(reg_list_count(0x0000u) == 0u);
    REQUIRE(reg_list_count(0x8001u) == 2u);
    REQUIRE(reg_list_count(0xFFFFu) == 16u);
    // Bits above bit 15 must be ignored (the encoded field is 16 bits).
    REQUIRE(reg_list_count(0xFFFF'0000u) == 0u);
}

int main() {
    test_ldm_ia_base_case();
    test_stm_ia_base_case();
    test_stm_ia_writeback();
    test_ldm_ia_writeback();
    test_all_addressing_modes();
    test_stmdb_normalization_smoke();
    test_stm_rn_in_list_lowest();
    test_stm_rn_in_list_not_lowest();
    test_ldm_rn_in_list_suppresses_writeback();
    test_ldm_r15_in_list_s0();
    test_stm_r15_in_list();
    test_block_addressing_helpers();

    std::puts("arm7_block_test: LDM/STM IA base cases + {IA,IB,DA,DB} matrix + DB "
              "normalization + STM Rn-in-list old/new base + LDM Rn-in-list "
              "writeback suppression + LDM/STM R15-in-list (S=0) + helpers verified");
    return 0;
}
