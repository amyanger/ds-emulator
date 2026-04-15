// arm7_block_test.cpp — ARMv4T block data transfer (LDM / STM) tests.
// Grown incrementally across slice 3b4: commit 3 proved the warn-stub
// dispatch wiring, commit 4 verified the addressing-mode helpers in
// isolation, commit 5 promoted the STMIA scaffold test into the first
// real-execution test (simplest STM IA base case), commit 6 added the
// mirror LDM IA base case: W=0, S=0, P=0, U=1, Rn not in list, R15
// not in list, non-empty list, and commit 7 adds the writeback (W=1)
// variants of both IA base cases — Rn is updated to wb_value = Rn +
// 4*n after the transfer. The Rn-in-list writeback rules from spec
// §5.5 are still gated out and covered in commits 9/10.

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
    test_block_addressing_helpers();

    std::puts("arm7_block_test: LDM/STM IA base cases + writeback + helpers verified");
    return 0;
}
