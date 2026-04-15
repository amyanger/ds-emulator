// arm7_block_test.cpp — scaffold test for the ARMv4T block data transfer
// (LDM / STM) dispatcher. Commit 3 of slice 3b4 only wires dispatch_block
// into the primary dispatch table as a warn stub; this test verifies the
// wiring by executing a recognizable LDM/STM encoding and confirming the
// stub behaves as a no-op (PC advances 4, cycles increment by 1, register
// file otherwise unchanged). Real LDM/STM semantics land in commits 4-15
// and will be covered by dedicated tests.

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

// LDMIA R0, {R1}  — 0xE8900002
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=0, L=1,
// Rn=0, reg_list=0x0002. This is the simplest LDM form and routes
// through the primary 0b100 dispatch slot into dispatch_block.
static void test_dispatch_block_stub_ldmia_does_not_crash() {
    NDS nds;

    // Seed a couple of GPRs so we can prove the stub leaves them alone.
    nds.cpu7().state().r[0] = 0x0380'0200u;
    nds.cpu7().state().r[1] = 0xDEAD'BEEFu;
    nds.cpu7().state().r[2] = 0xCAFE'BABEu;

    const RegSnapshot before = snapshot_regs(nds.cpu7().state());
    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE890'0002u);

    // PC advances by 4, cycles consumed == 1 (ARM7 tracks cycles in ARM7
    // units; run_one advances by one ARM9-cycle pair which nets one ARM7
    // cycle for a 1-cycle instruction).
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);

    // No bus traffic, no register changes.
    require_regs_unchanged(before, nds.cpu7().state());
}

// STMIA R0!, {R1}  — 0xE8A00002
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=0,
// Rn=0, reg_list=0x0002. Second-simplest STM form; also routes through
// the 0b100 slot. Confirms both the L=0 and L=1 cases reach the stub.
static void test_dispatch_block_stub_stmia_does_not_crash() {
    NDS nds;

    nds.cpu7().state().r[0] = 0x0380'0300u;
    nds.cpu7().state().r[1] = 0x1234'5678u;

    const RegSnapshot before = snapshot_regs(nds.cpu7().state());
    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8A0'0002u);

    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
    require_regs_unchanged(before, nds.cpu7().state());
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
    test_dispatch_block_stub_ldmia_does_not_crash();
    test_dispatch_block_stub_stmia_does_not_crash();
    test_block_addressing_helpers();

    std::puts("arm7_block_test: dispatch_block stub wiring verified");
    return 0;
}
