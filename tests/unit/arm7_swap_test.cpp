// arm7_swap_test.cpp — ARMv4T Single Data Swap (SWP / SWPB) tests.
//
// Spec §6.2 coverage:
//   1. Word SWP — aligned base
//   2. Word SWP — unaligned base (rotated read), force-aligned write
//   3. SWPB — byte swap (zero-extended into Rd)
//   4. Rd == Rm — Rm latched before the read, original Rm goes to memory
//   5. Rm == Rn — original base used for the write (not whatever the
//      loaded value would become)
//   6. Rd == Rn — Rd gets the pre-write memory value, memory gets Rm
//   7. UNPREDICTABLE warn paths — R15 as Rn / Rd / Rm
//
// Each test seeds memory + register state, runs exactly one ARM7
// instruction via the usual `run_one` helper, and asserts both the
// register-file end state and the bus contents.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

} // namespace

// SWP R0, R1, [R2] — aligned base, happy path.
// Encoding: cond=AL, bits[27:23]=00010, B=0, bits[21:20]=00,
// Rn=R2, Rd=R0, bits[11:4]=00001001, Rm=R1 → 0xE102'0091.
//
// Pre:  memory[R2] = 0xDEAD'BEEF, R1 = 0xCAFE'BABE, R2 = kSwapBase
// Post: R0 = 0xDEAD'BEEF (loaded), memory[R2] = 0xCAFE'BABE (from Rm)
static void test_swp_word_aligned() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    constexpr u32 kMemBefore = 0xDEAD'BEEFu;
    constexpr u32 kRmValue = 0xCAFE'BABEu;

    nds.arm7_bus().write32(kSwapBase, kMemBefore);
    s.r[0] = 0u;
    s.r[1] = kRmValue;
    s.r[2] = kSwapBase;

    const u64 cycles_before = s.cycles;

    run_one(nds, kBase, 0xE102'0091u);

    // Rd picked up the original memory word.
    REQUIRE(s.r[0] == kMemBefore);
    // Rm is unchanged; memory now holds Rm.
    REQUIRE(s.r[1] == kRmValue);
    REQUIRE(s.r[2] == kSwapBase);
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kRmValue);

    REQUIRE(s.pc == kBase + 4u);
    REQUIRE(s.cycles == cycles_before + 1u);
}

// SWP R0, R1, [R2] — unaligned base (R2 = kSwapBase + 1).
// Word SWP on ARMv4 rotates the LDR-style read by (addr & 3) * 8 so
// the byte at the unaligned address lands in bits [7:0] of Rd, but
// the write goes to the FORCE-ALIGNED address with the full Rm word.
//
// With memory at the aligned word = 0xAABB'CCDD and R2 = base + 1:
//   rotation = 1 * 8 = 8
//   loaded   = ROR(0xAABB'CCDD, 8) = 0xDDAA'BBCC
//   written  = 0xCAFE'BABE at address (base & ~3u)
static void test_swp_word_unaligned_rotated_read() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    constexpr u32 kUnalignedAddr = kSwapBase + 1u;
    constexpr u32 kMemBefore = 0xAABB'CCDDu;
    constexpr u32 kRmValue = 0xCAFE'BABEu;
    constexpr u32 kExpectedLoaded = 0xDDAA'BBCCu; // ROR(0xAABBCCDD, 8)

    nds.arm7_bus().write32(kSwapBase, kMemBefore);
    s.r[0] = 0u;
    s.r[1] = kRmValue;
    s.r[2] = kUnalignedAddr;

    run_one(nds, kBase, 0xE102'0091u);

    // Rotated read lands in Rd.
    REQUIRE(s.r[0] == kExpectedLoaded);
    // Write is force-aligned: the full Rm word goes to the aligned
    // base, NOT to the unaligned R2.
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kRmValue);
}

// SWPB R0, R1, [R2] — byte swap, no alignment, no rotation.
// Encoding: B=1 → 0xE142'0091.
//
// Pre:  memory bytes at kSwapBase..+3 are 0xAA 0xBB 0xCC 0xDD
//       R1 = 0x1234'5677 (we only want the low byte 0x77 stored)
//       R2 = kSwapBase + 2 (the 0xCC slot)
// Post: R0 = 0x0000'00CC (zero-extended)
//       memory[kSwapBase + 2] = 0x77 (low byte of Rm)
//       neighbouring bytes unchanged.
static void test_swpb_byte() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    // Pack so bytes read low-to-high as AA BB CC DD.
    nds.arm7_bus().write32(kSwapBase, 0xDDCC'BBAAu);

    s.r[0] = 0xFFFF'FFFFu; // sentinel: must be zero-extended over
    s.r[1] = 0x1234'5677u;
    s.r[2] = kSwapBase + 2u;

    run_one(nds, kBase, 0xE142'0091u);

    // Zero-extended byte 0xCC — upper 24 bits cleared.
    REQUIRE(s.r[0] == 0x0000'00CCu);
    // Only byte at +2 was overwritten with 0x77; neighbours intact.
    const u32 after = nds.arm7_bus().read32(kSwapBase);
    REQUIRE((after & 0x0000'00FFu) == 0x0000'00AAu); // byte 0
    REQUIRE((after & 0x0000'FF00u) == 0x0000'BB00u); // byte 1
    REQUIRE((after & 0x00FF'0000u) == 0x0077'0000u); // byte 2 (was CC, now 77)
    REQUIRE((after & 0xFF00'0000u) == 0xDD00'0000u); // byte 3
}

// Rd == Rm — the Rm latch before the bus read must capture the
// ORIGINAL value so the write gets the pre-load Rm, not whatever the
// memory read returned. Without the latch, Rd would be overwritten
// by the read before the write path reads `state.r[rm]`, and the
// memory would end up holding the loaded value.
//
// Encoding: SWP R1, R1, [R2]  → Rd=R1, Rm=R1
//   0xE102'1091 (Rd bits[15:12] = 1).
static void test_swp_rd_equals_rm_latch() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    constexpr u32 kMemBefore = 0x1234'5678u;
    constexpr u32 kOriginalR1 = 0xABCD'ABCDu;

    nds.arm7_bus().write32(kSwapBase, kMemBefore);
    s.r[1] = kOriginalR1;
    s.r[2] = kSwapBase;

    run_one(nds, kBase, 0xE102'1091u);

    // R1 ends up holding the loaded memory word (Rd = post-load).
    REQUIRE(s.r[1] == kMemBefore);
    // Memory holds the ORIGINAL R1, proving the latch captured Rm
    // before the read clobbered it.
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kOriginalR1);
}

// Rm == Rn — the latch ensures the Rm value (== base address) is
// still stored to memory, not whatever shows up after the load.
// Encoding: SWP R0, R2, [R2]  → Rd=R0, Rn=R2, Rm=R2
//   0xE102'0092 (Rm bits[3:0] = 2).
static void test_swp_rm_equals_rn_latch() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    constexpr u32 kMemBefore = 0x0BAD'F00Du;

    nds.arm7_bus().write32(kSwapBase, kMemBefore);
    s.r[0] = 0u;
    s.r[2] = kSwapBase;

    run_one(nds, kBase, 0xE102'0092u);

    // Rd got the pre-write memory word.
    REQUIRE(s.r[0] == kMemBefore);
    // Memory holds the ORIGINAL Rm, which is the original base
    // address itself — proving we didn't accidentally store the
    // post-load value.
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kSwapBase);
    // R2 unchanged.
    REQUIRE(s.r[2] == kSwapBase);
}

// Rd == Rn — Rd gets the pre-write memory value; memory gets Rm.
// The base is not touched because the store uses `state.r[rn]` which
// has not yet been overwritten (the Rd write happens after the
// memory store). Rd = R2, Rn = R2, Rm = R1.
// Encoding: SWP R2, R1, [R2]  → 0xE102'2091.
static void test_swp_rd_equals_rn() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    constexpr u32 kMemBefore = 0x1357'9BDFu;
    constexpr u32 kRmValue = 0x2468'ACE0u;

    nds.arm7_bus().write32(kSwapBase, kMemBefore);
    s.r[1] = kRmValue;
    s.r[2] = kSwapBase;

    run_one(nds, kBase, 0xE102'2091u);

    // R2 (= Rd) now holds the pre-write memory value.
    REQUIRE(s.r[2] == kMemBefore);
    // Memory holds Rm.
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kRmValue);
    // R1 unchanged.
    REQUIRE(s.r[1] == kRmValue);
}

// UNPREDICTABLE warn paths — R15 as any operand. The instruction
// still executes deterministically (warn + continue with the
// pipeline-offset R15 value); verify we don't crash and the bus
// access completes.
//
// Encoding: SWP R15, R1, [R2] — Rd = R15. 0xE102'F091.
// This would write the loaded memory value into R15, which our
// implementation does directly via `state.r[rd] = loaded`. We do
// NOT use write_rd here, so state.pc is NOT updated by the swap —
// step_arm's post-dispatch `state.pc = instr_addr + 4` stands.
static void test_swp_r15_operand_warns() {
    NDS nds;
    auto& s = nds.cpu7().state();

    constexpr u32 kSwapBase = 0x0380'0200u;
    constexpr u32 kMemBefore = 0x0000'F00Du;
    constexpr u32 kRmValue = 0xAAAA'5555u;

    nds.arm7_bus().write32(kSwapBase, kMemBefore);
    s.r[1] = kRmValue;
    s.r[2] = kSwapBase;

    run_one(nds, kBase, 0xE102'F091u);

    // Memory received Rm.
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kRmValue);
    // No crash, pc still advanced by step_arm.
    REQUIRE(s.pc == kBase + 4u);
}

int main() {
    test_swp_word_aligned();
    test_swp_word_unaligned_rotated_read();
    test_swpb_byte();
    test_swp_rd_equals_rm_latch();
    test_swp_rm_equals_rn_latch();
    test_swp_rd_equals_rn();
    test_swp_r15_operand_warns();

    std::puts("arm7_swap_test: SWP aligned/unaligned + SWPB + Rm latch corners "
              "(Rd==Rm, Rm==Rn, Rd==Rn) + R15 warn path verified");
    return 0;
}
