// Slice 3c commits 6–7 smoke tests: verify that Arm7::run_until
// correctly routes to step_thumb when CPSR.T=1, that the Thumb fetch
// uses 16-bit reads, that PC advances by 2 per Thumb instruction,
// and that the R15 pipeline-prefetch value (instr_addr + 4) is set
// correctly for both word-aligned and halfword-aligned start addresses.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static void thumb_run_until_advances_pc_by_two_and_consumes_one_cycle() {
    NDS nds;
    const u32 base = 0x0380'0000u;

    // Any Thumb halfword. 0x0000 falls into bucket 000
    // (shift/add/sub) — a warn stub in commit 6.
    nds.arm7_bus().write16(base, 0x0000);

    nds.cpu7().state().pc = base;
    nds.cpu7().state().cpsr |= (1u << 5); // set CPSR.T

    // Run exactly one ARM7 cycle (= 2 ARM9 cycles).
    nds.cpu7().run_until(2);

    REQUIRE(nds.cpu7().state().pc == base + 2);
    REQUIRE(nds.cpu7().state().r[15] == base + 4u); // Thumb pipeline prefetch
    REQUIRE(nds.cpu7().state().cycles == 1u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) != 0); // still Thumb
}

static void thumb_dispatch_covers_all_eight_buckets() {
    NDS nds;
    const u32 base = 0x0380'0000u;

    // One halfword per bucket. bits[15:13] walks 0..7.
    // Each encoding is just the bucket selector in bits 15:13 with
    // zeroes elsewhere. Every one hits a warn stub and returns 1 cycle.
    for (u32 bucket = 0; bucket < 8; ++bucket) {
        const u16 instr = static_cast<u16>(bucket << 13);
        nds.arm7_bus().write16(base + bucket * 2, instr);
    }

    nds.cpu7().state().pc = base;
    nds.cpu7().state().cpsr |= (1u << 5); // Thumb state

    // 8 Thumb instructions = 8 ARM7 cycles = 16 ARM9 cycles.
    nds.cpu7().run_until(16);

    REQUIRE(nds.cpu7().state().pc == base + 16);
    REQUIRE(nds.cpu7().state().cycles == 8u);
}

// Commit 7: verify R15 is instr_addr + 4 when starting at a
// non-word-aligned address (instr_addr % 4 == 2). This is where
// pc_read and pc_literal diverge: pc_read = addr+4 (not word-aligned),
// pc_literal = (addr+4) & ~2 (word-aligned). Bucket handlers are
// still stubs, so we can only verify the externally visible R15 value
// here. The pc_literal path gets tested when format handlers land.
static void thumb_r15_correct_at_halfword_aligned_addr() {
    NDS nds;
    // Start at an address where instr_addr % 4 == 2.
    const u32 base = 0x0380'0002u;

    nds.arm7_bus().write16(base, 0x0000); // bucket 000 stub

    nds.cpu7().state().pc = base;
    nds.cpu7().state().cpsr |= (1u << 5);

    nds.cpu7().run_until(2);

    REQUIRE(nds.cpu7().state().pc == base + 2);
    // R15 = instr_addr + 4 = 0x0380'0006 (not word-aligned)
    REQUIRE(nds.cpu7().state().r[15] == base + 4u);
    REQUIRE(nds.cpu7().state().cycles == 1u);
}

int main() {
    thumb_run_until_advances_pc_by_two_and_consumes_one_cycle();
    thumb_dispatch_covers_all_eight_buckets();
    thumb_r15_correct_at_halfword_aligned_addr();
    return 0;
}
