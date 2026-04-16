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
    // Each encoding is chosen so it either warns (stub family) or is a
    // deterministic non-branching no-op — PC must advance by exactly 2
    // regardless of which bucket is hit.
    //   buckets 0..5: the bare bucket selector is a harmless shift/add/dp
    //                 or load-store encoding that advances PC by 2.
    //   bucket 6 (0xC000): THUMB.15 STMIA r0, {} — empty list no-op.
    //   bucket 7 (0xF000): THUMB.19 first halfword — BL prefix with imm11=0.
    //                      Sets LR but does not touch PC. (0xE000 would be
    //                      THUMB.18 B #0 which self-loops; 0xE800 was the
    //                      old BLX-suffix warn stub but now UNDEFs per
    //                      slice 3d commit 5.)
    const u16 bucket_encoding[8] = {
        0x0000u, // 000
        0x2000u, // 001
        0x4000u, // 010
        0x6000u, // 011
        0x8000u, // 100
        0xA000u, // 101
        0xC000u, // 110 — STMIA r0, {} empty list
        0xF000u, // 111 — BL prefix imm11=0 (sets LR, no branch)
    };
    for (u32 bucket = 0; bucket < 8; ++bucket) {
        nds.arm7_bus().write16(base + bucket * 2, bucket_encoding[bucket]);
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
