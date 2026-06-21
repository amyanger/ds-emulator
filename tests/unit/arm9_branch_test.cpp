// Hand-assembled ARM9 branch (B / BL) tests. Each case loads a small program
// into ARM9 main RAM at 0x0200'0000, points PC there, and runs N full-rate
// ARM9 instructions (1 ARM9 cycle each, no halving). B/BL is bit-identical
// between ARMv4T and ARMv5TE (slice 3l §2.1, §5.7), so these mirror the ARM7
// branch tests and prove the ARM9-owned executor computes the right target,
// links the return address for BL, and honours the condition field. BX is a
// separate executor landing with LDR/STR in commit 8 and is not exercised
// here.

#include "bus/arm9_bus.hpp"
#include "cpu/arm9/arm9.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>
#include <initializer_list>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0200'0000u;

// Load a program into ARM9 main RAM at kBase, point PC there, and run exactly
// `steps` instructions. The ARM9 runs at full scheduler rate, so each
// instruction costs 1 ARM9 cycle (no /2). step_arm refreshes R15 = pc + 8 on
// every fetch, so only the initial PC needs seeding.
void load_and_run(NDS& nds, std::initializer_list<u32> program, u32 steps) {
    u32 i = 0;
    for (u32 word : program) {
        nds.arm9_bus().write32(kBase + i * 4, word);
        ++i;
    }
    nds.cpu9().state().pc = kBase;
    const u64 before = nds.cpu9().state().cycles;
    nds.cpu9().run_until(before + steps);
    REQUIRE(nds.cpu9().state().cycles == before + steps);
}

} // namespace

static void b_forward_takes_branch() {
    NDS nds;
    // B +8: target = pc + 8 + (offset24 << 2). For target = kBase + 16 from
    // pc = kBase, offset24 = (16 - 8) / 4 = 2 → 0xEA000002.
    load_and_run(nds, {0xEA00'0002u}, 1);
    REQUIRE(nds.cpu9().state().pc == kBase + 16u);
}

static void b_backward_takes_branch() {
    NDS nds;
    //   kBase + 0x00: MOV R0, #0x55   ; E3A0'0055
    //   kBase + 0x04: B   -3          ; target = (kBase+4) + 8 + (-3 << 2)
    //                                   = kBase + 0  → 0xEAFF'FFFD
    // After 2 steps (MOV, then B) pc must be back at kBase.
    load_and_run(nds, {0xE3A0'0055u, 0xEAFF'FFFDu}, 2);
    REQUIRE(nds.cpu9().state().r[0] == 0x55u);
    REQUIRE(nds.cpu9().state().pc == kBase + 0u);
}

static void b_condition_fail_does_not_branch() {
    NDS nds;
    // MOVS R0, #0 (sets Z=1), then BNE +8 (Z=1 → skipped; pc advances to +8).
    load_and_run(nds, {0xE3B0'0000u, 0x1A00'0002u}, 2);
    REQUIRE(nds.cpu9().state().pc == kBase + 8u);
}

static void b_max_negative_offset_wraps() {
    NDS nds;
    // B with offset24 = 0x800000 (most negative): target = pc + 8 +
    // (sign_extend(0x800000) << 2) = kBase + 8 + 0xFE00'0000 (mod 2^32),
    // ARM-aligned. Encoding 0xEA80'0000. We run exactly one step and only
    // check the computed pc (the target itself is unmapped).
    load_and_run(nds, {0xEA80'0000u}, 1);
    const u32 expected_pc = (kBase + 8u + 0xFE00'0000u) & ~0x3u;
    REQUIRE(nds.cpu9().state().pc == expected_pc);
}

static void bl_links_return_address_and_jumps() {
    NDS nds;
    // BL +8: R14 = kBase + 4 (instruction after the BL), pc = kBase + 16.
    // Encoding: cond=AL, 101, L=1, offset24=2 → 0xEB00'0002.
    load_and_run(nds, {0xEB00'0002u}, 1);
    REQUIRE(nds.cpu9().state().r[14] == kBase + 4u);
    REQUIRE(nds.cpu9().state().pc == kBase + 16u);
}

static void bl_then_mov_pc_lr_returns() {
    NDS nds;
    //   kBase + 0x00: BL +4          ; EB00'0001 (target = kBase + 12)
    //   kBase + 0x04: MOV R1, #0x11  ; E3A0'1011 — skipped by the BL
    //   kBase + 0x08: <hole>
    //   kBase + 0x0C: MOV PC, LR     ; E1A0'F00E
    // After 2 steps (BL, then MOV PC,LR) pc returns to kBase + 4.
    load_and_run(nds,
                 {0xEB00'0001u,  // BL +4
                  0xE3A0'1011u,  // MOV R1, #0x11 (skipped)
                  0x0000'0000u,  // hole
                  0xE1A0'F00Eu}, // MOV PC, LR
                 2);
    REQUIRE(nds.cpu9().state().r[14] == kBase + 4u);
    REQUIRE(nds.cpu9().state().r[1] == 0u); // MOV R1 was skipped
    REQUIRE(nds.cpu9().state().pc == kBase + 4u);
}

int main() {
    b_forward_takes_branch();
    b_backward_takes_branch();
    b_condition_fail_does_not_branch();
    b_max_negative_offset_wraps();
    bl_links_return_address_and_jumps();
    bl_then_mov_pc_lr_returns();
    std::puts("arm9_branch_test OK");
    return 0;
}
