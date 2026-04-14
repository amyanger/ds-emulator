// ARM7 slice 3b1 capstone: run a small program that uses LDR, DP, STR,
// B, BL, and MOV PC, LR in sequence. Verifies the top-level dispatcher
// routes each family correctly across many instructions.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void branch_load_store_sequence() {
    NDS nds;

    constexpr u32 kBase = 0x0380'0000u;
    constexpr u32 kData = 0x0380'0200u;

    // Program layout (little-endian words at kBase):
    //
    //   0x00: LDR  R0, [R1, #0]   ; R0 = *(R1 + 0)       = 10
    //   0x04: ADD  R0, R0, #5     ; R0 = 15
    //   0x08: STR  R0, [R1, #4]   ; *(R1 + 4) = 15
    //   0x0C: B    +4             ; jump to 0x14 (skip 0x10)
    //   0x10: MOV  R0, #0xFF      ; SKIPPED
    //   0x14: BL   +4             ; r14 = 0x18, jump to 0x1C
    //   0x18: MOV  PC, LR         ; return to caller (test harness will see this)
    //   0x1C: MOV  R2, #0x42      ; leaf body
    //   0x20: MOV  PC, LR         ; leaf return -> 0x18
    //
    // Execution order:
    //   step 1: LDR  R0, [R1, #0]       pc = 0x04
    //   step 2: ADD  R0, R0, #5          pc = 0x08
    //   step 3: STR  R0, [R1, #4]        pc = 0x0C
    //   step 4: B    +4                  pc = 0x14
    //   step 5: BL   +4                  pc = 0x1C, r14 = 0x18
    //   step 6: MOV  R2, #0x42           pc = 0x20
    //   step 7: MOV  PC, LR (from leaf)  pc = 0x18
    //
    // We stop after 7 cycles so step 8 (the outer MOV PC, LR at 0x18)
    // never runs, letting us pin final register state precisely.

    const u32 program[] = {
        0xE591'0000u,   // 0x00: LDR  R0, [R1, #0]
        0xE280'0005u,   // 0x04: ADD  R0, R0, #5
        0xE581'0004u,   // 0x08: STR  R0, [R1, #4]
        0xEA00'0000u,   // 0x0C: B    +0  (pc_target = 0x0C + 8 + 0 = 0x14)
        0xE3A0'00FFu,   // 0x10: MOV  R0, #0xFF  (skipped)
        0xEB00'0000u,   // 0x14: BL   +0  (pc_target = 0x14 + 8 + 0 = 0x1C)
        0xE1A0'F00Eu,   // 0x18: MOV  PC, LR  (outer return; not executed in this test)
        0xE3A0'2042u,   // 0x1C: MOV  R2, #0x42
        0xE1A0'F00Eu,   // 0x20: MOV  PC, LR  (leaf return)
    };
    for (u32 i = 0; i < 9; ++i) {
        nds.arm7_bus().write32(kBase + i * 4, program[i]);
    }

    // Seed R1 -> kData, and seed *(kData + 0) = 10.
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 0, 10u);

    nds.cpu7().state().pc = kBase;
    // 7 ARM7 cycles = 14 ARM9 cycles.
    nds.cpu7().run_until(14);

    REQUIRE(nds.cpu7().state().r[0] == 15u);                  // LDR + ADD
    REQUIRE(nds.cpu7().state().r[1] == kData);                // unchanged
    REQUIRE(nds.cpu7().state().r[2] == 0x42u);                // leaf body ran
    REQUIRE(nds.cpu7().state().r[14] == kBase + 0x18u);       // BL link addr
    REQUIRE(nds.cpu7().state().pc  == kBase + 0x18u);         // after leaf return
    REQUIRE(nds.arm7_bus().read32(kData + 4) == 15u);         // STR landed

    // Bonus sanity: the MOV R0, #0xFF at 0x10 must have been skipped,
    // else R0 would not be 15.
}

int main() {
    branch_load_store_sequence();
    std::puts("arm7_branch_load_sequence_test OK");
    return 0;
}
