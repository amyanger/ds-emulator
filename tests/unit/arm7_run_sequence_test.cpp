// Slice 3a capstone: run a multi-instruction sequence end-to-end via
// the real Arm7::run_until against Arm7Bus-backed memory. Proves that
// fetch, decode, execute, flag updates, and condition-code gating all
// agree with each other across consecutive instructions.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void arithmetic_sequence_computes_expected_result() {
    NDS nds;

    const u32 base = 0x0380'0000u;

    // Program:
    //   MOV R0, #5          ; R0 = 5
    //   MOV R1, #3          ; R1 = 3
    //   ADD R2, R0, R1      ; R2 = 8
    //   SUB R3, R2, #1      ; R3 = 7
    //   MOV R4, R3, LSL #2  ; R4 = 28
    const u32 program[] = {
        0xE3A0'0005u,  // MOV R0, #5
        0xE3A0'1003u,  // MOV R1, #3
        0xE080'2001u,  // ADD R2, R0, R1
        0xE242'3001u,  // SUB R3, R2, #1
        0xE1A0'4103u,  // MOV R4, R3, LSL #2
    };
    for (u32 i = 0; i < 5; ++i) {
        nds.arm7_bus().write32(base + i * 4, program[i]);
    }

    nds.cpu7().state().pc = base;

    // Drive Arm7::run_until directly rather than NDS::run_frame — the
    // latter would also spin the ARM9 Phase-0 stub, which is out of
    // scope for this slice-3a integration test.
    // Run exactly 5 ARM7 cycles (= 10 ARM9 cycles).
    nds.cpu7().run_until(10);

    REQUIRE(nds.cpu7().state().r[0] == 5u);
    REQUIRE(nds.cpu7().state().r[1] == 3u);
    REQUIRE(nds.cpu7().state().r[2] == 8u);
    REQUIRE(nds.cpu7().state().r[3] == 7u);
    REQUIRE(nds.cpu7().state().r[4] == 28u);
    REQUIRE(nds.cpu7().state().pc == base + 5 * 4);
    REQUIRE(nds.cpu7().state().cycles == 5u);
}

static void condition_codes_gate_execution_in_sequence() {
    NDS nds;
    const u32 base = 0x0380'0000u;

    // Program:
    //   MOV   R0, #0         ; R0 = 0 (no flag update)
    //   MOVS  R0, #0         ; R0 = 0, Z=1
    //   MOVEQ R1, #42        ; Z=1, taken   -> R1 = 42
    //   MOVNE R2, #99        ; Z=1, skipped -> R2 unchanged
    //   MOVS  R0, #1         ; R0 = 1, Z=0
    //   MOVEQ R3, #77        ; Z=0, skipped -> R3 unchanged
    const u32 program[] = {
        0xE3A0'0000u,  // MOV   R0, #0
        0xE3B0'0000u,  // MOVS  R0, #0
        0x03A0'102Au,  // MOVEQ R1, #42
        0x13A0'2063u,  // MOVNE R2, #99
        0xE3B0'0001u,  // MOVS  R0, #1
        0x03A0'304Du,  // MOVEQ R3, #77
    };
    for (u32 i = 0; i < 6; ++i) {
        nds.arm7_bus().write32(base + i * 4, program[i]);
    }

    nds.cpu7().state().pc = base;

    // Run exactly 6 ARM7 cycles (= 12 ARM9 cycles).
    nds.cpu7().run_until(12);

    REQUIRE(nds.cpu7().state().r[0] == 1u);
    REQUIRE(nds.cpu7().state().r[1] == 42u);
    REQUIRE(nds.cpu7().state().r[2] == 0u);  // MOVNE skipped
    REQUIRE(nds.cpu7().state().r[3] == 0u);  // MOVEQ skipped after Z=0
    REQUIRE(nds.cpu7().state().cycles == 6u);

    // Directly observe the Z flag too, not just inferred from conditional
    // execution: the final MOVS R0, #1 cleared Z, so the assertion below
    // would fail if a regression corrupted CPSR write-through.
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0u);
}

int main() {
    arithmetic_sequence_computes_expected_result();
    condition_codes_gate_execution_in_sequence();
    std::puts("arm7_run_sequence_test OK");
    return 0;
}
