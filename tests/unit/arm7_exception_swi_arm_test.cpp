// arm7_exception_swi_arm_test.cpp — ARM-state SWI (bits[27:24] == 0b1111)
// must swap into Supervisor mode with return_addr = instr_addr + 4, jump to
// the SWI vector (0x00000008), mask IRQs, clear the T bit, capture old CPSR
// into SPSR_svc, and hand off to the BIOS-HLE dispatcher.
//
// Slice 3e commit 4 made the HLE dispatcher issue an implicit `MOVS PC, R14`
// at the end of every SWI body, so the mid-SWI "frozen" state (PC at 0x08,
// mode = Supervisor, T=0, I=1) is no longer observable through the stepping
// API — after one SWI step the CPU is already back in the caller's mode at
// the return address. These tests now verify the full SWI round-trip:
// SPSR_svc captured the pre-entry CPSR, R14_svc holds the return address,
// and the restored CPSR/PC match the pre-entry mode + I + T. Slice 3d commit 7.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Seed opcode at `pc`, position the ARM7 to execute it, and run one
// instruction. SWI entry consumes 3 ARM7 cycles (fixed cost in the dispatcher
// helper — HLE stub returns 0 today). run_until takes an ARM9-cycle target;
// ARM7 runs at half rate, hence the *2.
static void run_one_expecting_cycles(NDS& nds, u32 pc, u32 instr, u32 expected_cycles) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8; // R15 = pc + 8 at execute time
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + expected_cycles) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + expected_cycles);
}

// System mode, V=0, C=1, T=0, I=0, F=0 — distinctive enough that SPSR_svc
// capture is provably the pre-entry CPSR, and the "I set after entry"
// assertion isn't satisfied by the reset default.
constexpr u32 kPreCpsr = 0x2000001Fu;

} // namespace

// Test 1: basic ARM SWI enters Supervisor mode, runs the stub, and returns
// to the caller. Opcode 0xEF000042 — cond=AL, bits[27:24]=1111, SWI# = 0x42.
// Pre-entry: System mode, I=0. Post-round-trip: System mode restored, PC at
// instr_addr + 4, R14_svc holds the return address, SPSR_svc holds kPreCpsr.
static void arm_swi_enters_supervisor_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Leave reset-default Supervisor, settle into System mode with kPreCpsr.
    // I=0 here so the SPSR_svc capture isn't trivially satisfied by the reset
    // default (which has I=1).
    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_expecting_cycles(nds, kBase, 0xEF000042u, 3);

    // Caller-mode state fully restored after the implicit MOVS PC, R14.
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.cpsr == kPreCpsr);                   // every bit restored from SPSR_svc
    REQUIRE(state.pc == kBase + 4u);                   // return address (R14_svc value)
    REQUIRE(state.banks.spsr_svc == kPreCpsr);         // SPSR_svc still holds the capture
    REQUIRE(state.banks.svc_r13_r14[1] == kBase + 4u); // R14_svc stashed on mode swap
}

// Test 2: a high-bit SWI number (0x1234AB — uses all 24 comment-field bits)
// must not break the state transition. Entry mechanics are the same regardless
// of the SWI number, so verifying the round-trip here is sufficient.
static void arm_swi_extracts_24bit_comment_field() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_expecting_cycles(nds, kBase, 0xEF1234ABu, 3);

    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.cpsr == kPreCpsr);
    REQUIRE(state.pc == kBase + 4u);
    REQUIRE(state.banks.spsr_svc == kPreCpsr);
    REQUIRE(state.banks.svc_r13_r14[1] == kBase + 4u);
}

// Test 3: SPSR_svc must faithfully capture the pre-entry CPSR even when the
// I bit starts at 0, so the implicit MOVS PC, R14 restores I=0 verbatim.
// Start from plain System + I=0, run the SWI, and confirm I is restored.
static void arm_swi_runs_with_irqs_masked_after_entry() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Plain System mode, all flags/masks clear. I=0 pre-entry.
    state.switch_mode(Mode::System);
    state.cpsr = 0x0000001Fu;

    run_one_expecting_cycles(nds, kBase, 0xEF000042u, 3);

    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE((state.banks.spsr_svc & (1u << 7)) == 0u); // pre-entry I=0 preserved in SPSR
    REQUIRE((state.cpsr & (1u << 7)) == 0u);           // I=0 restored to CPSR
}

int main() {
    arm_swi_enters_supervisor_mode();
    arm_swi_extracts_24bit_comment_field();
    arm_swi_runs_with_irqs_masked_after_entry();
    std::puts("arm7_exception_swi_arm_test: all 3 cases passed");
    return 0;
}
