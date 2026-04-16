// arm7_exception_swi_arm_test.cpp — ARM-state SWI (bits[27:24] == 0b1111)
// must swap into Supervisor mode with return_addr = instr_addr + 4, jump to
// the SWI vector (0x00000008), mask IRQs, clear the T bit, capture old CPSR
// into SPSR_svc, and hand off to the BIOS-HLE dispatcher. The HLE layer is a
// warn-only stub in slice 3d, so these tests verify only the exception-entry
// mechanics — SWI bodies land in a later slice. Slice 3d commit 7.

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

// Test 1: basic ARM SWI enters Supervisor mode with the documented state
// transition. Opcode 0xEF000042 — cond=AL, bits[27:24]=1111, SWI# = 0x42.
static void arm_swi_enters_supervisor_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Leave reset-default Supervisor, settle into System mode with kPreCpsr.
    // I=0 here so the "I set after entry" assertion proves the helper ran.
    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_expecting_cycles(nds, kBase, 0xEF000042u, 3);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.pc == 0x00000008u);   // SWI vector
    REQUIRE(state.r[14] == kBase + 4u); // return_addr = instr_addr + 4
    REQUIRE(state.banks.spsr_svc == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
    // F is carried through from the old CPSR (F=0 in kPreCpsr) — SWI does NOT
    // set the FIQ mask. Confirm that.
    REQUIRE((state.cpsr & (1u << 6)) == 0u);
}

// Test 2: a high-bit SWI number (0x1234AB — uses all 24 comment-field bits)
// must not break the state transition. The stub's warn output isn't asserted
// because we can't capture it portably, but the entry mechanics are the same
// regardless of the SWI number, so verifying those here is sufficient.
static void arm_swi_extracts_24bit_comment_field() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_expecting_cycles(nds, kBase, 0xEF1234ABu, 3);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.pc == 0x00000008u);
    REQUIRE(state.r[14] == kBase + 4u);
    REQUIRE(state.banks.spsr_svc == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
}

// Test 3: even if entry-time CPSR has I=0 (IRQs enabled), post-SWI CPSR
// must have I=1. SPSR_svc captures the original I=0 so MOVS PC, R14 later
// can restore it.
static void arm_swi_runs_with_irqs_masked_after_entry() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Plain System mode, all flags/masks clear. I=0 pre-entry.
    state.switch_mode(Mode::System);
    state.cpsr = 0x0000001Fu;

    run_one_expecting_cycles(nds, kBase, 0xEF000042u, 3);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE((state.cpsr & (1u << 7)) != 0u);           // I set by entry
    REQUIRE((state.banks.spsr_svc & (1u << 7)) == 0u); // pre-entry I=0 preserved
}

int main() {
    arm_swi_enters_supervisor_mode();
    arm_swi_extracts_24bit_comment_field();
    arm_swi_runs_with_irqs_masked_after_entry();
    std::puts("arm7_exception_swi_arm_test: all 3 cases passed");
    return 0;
}
