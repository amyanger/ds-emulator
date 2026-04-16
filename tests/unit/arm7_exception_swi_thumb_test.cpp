// arm7_exception_swi_thumb_test.cpp — Thumb-state SWI (THUMB.17, opcode
// 11011111 imm8) must swap into Supervisor mode with return_addr =
// instr_addr + 2 (Thumb SWI return contract per GBATEK / spec §5.3), jump
// to the SWI vector (0x00000008), mask IRQs, clear the T bit, capture old
// CPSR into SPSR_svc, and hand off to the BIOS-HLE dispatcher. The HLE
// layer is a warn-only stub in slice 3d, so these tests verify only the
// exception-entry mechanics. Slice 3d commit 8.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Seed a 16-bit Thumb opcode at `pc`, position the ARM7 to execute it in
// Thumb mode, and run one instruction. SWI entry consumes 3 ARM7 cycles
// (fixed cost in the dispatcher helper — HLE stub returns 0 today).
// run_until takes an ARM9-cycle target; ARM7 runs at half rate, hence *2.
static void run_one_thumb_expecting_cycles(NDS& nds, u32 pc, u16 instr, u32 expected_cycles) {
    nds.arm7_bus().write16(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 4; // Thumb: R15 reads as pc + 4 during execute
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + expected_cycles) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + expected_cycles);
}

// Pre-entry CPSR: System mode (M=0x1F), T=1 (Thumb), V=0, C=1, I=0, F=0.
// Distinctive enough that SPSR_svc capture is provably the pre-entry CPSR
// and the post-entry "I set / T clear" assertions aren't trivially
// satisfied by the reset default.
constexpr u32 kPreCpsr = 0x2000003Fu;

} // namespace

// Test 1: THUMB.17 SWI 0xAB enters Supervisor mode with the documented
// state transition. Opcode 0xDFAB — 11011111 imm8, imm8 = 0xAB.
static void thumb_swi_enters_supervisor_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_thumb_expecting_cycles(nds, kBase, 0xDFABu, 3);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.pc == 0x00000008u);   // SWI vector
    REQUIRE(state.r[14] == kBase + 2u); // Thumb return_addr = instr_addr + 2
    REQUIRE(state.banks.spsr_svc == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear (exceptions enter ARM state)
    // F is carried through from the old CPSR (F=0 in kPreCpsr) — SWI does NOT
    // set the FIQ mask. Confirm that.
    REQUIRE((state.cpsr & (1u << 6)) == 0u);
}

// Test 2: a different SWI number (0x00) must produce an identical state
// transition — the SWI number only matters to the HLE dispatcher, not to
// the entry machinery. Use 0xDF00 (SWI 0).
static void thumb_swi_extracts_8bit_number() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_thumb_expecting_cycles(nds, kBase, 0xDF00u, 3);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.pc == 0x00000008u);
    REQUIRE(state.r[14] == kBase + 2u);
    REQUIRE(state.banks.spsr_svc == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
}

// Test 3: pre-entry CPSR with I=0 (IRQs enabled) must yield post-entry
// CPSR with I=1. SPSR_svc captures the original I=0 so MOVS PC, R14 later
// can restore it. Start from plain System+Thumb, no flags.
static void thumb_swi_sets_i_mask_from_i_zero_start() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // System mode (M=0x1F), T=1, F=0, I=0, NZCV=0.
    state.switch_mode(Mode::System);
    state.cpsr = 0x0000003Fu;

    run_one_thumb_expecting_cycles(nds, kBase, 0xDFABu, 3);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE((state.cpsr & (1u << 7)) != 0u);           // I set by entry
    REQUIRE((state.cpsr & (1u << 5)) == 0u);           // T clear
    REQUIRE((state.banks.spsr_svc & (1u << 7)) == 0u); // pre-entry I=0 preserved
    REQUIRE((state.banks.spsr_svc & (1u << 5)) != 0u); // pre-entry T=1 preserved
}

int main() {
    thumb_swi_enters_supervisor_mode();
    thumb_swi_extracts_8bit_number();
    thumb_swi_sets_i_mask_from_i_zero_start();
    std::puts("arm7_exception_swi_thumb_test: all 3 cases passed");
    return 0;
}
