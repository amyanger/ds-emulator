// arm7_exception_undef_thumb_test.cpp — Thumb UNDEF dispatch drives two
// currently-reserved encodings through enter_exception() via the full
// step_thumb path:
//   1) THUMB.16 Bcond with cond == 0xE (reserved "always" code).
//   2) THUMB.19 second halfword with bits[15:11] == 11101 — the ARMv5
//      BLX-label form, which is UNDEF on ARMv4 (ARM7TDMI).
// Return address is instr_addr + 2 for both: step_thumb pre-advances
// state.pc before dispatch, so the Thumb R14_und contract (return addr
// = instruction after the faulting one) is already materialized there.
// Slice 3d commit 5.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Pre-entry CPSR: System mode (M=0x1F) with V=1, T=1, I=0. Distinctive
// enough that SPSR_und capture is provably the pre-entry value and the
// post-entry "I set / T clear" assertions aren't trivially satisfied.
constexpr u32 kPreCpsr = 0x1000003Fu;

// Seed a 16-bit Thumb opcode at `pc`, position the ARM7 to execute it in
// Thumb mode, and run one instruction. Exception entry costs 3 ARM7 cycles
// (coarse model in enter_exception()); run_until takes an ARM9-cycle
// target so we scale by 2.
static void run_one_thumb_expecting_cycles(NDS& nds, u32 pc, u16 instr, u32 expected_cycles) {
    nds.arm7_bus().write16(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 4; // Thumb: R15 reads as pc + 4 during execute
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + expected_cycles) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + expected_cycles);
}

} // namespace

// Test 1: THUMB.16 with cond == 0xE. Encoding 0b1101_1110_XXXX_XXXX.
// 0xDE00 picks cond=0xE with imm8=0x00 — the low 8 bits are don't-care.
static void thumb16_cond_0xe_enters_undef_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_thumb_expecting_cycles(nds, kBase, 0xDE00u, 3);

    REQUIRE(state.current_mode() == Mode::Undefined);
    REQUIRE(state.pc == 0x00000004u);   // UND vector
    REQUIRE(state.r[14] == kBase + 2u); // Thumb return_addr = instr_addr + 2
    REQUIRE(state.banks.spsr_und == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear (exceptions enter ARM state)
}

// Test 2: THUMB.19 second-halfword form with bits[15:11] == 11101 (BLX
// label, ARMv5 only — UNDEF on ARMv4). Encoding 0b1110_1XXX_XXXX_XXXX.
// dispatch_thumb_bl_suffix is reached unconditionally from the 111-space
// bits[12:11]==01 path, so executing the suffix in isolation exercises
// the UNDEF branch without needing to run a paired BL prefix first.
static void thumb19_blx_armv5_encoding_enters_undef_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_thumb_expecting_cycles(nds, kBase, 0xE800u, 3);

    REQUIRE(state.current_mode() == Mode::Undefined);
    REQUIRE(state.pc == 0x00000004u);   // UND vector
    REQUIRE(state.r[14] == kBase + 2u); // return_addr = instr_addr + 2
    REQUIRE(state.banks.spsr_und == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
}

int main() {
    thumb16_cond_0xe_enters_undef_mode();
    thumb19_blx_armv5_encoding_enters_undef_mode();
    std::puts("arm7_exception_undef_thumb_test: all 2 cases passed");
    return 0;
}
