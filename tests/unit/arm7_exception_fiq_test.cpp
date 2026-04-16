// arm7_exception_fiq_test.cpp — exercises the synthetic FIQ entry point on
// Arm7. The DS has no nFIQ input wired, so no real emulator code path ever
// triggers this; the vector exists for ARMv4T correctness. We drive
// raise_fiq() directly and verify:
//   - mode goes to FIQ (CPSR.M = 0x11)
//   - PC = 0x0000001C
//   - R14_fiq = state.pc + 4 (ARM-style post-advance)
//   - SPSR_fiq captures the pre-entry CPSR
//   - CPSR.T cleared, CPSR.I set, CPSR.F set (F is forced only on Reset+FIQ)
//   - R8..R12 swap from the User bank to the FIQ bank (distinguishes FIQ
//     entry from every other exception kind — only FIQ banks R8..R12)
// Slice 3d commit 9.

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Test 1: FIQ from System mode with Z=1, N=1, I=0, F=0, T=0. After entry:
// mode=FIQ, PC=0x1C, R14_fiq = pc_pre + 4, SPSR_fiq captures the old CPSR,
// T=0, I=1, F=1.
static void raise_fiq_enters_fiq_mode_and_sets_masks() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = 0xC000001Fu; // N=1, Z=1, System, T=0, I=0, F=0
    state.pc = 0x02004000u;

    nds.cpu7().raise_fiq();

    REQUIRE(state.current_mode() == Mode::Fiq);
    REQUIRE(state.pc == 0x0000001Cu);    // FIQ vector
    REQUIRE(state.r[14] == 0x02004004u); // state.pc + 4
    REQUIRE(state.banks.spsr_fiq == 0xC000001Fu);
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) != 0u); // F set (FIQ-specific)
}

// Test 2: FIQ banks R8..R12 — the single behavior that distinguishes FIQ
// from every other exception kind. Pre-load the User bank with distinctive
// values in R8..R12, enter FIQ (whose FIQ bank defaults to zero), verify
// the live registers now read zero. Then switch back to User and confirm
// the original values come back — proof that User's R8..R12 were saved to
// the User bank on entry, not clobbered.
static void raise_fiq_banks_r8_to_r12() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u; // User, flags clear, I=0, F=0, T=0
    state.r[8] = 0xAAAAAAAAu;
    state.r[9] = 0xBBBBBBBBu;
    state.r[10] = 0xCCCCCCCCu;
    state.r[11] = 0xDDDDDDDDu;
    state.r[12] = 0xEEEEEEEEu;
    state.pc = 0x02005000u;

    nds.cpu7().raise_fiq();

    // FIQ bank was never written, so R8..R12 should now read 0.
    REQUIRE(state.current_mode() == Mode::Fiq);
    REQUIRE(state.r[8] == 0u);
    REQUIRE(state.r[9] == 0u);
    REQUIRE(state.r[10] == 0u);
    REQUIRE(state.r[11] == 0u);
    REQUIRE(state.r[12] == 0u);

    // Return to User and verify the original R8..R12 come back from the
    // User bank — proof switch_mode saved them on FIQ entry.
    state.switch_mode(Mode::User);
    REQUIRE(state.r[8] == 0xAAAAAAAAu);
    REQUIRE(state.r[9] == 0xBBBBBBBBu);
    REQUIRE(state.r[10] == 0xCCCCCCCCu);
    REQUIRE(state.r[11] == 0xDDDDDDDDu);
    REQUIRE(state.r[12] == 0xEEEEEEEEu);
}

int main() {
    raise_fiq_enters_fiq_mode_and_sets_masks();
    raise_fiq_banks_r8_to_r12();
    std::puts("arm7_exception_fiq_test: all 2 cases passed");
    return 0;
}
