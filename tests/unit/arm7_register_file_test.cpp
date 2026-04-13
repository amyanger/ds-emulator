// Banked register behavior for Arm7State. Covers mode switching
// across User/SVC/IRQ/FIQ, preserving non-banked registers, and
// the reset default (SVC, CPSR=0xD3).

#include "cpu/arm7/arm7_state.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void reset_defaults_to_svc_with_irqs_disabled() {
    Arm7State s;
    s.reset();
    REQUIRE(s.cpsr == 0xD3u);
    REQUIRE(s.current_mode() == Mode::Supervisor);
    REQUIRE(s.r[15] == 0u);
}

static void non_banked_registers_are_visible_across_modes() {
    Arm7State s;
    s.reset();
    s.r[0] = 0xAABBCCDDu;
    s.switch_mode(Mode::User);
    REQUIRE(s.r[0] == 0xAABBCCDDu);
    s.switch_mode(Mode::Irq);
    REQUIRE(s.r[0] == 0xAABBCCDDu);
}

static void r13_r14_are_banked_between_svc_and_irq() {
    Arm7State s;
    s.reset();  // starts in SVC
    s.r[13] = 0x0300'7F00u;  // SVC stack
    s.r[14] = 0x0800'1234u;  // SVC lr

    s.switch_mode(Mode::Irq);
    REQUIRE(s.r[13] == 0u);  // IRQ bank initialized to zero
    REQUIRE(s.r[14] == 0u);
    s.r[13] = 0x0380'FF00u;
    s.r[14] = 0x0800'ABCDu;

    s.switch_mode(Mode::Supervisor);
    REQUIRE(s.r[13] == 0x0300'7F00u);  // SVC bank restored
    REQUIRE(s.r[14] == 0x0800'1234u);
}

static void reset_loads_svc_bank_before_first_switch() {
    // Regression for Fix 1: reset() must establish the invariant that
    // r[13..14] reflects banks[current_mode] from cycle zero. After
    // reset(), CPSR says SVC, so r[13]/r[14] must equal banks.svc_r13_r14.
    // Without Fix 1, reset() left r[13]/r[14] zero but never called
    // load_banked_registers, so the asymmetry would bite the moment a
    // later slice gives reset() non-zero defaults: the first switch_mode
    // out of SVC would store stale r[13]/r[14] over the SVC bank values.
    Arm7State s;
    s.reset();
    REQUIRE(s.current_mode() == Mode::Supervisor);
    REQUIRE(s.r[13] == s.banks.svc_r13_r14[0]);
    REQUIRE(s.r[14] == s.banks.svc_r13_r14[1]);

    // Round-trip: write SVC sp/lr through the visible registers (the
    // intended pattern), bounce through IRQ, and verify we come back
    // with the same values — the post-reset invariant is preserved
    // through a mode switch, not just at t=0.
    s.r[13] = 0x0380'FF80u;
    s.r[14] = 0xDEAD'BEEFu;
    s.switch_mode(Mode::Irq);
    s.switch_mode(Mode::Supervisor);
    REQUIRE(s.r[13] == 0x0380'FF80u);
    REQUIRE(s.r[14] == 0xDEAD'BEEFu);
}

static void fiq_banks_r8_through_r14() {
    Arm7State s;
    s.reset();
    // Fill R8..R14 with recognizable values while in SVC mode.
    for (u32 i = 8; i <= 14; ++i) s.r[i] = 0x1000u + i;

    s.switch_mode(Mode::Fiq);
    // FIQ has its own R8..R14 bank, which is fresh (zero).
    for (u32 i = 8; i <= 14; ++i) REQUIRE(s.r[i] == 0u);

    // Stomp them in FIQ mode with different values.
    for (u32 i = 8; i <= 14; ++i) s.r[i] = 0x2000u + i;

    s.switch_mode(Mode::Supervisor);
    // Back in SVC. R8..R12 are shared with the User bank — the SVC writes
    // of 0x1008..0x100C should come back because switching SVC→FIQ saved
    // them to the User bank (SVC uses the User bank for R8..R12).
    for (u32 i = 8; i <= 12; ++i) REQUIRE(s.r[i] == 0x1000u + i);
    // R13/R14 use the SVC-specific bank, restored from 0x100D/0x100E.
    REQUIRE(s.r[13] == 0x100Du);
    REQUIRE(s.r[14] == 0x100Eu);
}

int main() {
    reset_defaults_to_svc_with_irqs_disabled();
    non_banked_registers_are_visible_across_modes();
    r13_r14_are_banked_between_svc_and_irq();
    reset_loads_svc_bank_before_first_switch();
    fiq_banks_r8_through_r14();
    std::puts("arm7_register_file_test OK");
    return 0;
}
