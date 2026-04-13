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
    fiq_banks_r8_through_r14();
    std::puts("arm7_register_file_test OK");
    return 0;
}
