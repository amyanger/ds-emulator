// Arm9State register file, banking, and ARM9 reset defaults. Covers the
// high-vector reset (PC=0xFFFF0000), CPSR=0xD3, mode-banking round-trips, and
// the live Q flag (CPSR bit 27 must round-trip and never be masked).

#include "cpu/arm9/arm9_state.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void reset_defaults_high_vector_svc_irqs_disabled() {
    Arm9State s;
    s.reset();
    REQUIRE(s.pc == 0xFFFF0000u); // high reset vector, NOT 0
    REQUIRE(s.cpsr == 0xD3u);     // SVC, I=1, F=1, T=0, flags clear
    REQUIRE(s.cycles == 0u);
    REQUIRE(s.current_mode() == Mode::Supervisor);
    // Bit 27 (Q) clear out of reset (implied by cpsr=0xD3).
    REQUIRE((s.cpsr & (1u << 27)) == 0u);
}

static void r13_r14_are_banked_between_svc_and_irq() {
    Arm9State s;
    s.reset(); // starts in SVC
    s.r[13] = 0x0300'7F00u;
    s.r[14] = 0x0800'1234u;

    s.switch_mode(Mode::Irq);
    REQUIRE(s.r[13] == 0u); // IRQ bank initialized to zero
    REQUIRE(s.r[14] == 0u);
    s.r[13] = 0x0380'FF00u;
    s.r[14] = 0x0800'ABCDu;

    s.switch_mode(Mode::Supervisor);
    REQUIRE(s.r[13] == 0x0300'7F00u); // SVC bank restored
    REQUIRE(s.r[14] == 0x0800'1234u);
}

static void non_banked_registers_visible_across_modes() {
    Arm9State s;
    s.reset();
    s.r[0] = 0xAABBCCDDu;
    s.switch_mode(Mode::User);
    REQUIRE(s.r[0] == 0xAABBCCDDu);
    s.switch_mode(Mode::Irq);
    REQUIRE(s.r[0] == 0xAABBCCDDu);
}

static void q_bit_round_trips_and_is_not_masked() {
    // CPSR bit 27 is the live sticky-saturation (Q) flag on ARMv5TE. Setting
    // it directly must persist — Arm9State stores it in cpsr and never masks
    // it. A mode switch must not disturb it either (switch_mode only touches
    // the mode field, bits[4:0]).
    Arm9State s;
    s.reset();
    s.cpsr |= (1u << 27);
    REQUIRE((s.cpsr & (1u << 27)) != 0u);

    s.switch_mode(Mode::Irq);
    REQUIRE((s.cpsr & (1u << 27)) != 0u);
    s.switch_mode(Mode::Supervisor);
    REQUIRE((s.cpsr & (1u << 27)) != 0u);
}

int main() {
    reset_defaults_high_vector_svc_irqs_disabled();
    r13_r14_are_banked_between_svc_and_irq();
    non_banked_registers_visible_across_modes();
    q_bit_round_trips_and_is_not_masked();
    std::puts("arm9_state_test OK");
    return 0;
}
