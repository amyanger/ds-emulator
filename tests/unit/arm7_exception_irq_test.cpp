// arm7_exception_irq_test.cpp — exercises the IRQ entry path that slice 3d
// commit 10 wires in: the dedicated arm7_enter_irq() helper (with the
// direct-boot [0x0380FFFC] vector indirection), plus the top-of-loop
// sampling that Arm7::run_until now performs before each instruction step.
//
// Verified here:
//   - arm7_enter_irq from ARM state: mode/vector-indirection/R14/SPSR/mask
//     bits; state.cycles advances by the entry cost.
//   - arm7_enter_irq from Thumb state: same R14 formula (state.pc + 4 in
//     both states, matching GBATEK "ARM-style format"), CPSR.T cleared.
//   - run_until with the line asserted and CPSR.I=0 enters IRQ instead of
//     executing the instruction at state.pc.
//   - run_until with the line asserted but CPSR.I=1 (masked) does NOT
//     enter IRQ; the instruction at state.pc runs normally.
//   - run_until with the line clear does NOT enter IRQ.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_exception.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// ARM opcode: MOV r0, r0 — the canonical ARMv4 nop. Used as the "normal
// instruction at state.pc" in the sampling tests so step_arm has something
// valid to execute when an IRQ does NOT fire.
constexpr u32 kArmNop = 0xE1A00000u;

// Test 1: arm7_enter_irq from ARM state. Seed [0x0380FFFC] with a handler
// pointer, pre-load state with distinctive flags. Verify the helper enters
// IRQ mode, saves the old CPSR into SPSR_irq, puts state.pc + 4 in R14_irq,
// clears T, sets I, leaves F untouched, and — critically — overrides PC
// with the word at 0x0380FFFC (the direct-boot BIOS-thunk substitute).
static void arm7_enter_irq_from_arm_state() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Place a handler pointer at the BIOS-indirection slot.
    nds.arm7_bus().write32(0x0380FFFCu, 0x037F0000u);

    state.switch_mode(Mode::User);
    state.cpsr = 0x20000010u; // C=1, User, T=0, I=0, F=0
    state.pc = 0x02003000u;
    const u64 cycles_before = state.cycles;

    arm7_enter_irq(state, nds.arm7_bus());

    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == 0x037F0000u);    // indirected, not 0x18
    REQUIRE(state.r[14] == 0x02003004u); // state.pc + 4
    REQUIRE(state.banks.spsr_irq == 0x20000010u);
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T cleared
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) == 0u); // F unchanged (was 0)
    REQUIRE(state.cycles > cycles_before);   // entry advanced the clock
}

// Test 2: arm7_enter_irq from Thumb state. The GBATEK "ARM-style format"
// rule means the return-address formula is state.pc + 4 whether we came
// from ARM or Thumb. The entry itself always leaves CPSR.T = 0 (vectors
// are ARM code).
static void arm7_enter_irq_from_thumb_state() {
    NDS nds;
    auto& state = nds.cpu7().state();

    nds.arm7_bus().write32(0x0380FFFCu, 0x037F1000u);

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000030u; // User, T=1, I=0, F=0
    state.pc = 0x02004000u;

    arm7_enter_irq(state, nds.arm7_bus());

    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == 0x037F1000u);
    REQUIRE(state.r[14] == 0x02004004u);          // same +4 rule in Thumb
    REQUIRE(state.banks.spsr_irq == 0x00000030u); // captures T=1 for return
    REQUIRE((state.cpsr & (1u << 5)) == 0u);      // T cleared on entry
    REQUIRE((state.cpsr & (1u << 7)) != 0u);      // I set
}

// Test 3: run_until samples the IRQ line at the top of the loop. With the
// line asserted and CPSR.I=0, the CPU enters IRQ instead of executing the
// instruction at state.pc. We verify by placing a nop at state.pc that
// would clearly be observed (state.pc would advance past it) if it ran —
// instead state.pc lands at the indirected handler address.
static void run_until_enters_irq_when_line_asserted_and_unmasked() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Indirection target: a bogus but valid ARM7-WRAM address.
    nds.arm7_bus().write32(0x0380FFFCu, 0x037F2000u);

    // Plant a nop in main RAM at the "normal" PC — if IRQ entry fails to
    // pre-empt, step_arm would execute this and advance pc by 4.
    nds.arm7_bus().write32(0x02005000u, kArmNop);

    state.switch_mode(Mode::System);
    state.cpsr = 0x0000001Fu; // System, I=0, F=0, T=0
    state.pc = 0x02005000u;

    // Raise a fully-enabled IRQ source so the NDS glue asserts the line.
    nds.arm7_io_write32(IO_IE, 0x1u);
    nds.arm7_io_write32(IO_IME, 0x1u);
    nds.irq7().raise(0x1u);
    nds.arm7_io_write32(IO_IME, 0x1u); // trigger update_arm7_irq_line
    REQUIRE(nds.cpu7().irq_line() == true);

    // One tick: small arm9-target so only the IRQ-entry iteration runs.
    nds.cpu7().run_until(2);

    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == 0x037F2000u);        // at handler, not past the nop
    REQUIRE(state.r[14] == 0x02005004u);     // return to where pc pointed + 4
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set by entry
}

// Test 4: line asserted but CPSR.I=1 (masked) → no entry. step_arm runs
// the nop, advancing pc by 4. Mode stays System.
static void run_until_ignores_irq_when_cpsr_I_is_set() {
    NDS nds;
    auto& state = nds.cpu7().state();

    nds.arm7_bus().write32(0x0380FFFCu, 0x037F3000u);
    nds.arm7_bus().write32(0x02006000u, kArmNop);

    state.switch_mode(Mode::System);
    state.cpsr = 0x000000DFu; // System, I=1 (masked), F=1, T=0
    state.pc = 0x02006000u;

    nds.arm7_io_write32(IO_IE, 0x1u);
    nds.arm7_io_write32(IO_IME, 0x1u);
    nds.irq7().raise(0x1u);
    nds.arm7_io_write32(IO_IME, 0x1u);
    REQUIRE(nds.cpu7().irq_line() == true);

    nds.cpu7().run_until(2);

    REQUIRE(state.current_mode() == Mode::System); // no mode change
    REQUIRE(state.pc == 0x02006004u);              // nop ran, pc advanced
}

// Test 5: line clear → no entry regardless of CPSR.I. step_arm runs the
// nop as usual.
static void run_until_ignores_irq_when_line_clear() {
    NDS nds;
    auto& state = nds.cpu7().state();

    nds.arm7_bus().write32(0x0380FFFCu, 0x037F4000u);
    nds.arm7_bus().write32(0x02007000u, kArmNop);

    state.switch_mode(Mode::System);
    state.cpsr = 0x0000001Fu; // System, I=0, F=0, T=0
    state.pc = 0x02007000u;

    REQUIRE(nds.cpu7().irq_line() == false); // no IME/IE/IF writes → line clear

    nds.cpu7().run_until(2);

    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.pc == 0x02007004u);
}

int main() {
    arm7_enter_irq_from_arm_state();
    arm7_enter_irq_from_thumb_state();
    run_until_enters_irq_when_line_asserted_and_unmasked();
    run_until_ignores_irq_when_cpsr_I_is_set();
    run_until_ignores_irq_when_line_clear();
    std::puts("arm7_exception_irq_test: all 5 cases passed");
    return 0;
}
