// arm7_exception_helper_test.cpp — exercises the shared enter_exception()
// helper for every exception kind except Reset (Reset is handled by
// Arm7State::reset() and does not flow through this helper). For each kind
// we assert the full ARMv4 entry actions: vector, target mode, R14_<new>,
// SPSR_<new>, and CPSR.T / CPSR.I / CPSR.F. Slice 3d commit 2.

#include "cpu/arm7/arm7_exception.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Test 1: UNDEF entry from ARM state (User mode). Starting CPSR has V=1,
// C=1, T=0, I=0, F=0. After entry: mode=Undefined, PC=0x04, R14_und holds
// the caller-supplied return address, SPSR_und captures the old CPSR, T=0,
// I=1, F unchanged (stays 0).
static void enter_undef_from_arm_state() {
    Arm7State state;
    state.reset();

    state.switch_mode(Mode::User);
    state.cpsr = 0x60000010u; // V=1, C=1, User, T=0, I=0, F=0

    enter_exception(state, ExceptionKind::Undef, 0x02001238u);

    REQUIRE(state.current_mode() == Mode::Undefined);
    REQUIRE(state.pc == 0x00000004u);
    REQUIRE(state.r[14] == 0x02001238u);
    REQUIRE(state.banks.spsr_und == 0x60000010u);
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T cleared
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) == 0u); // F unchanged (was 0)
}

// Test 2: SWI entry from User. After entry: mode=Supervisor, PC=0x08,
// R14_svc holds return address, SPSR_svc == old CPSR, I set, F unchanged.
static void enter_swi_from_user() {
    Arm7State state;
    state.reset();

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u; // User, all flags clear, I=0, F=0

    enter_exception(state, ExceptionKind::Swi, 0x02000104u);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.pc == 0x00000008u);
    REQUIRE(state.r[14] == 0x02000104u);
    REQUIRE(state.banks.spsr_svc == 0x00000010u);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) == 0u); // F unchanged (was 0)
}

// Test 3: prefetch abort entry from System. Mode switches to Abort, vector
// 0x0C, R14_abt holds the return address, SPSR_abt captures old CPSR.
static void enter_prefetch_abort() {
    Arm7State state;
    state.reset();

    state.switch_mode(Mode::System);
    state.cpsr = 0x0000001Fu; // System mode

    enter_exception(state, ExceptionKind::PrefetchAbort, 0x02000200u);

    REQUIRE(state.current_mode() == Mode::Abort);
    REQUIRE(state.pc == 0x0000000Cu);
    REQUIRE(state.r[14] == 0x02000200u);
    REQUIRE(state.banks.spsr_abt == 0x0000001Fu);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
}

// Test 4: data abort entry from Supervisor with NZCV set. Vector 0x10,
// R14_abt holds the return address (instr_addr + 8 per the spec, though
// this helper takes return_addr directly), SPSR_abt captures old CPSR
// including the flag bits.
static void enter_data_abort() {
    Arm7State state;
    state.reset();

    // Reset default is Supervisor. Set NZCV into CPSR (keep M=Supervisor).
    state.cpsr = 0xF0000013u; // NZCV=1111, Supervisor, I=0, F=0

    enter_exception(state, ExceptionKind::DataAbort, 0x02000300u);

    REQUIRE(state.current_mode() == Mode::Abort);
    REQUIRE(state.pc == 0x00000010u);
    REQUIRE(state.r[14] == 0x02000300u);
    REQUIRE(state.banks.spsr_abt == 0xF0000013u);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
}

// Test 5: IRQ entry does NOT set the F bit — only Reset and FIQ do.
// Starting with both I and F clear, after IRQ entry I=1 but F stays 0.
static void enter_irq_does_not_set_fiq_mask() {
    Arm7State state;
    state.reset();

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u; // User, I=0, F=0

    enter_exception(state, ExceptionKind::Irq, 0x02000404u);

    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == 0x00000018u);
    REQUIRE(state.r[14] == 0x02000404u);
    REQUIRE(state.banks.spsr_irq == 0x00000010u);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) == 0u); // F still clear (IRQ does not raise F)
}

// Test 6: FIQ entry raises BOTH masks (I and F). Starting with both clear,
// after FIQ entry both are set.
static void enter_fiq_sets_both_masks() {
    Arm7State state;
    state.reset();

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u; // User, I=0, F=0

    enter_exception(state, ExceptionKind::Fiq, 0x02000500u);

    REQUIRE(state.current_mode() == Mode::Fiq);
    REQUIRE(state.pc == 0x0000001Cu);
    REQUIRE(state.r[14] == 0x02000500u);
    REQUIRE(state.banks.spsr_fiq == 0x00000010u);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) != 0u); // F set
}

int main() {
    enter_undef_from_arm_state();
    enter_swi_from_user();
    enter_prefetch_abort();
    enter_data_abort();
    enter_irq_does_not_set_fiq_mask();
    enter_fiq_sets_both_masks();
    std::puts("arm7_exception_helper_test: all 6 cases passed");
    return 0;
}
