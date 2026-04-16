// arm7_exception_abort_test.cpp — exercises the synthetic prefetch- and
// data-abort entry points on Arm7. No real bus-fault source wires into
// these yet (later slices attach them to the MMU / cart protocol / etc.),
// so for now the raise_* methods are test-only manual triggers that funnel
// straight through enter_exception. We verify the two abort kinds reach
// Abort mode with the correct vector, return address, SPSR, and CPSR
// mask bits. The +4 vs +8 return-address difference is the documented
// GBATEK quirk that distinguishes prefetch from data abort. Slice 3d commit 6.

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Test 1: prefetch abort from System mode with a distinctive pre-entry
// CPSR (Z=1, V=1, System, T=0, I=0, F=0). After raise_prefetch_abort:
// mode=Abort, PC=0x0C, R14_abt = instr_addr + 4, SPSR_abt captures the
// pre-entry CPSR, T cleared, I set, F unchanged (stays 0).
static void raise_prefetch_abort_enters_abort_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = 0x5000001Fu; // Z=1, V=1, System, T=0, I=0, F=0

    nds.cpu7().raise_prefetch_abort(0x02001234u);

    REQUIRE(state.current_mode() == Mode::Abort);
    REQUIRE(state.pc == 0x0000000Cu);    // prefetch abort vector
    REQUIRE(state.r[14] == 0x02001238u); // instr_addr + 4
    REQUIRE(state.banks.spsr_abt == 0x5000001Fu);
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) == 0u); // F unchanged (was 0)
}

// Test 2: data abort from User mode. Same entry machinery as prefetch
// abort, but R14_abt = instr_addr + 8 (the +8 quirk that lets the handler
// SUBS PC, R14, #8 back to the faulting load/store).
static void raise_data_abort_enters_abort_mode_with_plus_8() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u; // User, all flags clear, I=0, F=0

    nds.cpu7().raise_data_abort(0x02002000u);

    REQUIRE(state.current_mode() == Mode::Abort);
    REQUIRE(state.pc == 0x00000010u);    // data abort vector
    REQUIRE(state.r[14] == 0x02002008u); // instr_addr + 8 (not +4)
    REQUIRE(state.banks.spsr_abt == 0x00000010u);
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 6)) == 0u); // F unchanged (was 0)
}

int main() {
    raise_prefetch_abort_enters_abort_mode();
    raise_data_abort_enters_abort_mode_with_plus_8();
    std::puts("arm7_exception_abort_test: all 2 cases passed");
    return 0;
}
