// arm7_exception_undef_arm_test.cpp — ARMv4T ARM7TDMI has no coprocessor
// interface, so CDP / MRC / MCR / LDC / STC encodings (bits[27:25] = 0b110
// or 0b111 with bits[27:24] != 0b1111) must trap to the UND vector. Commit
// 2 already covered enter_exception() in isolation; this test drives the
// decoder through the full NDS + step_arm path to verify dispatch_arm()
// routes these encodings to UND mode with return_addr = instr_addr + 4.
// Slice 3d commit 4.

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
// instruction. Exception entry (UNDEF here) consumes 3 ARM7 cycles per the
// coarse model in enter_exception(), so the run_until target is scaled
// accordingly — run_until takes an ARM9-cycle target (ARM7 runs at half rate).
static void run_one_expecting_cycles(NDS& nds, u32 pc, u32 instr, u32 expected_cycles) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8; // R15 = pc + 8 at execute time
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + expected_cycles) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + expected_cycles);
}

// Pre-entry CPSR for both cases: System mode (M=0x1F) with V=1 and I=0.
// Distinctive so SPSR_und capture is provably the pre-entry value and the
// I-set assertion isn't trivially satisfied by the reset default.
constexpr u32 kPreCpsr = 0x1000001Fu;

} // namespace

// Test 1: CDP encoding (0xEE000010 — CDP p0, 0, c0, c0, c0, 0) enters UND.
// bits[27:25] = 0b111, bits[27:24] = 0b1110 — not SWI, so coproc → UND.
static void cdp_encoding_enters_undef_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Leave reset-default Supervisor, then settle into System mode with a
    // distinctive pre-entry CPSR. I=0 here so the "I set after entry" check
    // proves the helper ran.
    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_expecting_cycles(nds, kBase, 0xEE000010u, 3);

    REQUIRE(state.current_mode() == Mode::Undefined);
    REQUIRE(state.pc == 0x00000004u);   // UND vector
    REQUIRE(state.r[14] == kBase + 4u); // return_addr = instr_addr + 4
    REQUIRE(state.banks.spsr_und == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
}

// Test 2: MRC encoding (0xEE100010 — MRC p0, 0, R0, c0, c0, 0) enters UND.
// bits[27:25] = 0b111, bits[27:24] = 0b1110 — same routing as CDP, but with
// the "read from coprocessor" bit set. Both must UND on ARM7TDMI.
static void mrc_encoding_enters_undef_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::System);
    state.cpsr = kPreCpsr;

    run_one_expecting_cycles(nds, kBase, 0xEE100010u, 3);

    REQUIRE(state.current_mode() == Mode::Undefined);
    REQUIRE(state.pc == 0x00000004u);
    REQUIRE(state.r[14] == kBase + 4u);
    REQUIRE(state.banks.spsr_und == kPreCpsr);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I set
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T clear
}

int main() {
    cdp_encoding_enters_undef_mode();
    mrc_encoding_enters_undef_mode();
    std::puts("arm7_exception_undef_arm_test: all 2 cases passed");
    return 0;
}
