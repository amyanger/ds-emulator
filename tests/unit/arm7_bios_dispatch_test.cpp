// arm7_bios_dispatch_test.cpp — slice 3e commit 4. Verifies the ARM7 BIOS
// HLE dispatcher's return contract: after a stub SWI, control must resume at
// the instruction following the SWI with the pre-entry CPSR restored from
// SPSR_svc (mode, T, I, F all restored). This mirrors the "MOVS PC, R14"
// that real BIOS code would execute at the end of the handler; direct-boot
// has no BIOS ROM, so the dispatcher runs it implicitly.
//
// arm7_exception_sequence_test already proves enter_exception + MOVS PC, R14
// end-to-end via the IRQ path. This test targets the SWI-specific piece:
// that the dispatcher's return logic lands correctly for the warn-stub cases
// landed in commit 4 (SoftReset 0x00, decompressor 0x10–0x15, and the
// catch-all default for any other number).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kArmBase = 0x02000000u;
constexpr u32 kThumbBase = 0x02000000u;

// Drive the ARM7 through exactly one ARM SWI — entry costs 3 cycles, and
// the dispatcher's implicit MOVS PC, R14 runs inline with no further cycle
// cost. run_until takes an ARM9-cycle target; ARM7 runs at half rate, hence
// the *2. Mirrors arm7_exception_swi_arm_test's helper.
static void run_one_arm_swi(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 3u) * 2u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 3u);
}

// Same for a Thumb SWI (16-bit opcode, R15 reads as pc + 4 during execute).
static void run_one_thumb_swi(NDS& nds, u32 pc, u16 instr) {
    nds.arm7_bus().write16(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 4;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 3u) * 2u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 3u);
}

// Prepare the ARM7 for an ARM-state SWI executed from the given caller mode.
// Caller mode is set via switch_mode + a clean CPSR (I=0, F=0, T=0, NZCV=0)
// so the restored-CPSR assertions are anchored to a known value.
static u32 seed_arm_caller(NDS& nds, Mode caller_mode) {
    auto& state = nds.cpu7().state();
    state.switch_mode(caller_mode);
    state.cpsr = (state.cpsr & ~0x1Fu) | static_cast<u32>(caller_mode);
    // Clear I, F, T and NZCV — leave only the mode bits.
    state.cpsr &= 0x1Fu;
    return state.cpsr;
}

} // namespace

// Case 1: ARM-state SWI 0x00 (SoftReset warn-stub). After the dispatcher
// returns, PC must be at instr_addr + 4, and CPSR must be restored from
// SPSR_svc (User mode, I=0, T=0).
static void arm_swi_0x00_returns_to_caller() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000000u); // SWI #0x00

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE((state.cpsr & (1u << 7)) == 0u); // I restored to 0
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T restored to 0 (ARM)
    REQUIRE(state.pc == kArmBase + 4u);
}

// Case 2: Thumb-state SWI 0x00. Thumb return_addr = instr_addr + 2, and the
// restored CPSR must have T=1 (we came from Thumb).
static void thumb_swi_0x00_returns_to_caller() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // User mode + T=1, everything else clear.
    state.switch_mode(Mode::User);
    state.cpsr = static_cast<u32>(Mode::User) | (1u << 5);
    const u32 pre_cpsr = state.cpsr;

    run_one_thumb_swi(nds, kThumbBase, 0xDF00u); // Thumb SWI #0x00

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE((state.cpsr & (1u << 5)) != 0u); // T restored to 1
    REQUIRE((state.cpsr & (1u << 7)) == 0u); // I restored to 0
    REQUIRE(state.pc == kThumbBase + 2u);
}

// Case 3: ARM-state SWI 0x10 (decompressor stub, first of 0x10–0x15 range).
// Same return contract as SoftReset.
static void arm_swi_0x10_decompressor_stub() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000010u); // SWI #0x10

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// Case 4: ARM-state SWI 0x15 (upper bound of the decompressor stub range).
static void arm_swi_0x15_decompressor_stub() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000015u); // SWI #0x15

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// Case 5: ARM-state SWI 0x01 — invalid, hits the `default` warn path.
static void arm_swi_0x01_invalid_default() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000001u); // SWI #0x01

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// Case 6: ARM-state SWI 0x02 — invalid, hits the `default` warn path.
static void arm_swi_0x02_invalid_default() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000002u); // SWI #0x02

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// Case 7: Thumb-state SWI 0x0A — invalid, hits the `default` warn path.
static void thumb_swi_0x0a_invalid_default() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // User mode + T=1, everything else clear.
    state.switch_mode(Mode::User);
    state.cpsr = static_cast<u32>(Mode::User) | (1u << 5);
    const u32 pre_cpsr = state.cpsr;

    run_one_thumb_swi(nds, kThumbBase, 0xDF0Au); // Thumb SWI #0x0A

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE((state.cpsr & (1u << 5)) != 0u); // T restored to 1
    REQUIRE(state.pc == kThumbBase + 2u);
}

// Case 8: stubs don't touch user-visible general-purpose registers R0–R12.
// Seed distinct sentinels before the SWI, assert they survive the round
// trip. SWI 0x01 is chosen (hits default — representative of every stub).
//
// Note: R13 and R14 are banked. During SVC entry R13_svc / R14_svc become
// visible; when the dispatcher restores to User mode the User-bank R13/R14
// are reloaded, so those cannot participate in the sentinel check.
static void swi_stubs_preserve_r0_through_r12() {
    NDS nds;
    auto& state = nds.cpu7().state();

    seed_arm_caller(nds, Mode::User);
    for (u32 i = 0; i <= 12; ++i) {
        state.r[i] = 0xDEADBEEFu + i;
    }

    run_one_arm_swi(nds, kArmBase, 0xEF000001u);

    for (u32 i = 0; i <= 12; ++i) {
        REQUIRE(state.r[i] == 0xDEADBEEFu + i);
    }
}

// Case 9: SWI from System mode (no SPSR_sys — SPSR_svc is used because SWI
// entry swaps into SVC). The restore path must walk through SPSR_svc, so
// the caller mode round-trips correctly even when it is not User.
static void arm_swi_from_system_mode_returns_to_system() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::System);
    run_one_arm_swi(nds, kArmBase, 0xEF000001u);

    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

int main() {
    arm_swi_0x00_returns_to_caller();
    thumb_swi_0x00_returns_to_caller();
    arm_swi_0x10_decompressor_stub();
    arm_swi_0x15_decompressor_stub();
    arm_swi_0x01_invalid_default();
    arm_swi_0x02_invalid_default();
    thumb_swi_0x0a_invalid_default();
    swi_stubs_preserve_r0_through_r12();
    arm_swi_from_system_mode_returns_to_system();
    std::puts("arm7_bios_dispatch_test: all 9 cases passed");
    return 0;
}
