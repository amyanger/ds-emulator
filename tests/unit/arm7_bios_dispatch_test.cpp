// arm7_bios_dispatch_test.cpp — slice 3e commit 4, extended in slice 3f
// commit 1. Verifies the ARM7 BIOS HLE dispatcher's return contract: after a
// stub or scaffold SWI, control must resume at the instruction following the
// SWI with the pre-entry CPSR restored from SPSR_svc (mode, T, I, F all
// restored). This mirrors the "MOVS PC, R14" that real BIOS code would
// execute at the end of the handler; direct-boot has no BIOS ROM, so the
// dispatcher runs it implicitly.
//
// arm7_exception_sequence_test already proves enter_exception + MOVS PC, R14
// end-to-end via the IRQ path. This test targets the SWI-specific piece:
// that the dispatcher's return logic lands correctly for SoftReset 0x00, the
// catch-all default for invalid SWIs, and each of the six per-SWI decompressor
// stubs/impls landed in slice 3f commit 1 (0x10–0x15 — three ReadNormal
// scaffolds and three ReadByCallback warn-stubs).

#include "arm7_step.hpp"
#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_tables.hpp"
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

// ARM-state SWI 0x00 (SoftReset warn-stub). After the dispatcher
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

// Thumb-state SWI 0x00. Thumb return_addr = instr_addr + 2, and the
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

// ARM-state SWI 0x10 — BitUnPack scaffold (slice 3f commit 1).
// Returns 1 cycle with R0/CPSR untouched; commit 2 fills in the algorithm.
static void arm_swi_0x10_bit_unpack_scaffold() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000010u); // SWI #0x10

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x11 — LZ77UnComp(Wram) scaffold.
static void arm_swi_0x11_lz77_wram_scaffold() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000011u); // SWI #0x11

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x12 — LZ77UnComp(callback,Vram). Real handler
// invokes the trampoline, so cycle cost varies with callback instructions;
// we step-until-return and then verify the dispatcher's MOVS PC, R14 tail
// (PC at instr+4, CPSR restored from SPSR_svc). Minimal callbacks: Open
// returns a header with size=0 so the inner loop never runs; Close = 0.
static void arm_swi_0x12_lz77_callback_real() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    constexpr u32 kOpen = 0x0200'1000u;   // Open: MOV R0, #0x10; BX LR (header=size 0)
    constexpr u32 kStruct = 0x0200'1100u; // 5 × u32 callback struct
    constexpr u32 kDst = 0x0200'1200u;
    bus.write32(kOpen + 0u, 0xE3A0'0010u); // MOV R0, #0x10 (type=1, size=0)
    bus.write32(kOpen + 4u, 0xE12F'FF1Eu); // BX LR
    bus.write32(kStruct + 0x00u, kOpen);
    bus.write32(kStruct + 0x04u, 0u);
    bus.write32(kStruct + 0x08u, 0u);
    bus.write32(kStruct + 0x0Cu, 0u);
    bus.write32(kStruct + 0x10u, 0u);

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    state.r[1] = kDst;
    state.r[3] = kStruct;
    ds::test::run_until_returns(nds, kArmBase, 0xEF000012u, kArmBase + 4u);

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x13 — HuffUnComp(callback). Real handler invokes the
// trampoline (Open + tree fetch via Get_8bit), so cycle cost varies with guest
// callback instructions; step-until-return and verify the dispatcher's
// MOVS PC, R14 tail. Minimal callbacks: Open returns a header with size=0,
// type=2, data_size_bits=8; Get_8bit / Get_32bit return 0; Close=0.
static void arm_swi_0x13_huff_callback_real() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    constexpr u32 kOpen = 0x0200'1000u;   // Open: MOV R0, #0x28; BX LR (type=2, size=0, ds=8)
    constexpr u32 kGet8 = 0x0200'1040u;   // Get_8bit: MOV R0, #0; BX LR
    constexpr u32 kGet32 = 0x0200'1080u;  // Get_32bit: MOV R0, #0; BX LR
    constexpr u32 kStruct = 0x0200'1100u; // 5 × u32 callback struct
    constexpr u32 kDst = 0x0200'1200u;
    bus.write32(kOpen + 0u, 0xE3A0'0028u);  // MOV R0, #0x28 (type=2, ds=8, size=0)
    bus.write32(kOpen + 4u, 0xE12F'FF1Eu);  // BX LR
    bus.write32(kGet8 + 0u, 0xE3A0'0000u);  // MOV R0, #0
    bus.write32(kGet8 + 4u, 0xE12F'FF1Eu);  // BX LR
    bus.write32(kGet32 + 0u, 0xE3A0'0000u); // MOV R0, #0
    bus.write32(kGet32 + 4u, 0xE12F'FF1Eu); // BX LR
    bus.write32(kStruct + 0x00u, kOpen);
    bus.write32(kStruct + 0x04u, 0u);
    bus.write32(kStruct + 0x08u, kGet8);
    bus.write32(kStruct + 0x0Cu, 0u);
    bus.write32(kStruct + 0x10u, kGet32);

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    state.r[1] = kDst;
    state.r[3] = kStruct;
    ds::test::run_until_returns(nds, kArmBase, 0xEF000013u, kArmBase + 4u);

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x14 — RLUnComp(Wram) scaffold.
static void arm_swi_0x14_rl_wram_scaffold() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000014u); // SWI #0x14

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x15 — RLUnComp(callback,Vram). Real handler
// invokes the trampoline, so cycle cost varies with callback instructions;
// we step-until-return and then verify the dispatcher's MOVS PC, R14 tail
// (PC at instr+4, CPSR restored from SPSR_svc). Minimal callbacks: Open
// returns a header with size=0 so the inner loop never runs; Close = 0.
static void arm_swi_0x15_rl_callback_real() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    constexpr u32 kOpen = 0x0200'1000u;   // Open: MOV R0, #0x30; BX LR (header=type 3, size 0)
    constexpr u32 kStruct = 0x0200'1100u; // 5 × u32 callback struct
    constexpr u32 kDst = 0x0200'1200u;
    bus.write32(kOpen + 0u, 0xE3A0'0030u); // MOV R0, #0x30 (type=3, size=0)
    bus.write32(kOpen + 4u, 0xE12F'FF1Eu); // BX LR
    bus.write32(kStruct + 0x00u, kOpen);
    bus.write32(kStruct + 0x04u, 0u);
    bus.write32(kStruct + 0x08u, 0u);
    bus.write32(kStruct + 0x0Cu, 0u);
    bus.write32(kStruct + 0x10u, 0u);

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    state.r[1] = kDst;
    state.r[3] = kStruct;
    ds::test::run_until_returns(nds, kArmBase, 0xEF000015u, kArmBase + 4u);

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x1A — GetSineTable. Dispatcher routes to the real
// handler (slice 3h commit 2). R0 = 32 → 0x5A82 proves the case row landed
// on bios7_get_sine_table; the dispatcher's MOVS PC, R14 tail still applies.
static void arm_swi_0x1a_get_sine_table() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    state.r[0] = 32u;
    run_one_arm_swi(nds, kArmBase, 0xEF00001Au); // SWI #0x1A

    REQUIRE(state.r[0] == 0x5A82u);
    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x1B — GetPitchTable. Dispatcher routes to the real
// handler (slice 3h commit 3). R0 = 64 → pitch_table_lookup(64) proves the
// case row landed on bios7_get_pitch_table; the dispatcher's MOVS PC, R14
// tail still applies.
static void arm_swi_0x1b_get_pitch_table() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    state.r[0] = 64u;
    run_one_arm_swi(nds, kArmBase, 0xEF00001Bu); // SWI #0x1B

    REQUIRE(state.r[0] == pitch_table_lookup(64));
    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x01 — invalid, hits the `default` warn path.
static void arm_swi_0x01_invalid_default() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000001u); // SWI #0x01

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// ARM-state SWI 0x02 — invalid, hits the `default` warn path.
static void arm_swi_0x02_invalid_default() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 pre_cpsr = seed_arm_caller(nds, Mode::User);
    run_one_arm_swi(nds, kArmBase, 0xEF000002u); // SWI #0x02

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.cpsr == pre_cpsr);
    REQUIRE(state.pc == kArmBase + 4u);
}

// Thumb-state SWI 0x0A — invalid, hits the `default` warn path.
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

// Stubs don't touch user-visible general-purpose registers R0–R12.
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

// SWI from System mode (no SPSR_sys — SPSR_svc is used because SWI
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
    arm_swi_0x10_bit_unpack_scaffold();
    arm_swi_0x11_lz77_wram_scaffold();
    arm_swi_0x12_lz77_callback_real();
    arm_swi_0x13_huff_callback_real();
    arm_swi_0x14_rl_wram_scaffold();
    arm_swi_0x15_rl_callback_real();
    arm_swi_0x1a_get_sine_table();
    arm_swi_0x1b_get_pitch_table();
    arm_swi_0x01_invalid_default();
    arm_swi_0x02_invalid_default();
    thumb_swi_0x0a_invalid_default();
    swi_stubs_preserve_r0_through_r12();
    arm_swi_from_system_mode_returns_to_system();
    std::puts("arm7_bios_dispatch_test: all 15 cases passed");
    return 0;
}
