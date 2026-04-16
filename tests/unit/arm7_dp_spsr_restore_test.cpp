// arm7_dp_spsr_restore_test.cpp — ARMv4 exception-return path: when a
// data-processing instruction has S=1 and Rd=R15, the CPU copies SPSR of
// the current mode into CPSR (which re-banks R13/R14 for the restored
// mode) instead of updating NZCV from the result. This covers
// MOVS PC, R14 (SWI/UNDEF return) and SUBS PC, R14, #4 (IRQ/FIQ/prefetch
// abort return). In modes with no SPSR (User / System) the behavior is
// UNPREDICTABLE per ARMv4; we choose to leave CPSR untouched (matches
// melonDS). Slice 3d commit 1.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Preload a single instruction word at pc and run one instruction.
// Caller is responsible for having set up mode, banks, and CPSR before
// this is called. Does NOT overwrite state.cpsr beyond whatever step_arm
// does itself.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8; // R15 = pc + 8 at execute time
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1);
}

} // namespace

// Test 1: MOVS PC, R14 from Supervisor returns to the prior (System) mode.
// Reset default is Supervisor mode, so no explicit switch needed. We stash
// a pre-exception CPSR in SPSR_svc and a return address in R14_svc, then
// execute MOVS PC, R14 (0xE1B0F00E). After execute: CPSR == SPSR_svc,
// mode bits flipped to System, and the System bank has been swapped in.
static void movs_pc_r14_from_supervisor_restores_cpsr_and_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // We begin in Supervisor (reset default). Seed a System-bank R13 so
    // we can prove the bank swap happened after the return.
    state.banks.user_r8_r14[5] = 0x5A5A5A5Au; // System R13 (user/system share)
    state.banks.user_r8_r14[6] = 0xB0B0B0B0u; // System R14

    // Pre-exception CPSR: System mode (0x1F), T=0, I=0, F=0, NZCV=0.
    state.banks.spsr_svc = 0x0000001Fu;
    state.r[14] = 0x02000100u; // return address (already aligned)

    run_one(nds, kBase, 0xE1B0F00Eu);

    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.cpsr == 0x0000001Fu);
    REQUIRE(state.pc == 0x02000100u);
    // After switch_mode, visible R13/R14 come from the System (shared-user)
    // bank. R14 is clobbered by the instruction result (the MOV writes R15
    // only), so only R13 is a clean witness.
    REQUIRE(state.r[13] == 0x5A5A5A5Au);
}

// Test 2: SUBS PC, R14, #4 from IRQ returns to the prior (Supervisor)
// mode with the SPSR's flag bits preserved. Switch to IRQ first, seed
// SPSR_irq with SVC mode + V flag (0x10000013), and execute
// 0xE25EF004 (SUB{S} R15, R14, #4). Expected: CPSR == 0x10000013,
// mode == Supervisor, PC == R14 - 4.
static void subs_pc_r14_minus_4_from_irq_restores_cpsr_and_mode() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::Irq);
    // Seed SVC-bank R13 AFTER the switch away from Supervisor, otherwise
    // the store half of switch_mode(Irq) would overwrite this with the
    // visible R13 value at the time of the switch.
    state.banks.svc_r13_r14[0] = 0x03007F00u; // SVC R13
    state.banks.spsr_irq = 0x10000013u;       // V flag set, mode = Supervisor
    state.r[14] = 0x02001000u;                // return address
    // Clear flags in the current (IRQ) CPSR so the V bit we observe can
    // only have come from SPSR_irq via the exception-return copy.
    state.cpsr &= ~0xF0000000u;

    run_one(nds, kBase, 0xE25EF004u);

    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.cpsr == 0x10000013u);
    REQUIRE(state.pc == 0x02000FFCu);
    REQUIRE(state.r[13] == 0x03007F00u); // SVC bank swapped in
}

// Test 3: MOVS PC, R14 from User mode leaves CPSR untouched (User has no
// SPSR; ARMv4 calls this UNPREDICTABLE, we choose no-op to match melonDS).
// PC must still update from the instruction result.
static void movs_pc_r14_from_user_mode_leaves_cpsr_untouched() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u; // User mode, flags clear, I=0, F=0
    state.r[14] = 0x02000040u;

    run_one(nds, kBase, 0xE1B0F00Eu);

    REQUIRE(state.cpsr == 0x00000010u);
    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE(state.pc == 0x02000040u);
}

// Test 4: regression guard — a plain SUBS with Rd != R15 still does the
// normal flag update. SUBS R0, R1, #0 with R1=0 yields result 0 → Z=1.
static void subs_non_r15_still_does_flag_update() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[1] = 0u;

    // SUBS R0, R1, #0 -> 0xE2510000 (opcode=0x2, S=1, Rn=1, Rd=0, imm=0).
    run_one(nds, kBase, 0xE2510000u);

    REQUIRE(state.r[0] == 0u);
    REQUIRE((state.cpsr & (1u << 30)) != 0); // Z set
    // Mode bits unchanged — still Supervisor (reset default).
    REQUIRE(state.current_mode() == Mode::Supervisor);
}

// Test 5: MOV PC, R14 with S=0 is a plain write; no SPSR copy, no flag
// update, no mode change. Only PC changes.
static void mov_pc_r14_without_s_flag_does_not_touch_cpsr() {
    NDS nds;
    auto& state = nds.cpu7().state();

    const u32 cpsr_before = state.cpsr;
    state.r[14] = 0x02000040u;

    run_one(nds, kBase, 0xE1A0F00Eu); // MOV R15, R14 (S=0)

    REQUIRE(state.cpsr == cpsr_before);
    REQUIRE(state.current_mode() == Mode::Supervisor);
    REQUIRE(state.pc == 0x02000040u);
}

// Test 6: MOVS PC, R14 returning to Thumb state clears bit 0 but preserves
// bit 1. This is the BL-then-SWI-then-return path: the SWI handler returns
// with MOVS PC, R14; SPSR_svc has T=1 set, and R14's bit 0 carries the
// "return to Thumb" marker per ARM convention. ARMv4 aligns PC to halfword
// when the new T bit is 1, so bit 1 must survive — only bit 0 is cleared.
static void movs_pc_r14_returning_to_thumb_aligns_halfword() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Begin in Supervisor. Pre-exception CPSR: User mode (0x10), T=1 (0x20).
    state.banks.spsr_svc = 0x00000030u;
    state.r[14] = 0x02000103u; // bit 0 = thumb marker, bit 1 = 1 (must survive)

    run_one(nds, kBase, 0xE1B0F00Eu);

    REQUIRE(state.current_mode() == Mode::User);
    REQUIRE((state.cpsr & (1u << 5)) != 0); // T set
    REQUIRE(state.pc == 0x02000102u);       // bit 1 preserved, bit 0 cleared
}

// Test 7: MOVS PC, R14 returning to ARM state force-aligns to 4 bytes.
// SPSR has T=0, so ARMv4 aligns PC to word (bits 0 and 1 both cleared).
// Proves the alignment branch reacts to the *new* T bit after the CPSR copy.
static void movs_pc_r14_returning_to_arm_aligns_word() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.switch_mode(Mode::Irq);
    state.banks.spsr_irq = 0x0000001Fu; // System mode, T=0
    state.r[14] = 0x02000106u;          // bits 0 and 1 both set — both must clear
    state.cpsr &= ~0xF0000000u;

    run_one(nds, kBase, 0xE1B0F00Eu);

    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE((state.cpsr & (1u << 5)) == 0); // T clear
    REQUIRE(state.pc == 0x02000104u);       // ~0x3 alignment
}

int main() {
    movs_pc_r14_from_supervisor_restores_cpsr_and_mode();
    subs_pc_r14_minus_4_from_irq_restores_cpsr_and_mode();
    movs_pc_r14_from_user_mode_leaves_cpsr_untouched();
    subs_non_r15_still_does_flag_update();
    mov_pc_r14_without_s_flag_does_not_touch_cpsr();
    movs_pc_r14_returning_to_thumb_aligns_halfword();
    movs_pc_r14_returning_to_arm_aligns_word();
    std::puts("arm7_dp_spsr_restore_test: all 7 cases passed");
    return 0;
}
