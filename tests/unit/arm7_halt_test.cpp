// arm7_halt_test.cpp — slice 3e prerequisite infrastructure for ARM7 BIOS
// HLE halt-family SWIs. Verifies the HALTCNT register write path, the
// halted_ fast-path in Arm7::run_until, and the halt-wake signal semantics.
// No SWI implementations are exercised here; those land in later commits.
//
// Verified here:
//   - HALTCNT=0x80 (Power Down Mode 2, Halt) sets cpu7.is_halted().
//   - A halted CPU burns the scheduler quantum without executing any
//     instructions — state.cycles advances to the ARM7 target, state.pc
//     does not change.
//   - Halt wake with IME=0 / CPSR.I=1: halted_ clears but the CPU does
//     NOT enter the IRQ vector. GBATEK: "IME register is don't care" for
//     wake, but the ARMv4T IRQ sample still gates on IME and CPSR.I, so
//     the CPU simply resumes normal stepping.
//   - Halt wake with IME=1 / CPSR.I=0: halted_ clears AND the IRQ sample
//     fires, so the CPU enters the IRQ vector on the same quantum.
//   - HALTCNT mode decode covers all four Power Down Mode values:
//     0 (no function), 1 (GBA mode — no-op on DS), 2 (Halt), 3 (Sleep
//     — treated as halt this slice per spec §3).

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_halt.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// ARM opcode: MOV r0, #0x11. Used in TEST 3 to prove the halted CPU, once
// woken with IRQs masked, resumes normal instruction fetch at its pre-halt
// PC instead of entering the IRQ vector.
constexpr u32 kMovR0Imm11 = 0xE3A00011u;

// TEST 1 — HALTCNT=0x80 sets halted_.
// Power Down Mode 2 (Halt) in bits 6-7 of HALTCNT is the bit pattern games
// use for BIOS Halt SWI. A byte write through the ARM7 bus must route to
// arm7_io_write8 and flip the CPU's halted_ flag.
static void haltcnt_mode2_sets_halted() {
    NDS nds;
    REQUIRE(nds.cpu7().is_halted() == false);

    nds.arm7_bus().write8(IO_HALTCNT, 0x80u);

    REQUIRE(nds.cpu7().is_halted() == true);
}

// TEST 2 — halted CPU burns cycles without stepping.
// Plant a known instruction at the CPU's PC so that if run_until does
// step, we would see pc advance by 4. With halted_=true and no pending
// source, the fast-path short-circuits to arm7_target and returns; pc
// stays put and cycles jumps to the target.
static void halted_cpu_burns_cycles_without_stepping() {
    NDS nds;
    auto& state = nds.cpu7().state();

    constexpr u32 kProgAddr = 0x02000000u;
    nds.arm7_bus().write32(kProgAddr, kMovR0Imm11);

    state.switch_mode(Mode::System);
    state.cpsr = 0x0000001Fu; // System, I=0, F=0, T=0
    state.pc = kProgAddr;

    // Halt via HALTCNT write (production path exercised by TEST 1).
    nds.arm7_bus().write8(IO_HALTCNT, 0x80u);
    REQUIRE(nds.cpu7().is_halted() == true);

    const u32 pc_before = state.pc;
    const u64 cycles_before = state.cycles;
    REQUIRE(cycles_before == 0u);

    // run_until takes an ARM9-cycle target; ARM7 runs at half rate so the
    // internal target is 500. With no pending wake, the fast-path fills the
    // clock out to 500 and returns immediately.
    nds.cpu7().run_until(1000);

    REQUIRE(state.cycles == 500u);
    REQUIRE(state.pc == pc_before);          // no instruction executed
    REQUIRE(nds.cpu7().is_halted() == true); // still halted — no wake
    REQUIRE(state.r[0] == 0u);               // MOV r0, #0x11 did NOT land
}

// TEST 3 — wake with IME=0 / CPSR.I=1 clears halted_ but does NOT enter IRQ.
// GBATEK wake condition is (IE AND IF) != 0 regardless of IME or CPSR.I,
// so halted_ must clear. But the ARMv4T IRQ sample still gates on both,
// so the CPU must simply resume normal stepping at its pre-halt PC.
// Assertion: exactly one ARM instruction executes (pc advances by 4, mode
// unchanged, r0 gets the MOV immediate), proving we took the step_arm path
// and not the arm7_enter_irq path.
static void wake_with_irq_masked_resumes_stepping() {
    NDS nds;
    auto& state = nds.cpu7().state();

    constexpr u32 kProgAddr = 0x02000000u;
    nds.arm7_bus().write32(kProgAddr, kMovR0Imm11);

    // CPSR = System mode, I=1 (IRQ masked), F=1, T=0. Mode bits are
    // captured as cpsr_mode_snap so we can assert the IRQ path didn't
    // flip us to IRQ mode.
    state.switch_mode(Mode::System);
    state.cpsr = 0x000000DFu;
    state.pc = kProgAddr;

    // Halt via HALTCNT.
    nds.arm7_bus().write8(IO_HALTCNT, 0x80u);
    REQUIRE(nds.cpu7().is_halted() == true);

    // Drive the IRQ controller manually. IME stays 0 so the IRQ line is
    // false, but (IE AND IF) != 0, so the halt-wake signal is true.
    nds.irq7().write_ime(0);
    nds.irq7().write_ie(1);
    nds.irq7().raise(1);
    // Push the signals directly — matches what update_arm7_irq_signals
    // would emit: halt_wake_pending=true (IE&IF != 0), irq_line=false (IME=0).
    nds.cpu7().set_halt_wake_pending(true);
    nds.cpu7().set_irq_line(false);

    const u32 pc_snap = state.pc;
    const u32 cpsr_mode_snap = state.cpsr & 0x1Fu;

    // 2 ARM9 cycles → 1 ARM7 cycle budget; the while loop exits as soon as
    // state.cycles hits the ARM7 target, so exactly one instruction retires.
    // (Using a larger budget here would run the MOV and then march PC through
    // zero-filled RAM re-executing AND r0,r0,r0 nops until the budget drains.)
    nds.cpu7().run_until(2);

    REQUIRE(nds.cpu7().is_halted() == false);        // wake cleared the flag
    REQUIRE((state.cpsr & 0x1Fu) == cpsr_mode_snap); // mode UNCHANGED
    REQUIRE(state.pc == pc_snap + 4u);               // exactly one ARM instruction ran
    REQUIRE(state.r[0] == 0x11u);                    // the MOV landed
}

// TEST 4 — wake with IME=1 / CPSR.I=0 clears halted_ AND enters IRQ vector.
// Same halt-wake story as TEST 3, but this time the ARMv4T IRQ sample
// conditions are also satisfied. The fast-path clears halted_, then the
// existing IRQ sample branch fires on the same loop iteration, so we end
// up in IRQ mode at the direct-boot vector indirection target.
static void wake_with_irq_unmasked_enters_vector() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Seed the direct-boot IRQ-vector indirection slot with 0x18. This
    // matches the production IRQ entry path where [0x0380FFFC] normally
    // holds the handler pointer — writing 0x18 here keeps the asserted
    // "pc == 0x18" simple without needing a real handler.
    nds.arm7_bus().write32(0x0380FFFCu, 0x00000018u);

    // CPSR = User mode, I=0 (IRQ unmasked), F=0, T=0.
    state.switch_mode(Mode::User);
    state.cpsr = 0x00000010u;
    state.pc = 0x02000000u;

    // Halt via HALTCNT.
    nds.arm7_bus().write8(IO_HALTCNT, 0x80u);
    REQUIRE(nds.cpu7().is_halted() == true);

    // Program the controller with both IME and IE live, then raise source
    // bit 0 so (IE AND IF) != 0. Push the signals directly so the test is
    // deterministic about what state the CPU sees on the next run_until.
    nds.irq7().write_ime(1);
    nds.irq7().write_ie(1);
    nds.irq7().raise(1);
    nds.cpu7().set_halt_wake_pending(true);
    nds.cpu7().set_irq_line(true);

    // 2 ARM9 cycles → 1 ARM7 cycle: the fast-path clears halted_ and the
    // IRQ sample fires on the same loop iteration, so exactly one entry
    // happens (the entry itself advances state.cycles past the target).
    nds.cpu7().run_until(2);

    REQUIRE(nds.cpu7().is_halted() == false); // wake cleared
    REQUIRE((state.cpsr & 0x1Fu) == 0x12u);   // IRQ mode (0x12)
    REQUIRE(state.pc == 0x18u);               // IRQ vector target after arm7_enter_irq
}

// Helper for TEST 5: apply a HALTCNT byte write to a fresh NDS and return
// the halted state. Kept out-of-line so each invocation gets its own stack
// frame — holding four ~4 MB NDS objects in the same function (even in
// disjoint scopes) blows the 8 MB default macOS main-thread stack under
// ASan's red-zone padding, because the compiler reserves frame space for
// all scopes up-front in the function prologue.
static bool halted_after_haltcnt_write(u8 value) {
    NDS nds;
    nds.arm7_bus().write8(IO_HALTCNT, value);
    return nds.cpu7().is_halted();
}

// TEST 5 — HALTCNT mode decode: all four values.
// Sweep bits 6-7 of HALTCNT: 0x00 (no function), 0x40 (GBA mode —
// unreachable on DS), 0x80 (Halt), 0xC0 (Sleep — treated as halt).
// Expected truth table: {0x00: false, 0x40: false, 0x80: true, 0xC0: true}.
// Each case uses a fresh NDS so halted_ starts false — this catches any
// "mode 1 accidentally sets halted_" regression symmetrically to the
// positive mode-2/3 cases.
static void haltcnt_mode_decode_truth_table() {
    REQUIRE(halted_after_haltcnt_write(0x00u) == false); // mode 0: no function
    REQUIRE(halted_after_haltcnt_write(0x40u) == false); // mode 1: GBA mode no-op
    REQUIRE(halted_after_haltcnt_write(0x80u) == true);  // mode 2: Halt
    REQUIRE(halted_after_haltcnt_write(0xC0u) == true);  // mode 3: Sleep → halt
}

static void wait_by_loop_adds_four_cycles_per_count() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 100;
    const u64 before = state.cycles;
    bios7_wait_by_loop(state, nds.arm7_bus());
    REQUIRE(state.cycles == before + 400ull);
}

static void wait_by_loop_zero_count_is_noop() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0;
    const u64 before = state.cycles;
    bios7_wait_by_loop(state, nds.arm7_bus());
    REQUIRE(state.cycles == before);
}

// 4 * 0x40000000 = 0x100000000 — must fit in u64 without silent 32-bit
// truncation of the intermediate multiplication.
static void wait_by_loop_max_count_no_overflow() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0x40000000u;
    const u64 before = state.cycles;
    bios7_wait_by_loop(state, nds.arm7_bus());
    REQUIRE(state.cycles == before + static_cast<u64>(0x100000000ull));
}

// Hardware's `SUB R0,1 / BGT LOP` exits with R0 == 0.
static void wait_by_loop_clobbers_r0_to_zero() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 100;
    bios7_wait_by_loop(state, nds.arm7_bus());
    REQUIRE(state.r[0] == 0u);
}

int main() {
    haltcnt_mode2_sets_halted();
    halted_cpu_burns_cycles_without_stepping();
    wake_with_irq_masked_resumes_stepping();
    wake_with_irq_unmasked_enters_vector();
    haltcnt_mode_decode_truth_table();
    wait_by_loop_adds_four_cycles_per_count();
    wait_by_loop_zero_count_is_noop();
    wait_by_loop_max_count_no_overflow();
    wait_by_loop_clobbers_r0_to_zero();
    std::puts("arm7_halt_test: all 9 cases passed");
    return 0;
}
