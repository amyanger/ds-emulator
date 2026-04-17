// End-to-end slice 3e capstone. A mini ARM7 program that simulates a game's
// boot-wait-for-VBlank pattern, exercising every piece of the slice's plumbing
// against a real program executing in main RAM:
//
//   1. Load a setup body at 0x02000000 that stages I/O addresses into R2/R3/R4
//      and the BIOS shadow address into R5, enables IME and IE=VBlank, primes
//      R0=R1=1, then fires SWI 0x04 (IntrWait).
//   2. The BIOS HLE dispatcher's IntrWait handler forces IME=1, sees the
//      shadow is empty, halts via HALTCNT=0x80, and rewinds R14_svc by 4.
//      The dispatcher's implicit MOVS PC, R14 then lands us back at the SWI
//      instruction itself in System mode — halted, ready to re-execute.
//   3. An external VBlank raise wakes the halt AND enters the IRQ vector on
//      the same run_until iteration (tested in isolation by arm7_halt_test).
//      Execution indirects through [0x0380FFFC] to the game's handler at
//      0x02000080.
//   4. The handler (3 instructions + return) write-1-clears IF bit 0, ORs
//      bit 0 into the BIOS shadow at [0x0380FFF8], and SUBS PC, R14, #4
//      returns to System mode — back at the SWI, which now sees the shadow
//      bit set and consumes it instead of re-halting.
//   5. Execution resumes past the SWI on the post-wait marker (MOV R6, #0x77)
//      and then parks on a self-branch.
//
// Together this exercises: the halt fast-path, halt-wake signal, IRQ entry
// via slice 3d's direct-boot vector indirection, game-side shadow-write
// contract, IntrWait re-execute on halt-wake, dispatcher's implicit return
// (MOVS PC, R14 equivalent), and the full round-trip back to the caller. It
// is the single test that proves slice 3e's BIOS + slice 3d's exception glue
// work together.

#include "bus/arm7_bus.hpp"
#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "interrupt/irq_controller.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

// Program layout in main RAM. Four literals follow the setup body so the
// LDR PC-rel loads can pick the I/O addresses and the BIOS shadow address
// out of the pool without hand-stitching MOV / ORR sequences.
//
//   0x02000000:  LDR  R2, [PC, #0x28]   ; R2 = IO_IME    (literal at +0x30)
//   0x02000004:  LDR  R3, [PC, #0x28]   ; R3 = IO_IE     (literal at +0x34)
//   0x02000008:  LDR  R4, [PC, #0x28]   ; R4 = IO_IF     (literal at +0x38)
//   0x0200000C:  LDR  R5, [PC, #0x28]   ; R5 = 0x0380FFF8 (literal at +0x3C)
//   0x02000010:  MOV  R0, #1
//   0x02000014:  STR  R0, [R2]          ; IME = 1
//   0x02000018:  STR  R0, [R3]          ; IE  = 1 (VBlank)
//   0x0200001C:  MOV  R0, #1            ; R0 = discard-old flag
//   0x02000020:  MOV  R1, #1            ; R1 = mask = VBlank
//   0x02000024:  SWI  #0x04             ; IntrWait — halts on empty shadow
//   0x02000028:  MOV  R6, #0x77         ; post-wait marker
//   0x0200002C:  B    .                 ; self-loop; program parks here
//   0x02000030:  .word IO_IME
//   0x02000034:  .word IO_IE
//   0x02000038:  .word IO_IF
//   0x0200003C:  .word 0x0380FFF8       ; BIOS shadow
//
// Handler at 0x02000080 (clear of the literal pool). Uses R0 as a scratch
// value (R0 is shared across User/System/IRQ modes on ARMv4T) and relies on
// R4/R5 being pre-staged by the setup body with IO_IF and the shadow address.
//
//   0x02000080:  MOV  R0, #1
//   0x02000084:  STR  R0, [R4]          ; IF write-1-clear bit 0
//   0x02000088:  STR  R0, [R5]          ; OR bit 0 into BIOS shadow
//   0x0200008C:  SUBS PC, R14, #4       ; return from IRQ
constexpr u32 kProgBase = 0x02000000u;
constexpr u32 kSwiAddr = 0x02000024u;
constexpr u32 kPostWait = 0x02000028u;
constexpr u32 kSelfLoop = 0x0200002Cu;
constexpr u32 kHandlerAddr = 0x02000080u;
constexpr u32 kVectorSlot = 0x0380FFFCu; // direct-boot IRQ indirection
constexpr u32 kBiosShadow = 0x0380FFF8u; // BIOS "NDS7 IRQ Check Bits"

// Drive run_until forward by one tick — same helper shape as
// arm7_exception_sequence_test.cpp. One tick is big enough for either an
// instruction step or an IRQ-entry / halt-wake service to retire (the loop
// picks whichever is appropriate at the boundary). Assertions on observable
// state are what distinguish the branches.
static void step_tick(NDS& nds) {
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

static void load_program(NDS& nds) {
    auto& bus = nds.arm7_bus();
    bus.write32(kProgBase + 0x00, 0xE59F2028u); // LDR R2, [PC, #0x28]
    bus.write32(kProgBase + 0x04, 0xE59F3028u); // LDR R3, [PC, #0x28]
    bus.write32(kProgBase + 0x08, 0xE59F4028u); // LDR R4, [PC, #0x28]
    bus.write32(kProgBase + 0x0C, 0xE59F5028u); // LDR R5, [PC, #0x28]
    bus.write32(kProgBase + 0x10, 0xE3A00001u); // MOV R0, #1
    bus.write32(kProgBase + 0x14, 0xE5820000u); // STR R0, [R2]  (IME=1)
    bus.write32(kProgBase + 0x18, 0xE5830000u); // STR R0, [R3]  (IE=1)
    bus.write32(kProgBase + 0x1C, 0xE3A00001u); // MOV R0, #1    (discard)
    bus.write32(kProgBase + 0x20, 0xE3A01001u); // MOV R1, #1    (mask)
    bus.write32(kProgBase + 0x24, 0xEF000004u); // SWI #0x04     (IntrWait)
    bus.write32(kProgBase + 0x28, 0xE3A06077u); // MOV R6, #0x77
    bus.write32(kProgBase + 0x2C, 0xEAFFFFFEu); // B .  (self-loop)
    bus.write32(kProgBase + 0x30, IO_IME);
    bus.write32(kProgBase + 0x34, IO_IE);
    bus.write32(kProgBase + 0x38, IO_IF);
    bus.write32(kProgBase + 0x3C, kBiosShadow);

    bus.write32(kHandlerAddr + 0x0, 0xE3A00001u); // MOV R0, #1
    bus.write32(kHandlerAddr + 0x4, 0xE5840000u); // STR R0, [R4]  (ack IF)
    bus.write32(kHandlerAddr + 0x8, 0xE5850000u); // STR R0, [R5]  (OR shadow)
    bus.write32(kHandlerAddr + 0xC, 0xE25EF004u); // SUBS PC, R14, #4

    bus.write32(kVectorSlot, kHandlerAddr);
}

} // namespace

// The capstone: run the whole boot-wait-for-VBlank pattern against a real
// ARM program and assert at every observable hand-off.
static void intrwait_round_trip_through_real_program() {
    NDS nds;
    auto& state = nds.cpu7().state();

    load_program(nds);

    // Run in System mode (no SPSR of its own, full I/O privilege) with IRQ
    // unmasked. NZCV seeded nonzero (N=1, C=1) so the SPSR-save / CPSR-
    // restore assertions in Phase C / Phase E aren't satisfied trivially
    // by an all-zero flag word.
    state.switch_mode(Mode::System);
    state.cpsr = 0xA000001Fu; // System, NZCV = N+C, T=0, I=0, F=0
    state.pc = kProgBase;

    // Phase A — execute the setup body (4x LDR, MOV, 2x STR, 2x MOV) then
    // the SWI 0x04. The IntrWait handler halts the CPU and rewinds R14_svc
    // by 4; the dispatcher's implicit return lands PC back on the SWI
    // itself in System mode. Step until both conditions are true — safety
    // bound is generous (setup + SWI is ~10 instructions). Overshoot would
    // mean the SWI failed to halt and we ran on into the post-wait region,
    // so bail loudly.
    {
        int safety = 0;
        while (safety < 64 && !(state.pc == kSwiAddr && nds.cpu7().is_halted())) {
            step_tick(nds);
            ++safety;
        }
        REQUIRE(safety < 64);
    }

    REQUIRE(state.pc == kSwiAddr);
    REQUIRE(state.r[2] == IO_IME);
    REQUIRE(state.r[3] == IO_IE);
    REQUIRE(state.r[4] == IO_IF);
    REQUIRE(state.r[5] == kBiosShadow);
    REQUIRE(nds.irq7().read_ime() == 1u);
    REQUIRE(nds.irq7().read_ie() == 1u);
    REQUIRE(nds.irq7().read_if() == 0u);
    REQUIRE(nds.cpu7().is_halted() == true);
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(nds.cpu7().irq_line() == false);
    REQUIRE(nds.arm7_bus().read32(kBiosShadow) == 0u);

    // Phase B — halt burns cycles without spuriously waking. With no IRQ
    // raised, is_halted stays true across ticks and PC / shadow don't move.
    // This guards the halt fast-path against a bug where it prematurely
    // clears halted_ without a halt_wake signal.
    for (int i = 0; i < 4; ++i) {
        step_tick(nds);
        REQUIRE(nds.cpu7().is_halted() == true);
        REQUIRE(state.pc == kSwiAddr);
        REQUIRE(nds.arm7_bus().read32(kBiosShadow) == 0u);
    }

    // Phase C — external VBlank fires. raise() sets IF bit 0; the IME write
    // triggers update_arm7_irq_signals which pushes both halt_wake_pending
    // and irq_line into the CPU. With is_halted=true + halt_wake_pending=true
    // + irq_line=true + CPSR.I=0, the next run_until iteration clears halted_
    // AND enters the IRQ vector on the SAME iteration (proven by
    // arm7_halt_test's wake_with_irq_unmasked_enters_vector). So exactly one
    // tick gets us from "halted at SWI" to "in IRQ mode at the handler".
    nds.irq7().raise(0x1u);
    nds.arm7_io_write32(IO_IME, 0x1u);

    REQUIRE(nds.irq7().read_if() == 1u);
    REQUIRE(nds.cpu7().irq_line() == true);

    step_tick(nds);

    REQUIRE(nds.cpu7().is_halted() == false);
    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == kHandlerAddr);
    // R14_irq = pc-at-sample + 4 (ARM-style), and pc-at-sample was kSwiAddr
    // because the dispatcher's implicit return had parked PC on the SWI.
    REQUIRE(state.r[14] == kSwiAddr + 4u);
    REQUIRE(state.banks.spsr_irq == 0xA000001Fu);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I masked during handler

    // Phase D — handler body: MOV R0, #1 then STR R0, [R4] (acks IF) then
    // STR R0, [R5] (ORs bit 0 into the BIOS shadow). The STR to IO_IF routes
    // through arm7_io_write32, write-1-clears IF bit 0, and recomputes the
    // line. Once that store retires the line drops even though we're still
    // in the handler.
    step_tick(nds); // MOV R0, #1
    REQUIRE(state.r[0] == 1u);
    REQUIRE(state.pc == kHandlerAddr + 0x4u);

    step_tick(nds); // STR R0, [R4]  →  IF acked
    REQUIRE(nds.irq7().read_if() == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);
    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == kHandlerAddr + 0x8u);

    step_tick(nds); // STR R0, [R5]  →  shadow bit set
    REQUIRE(nds.arm7_bus().read32(kBiosShadow) == 0x1u);
    REQUIRE(state.pc == kHandlerAddr + 0xCu);

    // Phase E — SUBS PC, R14, #4 returns. The DP-op S=1 + Rd=R15 path copies
    // SPSR_irq back into CPSR (re-banking registers to the System mode),
    // and new PC = R14_irq - 4 = (kSwiAddr + 4) - 4 = kSwiAddr. Execution
    // resumes on the SWI instruction itself.
    step_tick(nds);
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.cpsr == 0xA000001Fu);
    REQUIRE(state.pc == kSwiAddr);

    // Phase F — SWI re-executes. This tick is a single boundary in run_until,
    // but internally it runs: (1) the ARM decoder dispatches SWI, entering
    // SVC with R14_svc = kSwiAddr + 4; (2) the HLE dispatcher calls
    // bios7_intr_wait, which now sees shadow & mask == 1, consumes the bit,
    // and returns without halting or rewinding; (3) the dispatcher's implicit
    // MOVS PC, R14 restores System mode with PC = kSwiAddr + 4 = kPostWait.
    // All in one boundary because the SWI dispatch is synchronous inside
    // step_arm().
    step_tick(nds);
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.pc == kPostWait);
    REQUIRE(nds.arm7_bus().read32(kBiosShadow) == 0u);
    REQUIRE(nds.cpu7().is_halted() == false);

    // Phase G — post-wait code runs: MOV R6, #0x77 then the self-branch. The
    // self-branch holds PC at kSelfLoop across any number of further steps,
    // proving the program successfully returned from the wait.
    step_tick(nds); // MOV R6, #0x77
    REQUIRE(state.r[6] == 0x77u);
    REQUIRE(state.pc == kSelfLoop);

    step_tick(nds); // B .
    REQUIRE(state.pc == kSelfLoop);

    step_tick(nds); // B . again — PC must not drift
    REQUIRE(state.pc == kSelfLoop);
}

int main() {
    intrwait_round_trip_through_real_program();
    std::puts("arm7_bios_sequence_test: capstone passed");
    return 0;
}
