// End-to-end ARM7 exception capstone. Drives every piece of the exception
// machinery against a real ARM program executing in main RAM:
//
//   1. Load a small program at 0x02000000 that enables IME, enables source
//      bit 0 (VBlank) in IE, then spins on a NOP loop.
//   2. Plant the handler address at the direct-boot IRQ-vector indirection
//      slot [0x0380FFFC] so arm7_enter_irq() jumps to the handler instead of
//      executing the (non-existent) BIOS thunk at 0x18.
//   3. Run the setup instructions. After they execute, IME and IE are
//      live, the line is still clear (IF == 0), and PC is parked on the NOP.
//   4. Externally raise source bit 0 via the IRQ controller and recompute
//      the line — simulates VBlank firing.
//   5. Step the CPU. The top-of-loop sample in run_until() preempts the
//      pending NOP fetch, enters IRQ mode, and indirects PC to the handler.
//   6. Step the handler instructions. The STR to IF (write-1-clear) acks
//      the source mid-handler; the NDS glue immediately recomputes the
//      line low. The closing SUBS PC, R14, #4 lands in execute_dp_op's
//      S=1 + Rd=R15 path which copies SPSR_irq back into CPSR, returning
//      to System mode with the flags / I-bit / mode bits intact and
//      PC = R14 - 4 (back at the NOP).
//   7. Step once more. Sample is clear (line is low), so step_arm runs the
//      NOP — proving the return resumed normal execution.
//
// Together this exercises: IRQ controller line computation, NDS glue
// pushing the line into Arm7::set_irq_line, run_until top-of-loop sampling,
// arm7_enter_irq direct-boot indirection through [0x0380FFFC], banked
// R14_irq save, SPSR_irq save, IF write-1-clear, and the SPSR -> CPSR
// restore on SUBS PC, R14, #N.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "interrupt/irq_controller.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

// Program layout in main RAM. Three literals follow the setup body so the
// LDR PC-rel loads can pick the I/O register addresses out without us
// hand-stitching MOV / ORR sequences for each constant.
//
//   0x02000000:  LDR  R1, [PC, #0x18]   ; R1 = IO_IME (literal at +0x20)
//   0x02000004:  LDR  R2, [PC, #0x18]   ; R2 = IO_IE  (literal at +0x24)
//   0x02000008:  LDR  R4, [PC, #0x18]   ; R4 = IO_IF  (literal at +0x28)
//   0x0200000C:  MOV  R0, #1            ; source bit 0 = VBlank
//   0x02000010:  STR  R0, [R2]          ; IE  = 1
//   0x02000014:  STR  R0, [R1]          ; IME = 1
//   0x02000018:  MOV  R0, R0            ; loop:  NOP (where the IRQ preempts)
//   0x0200001C:  B    loop              ; back to 0x02000018
//   0x02000020:  .word 0x04000208       ; literal: IO_IME
//   0x02000024:  .word 0x04000210       ; literal: IO_IE
//   0x02000028:  .word 0x04000214       ; literal: IO_IF
//
// Handler at 0x02000040 (kept clear of the literal pool). Uses R3 / R4 so
// the destructive writes don't clobber the setup state we want to inspect
// after return.
//
//   0x02000040:  MOV  R3, #1            ; ack mask (bit 0)
//   0x02000044:  STR  R3, [R4]          ; IF write-1-clear bit 0
//   0x02000048:  SUBS PC, R14, #4       ; return from IRQ
constexpr u32 kProgBase = 0x02000000u;
constexpr u32 kLoopAddr = 0x02000018u;
constexpr u32 kLoopBranch = 0x0200001Cu;
constexpr u32 kHandlerAddr = 0x02000040u;
constexpr u32 kVectorSlot = 0x0380FFFCu; // direct-boot IRQ indirection

// Drive run_until forward by one tick — long enough for either an
// instruction step or an IRQ-entry service to retire (whichever the loop
// picks at this boundary). Caller asserts on observable state to figure
// out which branch fired; the cycle bump is incidental.
static void step_tick(NDS& nds) {
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

static void load_program(NDS& nds) {
    auto& bus = nds.arm7_bus();
    bus.write32(kProgBase + 0x00, 0xE59F1018u); // LDR R1, [PC, #0x18]
    bus.write32(kProgBase + 0x04, 0xE59F2018u); // LDR R2, [PC, #0x18]
    bus.write32(kProgBase + 0x08, 0xE59F4018u); // LDR R4, [PC, #0x18]
    bus.write32(kProgBase + 0x0C, 0xE3A00001u); // MOV R0, #1
    bus.write32(kProgBase + 0x10, 0xE5820000u); // STR R0, [R2]
    bus.write32(kProgBase + 0x14, 0xE5810000u); // STR R0, [R1]
    bus.write32(kLoopAddr, 0xE1A00000u);        // MOV R0, R0 (NOP)
    bus.write32(kLoopBranch, 0xEAFFFFFDu);      // B loop (back to 0x02000018)
    bus.write32(kProgBase + 0x20, IO_IME);
    bus.write32(kProgBase + 0x24, IO_IE);
    bus.write32(kProgBase + 0x28, IO_IF);

    bus.write32(kHandlerAddr + 0x0, 0xE3A03001u); // MOV R3, #1
    bus.write32(kHandlerAddr + 0x4, 0xE5843000u); // STR R3, [R4]
    bus.write32(kHandlerAddr + 0x8, 0xE25EF004u); // SUBS PC, R14, #4

    bus.write32(kVectorSlot, kHandlerAddr);
}

} // namespace

// The capstone: drive the whole exception path with a real program and
// assert at every meaningful hand-off.
static void irq_round_trip_through_real_program() {
    NDS nds;
    auto& state = nds.cpu7().state();

    load_program(nds);

    // Run in System mode (no SPSR, full I/O privilege) with IRQ unmasked.
    // Reset puts us in Supervisor — this is the explicit pre-IRQ mode that
    // we will check we returned to after the handler exits. NZCV is seeded
    // to a distinctive nonzero pattern (N=1, C=1) so the SPSR-save and
    // CPSR-restore assertions are not satisfied trivially by an all-zero
    // flag word.
    state.switch_mode(Mode::System);
    state.cpsr = 0xA000001Fu; // System, NZCV = N+C, T=0, I=0, F=0
    state.pc = kProgBase;

    // Phase A — execute the setup body (3x LDR, MOV, 2x STR). Step until
    // PC parks on the NOP at kLoopAddr instead of hard-coding the count,
    // so a later edit to load_program doesn't silently desync the test.
    // The safety bound is generous — setup is ~6 instructions; any value
    // >> that would mean the program ran off the rails and we want to
    // bail loudly rather than spin forever inside the NOP loop.
    for (int safety = 0; safety < 64 && state.pc != kLoopAddr; ++safety) {
        step_tick(nds);
    }

    REQUIRE(state.pc == kLoopAddr);
    REQUIRE(state.r[1] == IO_IME);
    REQUIRE(state.r[2] == IO_IE);
    REQUIRE(state.r[4] == IO_IF);
    REQUIRE(state.r[0] == 1u);
    REQUIRE(nds.irq7().read_ime() == 1u);
    REQUIRE(nds.irq7().read_ie() == 1u);
    REQUIRE(nds.irq7().read_if() == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Phase B — externally raise source bit 0 (VBlank). raise() updates IF
    // but does not by itself recompute the line; bumping IME via the I/O
    // path forces NDS::update_arm7_irq_line(), the same mechanism the
    // existing IRQ unit test relies on.
    nds.irq7().raise(0x1u);
    nds.arm7_io_write32(IO_IME, 0x1u);

    REQUIRE(nds.irq7().read_if() == 1u);
    REQUIRE(nds.cpu7().irq_line() == true);

    // Phase C — top-of-loop sample preempts the NOP fetch and enters IRQ.
    // R14_irq must hold "ARM-style return = state.pc + 4", PC indirects
    // through [0x0380FFFC] to the handler, mode flips to IRQ, and the
    // pre-IRQ CPSR (System, I=0) lands in SPSR_irq verbatim. Snapshot
    // state.pc before stepping so the R14 assertion is anchored to the
    // observed pre-entry PC instead of re-deriving it from kLoopAddr —
    // catches any future regression that advances PC before sampling.
    const u32 pc_at_preempt = state.pc;
    const u32 cpsr_before_irq = state.cpsr;
    step_tick(nds);

    REQUIRE(state.current_mode() == Mode::Irq);
    REQUIRE(state.pc == kHandlerAddr);
    REQUIRE(state.r[14] == pc_at_preempt + 4u);
    REQUIRE(state.banks.spsr_irq == cpsr_before_irq);
    REQUIRE((state.cpsr & (1u << 7)) != 0u); // I masked during handler
    REQUIRE((state.cpsr & (1u << 5)) == 0u); // T cleared (vectors are ARM)
    // Line is still asserted at the controller — IF hasn't been ack'd yet.
    // The CPU just doesn't act on it because CPSR.I is now 1.
    REQUIRE(nds.cpu7().irq_line() == true);

    // Phase D — handler body: MOV R3, #1 then STR R3, [R4]. The STR routes
    // through arm7_io_write32 with addr=IO_IF, which write-1-clears bit 0
    // and immediately recomputes the line. Once that store retires the
    // line drops low even though we are still inside the handler.
    step_tick(nds); // MOV R3, #1
    REQUIRE(state.r[3] == 1u);
    REQUIRE(state.pc == kHandlerAddr + 0x4u);

    step_tick(nds); // STR R3, [R4]  →  IF acked
    REQUIRE(nds.irq7().read_if() == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);
    REQUIRE(state.current_mode() == Mode::Irq); // still in handler
    REQUIRE(state.pc == kHandlerAddr + 0x8u);

    // Phase E — return from IRQ via SUBS PC, R14, #4. The DP-op S=1 +
    // Rd=R15 path copies SPSR_irq into CPSR (re-banking R13/R14 back to
    // the System bank), and (R14_irq - 4) becomes the new PC.
    step_tick(nds); // SUBS PC, R14, #4
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.cpsr == cpsr_before_irq);
    REQUIRE(state.pc == kLoopAddr);

    // Phase F — normal execution resumes. The next sample is clear, so
    // step_arm runs the NOP and PC advances to the B-loop instruction.
    step_tick(nds);
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.pc == kLoopBranch);
    REQUIRE(nds.cpu7().irq_line() == false);
}

int main() {
    irq_round_trip_through_real_program();
    std::puts("arm7_exception_sequence_test: capstone passed");
    return 0;
}
