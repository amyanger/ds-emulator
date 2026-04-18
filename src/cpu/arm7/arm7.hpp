#pragma once

#include "cpu/arm7/arm7_state.hpp"
#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7Bus;

class Arm7 : public CpuCore {
public:
    void attach_bus(Arm7Bus& bus) { bus_ = &bus; }

    // Takes an ARM9-cycle target; internally converts to ARM7 cycles (half rate).
    void run_until(Cycle arm9_target) override;
    void reset() override;

    // IRQ line input. Set by NDS after every IRQ-controller state change
    // (IME/IE/IF writes or source raise). Sampled at instruction boundaries
    // in run_until — wired in commit 10. Level-triggered, not edge-triggered:
    // the handler must ack by writing IF to clear the source, or the line
    // stays asserted and re-fires the next boundary.
    void set_irq_line(bool level) { irq_line_ = level; }
    bool irq_line() const { return irq_line_; }

    // HALTCNT halt state. Set true by NDS when the game writes HALTCNT with
    // Power Down Mode 2 (Halt) or 3 (Sleep; treated as halt this slice).
    // Cleared in run_until when halt_wake_pending_ asserts.
    void set_halted(bool halted) { halted_ = halted; }
    bool is_halted() const { return halted_; }

    // Halt-wake signal. Mirrors (IE AND IF) != 0 — no IME gate, because
    // HALTCNT halt wake is IME-don't-care per GBATEK.
    void set_halt_wake_pending(bool pending) { halt_wake_pending_ = pending; }

    // Synthetic abort triggers. No real bus-fault source wires these yet —
    // later slices attach them to the MMU / cart protocol / etc. For now they
    // exist so unit tests can exercise the exception entry machinery.
    void raise_prefetch_abort(u32 instr_addr);
    void raise_data_abort(u32 instr_addr);

    // Synthetic FIQ trigger. DS has no nFIQ input wired, so nothing in the
    // emulator ever invokes this in real code. The path exists because
    // ARMv4T correctness requires a working FIQ vector — tests drive it
    // directly. R14_fiq = state.pc + 4 (ARM-style post-advance).
    void raise_fiq();

    // Internal-state accessors used by decoders (arm7_decode.cpp,
    // arm7_thumb_decode.cpp), the BIOS HLE dispatcher, and unit tests.
    Arm7State& state() { return state_; }
    const Arm7State& state() const { return state_; }
    Arm7Bus& bus() { return *bus_; }

    // Execute exactly one instruction at the current PC, selecting ARM or
    // Thumb by CPSR.T, after sampling the IRQ line at the instruction
    // boundary. Mirrors a single iteration of run_until's inner loop but
    // takes no cycle target — used by the BIOS-HLE trampoline, which steps
    // until a sentinel PC value is reached.
    //
    // Halt state is NOT honoured here — the trampoline must run the guest
    // callback regardless, and real BIOS callbacks never execute while the
    // CPU is halted anyway (halt clears on the IRQ that woke it).
    void step_one_instruction();

private:
    // Fetch one ARM instruction at pc_, advance pc_, set R15 to pc_+4
    // (= instruction_addr + 8), and execute. Defined in arm7_decode.cpp.
    void step_arm();

    // Fetch one Thumb instruction at pc, advance pc by 2, set R15 to pc_+4
    // (= instruction_addr + 4), and execute. Defined in arm7_thumb_decode.cpp.
    void step_thumb();

    Arm7State state_{};
    Arm7Bus* bus_ = nullptr;
    bool irq_line_ = false;
    bool halted_ = false;
    bool halt_wake_pending_ = false;
};

} // namespace ds
