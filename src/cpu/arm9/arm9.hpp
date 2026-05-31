#pragma once

#include "cpu/arm9/arm9_state.hpp"
#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm9Bus;

class Arm9 : public CpuCore {
public:
    void attach_bus(Arm9Bus& bus) { bus_ = &bus; }

    // Takes an ARM9-cycle target and runs at full scheduler rate — unlike
    // ARM7, the ARM9 does NOT halve the target. `now` is in ARM9 cycles, so
    // state_.cycles tracks 1:1 with the scheduler.
    void run_until(Cycle arm9_target) override;
    void reset() override;

    // IRQ line input. Set by NDS after every IRQ-controller state change
    // (IME/IE/IF writes or source raise). Sampled at instruction boundaries
    // in run_until. Level-triggered, not edge-triggered: the handler must ack
    // by writing IF to clear the source, or the line stays asserted and
    // re-fires the next boundary.
    void set_irq_line(bool level) { irq_line_ = level; }
    bool irq_line() const { return irq_line_; }

    // Internal-state accessors used by decoders (arm9_decode.cpp), future
    // BIOS-HLE, and unit tests.
    Arm9State& state() { return state_; }
    const Arm9State& state() const { return state_; }
    Arm9Bus& bus() { return *bus_; }

    // Execute exactly one instruction at the current PC, after sampling the
    // IRQ line at the instruction boundary. Mirrors a single iteration of
    // run_until's inner loop but takes no cycle target — used by tests and
    // future BIOS-HLE.
    void step_one_instruction();

private:
    // Fetch one ARM instruction at pc_, advance pc_, set R15 to pc_+8
    // (= instruction_addr + 8), and execute. Defined in arm9_decode.cpp
    // (commit 5). Stubbed this slice — burns one cycle.
    void step_arm();

    // ARM9 Thumb state is not implemented until slice 3n; nothing in 3l sets
    // CPSR.T, so this must never be reached.
    void step_thumb();

    Arm9State state_{};
    Arm9Bus* bus_ = nullptr;
    bool irq_line_ = false;
    // NOTE (3o): halted_/halt_wake_pending_ added with CP15 Wait-For-Interrupt
};

} // namespace ds
