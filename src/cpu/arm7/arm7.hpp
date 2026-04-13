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

    // Test access.
    Arm7State&       state()       { return state_; }
    const Arm7State& state() const { return state_; }

private:
    // Fetch one ARM instruction at pc_, advance pc_, set R15 to pc_+4
    // (= instruction_addr + 8), and execute. Defined in arm7_decode.cpp.
    void step_arm();

    Arm7State state_{};
    Arm7Bus*  bus_ = nullptr;
};

}  // namespace ds
