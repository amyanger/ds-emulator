#pragma once

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm9/arm9.hpp"
#include "ds/common.hpp"
#include "scheduler/scheduler.hpp"

namespace ds {

// Top-level system. Owns every subsystem. In Phase 0 it has only the
// scheduler and two CPU stubs; subsystems land in subsequent phases.
class NDS {
public:
    NDS();

    // Advance the emulator by one frame. Phase 0 just advances the clock a
    // fixed amount and returns — no CPU execution, no rendering.
    void run_frame();

    void reset();

    Scheduler& scheduler() { return scheduler_; }
    Arm9&      cpu9()      { return cpu9_; }
    Arm7&      cpu7()      { return cpu7_; }

private:
    Scheduler scheduler_;
    Arm9      cpu9_;
    Arm7      cpu7_;
    bool      frame_done_ = false;
};

}  // namespace ds
