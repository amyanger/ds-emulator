#pragma once

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm9/arm9.hpp"
#include "ds/common.hpp"
#include "scheduler/event.hpp"
#include "scheduler/scheduler.hpp"

namespace ds {

class NDS {
public:
    NDS();

    void run_frame();
    void reset();

    Scheduler& scheduler() { return scheduler_; }
    Arm9&      cpu9()      { return cpu9_; }
    Arm7&      cpu7()      { return cpu7_; }

private:
    void on_scheduler_event(const Event& ev);

    Scheduler scheduler_;
    Arm9      cpu9_;
    Arm7      cpu7_;
    bool      frame_done_ = false;
};

}  // namespace ds
