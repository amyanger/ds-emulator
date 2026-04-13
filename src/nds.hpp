#pragma once

#include "bus/arm9_bus.hpp"
#include "bus/wram_control.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm9/arm9.hpp"
#include "ds/common.hpp"
#include "scheduler/event.hpp"
#include "scheduler/scheduler.hpp"

#include <array>

namespace ds {

class NDS {
public:
    NDS();

    // Advance the emulator by one video frame via the scheduler event loop.
    void run_frame();

    void reset();

    Scheduler& scheduler() { return scheduler_; }
    Arm9&      cpu9()      { return cpu9_; }
    Arm7&      cpu7()      { return cpu7_; }
    Arm9Bus&   arm9_bus()  { return arm9_bus_; }

    // ARM9 I/O dispatch. All methods are stubs in this slice (return 0 /
    // ignore writes). The real WRAMCNT handler at 0x0400'0247 lands in a
    // later slice-2 task.
    u32  arm9_io_read32 (u32 addr);
    u16  arm9_io_read16 (u32 addr);
    u8   arm9_io_read8  (u32 addr);
    void arm9_io_write32(u32 addr, u32 value);
    void arm9_io_write16(u32 addr, u16 value);
    void arm9_io_write8 (u32 addr, u8  value);

private:
    // Central dispatch for scheduler events — the only place EventKind values
    // map to behavior. NDS owns this switch (scheduler is a pure data structure).
    void on_scheduler_event(const Event& ev);

    Scheduler scheduler_;
    Arm9      cpu9_;
    Arm7      cpu7_;
    bool      frame_done_ = false;

    // Physical memory owned by NDS. Sizes match real hardware; see design
    // spec §4.1. Bus classes (slice 2) hold raw pointers into these arrays.
    static constexpr std::size_t kMainRamBytes    = 4 * 1024 * 1024;  // 4 MB
    static constexpr std::size_t kSharedWramBytes = 32 * 1024;        // 32 KB
    static constexpr std::size_t kArm7WramBytes   = 64 * 1024;        // 64 KB

    std::array<u8, kMainRamBytes>    main_ram_{};
    std::array<u8, kSharedWramBytes> shared_wram_{};
    std::array<u8, kArm7WramBytes>   arm7_wram_{};

    WramControl wram_ctl_{};
    Arm9Bus     arm9_bus_;
};

}  // namespace ds
