#pragma once

#include "bus/arm7_bus.hpp"
#include "bus/arm9_bus.hpp"
#include "bus/wram_control.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm9/arm9.hpp"
#include "ds/common.hpp"
#include "interrupt/irq_controller.hpp"
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
    Arm9& cpu9() { return cpu9_; }
    Arm7& cpu7() { return cpu7_; }
    Arm9Bus& arm9_bus() { return arm9_bus_; }
    Arm7Bus& arm7_bus() { return arm7_bus_; }

    // Test/debug accessor for the ARM7 IRQ controller. NOT for cross-subsystem
    // use — subsystems must never reach through NDS to pull on another
    // subsystem's state. Use raise()/reg-writes via NDS glue instead.
    Arm7IrqController& irq7() { return irq7_ctrl_; }

    // ARM9 I/O dispatch. Only arm9_io_write8 handles a real register in
    // this slice — WRAMCNT at 0x0400'0247. All other I/O reads return 0
    // and writes are ignored until later slices wire more registers.
    u32 arm9_io_read32(u32 addr);
    u16 arm9_io_read16(u32 addr);
    u8 arm9_io_read8(u32 addr);
    void arm9_io_write32(u32 addr, u32 value);
    void arm9_io_write16(u32 addr, u16 value);
    void arm9_io_write8(u32 addr, u8 value);

    // ARM7 I/O dispatch. Slice 3d wires IME/IE/IF; other registers remain
    // stubbed until later slices land.
    u32 arm7_io_read32(u32 addr);
    u16 arm7_io_read16(u32 addr);
    u8 arm7_io_read8(u32 addr);
    void arm7_io_write32(u32 addr, u32 value);
    void arm7_io_write16(u32 addr, u16 value);
    void arm7_io_write8(u32 addr, u8 value);

private:
    // Central dispatch for scheduler events — the only place EventKind values
    // map to behavior. NDS owns this switch (scheduler is a pure data structure).
    void on_scheduler_event(const Event& ev);

    // Push both IRQ-derived signals into the CPU: the IRQ line and the
    // halt-wake-pending bit. Called after every write that can change
    // IME/IE/IF and after reset. Lives here (not on the controller) because
    // no subsystem may hold a pointer to another (rule 3).
    void update_arm7_irq_signals();

    Scheduler scheduler_;
    Arm9 cpu9_;
    Arm7 cpu7_;
    bool frame_done_ = false;

    // Physical memory owned by NDS. Sizes match real hardware; see design
    // spec §4.1. Bus classes (slice 2) hold raw pointers into these arrays.
    static constexpr std::size_t kMainRamBytes = 4 * 1024 * 1024; // 4 MB
    static constexpr std::size_t kSharedWramBytes = 32 * 1024;    // 32 KB
    static constexpr std::size_t kArm7WramBytes = 64 * 1024;      // 64 KB

    std::array<u8, kMainRamBytes> main_ram_{};
    std::array<u8, kSharedWramBytes> shared_wram_{};
    std::array<u8, kArm7WramBytes> arm7_wram_{};

    WramControl wram_ctl_{};
    Arm9Bus arm9_bus_;
    Arm7Bus arm7_bus_;
    Arm7IrqController irq7_ctrl_{};
};

} // namespace ds
