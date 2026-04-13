#include "nds.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380. Real
// PPU-derived timing lands when the PPU lands.
static constexpr Cycle kFrameCycles = 1'120'380;

NDS::NDS()
    : arm9_bus_(*this, main_ram_.data(), shared_wram_.data(), wram_ctl_),
      arm7_bus_(*this, main_ram_.data(), shared_wram_.data(), arm7_wram_.data(), wram_ctl_) {
    reset();
}

void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;

    main_ram_.fill(0);
    shared_wram_.fill(0);
    arm7_wram_.fill(0);

    wram_ctl_.reset();
    arm9_bus_.reset();
    arm7_bus_.reset();
}

void NDS::run_frame() {
    frame_done_ = false;
    scheduler_.schedule_in(kFrameCycles, EventKind::FrameEnd);

    while (!frame_done_) {
        const Cycle next = scheduler_.peek_next();
        if (next == Scheduler::kNoEvent) break;  // heap drained early; shouldn't happen today
        cpu9_.run_until(next);
        cpu7_.run_until(next);
        scheduler_.set_now(next);

        Event ev{};
        while (scheduler_.pop_due(next, ev)) {
            on_scheduler_event(ev);
        }
    }
}

void NDS::on_scheduler_event(const Event& ev) {
    switch (ev.kind) {
        case EventKind::FrameEnd:
            frame_done_ = true;
            break;
    }
}

u32 NDS::arm9_io_read32 (u32 /*addr*/) { return 0; }
u16 NDS::arm9_io_read16 (u32 /*addr*/) { return 0; }
u8  NDS::arm9_io_read8  (u32 /*addr*/) { return 0; }
void NDS::arm9_io_write32(u32 /*addr*/, u32 /*value*/) {}
void NDS::arm9_io_write16(u32 /*addr*/, u16 /*value*/) {}
void NDS::arm9_io_write8 (u32 /*addr*/, u8  /*value*/) {}

u32 NDS::arm7_io_read32 (u32 /*addr*/) { return 0; }
u16 NDS::arm7_io_read16 (u32 /*addr*/) { return 0; }
u8  NDS::arm7_io_read8  (u32 /*addr*/) { return 0; }
void NDS::arm7_io_write32(u32 /*addr*/, u32 /*value*/) {}
void NDS::arm7_io_write16(u32 /*addr*/, u16 /*value*/) {}
void NDS::arm7_io_write8 (u32 /*addr*/, u8  /*value*/) {}

}  // namespace ds
