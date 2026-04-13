#include "nds.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380. Real
// PPU-derived timing lands when the PPU lands.
static constexpr Cycle kFrameCycles = 1'120'380;

NDS::NDS() {
    reset();
}

void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;
}

void NDS::run_frame() {
    frame_done_ = false;
    scheduler_.schedule_in(kFrameCycles, EventKind::FrameEnd);

    while (!frame_done_) {
        const Cycle next = scheduler_.peek_next();
        cpu9_.run_until(next);
        cpu7_.run_until(next);
        scheduler_.set_now(next);

        Event ev;
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

}  // namespace ds
