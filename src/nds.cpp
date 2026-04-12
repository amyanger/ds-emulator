#include "nds.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380.
// Phase 0 only uses this as a stand-in "how far to advance the clock."
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
    // Phase 0: drive clock forward; CPUs are no-ops; scheduler has no events.
    const Cycle target = scheduler_.now() + kFrameCycles;
    cpu9_.run_until(target);
    cpu7_.run_until(target);
    scheduler_.advance_to(target);
    frame_done_ = true;
}

}  // namespace ds
