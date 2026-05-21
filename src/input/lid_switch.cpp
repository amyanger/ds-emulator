#include "input/lid_switch.hpp"

#include "interrupt/irq_controller.hpp"

namespace ds {

void LidSwitch::reset() {
    extkeyin_ = 0x007Fu;
    prev_unfolded_ = true;
}

void LidSwitch::set_closed(bool closed, IrqController& irq7) {
    // Only bit 7 is owned here; X/Y/DEBUG/pen come from other host paths.
    extkeyin_ = static_cast<u16>((extkeyin_ & ~kHingeBit) | (closed ? kHingeBit : 0u));

    const bool unfolded = (extkeyin_ & kHingeBit) == 0u;
    // IF.22 raises on the rising edge of "unfolded" (1->0 of bit 7). The hinge
    // has no source-side enable: the raise is unconditional on the edge, and
    // IE.22 gates only delivery. NDS7-only, so irq9 is never touched here.
    if (unfolded && !prev_unfolded_) {
        irq7.raise(1u << 22);
    }
    prev_unfolded_ = unfolded;
}

u16 LidSwitch::read_extkeyin() const {
    return extkeyin_;
}

bool LidSwitch::prev_unfolded() const {
    return prev_unfolded_;
}

} // namespace ds
