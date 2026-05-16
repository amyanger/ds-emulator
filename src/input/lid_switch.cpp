#include "input/lid_switch.hpp"

#include "interrupt/irq_controller.hpp"

namespace ds {

void LidSwitch::reset() {
    extkeyin_ = 0x007Fu;
    prev_unfolded_ = true;
}

void LidSwitch::set_closed(bool closed, IrqController& /*irq7*/) {
    // Only bit 7 is owned here; X/Y/DEBUG/pen come from other host paths.
    extkeyin_ = static_cast<u16>((extkeyin_ & ~kHingeBit) | (closed ? kHingeBit : 0u));
    prev_unfolded_ = (extkeyin_ & kHingeBit) == 0u;
}

u16 LidSwitch::read_extkeyin() const {
    return extkeyin_;
}

bool LidSwitch::prev_unfolded() const {
    return prev_unfolded_;
}

} // namespace ds
