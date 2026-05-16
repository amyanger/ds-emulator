#include "input/keypad.hpp"

#include "interrupt/irq_controller.hpp"

namespace ds {

void Keypad::reset() {
    keyinput_ = 0x03FFu;
    cnt_[0] = KeyCnt{};
    cnt_[1] = KeyCnt{};
    prev_condition_[0] = false;
    prev_condition_[1] = false;
}

void Keypad::set_keyinput(u16 v, IrqController& irq9, IrqController& irq7) {
    keyinput_ = static_cast<u16>(v & kButtonMask);
    recompute_irqs(irq9, irq7);
}

u16 Keypad::read_keyinput() const {
    return keyinput_;
}

u16 Keypad::read_keycnt(Side side) const {
    const KeyCnt& c = cell(side);
    u16 value = static_cast<u16>(c.select_mask & kButtonMask);
    if (c.enable) {
        value = static_cast<u16>(value | kKeyCntEnable);
    }
    if (c.mode_and) {
        value = static_cast<u16>(value | kKeyCntModeAnd);
    }
    return value;
}

void Keypad::write_keycnt(Side side, u16 v, IrqController& irq9, IrqController& irq7) {
    const u16 masked = static_cast<u16>(v & kKeyCntMask);
    KeyCnt& c = cell(side);
    c.select_mask = static_cast<u16>(masked & kButtonMask);
    c.enable = (masked & kKeyCntEnable) != 0u;
    c.mode_and = (masked & kKeyCntModeAnd) != 0u;
    recompute_irqs(irq9, irq7);
}

bool Keypad::prev_condition(Side side) const {
    return prev_condition_[static_cast<u8>(side)];
}

bool Keypad::compute_condition(Side /*side*/) const {
    return false;
}

void Keypad::recompute_irqs(IrqController& /*irq9*/, IrqController& /*irq7*/) {
    // No-op stub: real body raises IF.12 on each side's rising edge,
    // reading prev_condition_ before storing the new value.
}

} // namespace ds
