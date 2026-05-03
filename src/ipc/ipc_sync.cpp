#include "ipc/ipc_sync.hpp"

#include "interrupt/irq_controller.hpp"

namespace ds {

void IpcSync::reset() {
    arm9_ = SideState{};
    arm7_ = SideState{};
}

u16 IpcSync::read(Side side) const {
    const SideState& local = state(side);
    const SideState& remote = state(side == Side::Arm9 ? Side::Arm7 : Side::Arm9);
    u16 value = 0;
    value |= static_cast<u16>(remote.out_lo & 0xFu);       // bits 0-3
    value |= static_cast<u16>((local.out_lo & 0xFu) << 8); // bits 8-11
    if (local.irq_enable) {
        value |= static_cast<u16>(1u << 14); // bit 14
    }
    return value;
}

bool IpcSync::write(Side side, u16 value, [[maybe_unused]] IrqController& remote_irq) {
    SideState& local = state(side);
    local.out_lo = static_cast<u8>((value >> 8) & 0xFu);
    local.irq_enable = ((value >> 14) & 1u) != 0;
    return false;
}

IpcSync::SideState& IpcSync::state(Side side) {
    return side == Side::Arm9 ? arm9_ : arm7_;
}

const IpcSync::SideState& IpcSync::state(Side side) const {
    return side == Side::Arm9 ? arm9_ : arm7_;
}

} // namespace ds
