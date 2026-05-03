#pragma once

// IPCSYNC (0x4000180) — the lighter of the two IPC channels between the ARM9
// and ARM7. Each side sees a 16-bit register with a 4-bit "data out" field
// that surfaces as the *other* side's "data in", a one-shot "ping the remote
// CPU" bit (bit 13), and a per-side "accept pings from remote" enable bit
// (bit 14). The register can be accessed simultaneously by both CPUs without
// generating waitstates (per GBATEK).
//
// The remote IrqController is passed by reference into write() so the bit-13
// raise path can fire without IpcSync holding a pointer to another subsystem
// (CLAUDE.md rule 3).

#include "ds/common.hpp"

namespace ds {

class IrqController;

class IpcSync {
public:
    enum class Side { Arm9, Arm7 };

    void reset();

    // 16-bit register read assembled per GBATEK §IPCSYNC:
    //   bits 0-3   = remote.out_lo (this side's read of the remote's bits 8-11)
    //   bits 8-11  = local.out_lo
    //   bit  14    = local.irq_enable
    //   all other bits read as zero (bit 13 is write-only one-shot)
    u16 read(Side side) const;

    // 16-bit register write. Stores bits 8-11 (out_lo) and bit 14 (irq_enable).
    // Bit 13 (send IRQ to remote) is gated on remote.irq_enable and raises
    // remote IF.16 — return value reports whether the raise fired.
    bool write(Side side, u16 value, IrqController& remote_irq);

private:
    struct SideState {
        u8 out_lo = 0;
        bool irq_enable = false;
    };

    static constexpr Side other(Side side) { return side == Side::Arm9 ? Side::Arm7 : Side::Arm9; }

    SideState& state(Side side);
    const SideState& state(Side side) const;

    SideState arm9_{};
    SideState arm7_{};
};

} // namespace ds
