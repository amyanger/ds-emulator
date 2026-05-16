#pragma once

// EXTKEYIN (0x4000136, R, NDS7-only). Bit 7 is the hinge: 0 = open, 1 =
// closed. IF.22 is raised on ARM7 on the rising edge of "unfolded" —
// i.e. the 1→0 transition of bit 7. Closing the lid does NOT raise.

#include "ds/common.hpp"

namespace ds {

class IrqController;

class LidSwitch {
public:
    void reset();

    void set_closed(bool closed, IrqController& irq7);
    u16 read_extkeyin() const;

    bool prev_unfolded() const;

private:
    static constexpr u16 kHingeBit = 1u << 7;

    // Reset 0x007F: bits 0..6 = 1 (X/Y/DEBUG/pen released + GBATEK
    // "unknown / set" bits), bit 7 = 0 (open), bits 8..15 = 0.
    u16 extkeyin_ = 0x007Fu;

    // True at reset so the first set_closed(false) (lid stays open) does
    // not look like a rising edge against a default-zero bool.
    bool prev_unfolded_ = true;
};

} // namespace ds
