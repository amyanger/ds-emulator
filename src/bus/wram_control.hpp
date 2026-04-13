#pragma once

#include "ds/common.hpp"

namespace ds {

// Wrapper around the WRAMCNT I/O byte at 0x0400'0247. Only the bottom two
// bits are meaningful; the upper six are reserved and read back as zero.
// Writes come from `Arm9Bus::slow_write` via `NDS::arm9_io_write8` — the
// ARM7 bus cannot modify this register (rule 4: I/O routed through the
// bus it was issued from; WRAMCNT is ARM9-only).
class WramControl {
public:
    void reset() { value_ = 0; }

    u8 value() const { return value_; }

    void write(u8 raw) { value_ = raw & 0x3; }

private:
    u8 value_ = 0;
};

}  // namespace ds
