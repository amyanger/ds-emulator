#pragma once

// Per-CPU interrupt controller: IME, IE, IF plus line computation. Slice 3d
// wires this on the ARM7 side; an identical class will serve ARM9 in a later
// slice. No pointer to CPU or bus — NDS glues them together (rule 3). This
// header has no dependencies on cpu/ or bus/ code (rule 8): the controller
// is pure state plus pure functions of state.

#include "ds/common.hpp"

namespace ds {

class Arm7IrqController {
public:
    void reset() {
        ime_ = 0;
        ie_ = 0;
        if_ = 0;
    }

    void write_ime(u32 value) { ime_ = value & 1u; } // bit 0 only on DS
    void write_ie(u32 value) { ie_ = value; }
    void write_if(u32 value) { if_ &= ~value; } // write-1-clear
    void raise(u32 source_bits) { if_ |= source_bits; }

    u32 read_ime() const { return ime_ & 1u; }
    u32 read_ie() const { return ie_; }
    u32 read_if() const { return if_; }

    // The line asserts iff the master enable is set AND any enabled source
    // is pending. The CPU samples this level at instruction boundaries.
    bool line() const { return (ime_ & 1u) != 0 && (ie_ & if_) != 0; }

    // True iff any enabled interrupt source is pending. Unlike line(), this
    // does NOT gate on IME — the NDS7 HALTCNT halt wakes whenever (IE AND IF)
    // != 0 per GBATEK, regardless of IME or CPSR.I.
    bool halt_wake_pending() const { return (ie_ & if_) != 0; }

private:
    u32 ime_ = 0;
    u32 ie_ = 0;
    u32 if_ = 0;
};

} // namespace ds
