#pragma once

// ARM processor mode field (CPSR bits[4:0]).
//
// Architecture-neutral: the mode encoding is identical on ARMv4T (ARM7) and
// ARMv5TE (ARM9), so both cores' state structs (Arm7State, Arm9State) share
// this one definition. Keeping it here avoids an ARM9→ARM7 include edge
// (rule 8) and prevents an ODR clash when both state headers are pulled into
// the same translation unit (e.g. the dual-core integration test).

#include "ds/common.hpp"

namespace ds {

enum class Mode : u8 {
    User = 0x10,
    Fiq = 0x11,
    Irq = 0x12,
    Supervisor = 0x13,
    Abort = 0x17,
    Undefined = 0x1B,
    System = 0x1F,
};

} // namespace ds
