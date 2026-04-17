#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x03 — WaitByLoop.
u32 bios7_wait_by_loop(Arm7State& state, Arm7Bus& bus);

// SWI 0x06 — Halt. Writes HALTCNT = 0x80.
u32 bios7_halt(Arm7State& state, Arm7Bus& bus);

// SWI 0x07 — Sleep. Writes HALTCNT = 0xC0. Treated as halt this slice.
u32 bios7_sleep(Arm7State& state, Arm7Bus& bus);

// SWI 0x1F — CustomHalt. Writes HALTCNT = (R2 & 0xFF) verbatim.
u32 bios7_custom_halt(Arm7State& state, Arm7Bus& bus);

} // namespace ds
