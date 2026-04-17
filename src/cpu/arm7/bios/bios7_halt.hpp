#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x03 — WaitByLoop.
u32 bios7_wait_by_loop(Arm7State& state, Arm7Bus& bus);

// SWI 0x04 — IntrWait. R0 = discard-old flag, R1 = mask. Forces IME=1,
// consumes matching bits in [0x0380FFF8] or halts with R14_svc rewound to
// re-execute on wake.
u32 bios7_intr_wait(Arm7State& state, Arm7Bus& bus);

// SWI 0x05 — VBlankIntrWait. Sets R0=1, R1=1, tail-calls IntrWait.
u32 bios7_vblank_intr_wait(Arm7State& state, Arm7Bus& bus);

// SWI 0x06 — Halt. Writes HALTCNT = 0x80.
u32 bios7_halt(Arm7State& state, Arm7Bus& bus);

// SWI 0x07 — Sleep. Writes HALTCNT = 0xC0. Treated as halt this slice.
u32 bios7_sleep(Arm7State& state, Arm7Bus& bus);

// SWI 0x1F — CustomHalt. Writes HALTCNT = (R2 & 0xFF) verbatim.
u32 bios7_custom_halt(Arm7State& state, Arm7Bus& bus);

} // namespace ds
