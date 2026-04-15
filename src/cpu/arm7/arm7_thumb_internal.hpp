#pragma once

// arm7_thumb_internal.hpp — private declarations shared between the
// ARM7 Thumb decoder translation units.

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// Top-level Thumb instruction fan-out. Dispatches on bits[15:13] of
// `instr` into one of eight bucket handlers. Returns ARM7 cycles
// consumed (1 per instruction in slice 3c — coarse cost model).
u32 dispatch_thumb(Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr);

} // namespace ds
