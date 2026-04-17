#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x09 — Div.
u32 bios7_div(Arm7State& state, Arm7Bus& bus);

// SWI 0x0D — Sqrt.
u32 bios7_sqrt(Arm7State& state, Arm7Bus& bus);

} // namespace ds
