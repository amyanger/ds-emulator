#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x03 — WaitByLoop.
u32 bios7_wait_by_loop(Arm7State& state, Arm7Bus& bus);

} // namespace ds
