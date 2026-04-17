#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x0F — IsDebugger.
u32 bios7_is_debugger(Arm7State& state, Arm7Bus& bus);

// SWI 0x1D — GetBootProcs.
u32 bios7_get_boot_procs(Arm7State& state, Arm7Bus& bus);

} // namespace ds
