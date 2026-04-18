#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x08 — SoundBias.
u32 bios7_sound_bias(Arm7State& state, Arm7Bus& bus);

// SWI 0x0F — IsDebugger.
u32 bios7_is_debugger(Arm7State& state, Arm7Bus& bus);

// SWI 0x1D — GetBootProcs.
u32 bios7_get_boot_procs(Arm7State& state, Arm7Bus& bus);

// SWI 0x1A — GetSineTable.
inline constexpr u32 kSineTableSize = 0x40;
u16 sine_table_lookup(u32 index);
u32 bios7_get_sine_table(Arm7State& state, Arm7Bus& bus);

} // namespace ds
