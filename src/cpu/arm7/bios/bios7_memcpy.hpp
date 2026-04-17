#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// SWI 0x0B — CpuSet.
u32 bios7_cpu_set(Arm7State& state, Arm7Bus& bus);

// SWI 0x0C — CpuFastSet.
u32 bios7_cpu_fast_set(Arm7State& state, Arm7Bus& bus);

// SWI 0x0E — GetCRC16.
u32 bios7_get_crc16(Arm7State& state, Arm7Bus& bus);

} // namespace ds
