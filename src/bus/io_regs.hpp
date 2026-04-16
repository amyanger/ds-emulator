#pragma once

// DS I/O register addresses. Added here as they get wired — slice 3d adds
// IME/IE/IF on the ARM7 side. Keep this header dependency-free so both
// ARM9 and ARM7 bus code can include it cheaply.

#include "ds/common.hpp"

namespace ds {

// Interrupt control (ARM7 and ARM9 mirror — slice 3d wires the ARM7 side).
constexpr u32 IO_IME = 0x04000208u;
constexpr u32 IO_IE = 0x04000210u;
constexpr u32 IO_IF = 0x04000214u;

} // namespace ds
