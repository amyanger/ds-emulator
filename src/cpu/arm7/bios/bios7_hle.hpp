#pragma once

// ARM7 BIOS HLE — stub in slice 3d (warn-per-number), real jump table lands
// in a later slice. Called immediately after enter_exception(Swi, ...) has
// swapped the CPU into Supervisor mode, so this executes "as if" real BIOS
// code at 0x00000008 had dispatched to the handler.

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// Dispatch the SWI identified by `swi_number` (low 8 bits meaningful for
// every DS SWI in use; bits above that are ignored by real BIOS). Slice 3d
// stub warn-logs and returns 0 cycles consumed — the game's handler return
// (MOVS PC, R14) handles state restoration via the commit-1 DP fix.
u32 arm7_bios_hle_dispatch_swi(Arm7State& state, Arm7Bus& bus, u32 swi_number);

} // namespace ds
