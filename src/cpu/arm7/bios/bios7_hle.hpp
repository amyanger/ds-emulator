#pragma once

// ARM7 BIOS HLE dispatcher. Called immediately after enter_exception(Swi, ...)
// has swapped the CPU into Supervisor mode — this function executes "as if"
// real BIOS code at 0x00000008 had decoded the SWI number and jumped to the
// appropriate handler.
//
// Dispatch shape (slice 3e commit 4):
//   - Flat `switch (swi_number & 0xFFu)` — later commits convert numbers from
//     "warn-stub default" to real per-family implementations.
//   - Every path returns a cycle count for the SWI body.
//   - BEFORE the function returns, it replicates the `MOVS PC, R14` that real
//     BIOS code would issue at the end of the handler: SPSR_svc is copied
//     back into CPSR (re-banking R13/R14), and R14 (with Thumb/ARM alignment)
//     becomes the new PC. Without this, the CPU would stick at the SWI vector
//     (PC=0x08) forever since there is no real BIOS ROM to execute.
//
// Slice 3e wired in real handlers for every Pokemon-relevant SWI EXCEPT the
// six-entry decompressor family (0x10–0x15). Slice 3f commit 1 splits that
// family into six per-SWI dispatch entries: 0x10/0x11/0x14 route to scaffold
// stubs that commits 2/3/4 replace with real algorithms, while 0x12/0x13/0x15
// route to per-SWI callback warn-stubs deferred to slice 3g. SoftReset (0x00)
// and the catch-all default for invalid SWIs remain warn-stubs.

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// Dispatch the SWI identified by `swi_number` (low 8 bits are meaningful for
// every DS SWI; higher bits are ignored by real BIOS). Returns the cycle
// count consumed by the SWI body (the 3-cycle entry cost is charged by the
// caller in `dispatch_swi_coproc`). After the body runs, the dispatcher
// implicitly executes `MOVS PC, R14` to return to the caller.
u32 arm7_bios_hle_dispatch_swi(Arm7State& state, Arm7Bus& bus, u32 swi_number);

} // namespace ds
