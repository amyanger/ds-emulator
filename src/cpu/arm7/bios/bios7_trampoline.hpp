#pragma once

// ARM7 BIOS HLE — re-entrant guest-call primitive.
//
// The callback-variant decompressor SWIs (0x12 LZ77 Vram, 0x13 Huffman,
// 0x15 RLE Vram) invoke guest function pointers from inside a C++ HLE
// handler. Real BIOS does this trivially — real BIOS is itself ARM7 code
// running on the CPU, so a guest callback is just a `BX`/`BLX` away. Our
// HLE handlers are C++ and do not execute on the ARM7 core, so we need an
// adapter: save the ARM7 state we care about, set up the callback's
// register environment, step the interpreter until the callback returns
// to a sentinel PC, and capture R0.

#include "ds/common.hpp"

#include <span>

namespace ds {

class Arm7;

// Sentinel PC value — the trampoline writes this to LR before invoking the
// guest callback. When the callback's `BX LR` lands on this address, the
// step loop exits. Chosen to be in the unmapped high address space on
// ARM7 (main RAM ends at 0x023FFFFF, I/O at 0x04FFFFFF, no consumer maps
// above 0x10000000). The trampoline detects the sentinel via PC comparison
// BEFORE any fetch — the address itself is never dereferenced.
inline constexpr u32 kTrampolineSentinelPC = 0xFFFF'FFF0u;

// Maximum guest instructions per trampoline call. Real BIOS callbacks are
// ~3-10 instructions (typical: `LDRB R0, [R0]; BX LR`). A runaway callback
// hits this budget, warns, and returns 0. 1024 is ~50x headroom for the
// longest realistic callback; bump if a real ROM needs more.
inline constexpr u32 kTrampolineStepBudget = 1024u;

// Invokes a guest function pointer synchronously and returns the value
// left in R0 by the callback (or 0 on budget exhaustion / scaffold).
//
//   target_pc  — guest function entry. Low bit selects Thumb (1) vs ARM (0),
//                per the standard ARM interworking convention.
//   args       — four 32-bit values to pass in R0..R3. Unused slots should
//                be zero.
//
// Callbacks follow the AAPCS calling convention: args in R0-R3, return in
// R0, R4-R11 callee-saved, R12 scratch. Callbacks run in SVC mode (matching
// real BIOS, which never mode-switches back to User before invoking a
// callback). IRQs are NOT masked — a nested IRQ during a callback dispatches
// through the normal exception-entry path.
u32 arm7_bios_call_guest(Arm7& cpu, u32 target_pc, std::span<const u32, 4> args);

} // namespace ds
