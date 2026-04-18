#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;

// Shared 32-bit data header for LZ77 (SWI 0x11) and RLE (SWI 0x14). BitUnPack
// (SWI 0x10) uses a different header layout (UnpackInfo struct via R2) and
// does not call this helper.
//
// GBATEK layout:
//   bit  0-3   reserved (must be 0)
//   bit  4-7   compressed type (1 = LZ77, 3 = RLE)
//   bit  8-31  decompressed size in bytes
struct DecompHeader {
    u8 type;
    u32 size;
};

// Reads a single 32-bit word from the bus and splits it into the (type, size)
// pair documented above. The reserved bits 0-3 are ignored — real BIOS does
// not validate them either.
DecompHeader bios7_decomp_parse_header(Arm7Bus& bus, u32 src_addr);

// SWI 0x10 — BitUnPack. Bit-depth expansion with optional zero-bias offset.
// Real implementation lands in slice 3f commit 2; this scaffold warns and
// returns 1 cycle.
u32 bios7_bit_unpack(Arm7State& state, Arm7Bus& bus);

// SWI 0x11 — LZ77UnCompReadNormalWrite8bit. LZ77 decompression with source
// from R0 and 8-bit destination writes. Real implementation lands in slice 3f
// commit 3; this scaffold warns and returns 1 cycle.
u32 bios7_lz77_uncomp_wram(Arm7State& state, Arm7Bus& bus);

// SWI 0x14 — RLUnCompReadNormalWrite8bit. Run-length decompression with
// source from R0 and 8-bit destination writes. Real implementation lands in
// slice 3f commit 4; this scaffold warns and returns 1 cycle.
u32 bios7_rl_uncomp_wram(Arm7State& state, Arm7Bus& bus);

// SWI 0x12 — LZ77UnCompReadByCallbackWrite16bit. Callback-driven LZ77 with
// 16-bit destination writes. Deferred to slice 3g (requires a re-entrant
// interpreter trampoline for the BIOS-calls-guest-code path).
u32 bios7_lz77_callback_stub(Arm7State& state, Arm7Bus& bus);

// SWI 0x13 — HuffUnCompReadByCallback. Callback-driven Huffman decompression.
// Deferred to slice 3g.
u32 bios7_huff_callback_stub(Arm7State& state, Arm7Bus& bus);

// SWI 0x15 — RLUnCompReadByCallbackWrite16bit. Callback-driven RLE with
// 16-bit destination writes. Deferred to slice 3g.
u32 bios7_rl_callback_stub(Arm7State& state, Arm7Bus& bus);

} // namespace ds
