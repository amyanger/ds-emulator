#pragma once

#include "ds/common.hpp"

namespace ds {

struct Arm7State;
class Arm7Bus;
class Arm7;

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

// Splits a raw 32-bit header word into the (type, size) pair documented above.
// Used when the header is already in a register (callback variants where
// Open_and_get_32bit returned the word in R0).
DecompHeader bios7_decomp_split_header(u32 hdr_word);

// Reads a single 32-bit word from the bus and splits it into the (type, size)
// pair. Wraps bios7_decomp_split_header for the ReadNormal variants whose
// source is a guest memory pointer.
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

// Callback structure pointed to by entry-R3 for the three ReadByCallback
// decompressor SWIs (0x12 LZ77 Vram, 0x13 Huffman, 0x15 RLE Vram). Per GBATEK:
// five 32-bit function pointers at +0x00/+0x04/+0x08/+0x0C/+0x10. A `close`
// value of zero means "no Close callback" per GBATEK.
struct DecompCallbacks {
    u32 open_and_get_32bit; // +0x00
    u32 close;              // +0x04 (0 = skip)
    u32 get_8bit;           // +0x08
    u32 get_16bit;          // +0x0C (unused by LZ77 / RLE)
    u32 get_32bit;          // +0x10 (Huffman only)
};

// Reads the five callback pointers from the guest's callback structure at
// address `r3`. No validation — real BIOS dereferences as-is.
DecompCallbacks read_decomp_callbacks(Arm7Bus& bus, u32 r3);

// SWI 0x12 — LZ77UnCompReadByCallbackWrite16bit. Callback-driven LZ77 with
// 16-bit destination writes. Reads source bytes via `Get_8bit` callbacks;
// writes output as 16-bit halfwords. Takes `Arm7&` (not just `Arm7State&`)
// because the trampoline steps the interpreter to invoke guest callbacks.
u32 bios7_lz77_uncomp_vram(Arm7& cpu);

// SWI 0x13 — HuffUnCompReadByCallback warn-stub.
u32 bios7_huff_callback_stub(Arm7State& state, Arm7Bus& bus);

// SWI 0x15 — RLUnCompReadByCallbackWrite16bit. Callback-driven RLE with
// 16-bit destination writes. Reads source bytes via `Get_8bit` callbacks;
// writes output as 16-bit halfwords. Takes `Arm7&` (not just `Arm7State&`)
// because the trampoline steps the interpreter to invoke guest callbacks.
u32 bios7_rl_uncomp_vram(Arm7& cpu);

} // namespace ds
