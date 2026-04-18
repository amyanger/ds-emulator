// Slice 3g capstone — 0x12, 0x15, and 0x13 end-to-end through the SWI
// dispatcher, the bios7 trampoline, the ARM7 interpreter, and guest-mode
// ARM-stub callbacks installed in main RAM. Asserts post-SWI mode restore.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_hle.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <array>
#include <cstdio>
#include <initializer_list>

using namespace ds;

namespace {

constexpr u32 kCpsrSvcMode = 0x0000'0013u;
constexpr u32 kSpsrUserMode = 0x0000'0010u;
constexpr u32 kSwiPc = 0x0200'1000u;

constexpr u8 kSwiLz77Vram = 0x12u;
constexpr u8 kSwiHuffman = 0x13u;
constexpr u8 kSwiRleVram = 0x15u;

constexpr u32 kInstrBxLr = 0xE12F'FF1Eu;
constexpr u32 kInstrOpenLdrHandle0 = 0xE590'0000u;
constexpr u32 kInstrOpenLdrHandle1 = 0xE590'0004u;
constexpr u32 kInstrGet8LoadPtr0 = 0xE590'1000u;
constexpr u32 kInstrGet8LoadPtr4 = 0xE590'1004u;
constexpr u32 kInstrGet8LoadByte = 0xE5D1'2000u;
constexpr u32 kInstrGet8AdvancePtr = 0xE281'1001u;
constexpr u32 kInstrGet8StorePtr0 = 0xE580'1000u;
constexpr u32 kInstrGet8StorePtr4 = 0xE580'1004u;
constexpr u32 kInstrMovR0R2 = 0xE1A0'0002u;
constexpr u32 kInstrGet32LoadPtr8 = 0xE590'1008u;
constexpr u32 kInstrGet32LoadWord = 0xE591'2000u;
constexpr u32 kInstrGet32AdvancePtr = 0xE281'1004u;
constexpr u32 kInstrGet32StorePtr = 0xE580'1008u;

static constexpr u32 make_header(u8 type, u32 size) {
    return (static_cast<u32>(type & 0x0Fu) << 4) | (size << 8);
}

static constexpr u32 make_header_huffman(u8 data_size_bits, u8 type, u32 size) {
    return (static_cast<u32>(data_size_bits & 0x0Fu)) | (static_cast<u32>(type & 0x0Fu) << 4) |
           (size << 8);
}

static void enter_arm_swi_state(NDS& nds) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = kCpsrSvcMode;
    state.r[14] = kSwiPc + 4u;
    *state.spsr_slot() = kSpsrUserMode;
}

static void write_bytes(Arm7Bus& bus, u32 addr, std::initializer_list<u8> bytes) {
    u32 offset = 0u;
    for (u8 b : bytes) {
        bus.write8(addr + offset, b);
        ++offset;
    }
}

// 8 raw bytes ("DS:HG/SS") followed by 7 length-8 disp-8 back-references that
// copy the seed seven times. The trailing flag bit is unreached because written
// hits 64 after the seventh back-reference. disp=8 reads bytes already flushed
// past the halfword buffer, so the Vram variant produces the correct plaintext
// (no disp=0 caveat).
static void run_lz77_vram_case() {
    constexpr u32 kCbStruct = 0x0200'0000u;
    constexpr u32 kOpenCb = 0x0200'0020u;
    constexpr u32 kGet8Cb = 0x0200'0040u;
    constexpr u32 kHandle = 0x0200'0080u;
    constexpr u32 kStream = 0x0200'0100u;
    constexpr u32 kDest = 0x0200'0200u;

    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kOpenCb + 0u, kInstrOpenLdrHandle1);
    bus.write32(kOpenCb + 4u, kInstrBxLr);

    bus.write32(kGet8Cb + 0u, kInstrGet8LoadPtr0);
    bus.write32(kGet8Cb + 4u, kInstrGet8LoadByte);
    bus.write32(kGet8Cb + 8u, kInstrGet8AdvancePtr);
    bus.write32(kGet8Cb + 12u, kInstrGet8StorePtr0);
    bus.write32(kGet8Cb + 16u, kInstrMovR0R2);
    bus.write32(kGet8Cb + 20u, kInstrBxLr);

    bus.write32(kCbStruct + 0x00u, kOpenCb);
    bus.write32(kCbStruct + 0x04u, 0u);
    bus.write32(kCbStruct + 0x08u, kGet8Cb);
    bus.write32(kCbStruct + 0x0Cu, 0u);
    bus.write32(kCbStruct + 0x10u, 0u);

    constexpr std::array<u8, 64> expected = {
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S', 'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S', 'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S', 'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S', 'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
    };

    constexpr u8 kBackrefB0 = 0x50u; // length 8 (encoded 5)
    constexpr u8 kBackrefB1 = 0x07u; // disp 8 (encoded 7)

    write_bytes(bus, kStream, {0x00u,      'D',        'S',        ':',        'H',
                               'G',        '/',        'S',        'S',        0xFEu,
                               kBackrefB0, kBackrefB1, kBackrefB0, kBackrefB1, kBackrefB0,
                               kBackrefB1, kBackrefB0, kBackrefB1, kBackrefB0, kBackrefB1,
                               kBackrefB0, kBackrefB1, kBackrefB0, kBackrefB1});

    bus.write32(kHandle + 0u, kStream);
    bus.write32(kHandle + 4u, make_header(1u, 64u));

    enter_arm_swi_state(nds);
    state.r[0] = kHandle;
    state.r[1] = kDest;
    state.r[2] = 0u;
    state.r[3] = kCbStruct;

    arm7_bios_hle_dispatch_swi(nds.cpu7(), kSwiLz77Vram);

    REQUIRE(state.current_mode() == Mode::User);
    for (u32 i = 0; i < expected.size(); ++i) {
        REQUIRE(bus.read8(kDest + i) == expected[i]);
    }
}

// Three runs interleaved with two raw segments, totalling 32 bytes (even, so
// the halfword buffer flushes cleanly with no trailing zero).
static void run_rle_vram_case() {
    constexpr u32 kCbStruct = 0x0200'0000u;
    constexpr u32 kOpenCb = 0x0200'0020u;
    constexpr u32 kGet8Cb = 0x0200'0040u;
    constexpr u32 kHandle = 0x0200'0080u;
    constexpr u32 kStream = 0x0200'0100u;
    constexpr u32 kDest = 0x0200'0200u;

    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kOpenCb + 0u, kInstrOpenLdrHandle1);
    bus.write32(kOpenCb + 4u, kInstrBxLr);

    bus.write32(kGet8Cb + 0u, kInstrGet8LoadPtr0);
    bus.write32(kGet8Cb + 4u, kInstrGet8LoadByte);
    bus.write32(kGet8Cb + 8u, kInstrGet8AdvancePtr);
    bus.write32(kGet8Cb + 12u, kInstrGet8StorePtr0);
    bus.write32(kGet8Cb + 16u, kInstrMovR0R2);
    bus.write32(kGet8Cb + 20u, kInstrBxLr);

    bus.write32(kCbStruct + 0x00u, kOpenCb);
    bus.write32(kCbStruct + 0x04u, 0u);
    bus.write32(kCbStruct + 0x08u, kGet8Cb);
    bus.write32(kCbStruct + 0x0Cu, 0u);
    bus.write32(kCbStruct + 0x10u, 0u);

    constexpr std::array<u8, 32> expected = {
        0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu,                                    //
        0x11u, 0x22u, 0x33u,                                                  //
        0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu,               //
        0x10u, 0x20u, 0x30u, 0x40u,                                           //
        0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, //
        0xCCu, 0xCCu,
    };

    write_bytes(bus,
                kStream,
                {0x82u,
                 0xAAu, // run 5 of 0xAA
                 0x02u,
                 0x11u,
                 0x22u,
                 0x33u, // raw 3
                 0x85u,
                 0xBBu, // run 8 of 0xBB
                 0x03u,
                 0x10u,
                 0x20u,
                 0x30u,
                 0x40u, // raw 4
                 0x89u,
                 0xCCu}); // run 12 of 0xCC

    bus.write32(kHandle + 0u, kStream);
    bus.write32(kHandle + 4u, make_header(3u, 32u));

    enter_arm_swi_state(nds);
    state.r[0] = kHandle;
    state.r[1] = kDest;
    state.r[2] = 0u;
    state.r[3] = kCbStruct;

    arm7_bios_hle_dispatch_swi(nds.cpu7(), kSwiRleVram);

    REQUIRE(state.current_mode() == Mode::User);
    for (u32 i = 0; i < expected.size(); ++i) {
        REQUIRE(bus.read8(kDest + i) == expected[i]);
    }
}

// Tree layout (8 bytes after the leading tree_size byte, indices 0..7):
//   [0] = 0x00 root          (offset 0, no end flags) → children at 2, 3
//   [1] = 0x00 pad
//   [2] = 0xC0 L internal    (offset 0, both end flags) → leaves at 4, 5
//   [3] = 0xC1 R internal    (offset 1, both end flags) →
//                              child0 at (3 AND ~1)+1*2+2 = 6
//                              child1 at 7
//   [4] = 0x11 leaf for code 00
//   [5] = 0x22 leaf for code 01
//   [6] = 0x33 leaf for code 10
//   [7] = 0x44 leaf for code 11
//
// Every code is exactly 2 bits MSB-first within each 32-bit bitstream word
// (bit 31 consumed first per GBATEK §5.4). 48 plaintext bytes = 48 codes =
// 96 bits = exactly 3 bitstream words — exercises the per-word refill branch
// twice.

constexpr u8 huffman_code_for(u8 sym) {
    return sym == 0x11u ? 0u : sym == 0x22u ? 1u : sym == 0x33u ? 2u : 3u;
}

constexpr std::array<u8, 48> kHuffmanPlaintext = {
    // Frequencies: 0x11×18, 0x22×12, 0x33×10, 0x44×8 (sum = 48).
    0x11u, 0x22u, 0x33u, 0x44u, 0x11u, 0x22u, 0x33u, 0x44u, 0x11u, 0x22u, 0x33u, 0x44u,
    0x11u, 0x22u, 0x33u, 0x44u, 0x11u, 0x22u, 0x33u, 0x44u, 0x11u, 0x22u, 0x33u, 0x44u,
    0x11u, 0x22u, 0x33u, 0x44u, 0x11u, 0x22u, 0x33u, 0x44u, 0x11u, 0x11u, 0x22u, 0x22u,
    0x33u, 0x33u, 0x11u, 0x11u, 0x22u, 0x22u, 0x11u, 0x11u, 0x11u, 0x11u, 0x11u, 0x11u,
};

constexpr std::array<u32, 3> encode_huffman_bitstream() {
    std::array<u32, 3> words{};
    u32 word = 0u;
    int bits_in_word = 0;
    u32 word_idx = 0u;
    for (u8 sym : kHuffmanPlaintext) {
        const u32 code = huffman_code_for(sym);
        const int free_bits = 32 - bits_in_word;
        // 2-bit code occupies bits [free_bits-1 .. free_bits-2] (MSB-first).
        word |= code << (free_bits - 2);
        bits_in_word += 2;
        if (bits_in_word == 32) {
            words[word_idx++] = word;
            word = 0u;
            bits_in_word = 0;
        }
    }
    if (bits_in_word != 0) {
        words[word_idx] = word;
    }
    return words;
}

static void run_huffman_case() {
    constexpr u32 kCbStruct = 0x0200'0000u;
    constexpr u32 kOpenCb = 0x0200'0020u;
    constexpr u32 kGet8Cb = 0x0200'0040u;
    constexpr u32 kGet32Cb = 0x0200'0080u;
    constexpr u32 kHandle = 0x0200'0100u;
    constexpr u32 kTree = 0x0200'0180u;
    constexpr u32 kWordStream = 0x0200'0200u;
    constexpr u32 kDest = 0x0200'0300u;

    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kOpenCb + 0u, kInstrOpenLdrHandle0);
    bus.write32(kOpenCb + 4u, kInstrBxLr);

    bus.write32(kGet8Cb + 0u, kInstrGet8LoadPtr4);
    bus.write32(kGet8Cb + 4u, kInstrGet8LoadByte);
    bus.write32(kGet8Cb + 8u, kInstrGet8AdvancePtr);
    bus.write32(kGet8Cb + 12u, kInstrGet8StorePtr4);
    bus.write32(kGet8Cb + 16u, kInstrMovR0R2);
    bus.write32(kGet8Cb + 20u, kInstrBxLr);

    bus.write32(kGet32Cb + 0u, kInstrGet32LoadPtr8);
    bus.write32(kGet32Cb + 4u, kInstrGet32LoadWord);
    bus.write32(kGet32Cb + 8u, kInstrGet32AdvancePtr);
    bus.write32(kGet32Cb + 12u, kInstrGet32StorePtr);
    bus.write32(kGet32Cb + 16u, kInstrMovR0R2);
    bus.write32(kGet32Cb + 20u, kInstrBxLr);

    bus.write32(kCbStruct + 0x00u, kOpenCb);
    bus.write32(kCbStruct + 0x04u, 0u);
    bus.write32(kCbStruct + 0x08u, kGet8Cb);
    bus.write32(kCbStruct + 0x0Cu, 0u);
    bus.write32(kCbStruct + 0x10u, kGet32Cb);

    bus.write8(kTree + 0u, 0x03u); // tree_size_byte → tree spans 8 bytes
    write_bytes(bus, kTree + 1u, {0x00u, 0x00u, 0xC0u, 0xC1u, 0x11u, 0x22u, 0x33u, 0x44u});

    constexpr auto kBitstreamWords = encode_huffman_bitstream();
    for (u32 i = 0; i < kBitstreamWords.size(); ++i) {
        bus.write32(kWordStream + i * 4u, kBitstreamWords[i]);
    }

    bus.write32(kHandle + 0u, make_header_huffman(8u, 2u, 48u));
    bus.write32(kHandle + 4u, kTree);
    bus.write32(kHandle + 8u, kWordStream);

    enter_arm_swi_state(nds);
    state.r[0] = kHandle;
    state.r[1] = kDest;
    state.r[2] = 0u;
    state.r[3] = kCbStruct;

    arm7_bios_hle_dispatch_swi(nds.cpu7(), kSwiHuffman);

    REQUIRE(state.current_mode() == Mode::User);
    for (u32 i = 0; i < kHuffmanPlaintext.size(); ++i) {
        REQUIRE(bus.read8(kDest + i) == kHuffmanPlaintext[i]);
    }
}

} // namespace

int main() {
    run_lz77_vram_case();
    run_rle_vram_case();
    run_huffman_case();
    std::puts("arm7_bios_decomp_callback_sequence_test: all 3 sub-cases passed");
    return 0;
}
