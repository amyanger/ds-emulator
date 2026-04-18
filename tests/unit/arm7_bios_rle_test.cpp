// Exercises SWI 0x14 (RLUnCompReadNormalWrite8bit) by calling the family
// function directly. The dispatcher entry path is covered separately by
// arm7_bios_dispatch_test and arm7_bios_decomp_dispatch_test.

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

constexpr u32 kSrc = 0x0200'0000u;
constexpr u32 kDst = 0x0200'0200u;

// Header packing: bit 0-3 reserved, bit 4-7 type (3 = RLE), bit 8-31 size.
static constexpr u32 make_header(u8 type, u32 size) {
    return (static_cast<u32>(type & 0x0Fu) << 4) | (size << 8);
}

static void write_bytes(Arm7Bus& bus, u32 addr, std::initializer_list<u8> bytes) {
    u32 offset = 0u;
    for (u8 b : bytes) {
        bus.write8(addr + offset, b);
        ++offset;
    }
}

static void invoke(NDS& nds) {
    nds.cpu7().state().r[0] = kSrc;
    nds.cpu7().state().r[1] = kDst;
    bios7_rl_uncomp_wram(nds.cpu7().state(), nds.arm7_bus());
}

// Case 1: uncompressed length 1 — flag 0x00 stores (N-1)=0, followed by one raw byte.
static void rle_uncompressed_length_one() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(3u, 1u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x00u, 0xABu});
    invoke(nds);
    REQUIRE(nds.arm7_bus().read8(kDst) == 0xABu);
}

// Case 2: uncompressed length 128 (maximum) — flag 0x7F stores (N-1)=0x7F,
// followed by 128 distinct bytes copied verbatim.
static void rle_uncompressed_length_max() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(3u, 128u));
    nds.arm7_bus().write8(kSrc + 4u, 0x7Fu);
    for (u32 i = 0; i < 128u; ++i) {
        nds.arm7_bus().write8(kSrc + 5u + i, static_cast<u8>(i));
    }
    invoke(nds);
    for (u32 i = 0; i < 128u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == static_cast<u8>(i));
    }
}

// Case 3: compressed length 3 (minimum) — flag 0x80 stores (N-3)=0, one data
// byte repeated 3 times.
static void rle_compressed_length_min() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(3u, 3u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x80u, 0xCDu});
    invoke(nds);
    for (u32 i = 0; i < 3u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == 0xCDu);
    }
}

// Case 4: compressed length 130 (maximum) — flag 0xFF stores (N-3)=0x7F, one
// data byte repeated 130 times.
static void rle_compressed_length_max() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(3u, 130u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0xFFu, 0xEFu});
    invoke(nds);
    for (u32 i = 0; i < 130u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == 0xEFu);
    }
}

// Case 5: mixed stream — run of 3 (0xAA), 3 raw bytes (0x11, 0x22, 0x33),
// run of 5 (0xBB). Total decompressed = 11 bytes.
static void rle_mixed_stream() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(3u, 11u));
    write_bytes(
        nds.arm7_bus(), kSrc + 4u, {0x80u, 0xAAu, 0x02u, 0x11u, 0x22u, 0x33u, 0x82u, 0xBBu});
    invoke(nds);
    const u8 expected[] = {
        0xAAu, 0xAAu, 0xAAu, 0x11u, 0x22u, 0x33u, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu};
    for (u32 i = 0; i < 11u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == expected[i]);
    }
}

// Case 6: zero decompressed size. Outer loop must not enter; dest sentinel
// stays untouched.
static void rle_zero_size_no_writes() {
    NDS nds;
    const u8 sentinel = 0x5Au;
    for (u32 i = 0; i < 16u; ++i) {
        nds.arm7_bus().write8(kDst + i, sentinel);
    }
    nds.arm7_bus().write32(kSrc, make_header(3u, 0u));
    invoke(nds);
    for (u32 i = 0; i < 16u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == sentinel);
    }
}

int main() {
    rle_uncompressed_length_one();
    rle_uncompressed_length_max();
    rle_compressed_length_min();
    rle_compressed_length_max();
    rle_mixed_stream();
    rle_zero_size_no_writes();
    std::puts("arm7_bios_rle_test: all 6 cases passed");
    return 0;
}
