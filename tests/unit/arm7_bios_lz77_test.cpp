// Exercises SWI 0x11 (LZ77UnCompReadNormalWrite8bit) by calling the family
// function directly. The dispatcher entry path is covered by
// arm7_bios_dispatch_test and arm7_bios_decomp_dispatch_test (slice 3f
// commit 5).

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

constexpr u32 kSrc = 0x0200'0000u;
constexpr u32 kDst = 0x0200'0100u;

// Header packing: bit 0-3 reserved, bit 4-7 type, bit 8-31 decompressed size.
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
    bios7_lz77_uncomp_wram(nds.cpu7().state(), nds.arm7_bus());
}

// Case 1: single uncompressed block — flag bit 7 = 0, one raw byte.
static void lz77_single_uncompressed_block() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(1u, 1u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x00u, 0xABu});
    invoke(nds);
    REQUIRE(nds.arm7_bus().read8(kDst) == 0xABu);
}

// Case 2: raw byte then compressed back-reference (length=3, disp=1).
// Flag 0x40 = bits 7,6 are processed before written hits hdr.size=4.
static void lz77_compressed_via_raw_then_backref() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(1u, 4u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x40u, 0xABu, 0x00u, 0x00u});
    invoke(nds);
    for (u32 i = 0; i < 4u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == 0xABu);
    }
}

// Case 3: self-overlap RLE-via-LZ77. A 9-byte compressed run at disp=1 must
// see each byte synchronously after it's written, otherwise the run reads
// stale dest memory.
static void lz77_self_overlap_disp1_run() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(1u, 10u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x40u, 0xCDu, 0x60u, 0x00u});
    invoke(nds);
    for (u32 i = 0; i < 10u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == 0xCDu);
    }
}

// Case 4: maximum-length compressed run. Length encoded 0xF → 18 bytes.
// Flag 0x40 (raw, then compressed, rest unused-via-size guard).
static void lz77_max_length_run() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(1u, 19u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x40u, 0xEEu, 0xF0u, 0x00u});
    invoke(nds);
    for (u32 i = 0; i < 19u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == 0xEEu);
    }
}

// Case 5: disp byte-order check. b0 holds the high nibble of disp, b1 the
// low byte; if the impl swapped them the run would read the wrong dest
// offset and the assertion pattern would diverge.
static void lz77_disp_byte_order() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(1u, 5u));
    write_bytes(nds.arm7_bus(), kSrc + 4u, {0x20u, 0xAAu, 0xBBu, 0x00u, 0x01u});
    invoke(nds);
    REQUIRE(nds.arm7_bus().read8(kDst + 0u) == 0xAAu);
    REQUIRE(nds.arm7_bus().read8(kDst + 1u) == 0xBBu);
    REQUIRE(nds.arm7_bus().read8(kDst + 2u) == 0xAAu);
    REQUIRE(nds.arm7_bus().read8(kDst + 3u) == 0xBBu);
    REQUIRE(nds.arm7_bus().read8(kDst + 4u) == 0xAAu);
}

// Case 6: full flag byte 0xFF with all 8 bits compressed. Seed 8 raw bytes
// via flag 0x00, then a 0xFF flag drives 8 × (length=3, disp=1) self-overlap
// blocks for 24 more bytes. Total 32 bytes of 0xAA. Verifies the inner
// flag-bit loop walks all 8 bits MSB-first.
static void lz77_full_flag_byte_compressed() {
    NDS nds;
    nds.arm7_bus().write32(kSrc, make_header(1u, 32u));
    u32 off = 4u;
    nds.arm7_bus().write8(kSrc + off++, 0x00u);
    for (u32 i = 0; i < 8u; ++i) {
        nds.arm7_bus().write8(kSrc + off++, 0xAAu);
    }
    nds.arm7_bus().write8(kSrc + off++, 0xFFu);
    for (u32 i = 0; i < 8u; ++i) {
        nds.arm7_bus().write8(kSrc + off++, 0x00u);
        nds.arm7_bus().write8(kSrc + off++, 0x00u);
    }
    invoke(nds);
    for (u32 i = 0; i < 32u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == 0xAAu);
    }
}

// Case 7: zero decompressed size. Outer loop never enters; dest sentinel
// byte must be preserved exactly.
static void lz77_zero_size_no_writes() {
    NDS nds;
    const u8 sentinel = 0x5Au;
    for (u32 i = 0; i < 16u; ++i) {
        nds.arm7_bus().write8(kDst + i, sentinel);
    }
    nds.arm7_bus().write32(kSrc, make_header(1u, 0u));
    invoke(nds);
    for (u32 i = 0; i < 16u; ++i) {
        REQUIRE(nds.arm7_bus().read8(kDst + i) == sentinel);
    }
}

int main() {
    lz77_single_uncompressed_block();
    lz77_compressed_via_raw_then_backref();
    lz77_self_overlap_disp1_run();
    lz77_max_length_run();
    lz77_disp_byte_order();
    lz77_full_flag_byte_compressed();
    lz77_zero_size_no_writes();
    std::puts("arm7_bios_lz77_test: all 7 cases passed");
    return 0;
}
