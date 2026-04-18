// Slice 3f capstone — exercises the full ARM7 BIOS decompressor path end-to-end
// through real Arm7Bus and the SWI dispatcher. Per-algorithm tests cover the
// inner loops; this test confirms dispatcher routing, header parsing, bus
// access, and termination conditions compose correctly across the three
// ReadNormal SWIs (0x10 BitUnPack, 0x11 LZ77 Wram, 0x14 RLE Wram).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_hle.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <array>
#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kSrc = 0x0200'0100u;
constexpr u32 kDst = 0x0200'0200u;
constexpr u32 kInfo = 0x0200'0080u;
constexpr u32 kSwiPc = 0x0200'1000u;

static constexpr u32 make_header(u8 type, u32 size) {
    return (static_cast<u32>(type & 0x0Fu) << 4) | (size << 8);
}

// Pre-place the CPU in the post-exception-entry state the dispatcher expects.
// Mirrors arm7_bios_decomp_dispatch_test so the dispatcher's implicit
// MOVS PC, R14 tail restores cleanly without disturbing R0/R1.
static void enter_arm_swi_state(NDS& nds, u32 swi_addr) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = 0x00000013u;
    state.r[14] = swi_addr + 4u;
    *state.spsr_slot() = 0x00000010u;
}

static void write_payload(Arm7Bus& bus, u32 addr, const u8* bytes, u32 len) {
    for (u32 i = 0; i < len; ++i) {
        bus.write8(addr + i, bytes[i]);
    }
}

} // namespace

// Sub-case 1: LZ77 decompresses a multi-block payload to "DS:HG/SS" × 8.
// Payload exercises two flag bytes — one all-raw (initial 8-byte seed) and
// one all-backref (7 length-8 disp-8 copies that fill the remaining 56 bytes,
// with the last flag bit skipped via the inner-loop hdr.size guard).
static void lz77_end_to_end_dshgss_x8() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    // clang-format off
    constexpr std::array<u8, 64> expected = {
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
    };
    // clang-format on

    // length=8 → b0 high nibble = (8-3) = 5; disp=8 → encoded 7 spans
    // ((b0 & 0x0F) << 8) | b1 = 7, simplest as b0 low nibble 0, b1 = 0x07.
    constexpr u8 backref_b0 = 0x50u;
    constexpr u8 backref_b1 = 0x07u;

    // clang-format off
    constexpr std::array<u8, 24> payload = {
        // Flag 1: all 8 blocks are raw.
        0x00u, 'D', 'S', ':', 'H', 'G', '/', 'S', 'S',
        // Flag 2: 7 backrefs in bits 7..1; bit 0 unreached because written
        // hits 64 after the seventh backref.
        0xFEu,
        backref_b0, backref_b1, backref_b0, backref_b1,
        backref_b0, backref_b1, backref_b0, backref_b1,
        backref_b0, backref_b1, backref_b0, backref_b1,
        backref_b0, backref_b1,
    };
    // clang-format on

    bus.write32(kSrc, make_header(1u, 64u));
    write_payload(bus, kSrc + 4u, payload.data(), payload.size());

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSrc;
    state.r[1] = kDst;

    arm7_bios_hle_dispatch_swi(nds.cpu7(), 0x11u);

    for (u32 i = 0; i < expected.size(); ++i) {
        REQUIRE(bus.read8(kDst + i) == expected[i]);
    }
}

// Sub-case 2: RLE decompresses a 32-byte plaintext mixing compressed runs
// and raw segments — exercises both flag-byte modes and the cross-segment
// stream loop.
static void rle_end_to_end_mixed_32() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    // clang-format off
    constexpr std::array<u8, 32> expected = {
        0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu,                             // run 5 of 0xAA
        0x11u, 0x22u, 0x33u,                                           // raw 3
        0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu, 0xBBu,        // run 8 of 0xBB
        0x10u, 0x20u, 0x30u, 0x40u,                                    // raw 4
        0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu,
        0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu, 0xCCu,                      // run 12 of 0xCC
    };

    constexpr std::array<u8, 19> payload = {
        0x82u, 0xAAu,                                  // compressed: N-3=2 → len 5, byte 0xAA
        0x02u, 0x11u, 0x22u, 0x33u,                    // raw: N-1=2 → len 3, bytes
        0x85u, 0xBBu,                                  // compressed: N-3=5 → len 8, byte 0xBB
        0x03u, 0x10u, 0x20u, 0x30u, 0x40u,             // raw: N-1=3 → len 4, bytes
        0x89u, 0xCCu,                                  // compressed: N-3=9 → len 12, byte 0xCC
    };
    // clang-format on

    bus.write32(kSrc, make_header(3u, 32u));
    write_payload(bus, kSrc + 4u, payload.data(), payload.size());

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSrc;
    state.r[1] = kDst;

    arm7_bios_hle_dispatch_swi(nds.cpu7(), 0x14u);

    for (u32 i = 0; i < expected.size(); ++i) {
        REQUIRE(bus.read8(kDst + i) == expected[i]);
    }
}

// Sub-case 3: BitUnPack 1→8 expansion of a 4-byte source bitmap with
// data_offset=0x80 and zero-flag clear. Each non-zero source bit becomes
// 0x81 in the dest; each zero bit becomes 0x00. Verifies dest 32-bit
// buffered writes assemble correctly across multiple words.
static void bitunpack_end_to_end_1to8_offset_0x80() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write8(kSrc + 0u, 0x55u);
    bus.write8(kSrc + 1u, 0xAAu);
    bus.write8(kSrc + 2u, 0x33u);
    bus.write8(kSrc + 3u, 0xCCu);

    bus.write16(kInfo + 0u, 4u);
    bus.write8(kInfo + 2u, 1u);
    bus.write8(kInfo + 3u, 8u);
    bus.write32(kInfo + 4u, 0x0000'0080u);

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSrc;
    state.r[1] = kDst;
    state.r[2] = kInfo;

    arm7_bios_hle_dispatch_swi(nds.cpu7(), 0x10u);

    // clang-format off
    constexpr std::array<u8, 32> expected = {
        0x81u, 0x00u, 0x81u, 0x00u, 0x81u, 0x00u, 0x81u, 0x00u, // 0x55 LSB-first: 1,0,1,0,1,0,1,0
        0x00u, 0x81u, 0x00u, 0x81u, 0x00u, 0x81u, 0x00u, 0x81u, // 0xAA LSB-first: 0,1,0,1,0,1,0,1
        0x81u, 0x81u, 0x00u, 0x00u, 0x81u, 0x81u, 0x00u, 0x00u, // 0x33 LSB-first: 1,1,0,0,1,1,0,0
        0x00u, 0x00u, 0x81u, 0x81u, 0x00u, 0x00u, 0x81u, 0x81u, // 0xCC LSB-first: 0,0,1,1,0,0,1,1
    };
    // clang-format on

    for (u32 i = 0; i < expected.size(); ++i) {
        REQUIRE(bus.read8(kDst + i) == expected[i]);
    }
}

int main() {
    lz77_end_to_end_dshgss_x8();
    rle_end_to_end_mixed_32();
    bitunpack_end_to_end_1to8_offset_0x80();
    std::puts("arm7_bios_decomp_sequence_test: all 3 sub-cases passed");
    return 0;
}
