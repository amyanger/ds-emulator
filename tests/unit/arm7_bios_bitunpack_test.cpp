// Exercises SWI 0x10 (BitUnPack) by calling the family function directly.
// The dispatcher entry path is covered by arm7_bios_dispatch_test and
// arm7_bios_decomp_dispatch_test (slice 3f commit 5).

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

constexpr u32 kSrc = 0x0200'0000u;
constexpr u32 kDst = 0x0200'0100u;
constexpr u32 kInfo = 0x0200'0200u;

static void
write_unpack_info(Arm7Bus& bus, u32 info_addr, u16 src_len, u8 src_w, u8 dst_w, u32 offset_word) {
    bus.write16(info_addr + 0u, src_len);
    bus.write8(info_addr + 2u, src_w);
    bus.write8(info_addr + 3u, dst_w);
    bus.write32(info_addr + 4u, offset_word);
}

static void invoke(NDS& nds) {
    nds.cpu7().state().r[0] = kSrc;
    nds.cpu7().state().r[1] = kDst;
    nds.cpu7().state().r[2] = kInfo;
    bios7_bit_unpack(nds.cpu7().state(), nds.arm7_bus());
}

static void bitunpack_1to4_no_offset() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0xAAu);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 1u, 4u, 0u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst) == 0x10101010u);
}

static void bitunpack_1to8_offset_no_zero_flag() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0x55u);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 1u, 8u, 0x0000'0020u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst + 0u) == 0x0021'0021u);
    REQUIRE(nds.arm7_bus().read32(kDst + 4u) == 0x0021'0021u);
}

static void bitunpack_1to8_offset_zero_flag_set() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0x55u);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 1u, 8u, 0x8000'0020u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst + 0u) == 0x2021'2021u);
    REQUIRE(nds.arm7_bus().read32(kDst + 4u) == 0x2021'2021u);
}

static void bitunpack_2to8_no_offset() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0xE4u);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 2u, 8u, 0u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst) == 0x0302'0100u);
}

static void bitunpack_4to8_no_offset() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0x21u);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 4u, 8u, 0u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst) == 0x0000'0201u);
}

static void bitunpack_8to16_offset() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0x05u);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 8u, 16u, 0x0000'0100u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst) == 0x0000'0105u);
}

// dst_width=32 path; offset_word 0xFFFFFFFB exercises max 31-bit data_offset
// with zero-flag set. Spec §6.1's 0x80000005 expectation is unsatisfiable
// because bit 31 is the zero-flag, not part of data_offset.
static void bitunpack_8to32_max_offset() {
    NDS nds;
    nds.arm7_bus().write8(kSrc, 0x05u);
    write_unpack_info(nds.arm7_bus(), kInfo, 1u, 8u, 32u, 0xFFFF'FFFBu);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst) == 0x8000'0000u);
}

static void bitunpack_zero_length_no_writes() {
    NDS nds;
    const u32 sentinel = 0xDEAD'BEEFu;
    for (u32 i = 0; i < 4u; ++i) {
        nds.arm7_bus().write32(kDst + i * 4u, sentinel);
    }
    write_unpack_info(nds.arm7_bus(), kInfo, 0u, 1u, 8u, 0u);
    invoke(nds);
    for (u32 i = 0; i < 4u; ++i) {
        REQUIRE(nds.arm7_bus().read32(kDst + i * 4u) == sentinel);
    }
}

static void bitunpack_8to8_passthrough() {
    NDS nds;
    nds.arm7_bus().write8(kSrc + 0u, 0x11u);
    nds.arm7_bus().write8(kSrc + 1u, 0x22u);
    nds.arm7_bus().write8(kSrc + 2u, 0x33u);
    nds.arm7_bus().write8(kSrc + 3u, 0x44u);
    write_unpack_info(nds.arm7_bus(), kInfo, 4u, 8u, 8u, 0u);
    invoke(nds);
    REQUIRE(nds.arm7_bus().read32(kDst) == 0x4433'2211u);
}

int main() {
    bitunpack_1to4_no_offset();
    bitunpack_1to8_offset_no_zero_flag();
    bitunpack_1to8_offset_zero_flag_set();
    bitunpack_2to8_no_offset();
    bitunpack_4to8_no_offset();
    bitunpack_8to16_offset();
    bitunpack_8to32_max_offset();
    bitunpack_zero_length_no_writes();
    bitunpack_8to8_passthrough();
    std::puts("arm7_bios_bitunpack_test: all 9 cases passed");
    return 0;
}
