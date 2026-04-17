// Exercises SWI 0x0B (CpuSet), SWI 0x0C (CpuFastSet), and SWI 0x0E
// (GetCRC16) by calling the family functions directly; the SWI entry paths
// are covered by arm7_bios_dispatch_test.

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_memcpy.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// ----- CpuSet (SWI 0x0B) -----

static void cpu_set_copy_halfword_basic() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u16 values[4] = {0x1234, 0x5678, 0x9ABC, 0xDEF0};
    for (u32 i = 0; i < 4; ++i) {
        nds.arm7_bus().write16(src + i * 2u, values[i]);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 4u; // wide=0, fill=0, len=4 halfwords
    bios7_cpu_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 4; ++i) {
        REQUIRE(nds.arm7_bus().read16(dst + i * 2u) == values[i]);
    }
}

static void cpu_set_copy_word_basic() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u32 values[4] = {0x11111111u, 0x22222222u, 0x33333333u, 0x44444444u};
    for (u32 i = 0; i < 4; ++i) {
        nds.arm7_bus().write32(src + i * 4u, values[i]);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 4u | (1u << 26); // wide=1, fill=0, len=4 words
    bios7_cpu_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 4; ++i) {
        REQUIRE(nds.arm7_bus().read32(dst + i * 4u) == values[i]);
    }
}

static void cpu_set_fill_halfword_basic() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u16 sentinel = 0xBEEF;
    nds.arm7_bus().write16(src, sentinel);
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 8u | (1u << 24); // wide=0, fill=1, len=8 halfwords
    bios7_cpu_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read16(dst + i * 2u) == sentinel);
    }
}

static void cpu_set_fill_word_basic() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u32 sentinel = 0xDEAD'BEEFu;
    nds.arm7_bus().write32(src, sentinel);
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 8u | (1u << 24) | (1u << 26); // wide=1, fill=1, len=8 words
    bios7_cpu_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read32(dst + i * 4u) == sentinel);
    }
}

static void cpu_set_length_zero_noop() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    // Pre-fill dst with a sentinel so we can detect any spurious writes.
    const u16 pre = 0xAAAA;
    for (u32 i = 0; i < 8; ++i) {
        nds.arm7_bus().write16(dst + i * 2u, pre);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 0u | (1u << 26); // len=0 (wide flag irrelevant)
    bios7_cpu_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read16(dst + i * 2u) == pre);
    }
}

// Reserved bits (23:21, 25, 31:27) must be ignored; only bits 20:0 form the
// unit count. Set reserved bits 23 and 25 alongside len=4, wide=0.
static void cpu_set_unit_count_mask_20_bits() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u16 pre = 0xAAAA;
    const u16 values[4] = {0x1111, 0x2222, 0x3333, 0x4444};
    for (u32 i = 0; i < 4; ++i) {
        nds.arm7_bus().write16(src + i * 2u, values[i]);
    }
    // Halfword 5 at dst: pre-fill to verify it's not clobbered.
    for (u32 i = 4; i < 8; ++i) {
        nds.arm7_bus().write16(dst + i * 2u, pre);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 4u | (1u << 23) | (1u << 25); // reserved bits set
    bios7_cpu_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 4; ++i) {
        REQUIRE(nds.arm7_bus().read16(dst + i * 2u) == values[i]);
    }
    for (u32 i = 4; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read16(dst + i * 2u) == pre);
    }
}

// ----- CpuFastSet (SWI 0x0C) -----

static void cpu_fast_set_copy_8_words_basic() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u32 values[8] = {
        0x1111'1111u,
        0x2222'2222u,
        0x3333'3333u,
        0x4444'4444u,
        0x5555'5555u,
        0x6666'6666u,
        0x7777'7777u,
        0x8888'8888u,
    };
    for (u32 i = 0; i < 8; ++i) {
        nds.arm7_bus().write32(src + i * 4u, values[i]);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 8u; // len=8 words, copy
    bios7_cpu_fast_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read32(dst + i * 4u) == values[i]);
    }
}

// Hardware-accurate: len=1 rounds up to 8, so all 8 words at src get copied.
// Trailing words past the requested end are clobbered per GBATEK.
static void cpu_fast_set_copy_1_word_rounds_up_to_8() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u32 sentinel = 0xCAFE'BABEu;
    for (u32 i = 0; i < 8; ++i) {
        nds.arm7_bus().write32(src + i * 4u, sentinel);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 1u; // len=1 → rounds up to 8
    bios7_cpu_fast_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read32(dst + i * 4u) == sentinel);
    }
}

static void cpu_fast_set_fill_8_words() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u32 sentinel = 0x1234'5678u;
    nds.arm7_bus().write32(src, sentinel);
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 8u | (1u << 24); // fill=1, len=8
    bios7_cpu_fast_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read32(dst + i * 4u) == sentinel);
    }
}

static void cpu_fast_set_length_zero_noop() {
    NDS nds;
    const u32 src = 0x0200'0000u;
    const u32 dst = 0x0200'0100u;
    const u32 pre = 0xAAAA'AAAAu;
    for (u32 i = 0; i < 8; ++i) {
        nds.arm7_bus().write32(dst + i * 4u, pre);
    }
    nds.cpu7().state().r[0] = src;
    nds.cpu7().state().r[1] = dst;
    nds.cpu7().state().r[2] = 0u;
    bios7_cpu_fast_set(nds.cpu7().state(), nds.arm7_bus());
    for (u32 i = 0; i < 8; ++i) {
        REQUIRE(nds.arm7_bus().read32(dst + i * 4u) == pre);
    }
}

// ----- GetCRC16 (SWI 0x0E) -----

static void get_crc16_zero_length_returns_initial() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xFFFFu;
    nds.cpu7().state().r[1] = 0x0200'0000u;
    nds.cpu7().state().r[2] = 0u;
    bios7_get_crc16(nds.cpu7().state(), nds.arm7_bus());
    // R3 is undefined when len=0 on real hardware; we initialize last=0
    // internally but do not assert the value here.
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFu);
}

// Reference (Python):
//   TABLE = [0xC0C1, 0xC181, 0xC301, 0xC601, 0xCC01, 0xD801, 0xF001, 0xA001]
//   def crc16(crc, data):
//       for i in range(0, len(data), 2):
//           word = data[i] | (data[i+1] << 8)
//           for shift in (0, 8):
//               for bit in range(8):
//                   xn = ((crc ^ (word >> (bit + shift))) & 1)
//                   crc = (crc >> 1) ^ (TABLE[bit] if xn else 0)
//       return crc & 0xFFFF
//   crc16(0xFFFF, b'\x00\x00') == 0xCABE
static void get_crc16_known_vector() {
    NDS nds;
    const u32 ptr = 0x0200'0000u;
    nds.arm7_bus().write16(ptr, 0x0000u); // {0x00, 0x00}
    nds.cpu7().state().r[0] = 0xFFFFu;
    nds.cpu7().state().r[1] = ptr;
    nds.cpu7().state().r[2] = 2u;
    bios7_get_crc16(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0xCABEu);
}

// Two halfwords {0x1234, 0x5678}, initial CRC 0xFFFF.
// Reference (same Python as above) → CRC = 0x9E87, last halfword = 0x5678.
static void get_crc16_two_halfwords_r0_and_r3() {
    NDS nds;
    const u32 ptr = 0x0200'0000u;
    nds.arm7_bus().write16(ptr + 0u, 0x1234u);
    nds.arm7_bus().write16(ptr + 2u, 0x5678u);
    nds.cpu7().state().r[0] = 0xFFFFu;
    nds.cpu7().state().r[1] = ptr;
    nds.cpu7().state().r[2] = 4u;
    bios7_get_crc16(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0x9E87u);
    REQUIRE(nds.cpu7().state().r[3] == 0x5678u);
}

int main() {
    cpu_set_copy_halfword_basic();
    cpu_set_copy_word_basic();
    cpu_set_fill_halfword_basic();
    cpu_set_fill_word_basic();
    cpu_set_length_zero_noop();
    cpu_set_unit_count_mask_20_bits();
    cpu_fast_set_copy_8_words_basic();
    cpu_fast_set_copy_1_word_rounds_up_to_8();
    cpu_fast_set_fill_8_words();
    cpu_fast_set_length_zero_noop();
    get_crc16_zero_length_returns_initial();
    get_crc16_known_vector();
    get_crc16_two_halfwords_r0_and_r3();
    std::puts("arm7_bios_memcpy_test: all 13 cases passed");
    return 0;
}
