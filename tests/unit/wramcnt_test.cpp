// Unit tests for WRAMCNT banking. Each mode is exercised from both buses,
// and we verify that a WRAMCNT write through ARM9 bus rebuilds both views.

#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {
constexpr u32 kWramcntAddr = 0x0400'0247u;

void set_wramcnt(NDS& nds, u8 mode) {
    nds.arm9_bus().write8(kWramcntAddr, mode);
}
}  // namespace

static void mode_0_arm9_has_full_32k_arm7_sees_private() {
    NDS nds;
    set_wramcnt(nds, 0);

    nds.arm9_bus().write32(0x0300'0000, 0xAABB'CCDDu);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0xAABB'CCDDu);

    // 32 KB mirror: +0x0000'8000 is the same location, +0x0000'4000 is
    // the upper half (unwritten in this test, still zero).
    REQUIRE(nds.arm9_bus().read32(0x0300'4000) == 0);
    REQUIRE(nds.arm9_bus().read32(0x0300'8000) == 0xAABB'CCDDu);

    // ARM7 at 0x0300'0000 sees its own WRAM mirror, not shared WRAM.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0);
}

static void mode_1_splits_16k_16k_high_to_arm9_low_to_arm7() {
    NDS nds;
    set_wramcnt(nds, 1);

    // Mode 1 (verified against melonDS): ARM9 sees the second 16 KB
    // (offset 0x4000), ARM7 sees the first 16 KB (offset 0x0000).
    nds.arm9_bus().write32(0x0300'0000, 0x1111'1111u);
    nds.arm7_bus().write32(0x0300'0000, 0x2222'2222u);

    // ARM9 reads back its own 16 KB slice, mirrored across region 3.
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0x1111'1111u);
    REQUIRE(nds.arm9_bus().read32(0x0300'4000) == 0x1111'1111u);  // 16 KB mirror
    // ARM7 reads back its own 16 KB slice.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x2222'2222u);
    REQUIRE(nds.arm7_bus().read32(0x0300'4000) == 0x2222'2222u);  // 16 KB mirror
}

static void mode_2_splits_16k_16k_low_to_arm9_high_to_arm7() {
    NDS nds;
    set_wramcnt(nds, 2);

    // Mode 2: ARM9 sees the first 16 KB (offset 0x0000), ARM7 sees the
    // second 16 KB (offset 0x4000).
    nds.arm9_bus().write32(0x0300'0000, 0x3333'3333u);
    nds.arm7_bus().write32(0x0300'0000, 0x4444'4444u);

    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0x3333'3333u);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x4444'4444u);

    // Cross-check: ARM9's mode-2 slice (first 16 KB, offset 0x0000) is
    // the same physical bytes as ARM7's mode-1 slice. Switch to mode 1
    // and ARM7 should see 0x33333333.
    set_wramcnt(nds, 1);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x3333'3333u);
}

static void mode_3_arm7_has_full_32k_arm9_is_open_bus() {
    NDS nds;
    set_wramcnt(nds, 3);

    nds.arm7_bus().write32(0x0300'0000, 0x5555'5555u);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x5555'5555u);
    REQUIRE(nds.arm7_bus().read32(0x0300'8000) == 0x5555'5555u);  // 32 KB mirror

    // ARM9 at 0x0300'0000 is now unmapped — reads return 0, writes ignored.
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);
    nds.arm9_bus().write32(0x0300'0000, 0xFFFF'FFFFu);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);
    // The ARM7 data is still there.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x5555'5555u);
}

static void arm7_cannot_modify_wramcnt() {
    NDS nds;
    set_wramcnt(nds, 1);
    // ARM7 write to WRAMCNT: ARM7 I/O dispatch is a stub, so nothing
    // happens. Mode stays at 1.
    nds.arm7_bus().write8(kWramcntAddr, 3);

    // Prove: under mode 1 ARM9 still sees its slice (second 16 KB).
    nds.arm9_bus().write32(0x0300'0000, 0x7777'7777u);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0x7777'7777u);
    // ARM7 at 0x0300'0000 still sees its slice (first 16 KB), not ARM9's.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) != 0x7777'7777u);
}

int main() {
    mode_0_arm9_has_full_32k_arm7_sees_private();
    mode_1_splits_16k_16k_high_to_arm9_low_to_arm7();
    mode_2_splits_16k_16k_low_to_arm9_high_to_arm7();
    mode_3_arm7_has_full_32k_arm9_is_open_bus();
    arm7_cannot_modify_wramcnt();
    std::printf("wramcnt_test: all passed\n");
    return 0;
}
