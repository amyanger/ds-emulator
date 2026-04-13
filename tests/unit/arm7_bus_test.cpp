// Unit tests for Arm7Bus. Covers main RAM fast path (shared with Arm9Bus)
// plus the region-3 slow path which is ARM7-specific.

#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void main_ram_shared_with_arm9() {
    NDS nds;
    nds.arm9_bus().write32(0x0200'0000, 0xDEAD'BEEFu);
    // Both buses see the same main RAM.
    REQUIRE(nds.arm7_bus().read32(0x0200'0000) == 0xDEAD'BEEFu);

    nds.arm7_bus().write32(0x0200'1000, 0xFEED'FACEu);
    REQUIRE(nds.arm9_bus().read32(0x0200'1000) == 0xFEED'FACEu);
}

static void arm7_wram_round_trips_at_0x03800000() {
    NDS nds;
    nds.arm7_bus().write32(0x0380'0000, 0xA1B2'C3D4u);
    REQUIRE(nds.arm7_bus().read32(0x0380'0000) == 0xA1B2'C3D4u);
    // 64 KB mirror across the 8 MB upper half of region 3.
    REQUIRE(nds.arm7_bus().read32(0x0381'0000) == 0xA1B2'C3D4u);
    REQUIRE(nds.arm7_bus().read32(0x03FF'0000) == 0xA1B2'C3D4u);
    // ARM9 does not have a mapping at 0x0380'0000 → reads zero.
    REQUIRE(nds.arm9_bus().read32(0x0380'0000) == 0);
}

static void wramcnt_0_arm7_sees_arm7_wram_mirror_at_0x03000000() {
    NDS nds;
    // Default WRAMCNT = 0 at reset.
    // ARM7 write at 0x0380'0000 should be visible at 0x0300'0000 on ARM7
    // because 0x0300'0000 maps to ARM7 WRAM in this mode.
    nds.arm7_bus().write32(0x0380'0000, 0x1122'3344u);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x1122'3344u);
    // ARM9 at 0x0300'0000 sees shared WRAM (different backing), not ARM7 WRAM.
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);
}

static void io_region_routes_to_arm7_stub() {
    NDS nds;
    REQUIRE(nds.arm7_bus().read32(0x0400'0000) == 0);
    nds.arm7_bus().write32(0x0400'0000, 0xFFFF'FFFFu);
    // Should not crash; write ignored.
    REQUIRE(nds.arm7_bus().read32(0x0400'0000) == 0);
}

int main() {
    main_ram_shared_with_arm9();
    arm7_wram_round_trips_at_0x03800000();
    wramcnt_0_arm7_sees_arm7_wram_mirror_at_0x03000000();
    io_region_routes_to_arm7_stub();
    std::printf("arm7_bus_test: all passed\n");
    return 0;
}
