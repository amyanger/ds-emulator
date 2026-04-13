// Unit tests for Arm9Bus fast-path + slow-path behavior. Uses a live NDS
// because Arm9Bus holds a back-reference for I/O routing — there's no
// clean way to mock NDS in this slice, so we just construct one and hit
// its bus.

#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void main_ram_round_trip_all_widths() {
    NDS nds;

    nds.arm9_bus().write32(0x0200'0000, 0xDEAD'BEEFu);
    REQUIRE(nds.arm9_bus().read32(0x0200'0000) == 0xDEAD'BEEFu);
    REQUIRE(nds.arm9_bus().read16(0x0200'0000) == 0xBEEFu);
    REQUIRE(nds.arm9_bus().read16(0x0200'0002) == 0xDEADu);
    REQUIRE(nds.arm9_bus().read8 (0x0200'0000) == 0xEFu);
    REQUIRE(nds.arm9_bus().read8 (0x0200'0003) == 0xDEu);

    nds.arm9_bus().write16(0x0200'0010, 0xCAFEu);
    REQUIRE(nds.arm9_bus().read16(0x0200'0010) == 0xCAFEu);

    nds.arm9_bus().write8(0x0200'0020, 0x5Au);
    REQUIRE(nds.arm9_bus().read8(0x0200'0020) == 0x5Au);
}

static void main_ram_mirrors_four_times_across_the_region() {
    NDS nds;
    nds.arm9_bus().write32(0x0200'0000, 0x1234'5678u);

    // 4 MB mirrors at offsets 0, 0x0040'0000, 0x0080'0000, 0x00C0'0000.
    REQUIRE(nds.arm9_bus().read32(0x0240'0000) == 0x1234'5678u);
    REQUIRE(nds.arm9_bus().read32(0x0280'0000) == 0x1234'5678u);
    REQUIRE(nds.arm9_bus().read32(0x02C0'0000) == 0x1234'5678u);

    // Write near the far end of the 4 MB backing array, read near the far
    // end of the 16 MB region — should see the same byte.
    nds.arm9_bus().write32(0x023F'FFFC, 0xFACE'FEEDu);
    REQUIRE(nds.arm9_bus().read32(0x02FF'FFFC) == 0xFACE'FEEDu);
}

static void shared_wram_default_mode_is_arm9_32k() {
    NDS nds;
    // WRAMCNT = 0 at reset → ARM9 sees 32 KB at 0x0300'0000, mirrored.
    nds.arm9_bus().write32(0x0300'0000, 0xAABB'CCDDu);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0xAABB'CCDDu);
    // 32 KB mirror: +0x0000'8000 is the same location.
    REQUIRE(nds.arm9_bus().read32(0x0300'8000) == 0xAABB'CCDDu);
}

static void unmapped_reads_return_zero_and_writes_are_noops() {
    NDS nds;
    // 0x0A region: unmapped in this slice. Must not crash.
    REQUIRE(nds.arm9_bus().read32(0x0A00'0000) == 0);
    nds.arm9_bus().write32(0x0A00'0000, 0xFFFF'FFFFu);
    REQUIRE(nds.arm9_bus().read32(0x0A00'0000) == 0);
}

static void io_region_routes_to_nds_stub() {
    NDS nds;
    // ARM9 I/O at 0x0400'xxxx. In this slice NDS::arm9_io_readN returns 0
    // and writes are no-ops. Just verify we don't crash and we see 0.
    REQUIRE(nds.arm9_bus().read32(0x0400'0000) == 0);
    REQUIRE(nds.arm9_bus().read16(0x0400'0000) == 0);
    REQUIRE(nds.arm9_bus().read8 (0x0400'0000) == 0);
    nds.arm9_bus().write32(0x0400'0000, 0x1234'5678u);
    nds.arm9_bus().write16(0x0400'0000, 0x1234u);
    nds.arm9_bus().write8 (0x0400'0000, 0x12u);
}

int main() {
    main_ram_round_trip_all_widths();
    main_ram_mirrors_four_times_across_the_region();
    shared_wram_default_mode_is_arm9_32k();
    unmapped_reads_return_zero_and_writes_are_noops();
    io_region_routes_to_nds_stub();
    std::printf("arm9_bus_test: all passed\n");
    return 0;
}
