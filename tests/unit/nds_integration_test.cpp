// Slice-2 integration test. Proves the bus + WRAMCNT wiring survives a
// full NDS construction + reset + run_frame cycle, and that reset()
// zeroes memory as expected.

#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void run_frame_does_not_touch_bus_state() {
    NDS nds;
    nds.arm9_bus().write32(0x0200'0000, 0xCAFE'BABEu);
    nds.run_frame();
    REQUIRE(nds.arm9_bus().read32(0x0200'0000) == 0xCAFE'BABEu);
}

static void reset_zeroes_all_memory_and_resets_wramcnt() {
    NDS nds;

    // Dirty all three backing arrays + WRAMCNT.
    nds.arm9_bus().write32(0x0200'0000, 0x1111'1111u);
    nds.arm9_bus().write32(0x0300'0000, 0x2222'2222u);  // shared WRAM (mode 0)
    nds.arm7_bus().write32(0x0380'0000, 0x3333'3333u);  // ARM7 WRAM
    nds.arm9_bus().write8(0x0400'0247u, 2);             // WRAMCNT = 2

    // Under mode 2, ARM7 sees the second 16 KB.
    nds.arm7_bus().write32(0x0300'0000, 0x4444'4444u);

    nds.reset();

    REQUIRE(nds.arm9_bus().read32(0x0200'0000) == 0);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);  // back to mode 0, cleared
    REQUIRE(nds.arm7_bus().read32(0x0380'0000) == 0);
    // Mode is back to 0 — ARM7 at 0x0300'0000 mirrors its own cleared WRAM.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0);
}

int main() {
    run_frame_does_not_touch_bus_state();
    reset_zeroes_all_memory_and_resets_wramcnt();
    std::printf("nds_integration_test: all passed\n");
    return 0;
}
