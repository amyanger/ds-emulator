// arm7_bios_tables_test.cpp — ARM7 BIOS HLE tables-/misc-family tests.
// Covers SOUNDBIAS storage today; SWI-level tests get appended as each
// family SWI lands.

#include "bus/io_regs.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void soundbias_reset_value_is_0x0200() {
    NDS nds;
    REQUIRE(nds.soundbias() == 0x0200u);
}

static void soundbias_write32_stores_low_10_bits() {
    NDS nds;

    nds.arm7_bus().write32(IO_SOUNDBIAS, 0xFFFFFFFFu);
    REQUIRE(nds.soundbias() == 0x3FFu);

    nds.arm7_bus().write32(IO_SOUNDBIAS, 0x00000123u);
    REQUIRE(nds.soundbias() == 0x123u);
}

static void soundbias_write16_stores_low_10_bits() {
    NDS nds;

    nds.arm7_bus().write16(IO_SOUNDBIAS, 0xFFFFu);
    REQUIRE(nds.soundbias() == 0x3FFu);

    nds.arm7_bus().write16(IO_SOUNDBIAS, 0x0042u);
    REQUIRE(nds.soundbias() == 0x0042u);
}

// 8-bit writes are unrouted by design — SOUNDBIAS is 16/32-bit only.
static void soundbias_write8_is_ignored() {
    NDS nds;
    REQUIRE(nds.soundbias() == 0x0200u);

    nds.arm7_bus().write8(IO_SOUNDBIAS, 0xFFu);
    REQUIRE(nds.soundbias() == 0x0200u);

    nds.arm7_bus().write8(IO_SOUNDBIAS + 1u, 0xFFu);
    REQUIRE(nds.soundbias() == 0x0200u);
}

static void soundbias_reset_restores_default() {
    NDS nds;

    nds.arm7_bus().write16(IO_SOUNDBIAS, 0x0155u);
    REQUIRE(nds.soundbias() == 0x0155u);

    nds.reset();
    REQUIRE(nds.soundbias() == 0x0200u);
}

int main() {
    soundbias_reset_value_is_0x0200();
    soundbias_write32_stores_low_10_bits();
    soundbias_write16_stores_low_10_bits();
    soundbias_write8_is_ignored();
    soundbias_reset_restores_default();
    std::puts("arm7_bios_tables_test: all 5 cases passed");
    return 0;
}
