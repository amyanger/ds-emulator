// ARM7 BIOS HLE tables/misc-family tests.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_tables.hpp"
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

static void is_debugger_returns_zero() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xDEADBEEFu;
    bios7_is_debugger(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0u);
}

// Seed non-zero sentinels in the output registers so the readback proves
// the SWI cleared them, not that they were already zero. R2 is an input
// per GBATEK ("incoming r2") and must be preserved.
static void get_boot_procs_returns_zeros_in_r0_r1_r3() {
    NDS nds;
    auto& state = nds.cpu7().state();

    state.r[0] = 0xDEADBEEFu;
    state.r[1] = 0xDEADBEEFu;
    state.r[2] = 0xCAFEBABEu;
    state.r[3] = 0xDEADBEEFu;

    bios7_get_boot_procs(state, nds.arm7_bus());

    REQUIRE(state.r[0] == 0u);
    REQUIRE(state.r[1] == 0u);
    REQUIRE(state.r[2] == 0xCAFEBABEu);
    REQUIRE(state.r[3] == 0u);
}

int main() {
    soundbias_reset_value_is_0x0200();
    soundbias_write32_stores_low_10_bits();
    soundbias_write16_stores_low_10_bits();
    soundbias_write8_is_ignored();
    soundbias_reset_restores_default();
    is_debugger_returns_zero();
    get_boot_procs_returns_zeros_in_r0_r1_r3();
    std::puts("arm7_bios_tables_test: all 7 cases passed");
    return 0;
}
