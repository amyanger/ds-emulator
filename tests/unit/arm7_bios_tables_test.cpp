// ARM7 BIOS HLE tables/misc-family tests.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_tables.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>

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

// SWI 0x08 HLE: R0 == 0 writes 0x000 to SOUNDBIAS regardless of previous value.
static void sound_bias_swi_r0_zero_writes_0x000() {
    NDS nds;
    // Seed a non-target sentinel so a no-op would leave it there.
    nds.arm7_bus().write16(IO_SOUNDBIAS, 0x155u);
    REQUIRE(nds.soundbias() == 0x155u);

    auto& state = nds.cpu7().state();
    state.r[0] = 0;
    state.r[1] = 0xDEADBEEFu;
    bios7_sound_bias(state, nds.arm7_bus());

    REQUIRE(nds.soundbias() == 0x000u);
}

// SWI 0x08 HLE: any non-zero R0 writes exactly 0x200 (not R0 itself).
static void sound_bias_swi_r0_nonzero_writes_0x200() {
    NDS nds;
    nds.arm7_bus().write16(IO_SOUNDBIAS, 0x155u);
    REQUIRE(nds.soundbias() == 0x155u);

    auto& state = nds.cpu7().state();
    state.r[0] = 1;
    state.r[1] = 0;
    bios7_sound_bias(state, nds.arm7_bus());
    REQUIRE(nds.soundbias() == 0x200u);

    // Verify a large non-1 R0 still produces 0x200, not R0 truncated.
    nds.arm7_bus().write16(IO_SOUNDBIAS, 0x155u);
    state.r[0] = 0xDEADBEEFu;
    bios7_sound_bias(state, nds.arm7_bus());
    REQUIRE(nds.soundbias() == 0x200u);
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

// SWI 0x1A SineTable lookup (handler wire-up lands in commit 2).

static u16 sine_formula(u32 i) {
    const double angle = (static_cast<double>(i) * std::numbers::pi) / 128.0;
    return static_cast<u16>(std::lround(std::sin(angle) * 32768.0));
}

static void sine_table_index_zero_returns_zero() {
    REQUIRE(sine_table_lookup(0) == 0u);
}

static void sine_table_index_thirty_two_is_quarter_pi() {
    REQUIRE(sine_table_lookup(32) == sine_formula(32));
    REQUIRE(sine_table_lookup(32) == 0x5A82u);
}

// GBATEK documents the table max as 0x7FF5; this test only locks formula
// equivalence, not GBATEK's exact byte.
static void sine_table_index_sixty_three_matches_formula() {
    REQUIRE(sine_table_lookup(63) == sine_formula(63));
}

static void sine_table_full_table_matches_formula() {
    for (u32 i = 0; i < kSineTableSize; ++i) {
        REQUIRE(sine_table_lookup(i) == sine_formula(i));
    }
}

static void sine_table_is_monotonic_non_decreasing() {
    for (u32 i = 1; i < kSineTableSize; ++i) {
        REQUIRE(sine_table_lookup(i) >= sine_table_lookup(i - 1));
    }
}

int main() {
    soundbias_reset_value_is_0x0200();
    soundbias_write32_stores_low_10_bits();
    soundbias_write16_stores_low_10_bits();
    soundbias_write8_is_ignored();
    soundbias_reset_restores_default();
    sound_bias_swi_r0_zero_writes_0x000();
    sound_bias_swi_r0_nonzero_writes_0x200();
    is_debugger_returns_zero();
    get_boot_procs_returns_zeros_in_r0_r1_r3();
    sine_table_index_zero_returns_zero();
    sine_table_index_thirty_two_is_quarter_pi();
    sine_table_index_sixty_three_matches_formula();
    sine_table_full_table_matches_formula();
    sine_table_is_monotonic_non_decreasing();
    std::puts("arm7_bios_tables_test: all 14 cases passed");
    return 0;
}
