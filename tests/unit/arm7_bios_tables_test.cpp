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

// SWI 0x1A handler: routes through to the lookup, returns 1 cycle, and
// clamps any out-of-range R0 (logging a one-shot warn the test does not
// assert on).
static void get_sine_table_handler_routes_to_lookup() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 32u;
    bios7_get_sine_table(state, nds.arm7_bus());
    REQUIRE(state.r[0] == 0x5A82u);
}

static void get_sine_table_handler_returns_one_cycle() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0u;
    const u32 cycles = bios7_get_sine_table(state, nds.arm7_bus());
    REQUIRE(cycles == 1u);
}

static void get_sine_table_handler_clamps_one_past_end() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0x40u;
    bios7_get_sine_table(state, nds.arm7_bus());
    REQUIRE(state.r[0] == sine_table_lookup(0x3Fu));
}

static void get_sine_table_handler_clamps_max_u32() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0xFFFFFFFFu;
    bios7_get_sine_table(state, nds.arm7_bus());
    REQUIRE(state.r[0] == sine_table_lookup(0x3Fu));
}

// SWI 0x1B PitchTable lookup + handler (slice 3h commit 3).
//
// Table correctness is verified against the formula recomputed in the test
// rather than against an external reference — the spec ships the formula
// values (§4.6, §5.3), and a 0x20 delta versus the GBATEK-documented max is
// expected at the top of the table.

static u16 pitch_formula(u32 i) {
    const double size = static_cast<double>(kPitchTableSize);
    const double ratio = std::pow(2.0, static_cast<double>(i) / size);
    return static_cast<u16>(std::lround((ratio - 1.0) * 65536.0));
}

static void pitch_table_index_zero_returns_zero() {
    REQUIRE(pitch_table_lookup(0) == 0u);
}

// Index 64 is the one-semitone ratio (64/768 = 1/12). Anchor to a hand-worked
// landmark so a formula transcription that also slipped into pitch_formula
// cannot hide behind self-agreement. Value: round(65536 · (2^(1/12) − 1))
// = round(3897.47) = 3897 = 0x0F39. (Spec §5.3 rounds 0.0594631 · 65536 to
// 3898 = 0x0F3A; the exact double arithmetic lands one LSB lower.)
static void pitch_table_index_sixty_four_one_semitone() {
    REQUIRE(pitch_table_lookup(64) == pitch_formula(64));
    REQUIRE(pitch_table_lookup(64) == 0x0F39u);
}

static void pitch_table_index_max_matches_formula() {
    REQUIRE(pitch_table_lookup(767) == pitch_formula(767));
}

// Load-bearing test: locks the production table to the formula the spec
// specifies, byte for byte.
static void pitch_table_full_table_matches_formula() {
    for (u32 i = 0; i < kPitchTableSize; ++i) {
        REQUIRE(pitch_table_lookup(i) == pitch_formula(i));
    }
}

static void pitch_table_is_monotonic_non_decreasing() {
    for (u32 i = 1; i < kPitchTableSize; ++i) {
        REQUIRE(pitch_table_lookup(i) >= pitch_table_lookup(i - 1));
    }
}

static void get_pitch_table_handler_routes_to_lookup() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 64u;
    bios7_get_pitch_table(state, nds.arm7_bus());
    REQUIRE(state.r[0] == pitch_formula(64));
}

static void get_pitch_table_handler_returns_one_cycle() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0u;
    const u32 cycles = bios7_get_pitch_table(state, nds.arm7_bus());
    REQUIRE(cycles == 1u);
}

static void get_pitch_table_handler_clamps_one_past_end() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0x300u;
    bios7_get_pitch_table(state, nds.arm7_bus());
    REQUIRE(state.r[0] == pitch_table_lookup(0x2FFu));
}

static void get_pitch_table_handler_clamps_max_u32() {
    NDS nds;
    auto& state = nds.cpu7().state();
    state.r[0] = 0xFFFFFFFFu;
    bios7_get_pitch_table(state, nds.arm7_bus());
    REQUIRE(state.r[0] == pitch_table_lookup(0x2FFu));
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
    get_sine_table_handler_routes_to_lookup();
    get_sine_table_handler_returns_one_cycle();
    get_sine_table_handler_clamps_one_past_end();
    get_sine_table_handler_clamps_max_u32();
    pitch_table_index_zero_returns_zero();
    pitch_table_index_sixty_four_one_semitone();
    pitch_table_index_max_matches_formula();
    pitch_table_full_table_matches_formula();
    pitch_table_is_monotonic_non_decreasing();
    get_pitch_table_handler_routes_to_lookup();
    get_pitch_table_handler_returns_one_cycle();
    get_pitch_table_handler_clamps_one_past_end();
    get_pitch_table_handler_clamps_max_u32();
    std::puts("arm7_bios_tables_test: all 27 cases passed");
    return 0;
}
