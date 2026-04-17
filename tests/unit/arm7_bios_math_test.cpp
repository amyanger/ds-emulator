// Exercises SWI 0x0D (Sqrt) by calling the family function directly;
// the SWI entry path is covered by arm7_bios_dispatch_test.

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_math.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void sqrt_zero_returns_zero() {
    NDS nds;
    nds.cpu7().state().r[0] = 0;
    bios7_sqrt(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0u);
}

static void sqrt_one_returns_one() {
    NDS nds;
    nds.cpu7().state().r[0] = 1;
    bios7_sqrt(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 1u);
}

static void sqrt_four_returns_two() {
    NDS nds;
    nds.cpu7().state().r[0] = 4;
    bios7_sqrt(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 2u);
}

// Non-perfect square — truncates toward zero (3*3=9 <= 10 < 4*4=16).
static void sqrt_ten_truncates_to_three() {
    NDS nds;
    nds.cpu7().state().r[0] = 10;
    bios7_sqrt(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 3u);
}

static void sqrt_mid_range_0x100_returns_0x10() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x100u;
    bios7_sqrt(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0x10u);
}

// Boundary — largest u32 input. floor(sqrt(0xFFFFFFFF)) == 0xFFFF exactly.
// Also proves the result is masked to 16 bits per BIOS behavior.
static void sqrt_max_input_returns_0xffff() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xFFFFFFFFu;
    bios7_sqrt(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFu);
}

int main() {
    sqrt_zero_returns_zero();
    sqrt_one_returns_one();
    sqrt_four_returns_two();
    sqrt_ten_truncates_to_three();
    sqrt_mid_range_0x100_returns_0x10();
    sqrt_max_input_returns_0xffff();
    std::puts("arm7_bios_math_test: all 6 cases passed");
    return 0;
}
