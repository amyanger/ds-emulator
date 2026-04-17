// Exercises SWI 0x09 (Div) and SWI 0x0D (Sqrt) by calling the family
// functions directly; the SWI entry paths are covered by
// arm7_bios_dispatch_test.

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

static void div_positive_by_positive() {
    NDS nds;
    nds.cpu7().state().r[0] = 10;
    nds.cpu7().state().r[1] = 3;
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 3u);
    REQUIRE(nds.cpu7().state().r[1] == 1u);
    REQUIRE(nds.cpu7().state().r[3] == 3u);
}

// GBATEK's worked example: Div(-1234, 10) → quotient=-123, remainder=-4,
// |quotient|=123. Remainder takes the sign of the dividend.
static void div_negative_by_positive_matches_gbatek_example() {
    NDS nds;
    nds.cpu7().state().r[0] = static_cast<u32>(-1234);
    nds.cpu7().state().r[1] = 10;
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFF85u); // -123
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFCu); // -4
    REQUIRE(nds.cpu7().state().r[3] == 123u);
}

static void div_positive_by_negative() {
    NDS nds;
    nds.cpu7().state().r[0] = 10;
    nds.cpu7().state().r[1] = static_cast<u32>(-3);
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFDu); // -3
    REQUIRE(nds.cpu7().state().r[1] == 1u);
    REQUIRE(nds.cpu7().state().r[3] == 3u);
}

static void div_negative_by_negative() {
    NDS nds;
    nds.cpu7().state().r[0] = static_cast<u32>(-10);
    nds.cpu7().state().r[1] = static_cast<u32>(-3);
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 3u);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFFu); // -1 (sign of dividend)
    REQUIRE(nds.cpu7().state().r[3] == 3u);
}

static void div_zero_dividend() {
    NDS nds;
    nds.cpu7().state().r[0] = 0;
    nds.cpu7().state().r[1] = 5;
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE(nds.cpu7().state().r[1] == 0u);
    REQUIRE(nds.cpu7().state().r[3] == 0u);
}

static void div_by_one() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x12345678u;
    nds.cpu7().state().r[1] = 1;
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0x12345678u);
    REQUIRE(nds.cpu7().state().r[1] == 0u);
    REQUIRE(nds.cpu7().state().r[3] == 0x12345678u);
}

static void div_by_negative_one() {
    NDS nds;
    nds.cpu7().state().r[0] = 12345;
    nds.cpu7().state().r[1] = static_cast<u32>(-1);
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFCFC7u); // -12345
    REQUIRE(nds.cpu7().state().r[1] == 0u);
    REQUIRE(nds.cpu7().state().r[3] == 12345u);
}

// Relies on 2's-complement wrap; see bios7_math.cpp header.
static void div_int32_min_by_negative_one_overflow() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x80000000u; // INT32_MIN
    nds.cpu7().state().r[1] = static_cast<u32>(-1);
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
    REQUIRE(nds.cpu7().state().r[1] == 0u);
    REQUIRE(nds.cpu7().state().r[3] == 0x80000000u); // -INT32_MIN wraps back
}

// Div-by-zero: HLE returns R0=0, R1=dividend, R3=0 (melonDS compatibility).
static void div_by_zero_returns_dividend_in_r1() {
    NDS nds;
    nds.cpu7().state().r[0] = 42;
    nds.cpu7().state().r[1] = 0;
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE(nds.cpu7().state().r[1] == 42u);
    REQUIRE(nds.cpu7().state().r[3] == 0u);
}

static void div_by_zero_with_negative_dividend() {
    NDS nds;
    nds.cpu7().state().r[0] = static_cast<u32>(-42);
    nds.cpu7().state().r[1] = 0;
    bios7_div(nds.cpu7().state(), nds.arm7_bus());
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFD6u); // -42
    REQUIRE(nds.cpu7().state().r[3] == 0u);
}

int main() {
    sqrt_zero_returns_zero();
    sqrt_one_returns_one();
    sqrt_four_returns_two();
    sqrt_ten_truncates_to_three();
    sqrt_mid_range_0x100_returns_0x10();
    sqrt_max_input_returns_0xffff();
    div_positive_by_positive();
    div_negative_by_positive_matches_gbatek_example();
    div_positive_by_negative();
    div_negative_by_negative();
    div_zero_dividend();
    div_by_one();
    div_by_negative_one();
    div_int32_min_by_negative_one_overflow();
    div_by_zero_returns_dividend_in_r1();
    div_by_zero_with_negative_dividend();
    std::puts("arm7_bios_math_test: all 16 cases passed");
    return 0;
}
