// Boundary-case coverage for the adc/sbc helpers in arm7_alu.hpp.
// Every ARMv4T arithmetic data-processing instruction's flag output
// funnels through these two functions, so the NZCV correctness for
// the entire CPU rides on this test file.

#include "cpu/arm7/arm7_alu.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void adc_zero_plus_zero_no_carry() {
    AddResult r = adc(0u, 0u, false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == false);
    REQUIRE(r.overflow == false);
}

static void adc_wraparound_sets_carry_no_overflow() {
    // 0xFFFFFFFF + 1 = 0, carry out, no signed overflow (-1 + 1 = 0 is fine).
    AddResult r = adc(0xFFFF'FFFFu, 1u, false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);
    REQUIRE(r.overflow == false);
}

static void adc_signed_overflow_positive_to_negative() {
    // 0x7FFFFFFF + 1 = 0x80000000, no carry, signed overflow (INT_MAX + 1).
    AddResult r = adc(0x7FFF'FFFFu, 1u, false);
    REQUIRE(r.value == 0x8000'0000u);
    REQUIRE(r.carry == false);
    REQUIRE(r.overflow == true);
}

static void adc_two_negatives_set_both_carry_and_overflow() {
    // 0x80000000 + 0x80000000 = 0, carry out, signed overflow (INT_MIN + INT_MIN).
    AddResult r = adc(0x8000'0000u, 0x8000'0000u, false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);
    REQUIRE(r.overflow == true);
}

static void adc_uses_carry_in() {
    // 0xFFFFFFFF + 0xFFFFFFFF + 1 (c_in) = 0x1'FFFFFFFF, low word 0xFFFFFFFF, carry.
    AddResult r = adc(0xFFFF'FFFFu, 0xFFFF'FFFFu, true);
    REQUIRE(r.value == 0xFFFF'FFFFu);
    REQUIRE(r.carry == true);
    REQUIRE(r.overflow == false);  // -1 + -1 = -2, not a signed overflow.
}

static void sbc_plain_sub_zero_minus_zero() {
    // Plain SUB: c_in=true. 0 - 0 = 0, no borrow → c=1, no overflow.
    AddResult r = sbc(0u, 0u, true);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);  // C = NOT Borrow, no borrow means C=1.
    REQUIRE(r.overflow == false);
}

static void sbc_with_borrow_zero_minus_zero_minus_one() {
    // SBC 0 - 0 - 1 (because c_in=false means "borrow in"): result = 0xFFFFFFFF,
    // borrow out → c=0, no signed overflow (0 - 0 - 1 = -1 fits in i32).
    AddResult r = sbc(0u, 0u, false);
    REQUIRE(r.value == 0xFFFF'FFFFu);
    REQUIRE(r.carry == false);
    REQUIRE(r.overflow == false);
}

static void sbc_signed_overflow_int_min_minus_one() {
    // 0x80000000 - 1 = 0x7FFFFFFF, no borrow → c=1, signed overflow
    // (INT_MIN - 1 wraps to INT_MAX).
    AddResult r = sbc(0x8000'0000u, 1u, true);
    REQUIRE(r.value == 0x7FFF'FFFFu);
    REQUIRE(r.carry == true);
    REQUIRE(r.overflow == true);
}

static void sbc_zero_minus_one_borrows() {
    // 0 - 1 = 0xFFFFFFFF, borrow → c=0, no signed overflow (0 - 1 = -1).
    AddResult r = sbc(0u, 1u, true);
    REQUIRE(r.value == 0xFFFF'FFFFu);
    REQUIRE(r.carry == false);
    REQUIRE(r.overflow == false);
}

static void sbc_int_max_minus_neg_one_overflows() {
    // 0x7FFFFFFF - 0xFFFFFFFF = 0x80000000. Unsigned: 0x7FFFFFFF - 0xFFFFFFFF
    // wraps, so borrow → c=0. Signed: INT_MAX - (-1) = INT_MAX + 1 = overflow.
    AddResult r = sbc(0x7FFF'FFFFu, 0xFFFF'FFFFu, true);
    REQUIRE(r.value == 0x8000'0000u);
    REQUIRE(r.carry == false);
    REQUIRE(r.overflow == true);
}

int main() {
    adc_zero_plus_zero_no_carry();
    adc_wraparound_sets_carry_no_overflow();
    adc_signed_overflow_positive_to_negative();
    adc_two_negatives_set_both_carry_and_overflow();
    adc_uses_carry_in();
    sbc_plain_sub_zero_minus_zero();
    sbc_with_borrow_zero_minus_zero_minus_one();
    sbc_zero_minus_one_borrows();
    sbc_signed_overflow_int_min_minus_one();
    sbc_int_max_minus_neg_one_overflows();
    std::puts("arm7_adc_sbc_test OK");
    return 0;
}
