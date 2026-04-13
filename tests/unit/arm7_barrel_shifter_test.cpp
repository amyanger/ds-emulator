// Barrel shifter edge cases. ARMv4T shifts have some non-obvious
// corner cases documented in GBATEK — specifically LSR #0, ASR #0,
// ROR #0 (which is RRX), and LSL #32 (carry-out is bit 0 of operand).

#include "cpu/arm7/arm7_alu.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void lsl_zero_is_identity_with_unchanged_carry() {
    ShifterResult r = barrel_shift_imm(0xF000'0000u, ShiftType::Lsl, 0, /*c_in=*/true);
    REQUIRE(r.value == 0xF000'0000u);
    REQUIRE(r.carry == true);
    r = barrel_shift_imm(0xF000'0000u, ShiftType::Lsl, 0, /*c_in=*/false);
    REQUIRE(r.carry == false);
}

static void lsl_by_1_carries_out_bit_31() {
    ShifterResult r = barrel_shift_imm(0x8000'0001u, ShiftType::Lsl, 1, /*c_in=*/false);
    REQUIRE(r.value == 0x0000'0002u);
    REQUIRE(r.carry == true);
}

static void lsl_by_32_carries_out_bit_0_and_zeroes_value() {
    // Encoded as immediate-shift amount 0 with LSL is LSL #0 (identity),
    // but register-shift amount 32 with LSL produces zero result and
    // carry-out = bit 0 of operand. Our immediate helper does not see
    // amount 32 (imm5 max = 31), so we test this via the "amount" param
    // the register-shift path will use in slice 3b.
    ShifterResult r = barrel_shift_reg(0x0000'0001u, ShiftType::Lsl, 32);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);
}

static void lsr_imm_zero_is_lsr_32() {
    // ARMv4T: LSR #0 in immediate-shift form is actually LSR #32.
    // Result is zero, carry is bit 31 of operand.
    ShifterResult r = barrel_shift_imm(0x8000'0000u, ShiftType::Lsr, 0, /*c_in=*/false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);
}

static void asr_imm_zero_is_asr_32_sign_extends() {
    ShifterResult r = barrel_shift_imm(0x8000'0000u, ShiftType::Asr, 0, /*c_in=*/false);
    REQUIRE(r.value == 0xFFFF'FFFFu);
    REQUIRE(r.carry == true);
    r = barrel_shift_imm(0x7FFF'FFFFu, ShiftType::Asr, 0, /*c_in=*/false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == false);
}

static void ror_imm_zero_is_rrx() {
    // ARMv4T: ROR #0 in immediate-shift form is RRX (rotate-right-extend):
    // result = (C << 31) | (op >> 1), new C = bit 0 of operand.
    ShifterResult r = barrel_shift_imm(0x0000'0001u, ShiftType::Ror, 0, /*c_in=*/true);
    REQUIRE(r.value == 0x8000'0000u);
    REQUIRE(r.carry == true);
    r = barrel_shift_imm(0x0000'0002u, ShiftType::Ror, 0, /*c_in=*/false);
    REQUIRE(r.value == 0x0000'0001u);
    REQUIRE(r.carry == false);
}

static void rotated_immediate_no_rotation_passes_carry_through() {
    // Data-processing immediate with rotate==0: result is the imm8
    // zero-extended, carry-out is c_in unchanged.
    ShifterResult r = rotated_imm(0xFFu, 0, /*c_in=*/true);
    REQUIRE(r.value == 0xFFu);
    REQUIRE(r.carry == true);
}

static void rotated_immediate_with_rotation_updates_carry() {
    // imm8=0xFF, rotate field bits [11:8]=2 → real rotate amount = 4.
    // Result = ROR(0xFF, 4) = 0xF000'000F, carry = bit 31 of result.
    ShifterResult r = rotated_imm(0xFFu, 2, /*c_in=*/false);
    REQUIRE(r.value == 0xF000'000Fu);
    REQUIRE(r.carry == true);  // bit 31 of rotated result
}

int main() {
    lsl_zero_is_identity_with_unchanged_carry();
    lsl_by_1_carries_out_bit_31();
    lsl_by_32_carries_out_bit_0_and_zeroes_value();
    lsr_imm_zero_is_lsr_32();
    asr_imm_zero_is_asr_32_sign_extends();
    ror_imm_zero_is_rrx();
    rotated_immediate_no_rotation_passes_carry_through();
    rotated_immediate_with_rotation_updates_carry();
    std::puts("arm7_barrel_shifter_test OK");
    return 0;
}
