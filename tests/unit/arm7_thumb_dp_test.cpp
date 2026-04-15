// arm7_thumb_dp_test.cpp — THUMB.1 shift imm + THUMB.3 imm8 DP tests.
// THUMB.1: LSL/LSR/ASR imm5 with ARMv4T zero-amount quirks.
// THUMB.3: MOV/CMP/ADD/SUB imm8 with MOV NZ-only flag rule.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

// Encode THUMB.1: 000 op2 off5 Rs3 Rd3  (op: 0=LSL, 1=LSR, 2=ASR)
static u16 thumb1(u32 op, u32 offset, u32 rs, u32 rd) {
    return static_cast<u16>((op << 11) | (offset << 6) | (rs << 3) | rd);
}

// Encode THUMB.3: 001 op2 Rd3 imm8
static u16 thumb3(u32 op, u32 rd, u32 imm8) {
    return static_cast<u16>((0b001u << 13) | (op << 11) | (rd << 8) | imm8);
}

static constexpr u32 BASE = 0x0380'0000u;

static bool flag_n(u32 cpsr) {
    return (cpsr & (1u << 31)) != 0;
}
static bool flag_z(u32 cpsr) {
    return (cpsr & (1u << 30)) != 0;
}
static bool flag_c(u32 cpsr) {
    return (cpsr & (1u << 29)) != 0;
}
static bool flag_v(u32 cpsr) {
    return (cpsr & (1u << 28)) != 0;
}

static void setup_thumb(NDS& nds, u32 addr) {
    nds.cpu7().state().pc = addr;
    nds.cpu7().state().cpsr |= (1u << 5);
}

static void step_one(NDS& nds) {
    const u64 before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((before + 1) * 2);
}

// Encode THUMB.2: 00011 I op Rn3/imm3 Rs3 Rd3
// I=0,op=0: ADD reg  I=0,op=1: SUB reg  I=1,op=0: ADD imm3  I=1,op=1: SUB imm3
static u16 thumb2(u32 i, u32 op, u32 rn_or_imm, u32 rs, u32 rd) {
    return static_cast<u16>((0b00011u << 11) | (i << 10) | (op << 9) | (rn_or_imm << 6) |
                            (rs << 3) | rd);
}

// ==== THUMB.2 — add/sub ====

static void thumb2_add_reg() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(0, 0, 2, 1, 0)); // ADD R0, R1, R2
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = 100;
    nds.cpu7().state().r[2] = 50;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 150);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
}

static void thumb2_sub_reg() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(0, 1, 3, 4, 5)); // SUB R5, R4, R3
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 200;
    nds.cpu7().state().r[3] = 50;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 150);
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // no borrow
}

static void thumb2_add_imm3() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(1, 0, 7, 0, 1)); // ADD R1, R0, #7
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 10;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 17);
}

static void thumb2_sub_imm3() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(1, 1, 3, 6, 7)); // SUB R7, R6, #3
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[6] = 10;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[7] == 7);
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // no borrow
}

static void thumb2_mov_pseudo_updates_all_flags() {
    // ADD Rd, Rs, #0 is the MOV pseudo. GBATEK: full NZCV even for #0.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(1, 0, 0, 2, 3)); // ADD R3, R2, #0
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[2] = 0;
    nds.cpu7().state().cpsr |= (1u << 29) | (1u << 28); // C=1, V=1 before
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // 0+0 = no carry
    REQUIRE(!flag_v(nds.cpu7().state().cpsr)); // 0+0 = no overflow
}

static void thumb2_sub_underflow() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(1, 1, 1, 0, 1)); // SUB R1, R0, #1
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFF'FFFFu);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // borrow
}

static void thumb2_add_signed_overflow() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb2(1, 0, 1, 4, 5)); // ADD R5, R4, #1
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 0x7FFF'FFFFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 0x8000'0000u);
    REQUIRE(flag_v(nds.cpu7().state().cpsr));
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
}

// ==== THUMB.1 — shift imm ====

static void thumb1_lsl_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(0, 4, 1, 0)); // LSL R0, R1, #4
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = 0x0000'000Fu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0x0000'00F0u);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
}

static void thumb1_lsl_zero_no_shift_c_preserved() {
    // LSL #0 = no shift, C flag unchanged.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(0, 0, 2, 3)); // LSL R3, R2, #0
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[2] = 42;
    nds.cpu7().state().cpsr |= (1u << 29); // C=1 before
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 42);
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // C preserved
}

static void thumb1_lsl_zero_c_clear_preserved() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(0, 0, 4, 5)); // LSL R5, R4, #0
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 1;
    nds.cpu7().state().cpsr &= ~(1u << 29); // C=0 before
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 1);
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // C preserved
}

static void thumb1_lsl_sets_c_from_last_shifted_out() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(0, 1, 0, 1)); // LSL R1, R0, #1
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x8000'0000u; // bit 31 set
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // bit 31 shifted out
}

static void thumb1_lsr_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(1, 8, 0, 1)); // LSR R1, R0, #8
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xFF00u;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFu);
}

static void thumb1_lsr_zero_means_32() {
    // LSR #0 encodes shift-by-32: result = 0, C = bit 31 of source.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(1, 0, 3, 4)); // LSR R4, R3, #0 (=32)
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = 0x8000'0001u;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[4] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // bit 31 was set
}

static void thumb1_lsr_zero_c_clear_when_bit31_clear() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(1, 0, 5, 6)); // LSR R6, R5, #0 (=32)
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[5] = 0x7FFF'FFFFu; // bit 31 clear
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[6] == 0);
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // bit 31 was clear
}

static void thumb1_asr_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(2, 16, 0, 1)); // ASR R1, R0, #16
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xFFFF'0000u; // negative
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFF'FFFFu); // sign-extended
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
}

static void thumb1_asr_zero_means_32_negative() {
    // ASR #0 encodes shift-by-32: if negative, result = 0xFFFFFFFF, C = 1.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(2, 0, 2, 3)); // ASR R3, R2, #0 (=32)
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[2] = 0x8000'0000u;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 0xFFFF'FFFFu);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

static void thumb1_asr_zero_means_32_positive() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(2, 0, 4, 5)); // ASR R5, R4, #0 (=32)
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 0x7FFF'FFFFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
}

static void thumb1_v_flag_preserved() {
    // All THUMB.1 ops leave V unchanged.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb1(0, 4, 0, 1)); // LSL R1, R0, #4
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 1;
    nds.cpu7().state().cpsr |= (1u << 28); // V=1 before
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 16);
    REQUIRE(flag_v(nds.cpu7().state().cpsr)); // V preserved
}

// ==== THUMB.3 — MOV/CMP/ADD/SUB imm8 ====

// ---- MOV ----

static void thumb3_mov_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 3, 42));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 42);
}

static void thumb3_mov_zero_sets_z_clears_n() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 0, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().cpsr |= (1u << 31); // N set before
    nds.cpu7().state().r[0] = 0xDEAD'BEEFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
}

static void thumb3_mov_preserves_c_and_v_when_set() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 1, 0xFF));
    setup_thumb(nds, BASE);
    nds.cpu7().state().cpsr |= (1u << 29) | (1u << 28); // C=1, V=1
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0xFF);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // preserved
    REQUIRE(flag_v(nds.cpu7().state().cpsr)); // preserved
}

static void thumb3_mov_preserves_c_and_v_when_clear() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 2, 5));
    setup_thumb(nds, BASE);
    nds.cpu7().state().cpsr &= ~((1u << 29) | (1u << 28)); // C=0, V=0
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[2] == 5);
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
    REQUIRE(!flag_v(nds.cpu7().state().cpsr));
}

static void thumb3_mov_max_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 7, 0xFF));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[7] == 0xFF);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
}

// ---- CMP ----

static void thumb3_cmp_equal_sets_z_and_c() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 0, 42));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 42;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 42); // no writeback
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // a >= b, no borrow
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_v(nds.cpu7().state().cpsr));
}

static void thumb3_cmp_less_sets_n_clears_c() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 1, 100));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = 50;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 50);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // borrow
}

static void thumb3_cmp_greater() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 2, 10));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[2] = 200;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[2] == 200);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

static void thumb3_cmp_clears_stale_v() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 3, 5));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = 10;
    nds.cpu7().state().cpsr |= (1u << 28); // V=1 before
    step_one(nds);
    // 10 - 5 = 5: no signed overflow, V must be cleared
    REQUIRE(nds.cpu7().state().r[3] == 10);
    REQUIRE(!flag_v(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

// ---- ADD ----

static void thumb3_add_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 3, 10));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = 100;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 110);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
}

static void thumb3_add_wrap_to_zero() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 0, 1));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xFFFF'FFFFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // carry out
}

static void thumb3_add_signed_overflow() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 4, 1));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 0x7FFF'FFFFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[4] == 0x8000'0000u);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(flag_v(nds.cpu7().state().cpsr));
}

static void thumb3_add_zero_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 5, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[5] = 42;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 42);
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
}

// ---- SUB ----

static void thumb3_sub_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 5, 20));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[5] = 100;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 80);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // no borrow
}

static void thumb3_sub_to_zero() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 6, 50));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[6] = 50;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[6] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // equal means no borrow
}

static void thumb3_sub_underflow() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 7, 1));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[7] = 0;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[7] == 0xFFFF'FFFFu);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // borrow
}

static void thumb3_sub_max_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 0, 0xFF));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x100;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 1);
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

int main() {
    // THUMB.2
    thumb2_add_reg();
    thumb2_sub_reg();
    thumb2_add_imm3();
    thumb2_sub_imm3();
    thumb2_mov_pseudo_updates_all_flags();
    thumb2_sub_underflow();
    thumb2_add_signed_overflow();

    // THUMB.1
    thumb1_lsl_basic();
    thumb1_lsl_zero_no_shift_c_preserved();
    thumb1_lsl_zero_c_clear_preserved();
    thumb1_lsl_sets_c_from_last_shifted_out();
    thumb1_lsr_basic();
    thumb1_lsr_zero_means_32();
    thumb1_lsr_zero_c_clear_when_bit31_clear();
    thumb1_asr_basic();
    thumb1_asr_zero_means_32_negative();
    thumb1_asr_zero_means_32_positive();
    thumb1_v_flag_preserved();

    // THUMB.3
    thumb3_mov_basic();
    thumb3_mov_zero_sets_z_clears_n();
    thumb3_mov_preserves_c_and_v_when_set();
    thumb3_mov_preserves_c_and_v_when_clear();
    thumb3_mov_max_imm();

    thumb3_cmp_equal_sets_z_and_c();
    thumb3_cmp_less_sets_n_clears_c();
    thumb3_cmp_greater();
    thumb3_cmp_clears_stale_v();

    thumb3_add_basic();
    thumb3_add_wrap_to_zero();
    thumb3_add_signed_overflow();
    thumb3_add_zero_imm();

    thumb3_sub_basic();
    thumb3_sub_to_zero();
    thumb3_sub_underflow();
    thumb3_sub_max_imm();

    return 0;
}
