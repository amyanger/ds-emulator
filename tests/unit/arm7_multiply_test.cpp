// arm7_multiply_test.cpp — ARMv4T multiply family tests. Exercises MUL
// and MLA in slice 3b2 Task 5. UMULL/SMULL land in Task 6, UMLAL/SMLAL
// in Task 7, UNPREDICTABLE paths in Task 8.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Preload a single instruction word at `pc`, set PC and R15 for the ARM
// pipeline model, and run exactly one ARM7 cycle.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

constexpr u32 AL_COND = 0xEu << 28;

// MUL: cond 0000000 S Rd 0000 Rs 1001 Rm
u32 encode_mul(bool s, u32 rd, u32 rm, u32 rs) {
    u32 instr = AL_COND | 0x00000090u;
    if (s) instr |= (1u << 20);
    instr |= (rd & 0xFu) << 16;
    instr |= (rs & 0xFu) << 8;
    instr |= (rm & 0xFu);
    return instr;
}

// MLA: cond 0000001 S Rd Rn Rs 1001 Rm
u32 encode_mla(bool s, u32 rd, u32 rm, u32 rs, u32 rn) {
    u32 instr = AL_COND | 0x00200090u;
    if (s) instr |= (1u << 20);
    instr |= (rd & 0xFu) << 16;
    instr |= (rn & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (rm & 0xFu);
    return instr;
}

// Long multiplies: cond 00001 U A S RdHi RdLo Rs 1001 Rm
//   bit 22 (U): 1=signed, 0=unsigned
//   bit 21 (A): 1=accumulate, 0=plain
u32 encode_long_mul(bool signed_mul, bool accumulate, bool s,
                    u32 rd_hi, u32 rd_lo, u32 rm, u32 rs) {
    u32 instr = AL_COND | 0x00800090u;  // bits[27:23]=00001
    if (signed_mul) instr |= (1u << 22);
    if (accumulate) instr |= (1u << 21);
    if (s)          instr |= (1u << 20);
    instr |= (rd_hi & 0xFu) << 16;
    instr |= (rd_lo & 0xFu) << 12;
    instr |= (rs    & 0xFu) << 8;
    instr |= (rm    & 0xFu);
    return instr;
}

}  // namespace

static void mul_plain_small_operands() {
    NDS nds;
    nds.cpu7().state().r[1] = 7u;
    nds.cpu7().state().r[2] = 6u;
    run_one(nds, kBase, encode_mul(false, 0, 1, 2));
    REQUIRE(nds.cpu7().state().r[0] == 42u);
}

static void mul_overflow_low32_truncates() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFFFFFFu;
    nds.cpu7().state().r[2] = 0x2u;
    run_one(nds, kBase, encode_mul(false, 0, 1, 2));
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFEu);
}

static void muls_zero_result_sets_z_clears_n() {
    NDS nds;
    nds.cpu7().state().r[1] = 0u;
    nds.cpu7().state().r[2] = 42u;
    run_one(nds, kBase, encode_mul(true, 0, 1, 2));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
    REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) == 0);  // N clear
}

static void muls_bit31_result_sets_n_clears_z() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x80000000u;
    nds.cpu7().state().r[2] = 1u;
    run_one(nds, kBase, encode_mul(true, 0, 1, 2));
    REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) != 0);  // N set
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
}

static void muls_preserves_c_and_v() {
    NDS nds;
    nds.cpu7().state().r[1] = 7u;
    nds.cpu7().state().r[2] = 6u;
    nds.cpu7().state().cpsr |= (1u << 29) | (1u << 28);  // pre-set C and V
    run_one(nds, kBase, encode_mul(true, 0, 1, 2));
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C preserved
    REQUIRE((nds.cpu7().state().cpsr & (1u << 28)) != 0);  // V preserved
}

static void mla_plain_accumulate() {
    NDS nds;
    nds.cpu7().state().r[1] = 3u;
    nds.cpu7().state().r[2] = 4u;
    nds.cpu7().state().r[3] = 100u;
    run_one(nds, kBase, encode_mla(false, 0, 1, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 112u);  // 3*4 + 100
}

static void mla_accumulator_overflow_wraps() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFFFFFFu;
    nds.cpu7().state().r[2] = 0x2u;
    nds.cpu7().state().r[3] = 0x4u;
    run_one(nds, kBase, encode_mla(false, 0, 1, 2, 3));
    // 0xFFFFFFFF * 2 mod 2^32 = 0xFFFFFFFE
    // 0xFFFFFFFE + 0x4 mod 2^32 = 0x00000002
    REQUIRE(nds.cpu7().state().r[0] == 0x00000002u);
}

static void mlas_flag_behavior_matches_muls() {
    NDS nds;
    nds.cpu7().state().r[1] = 0u;
    nds.cpu7().state().r[2] = 0u;
    nds.cpu7().state().r[3] = 0u;
    run_one(nds, kBase, encode_mla(true, 0, 1, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
}

static void umull_0xffffffff_squared() {
    NDS nds;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;
    nds.cpu7().state().r[3] = 0xFFFFFFFFu;
    run_one(nds, kBase, encode_long_mul(false, false, false, 1, 0, 2, 3));
    // 0xFFFFFFFF * 0xFFFFFFFF == 0xFFFFFFFE_00000001
    REQUIRE(nds.cpu7().state().r[0] == 0x00000001u);  // RdLo
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFEu);  // RdHi
}

static void umull_zero_operand_zeros_both_halves() {
    NDS nds;
    nds.cpu7().state().r[2] = 0u;
    nds.cpu7().state().r[3] = 0xDEADBEEFu;
    run_one(nds, kBase, encode_long_mul(false, false, false, 1, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE(nds.cpu7().state().r[1] == 0u);
}

static void umulls_bit63_clear_clears_n_and_z() {
    NDS nds;
    nds.cpu7().state().r[2] = 2u;
    nds.cpu7().state().r[3] = 3u;
    run_one(nds, kBase, encode_long_mul(false, false, true, 1, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 6u);
    REQUIRE(nds.cpu7().state().r[1] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) == 0);  // N clear
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
}

static void umulls_zero_product_sets_z() {
    NDS nds;
    nds.cpu7().state().r[2] = 0u;
    nds.cpu7().state().r[3] = 0u;
    run_one(nds, kBase, encode_long_mul(false, false, true, 1, 0, 2, 3));
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
}

static void smull_negative_times_negative_is_one() {
    NDS nds;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
    nds.cpu7().state().r[3] = 0xFFFFFFFFu;  // -1
    run_one(nds, kBase, encode_long_mul(true, false, false, 1, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0x00000001u);
    REQUIRE(nds.cpu7().state().r[1] == 0x00000000u);
}

static void smull_negative_times_one_is_negative() {
    NDS nds;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
    nds.cpu7().state().r[3] = 0x00000001u;  //  1
    run_one(nds, kBase, encode_long_mul(true, false, false, 1, 0, 2, 3));
    // -1 sign-extended to 64 bits: 0xFFFFFFFFFFFFFFFF
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFFu);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFFu);
}

static void smull_int_min_squared() {
    NDS nds;
    nds.cpu7().state().r[2] = 0x80000000u;  // INT_MIN
    nds.cpu7().state().r[3] = 0x80000000u;  // INT_MIN
    run_one(nds, kBase, encode_long_mul(true, false, false, 1, 0, 2, 3));
    // (-2^31) * (-2^31) = +2^62 = 0x4000000000000000
    REQUIRE(nds.cpu7().state().r[0] == 0x00000000u);
    REQUIRE(nds.cpu7().state().r[1] == 0x40000000u);
}

static void smulls_negative_product_sets_n() {
    NDS nds;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
    nds.cpu7().state().r[3] = 0x00000001u;
    run_one(nds, kBase, encode_long_mul(true, false, true, 1, 0, 2, 3));
    REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) != 0);  // N set (bit 63 of result)
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
}

int main() {
    mul_plain_small_operands();
    mul_overflow_low32_truncates();
    muls_zero_result_sets_z_clears_n();
    muls_bit31_result_sets_n_clears_z();
    muls_preserves_c_and_v();
    mla_plain_accumulate();
    mla_accumulator_overflow_wraps();
    mlas_flag_behavior_matches_muls();
    umull_0xffffffff_squared();
    umull_zero_operand_zeros_both_halves();
    umulls_bit63_clear_clears_n_and_z();
    umulls_zero_product_sets_z();
    smull_negative_times_negative_is_one();
    smull_negative_times_one_is_negative();
    smull_int_min_squared();
    smulls_negative_product_sets_n();
    std::puts("arm7_multiply_test: all 16 cases passed");
    return 0;
}
