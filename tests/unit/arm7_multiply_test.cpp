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

static void umlal_plain_accumulate() {
    // RdHi:RdLo starts at 0x00000000_00000100
    // product = 2 * 3 = 6
    // result  = 0x00000000_00000106
    NDS nds;
    nds.cpu7().state().r[0] = 0x00000100u;  // RdLo pre-load
    nds.cpu7().state().r[1] = 0x00000000u;  // RdHi pre-load
    nds.cpu7().state().r[2] = 2u;
    nds.cpu7().state().r[3] = 3u;
    run_one(nds, kBase, encode_long_mul(false, true, false, 1, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0x00000106u);
    REQUIRE(nds.cpu7().state().r[1] == 0x00000000u);
}

static void umlal_carry_propagates_lo_to_hi() {
    // RdLo pre-load = 0xFFFFFFFF
    // RdHi pre-load = 0x00000000
    // product = 1 * 2 = 2
    // accumulate: 0x00000000_FFFFFFFF + 2 = 0x00000001_00000001
    NDS nds;
    nds.cpu7().state().r[0] = 0xFFFFFFFFu;
    nds.cpu7().state().r[1] = 0x00000000u;
    nds.cpu7().state().r[2] = 1u;
    nds.cpu7().state().r[3] = 2u;
    run_one(nds, kBase, encode_long_mul(false, true, false, 1, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0x00000001u);
    REQUIRE(nds.cpu7().state().r[1] == 0x00000001u);
}

static void smlal_signed_negative_accumulator() {
    // RdHi:RdLo starts at 0xFFFFFFFF_FFFFFFFF == -1 (signed)
    // product = (-1) * 1 = -1 == 0xFFFFFFFF_FFFFFFFF
    // sum = -2 == 0xFFFFFFFF_FFFFFFFE
    NDS nds;
    nds.cpu7().state().r[0] = 0xFFFFFFFFu;
    nds.cpu7().state().r[1] = 0xFFFFFFFFu;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
    nds.cpu7().state().r[3] = 0x00000001u;  //  1
    run_one(nds, kBase, encode_long_mul(true, true, false, 1, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFEu);
    REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFFu);
}

static void smlals_flag_reflects_full_64_bit_result() {
    // Accumulator starts at 0, product = -1 * 1 = -1.
    // Final 64-bit result = 0xFFFFFFFFFFFFFFFF. N set (bit 63), Z clear.
    NDS nds;
    nds.cpu7().state().r[0] = 0x00000000u;
    nds.cpu7().state().r[1] = 0x00000000u;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
    nds.cpu7().state().r[3] = 0x00000001u;
    run_one(nds, kBase, encode_long_mul(true, true, true, 1, 0, 2, 3));
    REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) != 0);  // N set (bit 63)
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
}

static void mul_rm_equals_rd_warn_path() {
    // Rm == Rd is UNPREDICTABLE on ARMv4T. We warn and execute as written.
    // The result is the standard product; there is no side-effect to Rm
    // before the multiplication, because we read both operands into locals
    // before writing Rd.
    NDS nds;
    nds.cpu7().state().r[0] = 3u;
    nds.cpu7().state().r[1] = 4u;
    // encode_mul(s=false, rd=0, rm=0, rs=1): Rd=R0, Rm=R0, Rs=R1
    run_one(nds, kBase, encode_mul(false, 0, 0, 1));
    REQUIRE(nds.cpu7().state().r[0] == 12u);  // 3 * 4
}

static void mul_rd_equals_r15_warn_path() {
    // Rd == R15 is UNPREDICTABLE. We warn and still perform the write.
    // write_rd masks R15 to 4-byte alignment when it sets state.pc.
    // We use an aligned target so no masking loss occurs, and verify PC
    // lands there (test does NOT verify the post-write pipeline flush
    // because we don't model a real pipeline — R15 is a single register).
    NDS nds;
    nds.cpu7().state().r[1] = 0x02000100u;
    nds.cpu7().state().r[2] = 1u;
    // encode_mul(s=false, rd=15, rm=1, rs=2)
    run_one(nds, kBase, encode_mul(false, 15, 1, 2));
    // 0x02000100 * 1 == 0x02000100; write_rd aligns to word boundary;
    // state.pc is updated because write_rd stomps pc on R15 writes.
    REQUIRE(nds.cpu7().state().pc == 0x02000100u);
}

static void umull_rd_hi_equals_rd_lo_warn_path() {
    // RdHi == RdLo is UNPREDICTABLE. We warn and execute the two writes
    // in the documented order: RdLo first, then RdHi. The second write
    // wins, so the final register value is whatever bit[63:32] of the
    // 64-bit product is.
    NDS nds;
    nds.cpu7().state().r[2] = 0xFFFFFFFFu;
    nds.cpu7().state().r[3] = 0xFFFFFFFFu;
    // encode_long_mul(signed=false, acc=false, s=false, rd_hi=0, rd_lo=0, rm=2, rs=3)
    // Both halves target R0. Product = 0xFFFFFFFE_00000001.
    // RdLo writes 0x00000001 to R0 first, then RdHi writes 0xFFFFFFFE.
    // Second write wins.
    run_one(nds, kBase, encode_long_mul(false, false, false, 0, 0, 2, 3));
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFEu);
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
    umlal_plain_accumulate();
    umlal_carry_propagates_lo_to_hi();
    smlal_signed_negative_accumulator();
    smlals_flag_reflects_full_64_bit_result();
    mul_rm_equals_rd_warn_path();
    mul_rd_equals_r15_warn_path();
    umull_rd_hi_equals_rd_lo_warn_path();
    std::puts("arm7_multiply_test: all 23 cases passed");
    return 0;
}
