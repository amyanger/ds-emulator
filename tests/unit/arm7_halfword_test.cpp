// arm7_halfword_test.cpp — ARMv4T halfword and signed-byte transfer
// tests. Task 4 covers the LDRH happy path (aligned addresses, both
// offset forms, every P/U/W combination, zero-extension). Task 5 adds
// STRH coverage: same addressing matrix, the low-16 truncation rule,
// and the Rd==R15 pipeline quirk (reads as PC+12). Unaligned
// LDRH/STRH/LDRSH behavior is pinned (matches melonDS — mask bit 0,
// read or write the aligned halfword).

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
    nds.cpu7().state().pc    = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

// Encode the immediate-offset form of a halfword transfer.
//   cond | 000 | P | U | 1 | W | L | Rn | Rd | imm[7:4] | 1 SH 1 | imm[3:0]
static u32 encode_halfword_imm(u8 cond, u8 p, u8 u, u8 w, u8 l,
                               u8 rn, u8 rd, u8 sh, u8 offset8) {
    return (u32(cond) << 28)
         | (0b000u << 25)               // bits[27:25] = 000
         | (u32(p) << 24)
         | (u32(u) << 23)
         | (1u << 22)                   // I = 1 (immediate)
         | (u32(w) << 21)
         | (u32(l) << 20)
         | (u32(rn) << 16)
         | (u32(rd) << 12)
         | (u32(offset8 & 0xF0u) << 4)  // upper 4 bits -> bits[11:8]
         | (1u << 7)                    // fixed bit 7
         | (u32(sh) << 5)
         | (1u << 4)                    // fixed bit 4
         | u32(offset8 & 0x0Fu);        // lower 4 bits -> bits[3:0]
}

// Encode the register-offset form of a halfword transfer.
//   cond | 000 | P | U | 0 | W | L | Rn | Rd | 0000 | 1 SH 1 | Rm
static u32 encode_halfword_reg(u8 cond, u8 p, u8 u, u8 w, u8 l,
                               u8 rn, u8 rd, u8 sh, u8 rm) {
    return (u32(cond) << 28)
         | (0b000u << 25)
         | (u32(p) << 24)
         | (u32(u) << 23)
         // I = 0 (register offset)
         | (u32(w) << 21)
         | (u32(l) << 20)
         | (u32(rn) << 16)
         | (u32(rd) << 12)
         // bits[11:8] = 0
         | (1u << 7)
         | (u32(sh) << 5)
         | (1u << 4)
         | u32(rm);
}

constexpr u8 kCondAL = 0xE;
constexpr u8 kSHhalf = 1;  // SH=1 -> halfword unsigned
constexpr u8 kSHsb   = 2;  // SH=2 -> signed byte
constexpr u8 kSHsh   = 3;  // SH=3 -> signed halfword

}  // namespace

// ---- Basic aligned loads ---------------------------------------------------

// LDRH R0, [R1, #4]  — pre-index, up, no writeback.
static void test_ldrh_imm_preindex_up_no_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.arm7_bus().write16(0x0380'0104u, 0xBEEFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/4));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'BEEFu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0100u);  // unchanged
}

// LDRH R0, [R1, #4]!  — pre-index, up, writeback.
static void test_ldrh_imm_preindex_up_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.arm7_bus().write16(0x0380'0104u, 0xBEEFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/4));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'BEEFu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0104u);
}

// LDRH R0, [R1, #-2]!  — pre-index, down, writeback.
static void test_ldrh_imm_preindex_down_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0106u;
    nds.arm7_bus().write16(0x0380'0104u, 0x1234u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/0, /*W*/1, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'1234u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0104u);
}

// LDRH R0, [R1], #2  — post-index, up.
static void test_ldrh_imm_postindex_up() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0108u;
    nds.arm7_bus().write16(0x0380'0108u, 0xCAFEu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/0, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'CAFEu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'010Au);
}

// LDRH R0, [R1], #-2  — post-index, down.
static void test_ldrh_imm_postindex_down() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'010Au;
    nds.arm7_bus().write16(0x0380'010Au, 0xDEADu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/0, /*U*/0, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'DEADu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0108u);
}

// LDRH R0, [R1, R2]  — register offset, pre-index, up, no writeback.
static void test_ldrh_reg_preindex_up_no_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.cpu7().state().r[2] = 0x10u;
    nds.arm7_bus().write16(0x0380'0110u, 0x5678u);

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*Rm*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'5678u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0100u);
    REQUIRE(nds.cpu7().state().r[2] == 0x10u);
}

// ---- Zero-extension verification ------------------------------------------

// The whole point of LDRH vs LDRSH: the high 16 bits must be zero even
// when bit 15 of the loaded halfword is set.
static void test_ldrh_zero_extension() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.arm7_bus().write16(0x0380'0200u, 0x8000u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'8000u);  // not 0xFFFF'8000
}

// ---- Offset composition edge cases ----------------------------------------

// Immediate offset 0xFE — exercises both halves of the split imm field:
// upper nibble (0xF -> bits[11:8]) and lower nibble (0xE -> bits[3:0]).
static void test_ldrh_imm_offset_0xFE() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.arm7_bus().write16(0x0380'01FEu, 0xBABEu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/0xFE));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'BABEu);
}

// LDRH R0, [R1]  — offset 0 / load from Rn itself.
static void test_ldrh_imm_offset_zero() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0220u;
    nds.arm7_bus().write16(0x0380'0220u, 0x00FFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'00FFu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0220u);
}

// ---- Unaligned LDRH (pinned: matches melonDS, no rotate) ------------------

// LDRH R0, [R1, #5]  — odd effective address 0x0380'0105. The ARM7TDMI
// rotate-by-8 quirk is NOT modeled (matches melonDS ARMv4::DataRead16):
// bit 0 is masked and we read the aligned halfword at 0x0380'0104.
static void test_ldrh_unaligned_odd_address_imm_masks_low_bit() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.arm7_bus().write16(0x0380'0104u, 0xBEEFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/5));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'BEEFu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0100u);  // unchanged
}

// LDRH R0, [R1, R2]  — odd effective address via register offset. Same
// rule: no rotate, mask bit 0, aligned halfword read (matches melonDS).
static void test_ldrh_unaligned_odd_address_reg_masks_low_bit() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.cpu7().state().r[2] = 0x0000'0001u;
    nds.arm7_bus().write16(0x0380'0200u, 0xCAFEu);

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*Rm*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'CAFEu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0200u);  // unchanged
    REQUIRE(nds.cpu7().state().r[2] == 0x0000'0001u);  // unchanged
}

// ---- STRH aligned cases ---------------------------------------------------

// STRH R0, [R1, #4]  — pre-index, up, no writeback.
static void test_strh_imm_preindex_up_no_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.cpu7().state().r[0] = 0x0000'ABCDu;

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/4));

    REQUIRE(nds.arm7_bus().read16(0x0380'0204u) == 0xABCDu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0200u);  // unchanged
}

// STRH R0, [R1, #4]!  — pre-index, up, writeback.
static void test_strh_imm_preindex_up_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.cpu7().state().r[0] = 0x0000'ABCDu;

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/4));

    REQUIRE(nds.arm7_bus().read16(0x0380'0204u) == 0xABCDu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0204u);
}

// STRH R0, [R1, #-2]!  — pre-index, down, writeback.
static void test_strh_imm_preindex_down_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0206u;
    nds.cpu7().state().r[0] = 0x0000'1234u;

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/0, /*W*/1, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/2));

    REQUIRE(nds.arm7_bus().read16(0x0380'0204u) == 0x1234u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0204u);
}

// STRH R0, [R1], #2  — post-index, up. Access happens at the original
// Rn, then Rn is advanced by the offset.
static void test_strh_imm_postindex_up() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0208u;
    nds.cpu7().state().r[0] = 0x0000'CAFEu;

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/0, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/2));

    REQUIRE(nds.arm7_bus().read16(0x0380'0208u) == 0xCAFEu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'020Au);
}

// STRH R0, [R1, R2]  — register offset, pre-index, up, no writeback.
static void test_strh_reg_preindex_up_no_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.cpu7().state().r[2] = 0x10u;
    nds.cpu7().state().r[0] = 0x0000'5678u;

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*Rm*/2));

    REQUIRE(nds.arm7_bus().read16(0x0380'0210u) == 0x5678u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0200u);
    REQUIRE(nds.cpu7().state().r[2] == 0x10u);
}

// STRH must write only the low 16 bits of Rd and must not touch the
// adjacent halfword. Pre-seed the containing word so the upper half has
// a known sentinel; after STRH, the lower half should be BEEF (low 16
// of 0xDEAD_BEEF) and the upper half should still be FACE.
static void test_strh_only_low_16_bits_written() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0300u;
    nds.cpu7().state().r[0] = 0xDEAD'BEEFu;
    // Little-endian: word 0xFACE0000 puts 0x0000 at +0, 0xFACE at +2.
    nds.arm7_bus().write32(0x0380'0300u, 0xFACE'0000u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/0));

    REQUIRE(nds.arm7_bus().read16(0x0380'0300u) == 0xBEEFu);   // low 16 only
    REQUIRE(nds.arm7_bus().read16(0x0380'0302u) == 0xFACEu);   // untouched
    // Little-endian word read: low half first, high half second.
    REQUIRE(nds.arm7_bus().read32(0x0380'0300u) == 0xFACE'BEEFu);
}

// STRH PC, [R0]  — Rd==R15 reads as PC+12 (= instr_addr + 12), matching
// the word-STR pipeline convention. The stored halfword is the low 16
// bits of that value. With instr_addr = kBase = 0x03800000, the expected
// stored halfword is (0x03800000 + 12) & 0xFFFF = 0x000C.
static void test_strh_rd_eq_r15_reads_pc_plus_12() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x0380'0400u;
    nds.arm7_bus().write16(0x0380'0400u, 0xAAAAu);  // sentinel

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/0, /*Rd*/15, kSHhalf, /*off*/0));

    // (kBase + 12) & 0xFFFF == 0x000C
    REQUIRE(nds.arm7_bus().read16(0x0380'0400u) == 0x000Cu);
}

// ---- Unaligned STRH (pinned: matches melonDS, mask bit 0) -----------------

// STRH R0, [R1, #5]  — odd effective address 0x0380'0105. STRH masks
// bit 0 on odd addresses and writes the aligned halfword — no rotate,
// no exception (matches melonDS ARMv4::DataWrite16).
static void test_strh_unaligned_odd_address_imm_masks_low_bit() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xDEAD'BEEFu;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.arm7_bus().write16(0x0380'0104u, 0x0000u);  // clean slate
    nds.arm7_bus().write16(0x0380'0106u, 0x0000u);  // clean slate

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*off*/5));

    REQUIRE(nds.arm7_bus().read16(0x0380'0104u) == 0xBEEFu);  // addr & ~1
    REQUIRE(nds.arm7_bus().read16(0x0380'0106u) == 0x0000u);  // untouched
    REQUIRE(nds.cpu7().state().r[0] == 0xDEAD'BEEFu);         // unchanged
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0100u);         // unchanged
}

// STRH R0, [R1, R2]  — odd effective address via register offset. Same
// rule: mask bit 0, write the aligned halfword (matches melonDS).
static void test_strh_unaligned_odd_address_reg_masks_low_bit() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xCAFE'BABEu;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.cpu7().state().r[2] = 0x0000'0001u;
    nds.arm7_bus().write16(0x0380'0200u, 0x0000u);  // clean slate
    nds.arm7_bus().write16(0x0380'0202u, 0x0000u);  // clean slate

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/0,
                                /*Rn*/1, /*Rd*/0, kSHhalf, /*Rm*/2));

    REQUIRE(nds.arm7_bus().read16(0x0380'0200u) == 0xBABEu);  // addr & ~1
    REQUIRE(nds.arm7_bus().read16(0x0380'0202u) == 0x0000u);  // untouched
    REQUIRE(nds.cpu7().state().r[0] == 0xCAFE'BABEu);         // unchanged
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0200u);         // unchanged
    REQUIRE(nds.cpu7().state().r[2] == 0x0000'0001u);         // unchanged
}

// ---- LDRSB cases ----------------------------------------------------------

// LDRSB R0, [R1]  — positive byte (bit 7 clear). Must zero-extend
// because the sign bit is zero.
static void test_ldrsb_positive_byte() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0500u;
    nds.arm7_bus().write8(0x0380'0500u, 0x7Fu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsb, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'007Fu);
}

// LDRSB R0, [R1]  — negative byte 0x80 (bit 7 set, minimum signed i8).
// Must sign-extend to 0xFFFF'FF80.
static void test_ldrsb_negative_byte_sign_extended() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0501u;
    nds.arm7_bus().write8(0x0380'0501u, 0x80u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsb, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FF80u);
}

// LDRSB R0, [R1]  — 0xFF is the strongest sign-extension verification:
// every bit of the result must be 1.
static void test_ldrsb_negative_byte_ff() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0502u;
    nds.arm7_bus().write8(0x0380'0502u, 0xFFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsb, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
}

// LDRSB R0, [R1], #4  — post-index form with a negative byte. Verifies
// sign-extension and post-index writeback compose correctly.
static void test_ldrsb_imm_postindex_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0300u;
    nds.arm7_bus().write8(0x0380'0300u, 0x81u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/0, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsb, /*off*/4));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FF81u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0304u);
}

// LDRSB R0, [R1, R2]  — register-offset form, positive byte.
static void test_ldrsb_reg_preindex_up() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0300u;
    nds.cpu7().state().r[2] = 0x10u;
    nds.arm7_bus().write8(0x0380'0310u, 0x40u);

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsb, /*Rm*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'0040u);
}

// LDRSB R0, [R1]  — odd (unaligned) address. Byte loads have NO
// alignment rule, so this must work cleanly with no mask applied.
static void test_ldrsb_unaligned_address_is_fine() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0305u;  // odd
    nds.arm7_bus().write8(0x0380'0305u, 0xC3u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsb, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFC3u);
}

// ---- LDRSH cases ----------------------------------------------------------

// LDRSH R0, [R1]  — positive halfword 0x1234 (bit 15 clear). The high 16
// bits must be zero because the sign bit is zero.
static void test_ldrsh_positive_halfword() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0600u;
    nds.arm7_bus().write16(0x0380'0600u, 0x1234u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'1234u);
}

// LDRSH R0, [R1]  — 0x8000 is the minimum signed i16 (bit 15 set).
// Must sign-extend to 0xFFFF'8000.
static void test_ldrsh_negative_halfword_sign_extended() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0602u;
    nds.arm7_bus().write16(0x0380'0602u, 0x8000u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'8000u);
}

// LDRSH R0, [R1]  — 0xFFFF is the strongest sign-extension verification:
// every bit of the result must be 1.
static void test_ldrsh_negative_halfword_ffff() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0604u;
    nds.arm7_bus().write16(0x0380'0604u, 0xFFFFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
}

// LDRSH R0, [R1, #4]!  — pre-index, up, writeback. Target halfword 0xABCD
// has bit 15 set, so R0 must be 0xFFFF'ABCD, and R1 must update to the
// pre-indexed address.
static void test_ldrsh_imm_preindex_up_writeback() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0700u;
    nds.arm7_bus().write16(0x0380'0704u, 0xABCDu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/4));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'ABCDu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0704u);
}

// LDRSH R0, [R1], #-2  — post-index, down. Access happens at the
// original Rn, then Rn is decremented by 2.
static void test_ldrsh_imm_postindex_down() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0710u;
    nds.arm7_bus().write16(0x0380'0710u, 0x7FFFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/0, /*U*/0, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'7FFFu);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'070Eu);
}

// LDRSH R0, [R1, R2]  — register-offset form, negative halfword.
static void test_ldrsh_reg_preindex_up() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0720u;
    nds.cpu7().state().r[2] = 0x10u;
    nds.arm7_bus().write16(0x0380'0730u, 0x9001u);

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*Rm*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'9001u);
}

// LDRSH R0, [R1]  — 0x7FFF is the largest positive i16. Verifies the
// boundary just below the sign-extension threshold: high 16 must be zero.
static void test_ldrsh_positive_halfword_7fff() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0740u;
    nds.arm7_bus().write16(0x0380'0740u, 0x7FFFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/0));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'7FFFu);
}

// ---- Unaligned LDRSH (pinned: matches melonDS, no byte-read quirk) --------

// LDRSH R0, [R1, #5]  — odd effective address 0x0380'0105. The ARM7TDMI
// TRM documents an odd-address quirk where LDRSH degenerates to an LDRSB
// of the single odd byte, but melonDS does NOT model it (see A_LDRSH in
// src/ARMInterpreter_LoadStore.cpp): bit 0 is masked and we read the
// aligned halfword at 0x0380'0104, then sign-extend. The sentinel 0x80F0
// is chosen so the correct result (0xFFFF'80F0) differs from both byte
// reads (0xFFFF'FF80 high byte / 0xFFFF'FFF0 low byte) and any rotated
// form, so a regression toward the TRM quirk would be caught here.
static void test_ldrsh_unaligned_odd_address_imm_masks_low_bit() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0100u;
    nds.arm7_bus().write16(0x0380'0104u, 0x80F0u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*off*/5));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'80F0u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0100u);  // unchanged
}

// LDRSH R0, [R1, R2]  — odd effective address via register offset. Same
// rule: mask bit 0, aligned halfword read, sign-extend (matches melonDS,
// no byte-read quirk). Sentinel 0x8001 — correct result 0xFFFF'8001 is
// distinct from any byte-read or rotated interpretation.
static void test_ldrsh_unaligned_odd_address_reg_masks_low_bit() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0200u;
    nds.cpu7().state().r[2] = 0x0000'0001u;
    nds.arm7_bus().write16(0x0380'0200u, 0x8001u);

    run_one(nds, kBase,
            encode_halfword_reg(kCondAL, /*P*/1, /*U*/1, /*W*/0, /*L*/1,
                                /*Rn*/1, /*Rd*/0, kSHsh, /*Rm*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'8001u);
    REQUIRE(nds.cpu7().state().r[1] == 0x0380'0200u);  // unchanged
    REQUIRE(nds.cpu7().state().r[2] == 0x0000'0001u);  // unchanged
}

// ---- Rn == Rd writeback suppression (loads) -------------------------------
//
// ARMv4T rule: when a load with writeback targets the same register as its
// base (Rn == Rd), the loaded value wins and the writeback is suppressed.
// The shared write_rd_and_writeback helper in arm7_halfword.cpp encodes this
// with `if (addr.writeback && addr.rn != rd)`. These tests exercise all
// three load variants so any future regression that drops the guard on one
// variant is caught immediately. Stores are NOT subject to the rule —
// test_strh_rn_eq_rd_writeback_proceeds_normally is the counter-example.

// LDRH R0, [R0, #4]!  — load wins, writeback suppressed.
static void test_ldrh_rn_eq_rd_writeback_suppressed() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x0380'0500u;
    nds.arm7_bus().write16(0x0380'0504u, 0xBEEFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/1,
                                /*Rn*/0, /*Rd*/0, kSHhalf, /*off*/4));

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'BEEFu);   // loaded value wins
    REQUIRE(nds.cpu7().state().r[0] != 0x0380'0504u);   // writeback suppressed
}

// LDRSB R0, [R0, #1]!  — sign-extended byte wins, writeback suppressed.
static void test_ldrsb_rn_eq_rd_writeback_suppressed() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x0380'0600u;
    nds.arm7_bus().write8(0x0380'0601u, 0xFFu);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/1,
                                /*Rn*/0, /*Rd*/0, kSHsb, /*off*/1));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);   // sign-extended
    REQUIRE(nds.cpu7().state().r[0] != 0x0380'0601u);   // writeback suppressed
}

// LDRSH R0, [R0, #2]!  — sign-extended halfword wins, writeback suppressed.
static void test_ldrsh_rn_eq_rd_writeback_suppressed() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x0380'0700u;
    nds.arm7_bus().write16(0x0380'0702u, 0x8000u);

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/1,
                                /*Rn*/0, /*Rd*/0, kSHsh, /*off*/2));

    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'8000u);   // sign-extended
    REQUIRE(nds.cpu7().state().r[0] != 0x0380'0702u);   // writeback suppressed
}

// STRH R0, [R0, #4]!  — counter-example: stores write Rd's pre-writeback
// value to memory, and writeback proceeds normally. The suppression rule
// only applies to loads (because loads overwrite Rd and would clobber the
// writeback). Here the stored halfword is the low 16 of the OLD R0
// (0x03800800 & 0xFFFF == 0x0800) and R0 is updated to 0x03800804.
static void test_strh_rn_eq_rd_writeback_proceeds_normally() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x0380'0800u;

    run_one(nds, kBase,
            encode_halfword_imm(kCondAL, /*P*/1, /*U*/1, /*W*/1, /*L*/0,
                                /*Rn*/0, /*Rd*/0, kSHhalf, /*off*/4));

    REQUIRE(nds.arm7_bus().read16(0x0380'0804u) == 0x0800u);  // low 16 of old R0
    REQUIRE(nds.cpu7().state().r[0] == 0x0380'0804u);          // writeback fired
}

int main() {
    test_ldrh_imm_preindex_up_no_writeback();
    test_ldrh_imm_preindex_up_writeback();
    test_ldrh_imm_preindex_down_writeback();
    test_ldrh_imm_postindex_up();
    test_ldrh_imm_postindex_down();
    test_ldrh_reg_preindex_up_no_writeback();
    test_ldrh_zero_extension();
    test_ldrh_imm_offset_0xFE();
    test_ldrh_imm_offset_zero();
    test_ldrh_unaligned_odd_address_imm_masks_low_bit();
    test_ldrh_unaligned_odd_address_reg_masks_low_bit();

    test_strh_imm_preindex_up_no_writeback();
    test_strh_imm_preindex_up_writeback();
    test_strh_imm_preindex_down_writeback();
    test_strh_imm_postindex_up();
    test_strh_reg_preindex_up_no_writeback();
    test_strh_only_low_16_bits_written();
    test_strh_rd_eq_r15_reads_pc_plus_12();
    test_strh_unaligned_odd_address_imm_masks_low_bit();
    test_strh_unaligned_odd_address_reg_masks_low_bit();

    test_ldrsb_positive_byte();
    test_ldrsb_negative_byte_sign_extended();
    test_ldrsb_negative_byte_ff();
    test_ldrsb_imm_postindex_writeback();
    test_ldrsb_reg_preindex_up();
    test_ldrsb_unaligned_address_is_fine();

    test_ldrsh_positive_halfword();
    test_ldrsh_negative_halfword_sign_extended();
    test_ldrsh_negative_halfword_ffff();
    test_ldrsh_imm_preindex_up_writeback();
    test_ldrsh_imm_postindex_down();
    test_ldrsh_reg_preindex_up();
    test_ldrsh_positive_halfword_7fff();
    test_ldrsh_unaligned_odd_address_imm_masks_low_bit();
    test_ldrsh_unaligned_odd_address_reg_masks_low_bit();

    test_ldrh_rn_eq_rd_writeback_suppressed();
    test_ldrsb_rn_eq_rd_writeback_suppressed();
    test_ldrsh_rn_eq_rd_writeback_suppressed();
    test_strh_rn_eq_rd_writeback_proceeds_normally();

    std::puts("arm7_halfword_test: all LDRH, STRH, LDRSB, and LDRSH cases passed");
    return 0;
}
