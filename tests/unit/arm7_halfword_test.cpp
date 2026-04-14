// arm7_halfword_test.cpp — ARMv4T halfword and signed-byte transfer
// tests. Task 4 covers the LDRH happy path only: aligned addresses,
// both immediate- and register-offset forms, every P/U/W combination,
// and a dedicated zero-extension verification. The unaligned LDRH
// rotate-by-8 quirk is deferred to Task 9 of slice 3b3.

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

    std::puts("arm7_halfword_test: all LDRH cases passed");
    return 0;
}
