// Hand-assembled ARM9 data-processing tests. Each case loads a single
// instruction word into ARM9 main RAM at 0x0200'0000, sets pc, runs exactly
// one (full-rate) ARM9 instruction via the shared run_one fixture, and checks
// the post-execution register file / flags.
//
// The DP family is bit-identical between ARMv4T and ARMv5TE (slice 3l §5.7),
// so these mirror the ARM7 DP tests; they prove the ARM9-owned executor works
// and matches the documented ARM7 behavior — operands, flags, the non-
// writeback compares, the barrel-shifter operand forms, and PC advance.

#include "arm9_step.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;
using ds::test::run_one;

constexpr u32 kBase = 0x0200'0000u;

// CPSR flag bit positions.
constexpr u32 kN = 1u << 31;
constexpr u32 kZ = 1u << 30;
constexpr u32 kC = 1u << 29;
constexpr u32 kV = 1u << 28;

static void mov_imm_zero_writes_register_and_advances_pc() {
    NDS nds;
    // MOV R0, #0 -> 0xE3A0'0000
    run_one(nds, kBase, 0xE3A0'0000u);
    REQUIRE(nds.cpu9().state().r[0] == 0u);
    REQUIRE(nds.cpu9().state().pc == kBase + 4u);
}

static void mov_imm_0x42_lands_in_r1() {
    NDS nds;
    // MOV R1, #0x42 -> 0xE3A0'1042
    run_one(nds, kBase, 0xE3A0'1042u);
    REQUIRE(nds.cpu9().state().r[1] == 0x42u);
}

static void condition_ne_skips_when_z_set() {
    NDS nds;
    // MOVNE R2, #0x77 -> 0x13A0'2077, with Z set so it skips.
    nds.cpu9().state().cpsr |= kZ;
    run_one(nds, kBase, 0x13A0'2077u);
    REQUIRE(nds.cpu9().state().r[2] == 0u);
    REQUIRE(nds.cpu9().state().pc == kBase + 4u); // still advances
}

static void add_sub_rsb_immediate() {
    NDS nds;
    nds.cpu9().state().r[1] = 10;
    run_one(nds, kBase, 0xE281'0005u); // ADD R0, R1, #5
    REQUIRE(nds.cpu9().state().r[0] == 15u);

    nds.cpu9().state().r[1] = 10;
    run_one(nds, kBase, 0xE241'0003u); // SUB R0, R1, #3
    REQUIRE(nds.cpu9().state().r[0] == 7u);

    nds.cpu9().state().r[1] = 3;
    run_one(nds, kBase, 0xE261'0014u); // RSB R0, R1, #0x14 -> 0x14 - 3
    REQUIRE(nds.cpu9().state().r[0] == 0x11u);
}

static void logical_ops_and_eor_orr_bic_mvn() {
    NDS nds;
    nds.cpu9().state().r[1] = 0xF0F0'F0F0u;

    run_one(nds, kBase, 0xE201'00FFu); // AND R0, R1, #0xFF
    REQUIRE(nds.cpu9().state().r[0] == 0xF0u);

    nds.cpu9().state().r[1] = 0xF0F0'F0F0u;
    run_one(nds, kBase, 0xE221'00FFu); // EOR R0, R1, #0xFF
    REQUIRE(nds.cpu9().state().r[0] == 0xF0F0'F00Fu);

    nds.cpu9().state().r[1] = 0xF0F0'0000u;
    run_one(nds, kBase, 0xE381'00FFu); // ORR R0, R1, #0xFF
    REQUIRE(nds.cpu9().state().r[0] == 0xF0F0'00FFu);

    nds.cpu9().state().r[1] = 0x0000'00FFu;
    run_one(nds, kBase, 0xE3C1'000Fu); // BIC R0, R1, #0x0F
    REQUIRE(nds.cpu9().state().r[0] == 0x0000'00F0u);

    run_one(nds, kBase, 0xE3E0'0000u); // MVN R0, #0
    REQUIRE(nds.cpu9().state().r[0] == 0xFFFF'FFFFu);
}

static void movs_sets_n_and_z() {
    NDS nds;
    run_one(nds, kBase, 0xE3B0'0000u); // MOVS R0, #0 -> Z set, N clear
    REQUIRE((nds.cpu9().state().cpsr & kZ) != 0u);
    REQUIRE((nds.cpu9().state().cpsr & kN) == 0u);

    nds.cpu9().state().r[1] = 0xFFFF'FFFFu;
    run_one(nds, kBase, 0xE1B0'0001u); // MOVS R0, R1 -> N set, Z clear
    REQUIRE((nds.cpu9().state().cpsr & kN) != 0u);
    REQUIRE((nds.cpu9().state().cpsr & kZ) == 0u);
}

static void adds_carry_and_overflow() {
    NDS nds;
    // 0xFFFFFFFF + 1 -> 0, carry out set, no signed overflow.
    nds.cpu9().state().r[1] = 0xFFFF'FFFFu;
    run_one(nds, kBase, 0xE291'0001u); // ADDS R0, R1, #1
    REQUIRE(nds.cpu9().state().r[0] == 0u);
    REQUIRE((nds.cpu9().state().cpsr & kC) != 0u);
    REQUIRE((nds.cpu9().state().cpsr & kZ) != 0u);
    REQUIRE((nds.cpu9().state().cpsr & kV) == 0u);

    // 0x7FFFFFFF + 1 -> 0x80000000, signed overflow set, carry clear.
    nds.cpu9().state().r[1] = 0x7FFF'FFFFu;
    run_one(nds, kBase, 0xE291'0001u); // ADDS R0, R1, #1
    REQUIRE(nds.cpu9().state().r[0] == 0x8000'0000u);
    REQUIRE((nds.cpu9().state().cpsr & kV) != 0u);
    REQUIRE((nds.cpu9().state().cpsr & kN) != 0u);
    REQUIRE((nds.cpu9().state().cpsr & kC) == 0u);
}

static void subs_sets_carry_as_no_borrow() {
    NDS nds;
    // 10 - 3 -> 7, no borrow so carry SET (ARM convention).
    nds.cpu9().state().r[1] = 10;
    run_one(nds, kBase, 0xE251'0003u); // SUBS R0, R1, #3
    REQUIRE(nds.cpu9().state().r[0] == 7u);
    REQUIRE((nds.cpu9().state().cpsr & kC) != 0u);

    // 3 - 10 -> negative, borrow so carry CLEAR, N set.
    nds.cpu9().state().r[1] = 3;
    run_one(nds, kBase, 0xE251'000Au); // SUBS R0, R1, #10
    REQUIRE((nds.cpu9().state().cpsr & kC) == 0u);
    REQUIRE((nds.cpu9().state().cpsr & kN) != 0u);
}

static void cmp_and_tst_do_not_writeback() {
    NDS nds;
    // CMP R1, #5 with R1==5 -> Z set, Rd (R0) untouched.
    nds.cpu9().state().r[0] = 0xDEAD'BEEFu;
    nds.cpu9().state().r[1] = 5;
    run_one(nds, kBase, 0xE351'0005u);                // CMP R1, #5
    REQUIRE(nds.cpu9().state().r[0] == 0xDEAD'BEEFu); // no writeback
    REQUIRE((nds.cpu9().state().cpsr & kZ) != 0u);

    // TST R1, #0 -> Z set, no writeback.
    nds.cpu9().state().r[1] = 0xF0;
    run_one(nds, kBase, 0xE311'0000u); // TST R1, #0
    REQUIRE(nds.cpu9().state().r[0] == 0xDEAD'BEEFu);
    REQUIRE((nds.cpu9().state().cpsr & kZ) != 0u);
}

static void barrel_shifter_lsl_immediate_operand() {
    NDS nds;
    nds.cpu9().state().r[1] = 1;
    // MOV R0, R1, LSL #4 -> 0xE1A0'0201
    run_one(nds, kBase, 0xE1A0'0201u);
    REQUIRE(nds.cpu9().state().r[0] == 0x10u);
}

static void register_shift_operand_form() {
    NDS nds;
    nds.cpu9().state().r[1] = 1;
    nds.cpu9().state().r[2] = 8; // shift amount in Rs
    // MOV R0, R1, LSL R2 -> 0xE1A0'0211
    run_one(nds, kBase, 0xE1A0'0211u);
    REQUIRE(nds.cpu9().state().r[0] == 0x100u);
}

int main() {
    mov_imm_zero_writes_register_and_advances_pc();
    mov_imm_0x42_lands_in_r1();
    condition_ne_skips_when_z_set();
    add_sub_rsb_immediate();
    logical_ops_and_eor_orr_bic_mvn();
    movs_sets_n_and_z();
    adds_carry_and_overflow();
    subs_sets_carry_as_no_borrow();
    cmp_and_tst_do_not_writeback();
    barrel_shifter_lsl_immediate_operand();
    register_shift_operand_form();
    std::puts("arm9_dp_test OK");
    return 0;
}
