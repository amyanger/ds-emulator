// Hand-assembled ARM data-processing tests. Each case loads a single
// instruction word into ARM7 WRAM at 0x0380'0000, sets pc_, calls
// Arm7::run_until() for one instruction's worth of cycles, and checks
// the post-execution register file.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Preload a single instruction word at pc and run one instruction.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;  // R15 = pc + 8 at execute time
    const u64 cycles_before = nds.cpu7().state().cycles;
    // Ask Arm7 to run for 1 more ARM7 cycle (= 2 ARM9 cycles; Arm7 is half-rate).
    nds.cpu7().run_until((cycles_before + 1) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1);
}

static void mov_imm_zero_writes_register_and_advances_pc() {
    NDS nds;
    // Encoding: cond=AL, 00 I=1 opcode=MOV S=0 Rn=0 Rd=0 rotate=0 imm=0
    //          1110 00 1 1101 0 0000 0000 0000 0000 0000
    //          0xE3A0'0000 = MOV R0, #0
    run_one(nds, 0x0380'0000u, 0xE3A0'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);
}

static void mov_imm_0x42_lands_in_r1() {
    NDS nds;
    // MOV R1, #0x42 -> 0xE3A0'1042
    run_one(nds, 0x0380'0000u, 0xE3A0'1042u);
    REQUIRE(nds.cpu7().state().r[1] == 0x42u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);
}

static void condition_ne_skips_when_z_set() {
    NDS nds;
    // MOVNE R2, #0x77 -> 0x13A0'2077
    // Pre-set CPSR Z flag so the instruction is skipped.
    nds.cpu7().state().cpsr |= (1u << 30);
    run_one(nds, 0x0380'0000u, 0x13A0'2077u);
    REQUIRE(nds.cpu7().state().r[2] == 0u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);  // still advances
}

static void add_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    // ADD R0, R1, #5 -> 0xE281'0005
    run_one(nds, 0x0380'0000u, 0xE281'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 15u);
}

static void sub_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    // SUB R0, R1, #3 -> 0xE241'0003
    run_one(nds, 0x0380'0000u, 0xE241'0003u);
    REQUIRE(nds.cpu7().state().r[0] == 7u);
}

static void rsb_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 3;
    // RSB R0, R1, #10 -> 0xE261'000A
    run_one(nds, 0x0380'0000u, 0xE261'000Au);
    REQUIRE(nds.cpu7().state().r[0] == 7u);
}

static void and_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xF0F0'F0F0u;
    // AND R0, R1, #0xFF -> 0xE201'00FF
    run_one(nds, 0x0380'0000u, 0xE201'00FFu);
    REQUIRE(nds.cpu7().state().r[0] == 0xF0u);
}

static void eor_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFF'FFFFu;
    // EOR R0, R1, #0xFF -> 0xE221'00FF
    run_one(nds, 0x0380'0000u, 0xE221'00FFu);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FF00u);
}

static void orr_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0000'000Fu;
    // ORR R0, R1, #0xF0 -> 0xE381'00F0
    run_one(nds, 0x0380'0000u, 0xE381'00F0u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFu);
}

static void bic_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFu;
    // BIC R0, R1, #0x0F -> 0xE3C1'000F
    run_one(nds, 0x0380'0000u, 0xE3C1'000Fu);
    REQUIRE(nds.cpu7().state().r[0] == 0xF0u);
}

static void mvn_imm() {
    NDS nds;
    // MVN R0, #0 -> 0xE3E0'0000 (result 0xFFFFFFFF)
    run_one(nds, 0x0380'0000u, 0xE3E0'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
}

static void adc_imm_with_carry_set() {
    NDS nds;
    nds.cpu7().state().cpsr |= (1u << 29);   // C = 1
    nds.cpu7().state().r[1] = 10;
    // ADC R0, R1, #5 -> 0xE2A1'0005
    run_one(nds, 0x0380'0000u, 0xE2A1'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 16u);
}

static void sbc_imm_with_carry_clear() {
    NDS nds;
    // C = 0 means SBC subtracts an extra 1.
    nds.cpu7().state().r[1] = 10;
    // SBC R0, R1, #3 -> 0xE2C1'0003
    run_one(nds, 0x0380'0000u, 0xE2C1'0003u);
    REQUIRE(nds.cpu7().state().r[0] == 6u);
}

static void rsc_imm_with_carry_set() {
    NDS nds;
    nds.cpu7().state().cpsr |= (1u << 29);  // C = 1 -> no extra decrement
    nds.cpu7().state().r[1] = 3;
    // RSC R0, R1, #10 -> 0xE2E1'000A
    run_one(nds, 0x0380'0000u, 0xE2E1'000Au);
    REQUIRE(nds.cpu7().state().r[0] == 7u);
}

// --- Task 7: immediate-shifted register operand2 --------------------------

static void mov_reg_no_shift() {
    NDS nds;
    nds.cpu7().state().r[2] = 0xDEAD'BEEFu;
    // MOV R0, R2 -> 0xE1A0'0002 (LSL #0 = identity)
    run_one(nds, 0x0380'0000u, 0xE1A0'0002u);
    REQUIRE(nds.cpu7().state().r[0] == 0xDEAD'BEEFu);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);
}

static void add_reg_lsl_3() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x100u;
    nds.cpu7().state().r[2] = 0x2u;
    // ADD R0, R1, R2, LSL #3 -> 0xE081'0182
    // r2 << 3 = 0x10, r1 + 0x10 = 0x110
    run_one(nds, 0x0380'0000u, 0xE081'0182u);
    REQUIRE(nds.cpu7().state().r[0] == 0x110u);
}

static void sub_reg_lsr_1() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x20u;
    nds.cpu7().state().r[2] = 0x10u;
    // SUB R0, R1, R2, LSR #1 -> 0xE041'00A2
    // r2 >> 1 = 0x8, r1 - 0x8 = 0x18
    run_one(nds, 0x0380'0000u, 0xE041'00A2u);
    REQUIRE(nds.cpu7().state().r[0] == 0x18u);
}

static void mov_reg_asr_31_sign_extends() {
    NDS nds;
    nds.cpu7().state().r[2] = 0x8000'0000u;
    // MOV R0, R2, ASR #31 -> 0xE1A0'0FC2
    // ASR by 31 sign-extends top bit down -> 0xFFFFFFFF
    run_one(nds, 0x0380'0000u, 0xE1A0'0FC2u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
}

static void mov_reg_to_pc_stomps_pc() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0380'0000u;
    // MOV R15, R1 -> 0xE1A0'F001
    // Exercises write_rd's rd==15 branch: sets state.r[15] AND stomps
    // state.pc (with the ~0x3 alignment mask). A regression that updates
    // only one of the two would previously pass all other tests.
    run_one(nds, 0x0380'0000u, 0xE1A0'F001u);
    REQUIRE(nds.cpu7().state().r[15] == 0x0380'0000u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0000u);
}

static void movs_reg_lsl_1_sets_c_from_shifter() {
    NDS nds;
    nds.cpu7().state().r[2] = 0x8000'0000u;
    // MOVS R0, R2, LSL #1 -> 0xE1B0'0082
    //   cond=E, 00, I=0, opcode=MOV(0xD), S=1, Rn=0, Rd=0,
    //   amount=1, type=LSL(0), bit4=0, Rm=2
    // Shifting 0x80000000 left by 1 drops bit 31 into the carry,
    // so: op2.value = 0, op2.carry = 1. Result (MOV) = 0 -> Z=1, N=0.
    // Logical form -> C = op2.carry = 1, V unchanged. Closes the
    // Task 6 + Task 7 composition gap (shifter carry -> set_c).
    run_one(nds, 0x0380'0000u, 0xE1B0'0082u);
    REQUIRE(nds.cpu7().state().r[0] == 0u);

    const u32 cpsr = nds.cpu7().state().cpsr;
    REQUIRE((cpsr & (1u << 30)) != 0);  // Z set
    REQUIRE((cpsr & (1u << 31)) == 0);  // N clear
    REQUIRE((cpsr & (1u << 29)) != 0);  // C from shifter carry-out
}

int main() {
    mov_imm_zero_writes_register_and_advances_pc();
    mov_imm_0x42_lands_in_r1();
    condition_ne_skips_when_z_set();
    add_imm(); sub_imm(); rsb_imm();
    and_imm(); eor_imm(); orr_imm(); bic_imm(); mvn_imm();
    adc_imm_with_carry_set();
    sbc_imm_with_carry_clear();
    rsc_imm_with_carry_set();
    mov_reg_no_shift();
    add_reg_lsl_3();
    sub_reg_lsr_1();
    mov_reg_asr_31_sign_extends();
    mov_reg_to_pc_stomps_pc();
    movs_reg_lsl_1_sets_c_from_shifter();
    std::puts("arm7_data_processing_test OK");
    return 0;
}
