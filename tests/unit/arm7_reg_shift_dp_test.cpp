// arm7_reg_shift_dp_test.cpp — ARMv4T register-shifted-register
// data-processing tests. Exercises the bit-4==1 form of DP operand2
// (shift amount comes from Rs[7:0]) for all four shift types, including
// the amount==0 / 32 / >32 boundary rules. PC+12 pipeline quirk tests
// and Rd==R15 S-bit warn tests land in slice 3b2 Task 3 and Task 4.

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
constexpr u32 kOpMOV = 0xDu;
constexpr u32 kOpADD = 0x4u;
constexpr u32 kLSL = 0x0u;
constexpr u32 kLSR = 0x1u;
constexpr u32 kASR = 0x2u;
constexpr u32 kROR = 0x3u;

// Assemble a reg-shift DP instruction:
//   cond = AL (0xE), I=0, bit4=1, bit7=0
//   |31..28 cond| 00 | 0 opcode S | Rn | Rd | Rs | 0 shift 1 | Rm |
u32 encode_reg_shift_dp(u32 opcode, bool s, u32 rn, u32 rd, u32 rs, u32 shift_type, u32 rm) {
    u32 instr = AL_COND;
    instr |= (opcode & 0xFu) << 21;
    if (s)
        instr |= (1u << 20);
    instr |= (rn & 0xFu) << 16;
    instr |= (rd & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (shift_type & 0x3u) << 5;
    instr |= (1u << 4); // bit4 = 1 (reg-shift)
    instr |= (rm & 0xFu);
    return instr;
}

} // namespace

// Test 1: MOV R0, R1, LSL R2 with R1=0x1, R2=3 → R0 == 0x8
static void mov_lsl_reg_amount_3() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x1u;
    nds.cpu7().state().r[2] = 3u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 2, kLSL, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0x8u);
}

// Test 2: MOV R0, R1, LSL R2 with R2=0 → R0 == R1, CPSR.C preserved
static void mov_lsl_reg_amount_zero_preserves_carry() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xDEADBEEFu;
    nds.cpu7().state().r[2] = 0u;
    nds.cpu7().state().cpsr |= (1u << 29); // set C before the instruction
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0xDEADBEEFu);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0); // C preserved
}

// Test 3: MOV R0, R1, LSL R2 with R2=32 → R0 == 0, C == bit 0 of R1
static void mov_lsl_reg_amount_32() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x80000001u; // bit 0 set
    nds.cpu7().state().r[2] = 32u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0); // C = bit 0 of R1
}

// Test 4: MOV R0, R1, LSL R2 with R2=33 → R0 == 0, C == 0
static void mov_lsl_reg_amount_33() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFFFFFFu;
    nds.cpu7().state().r[2] = 33u;
    nds.cpu7().state().cpsr |= (1u << 29); // pre-set C to verify it gets cleared
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) == 0); // C cleared
}

// Test 5: MOV R0, R1, LSR R2 with R2=32 → R0 == 0, C == bit 31 of R1
static void mov_lsr_reg_amount_32() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x80000000u;
    nds.cpu7().state().r[2] = 32u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSR, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
}

// Test 6: MOV R0, R1, ASR R2 with R1=0x80000000, R2=32 → R0 == 0xFFFFFFFF
static void mov_asr_reg_amount_32() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x80000000u;
    nds.cpu7().state().r[2] = 32u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 2, kASR, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFFu);
}

// Test 7: MOV R0, R1, ROR R2 with R2=32 → R0 == R1, C == bit 31 of R1
static void mov_ror_reg_amount_32() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x80000000u;
    nds.cpu7().state().r[2] = 32u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kROR, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
}

// Test 8: MOV R0, R1, ROR R2 with R2=64 → same as R2=32
static void mov_ror_reg_amount_64_folds_to_32() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x80000000u;
    nds.cpu7().state().r[2] = 64u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kROR, 1));
    REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
}

// Test 9: "only low byte of Rs matters" rule — ADD R0, R1, R2, LSL R3
// with R3=0xFFFFFF04 should behave exactly like LSL #4 (low byte = 0x04).
static void low_byte_of_rs_only() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x10u;
    nds.cpu7().state().r[2] = 0x1u;
    nds.cpu7().state().r[3] = 0xFFFFFF04u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpADD, false, 1, 0, 3, kLSL, 2));
    REQUIRE(nds.cpu7().state().r[0] == 0x10u + (0x1u << 4));
}

// Test 10: ADDS R0, R1, R2, LSL R3 flag update matches imm-shift path.
// R1 = 0xFFFFFFFF, R2 = 1, R3 = 0 (no shift). Result wraps to 0, carry out.
static void adds_reg_shift_matches_imm_shift() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFFFFFFu;
    nds.cpu7().state().r[2] = 0x1u;
    nds.cpu7().state().r[3] = 0u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpADD, true, 1, 0, 3, kLSL, 2));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0); // Z set
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0); // C set (carry out)
}

// Test 11: MOV R0, R15, LSL R1 with R1=0 → R0 == PC+12, not PC+8
// Instruction lives at kBase. r[15] at execute time is kBase+8.
// Reg-shift form reads r[15] as PC+12 == kBase+12.
static void mov_rm_is_pc_reads_pc12() {
    NDS nds;
    nds.cpu7().state().r[1] = 0u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 1, kLSL, 15));
    REQUIRE(nds.cpu7().state().r[0] == kBase + 12u);
}

// Test 12: ADD R0, R15, R2, LSL R3 with R3=0 → R0 == (PC+12) + R2
// Same PC+12 rule for Rn==15 in reg-shift form.
static void add_rn_is_pc_reads_pc12() {
    NDS nds;
    nds.cpu7().state().r[2] = 0x100u;
    nds.cpu7().state().r[3] = 0u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpADD, false, 15, 0, 3, kLSL, 2));
    REQUIRE(nds.cpu7().state().r[0] == (kBase + 12u) + 0x100u);
}

// Test 13: MOV R0, R1, LSL R15 → warn logged (Rs == 15 is UNPREDICTABLE),
// result uses PC+12 == kBase+12 as the shift amount. Low byte of
// that address determines the effective shift; with kBase = 0x03800000,
// (kBase + 12) & 0xFF == 0x0C, so this behaves as LSL #12.
static void mov_rs_is_pc_warn_and_pc12() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x1u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 15, kLSL, 1));
    REQUIRE(nds.cpu7().state().r[0] == (0x1u << ((kBase + 12u) & 0xFFu)));
}

// Test 14: MOVS R15, R1, LSL R2 via reg-shift form is an ARMv4
// exception-return. With SPSR_svc seeded to 0x0000001F (System mode, T=0,
// flags clear) and R1 holding the return address, executing the instruction
// copies SPSR into CPSR, re-banks R13/R14 into the System (shared-user)
// bank, and writes PC. NZCV is NOT updated from the result — the
// exception-return path replaces the normal flag update.
static void movs_rd_is_pc_restores_spsr() {
    NDS nds;
    auto& state = nds.cpu7().state();

    // Start in Supervisor (reset default). Seed System R13 so we can verify
    // the bank swap happened.
    state.banks.user_r8_r14[5] = 0x5A5A5A5Au; // System R13
    state.banks.spsr_svc = 0x0000001Fu;       // System, T=0, flags clear

    state.r[1] = 0x02000100u;
    state.r[2] = 0u;
    run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 15, 2, kLSL, 1));

    REQUIRE((state.cpsr & 0x1Fu) == 0x1Fu); // System mode
    REQUIRE(state.cpsr == 0x0000001Fu);     // SPSR contents copied verbatim
    REQUIRE(state.current_mode() == Mode::System);
    REQUIRE(state.pc == 0x02000100u);
    REQUIRE(state.r[13] == 0x5A5A5A5Au); // System bank swapped in
}

int main() {
    mov_lsl_reg_amount_3();
    mov_lsl_reg_amount_zero_preserves_carry();
    mov_lsl_reg_amount_32();
    mov_lsl_reg_amount_33();
    mov_lsr_reg_amount_32();
    mov_asr_reg_amount_32();
    mov_ror_reg_amount_32();
    mov_ror_reg_amount_64_folds_to_32();
    low_byte_of_rs_only();
    adds_reg_shift_matches_imm_shift();
    mov_rm_is_pc_reads_pc12();
    add_rn_is_pc_reads_pc12();
    mov_rs_is_pc_warn_and_pc12();
    movs_rd_is_pc_restores_spsr();

    std::puts("arm7_reg_shift_dp_test: all 14 cases passed");
    return 0;
}
