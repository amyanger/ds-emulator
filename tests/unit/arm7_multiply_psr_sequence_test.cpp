// arm7_multiply_psr_sequence_test.cpp — end-to-end capstone for slice 3b2.
// Runs a small program that exercises register-shift DP, MUL, SMULL, MRS,
// and MSR in sequence. Catches inter-family bugs where each family works
// in isolation but corrupts a register another family is about to read.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 AL_COND = 0xEu << 28;
constexpr u32 kBase   = 0x0380'0000u;  // ARM7 WRAM

// --- Instruction encoders ---

u32 mov_imm(u32 rd, u32 imm8) {
    // MOV Rd, #imm8 (rot=0) — imm form DP.
    return AL_COND | 0x03A00000u | ((rd & 0xFu) << 12) | (imm8 & 0xFFu);
}

u32 mov_neg_one(u32 rd) {
    // MVN Rd, #0 == 0xFFFFFFFF — imm form DP.
    return AL_COND | 0x03E00000u | ((rd & 0xFu) << 12);
}

u32 reg_shift_mov(u32 rd, u32 rm, u32 rs) {
    // MOV Rd, Rm, LSL Rs — reg-shift DP, opcode=MOV(0xD), shift_type=LSL(0).
    u32 instr = AL_COND | (0xDu << 21);
    instr |= (rd & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (1u << 4);  // reg-shift marker
    instr |= (rm & 0xFu);
    return instr;
}

u32 mul_instr(u32 rd, u32 rm, u32 rs) {
    return AL_COND | 0x00000090u
         | ((rd & 0xFu) << 16)
         | ((rs & 0xFu) << 8)
         | (rm & 0xFu);
}

u32 smull_instr(u32 rd_hi, u32 rd_lo, u32 rm, u32 rs) {
    // 00C = long (bit 23) + signed (bit 22)
    return AL_COND | 0x00C00090u
         | ((rd_hi & 0xFu) << 16)
         | ((rd_lo & 0xFu) << 12)
         | ((rs & 0xFu) << 8)
         | (rm & 0xFu);
}

u32 mrs_cpsr(u32 rd) {
    return AL_COND | 0x010F0000u | ((rd & 0xFu) << 12);
}

u32 msr_cpsr_c_reg(u32 rm) {
    // MSR CPSR_c, Rm: cond 00010 0 10 0001 1111 00000000 Rm
    // bits[19:16] = 0001 (mask = c), bits[15:12] = 1111
    return AL_COND | 0x0120F000u | 0x00010000u | (rm & 0xFu);
}

u32 orr_imm(u32 rd, u32 rn, u32 imm8) {
    return AL_COND | 0x03800000u
         | ((rn & 0xFu) << 16)
         | ((rd & 0xFu) << 12)
         | (imm8 & 0xFFu);
}

u32 bic_imm(u32 rd, u32 rn, u32 imm8) {
    return AL_COND | 0x03C00000u
         | ((rn & 0xFu) << 16)
         | ((rd & 0xFu) << 12)
         | (imm8 & 0xFFu);
}

}  // namespace

int main() {
    NDS nds;

    // Starting CPSR is 0xD3 (Supervisor, I=1, F=1, ARM). The disable-
    // interrupts idiom at the end of this program is a no-op on the I bit
    // because it's already set — but it's still a meaningful test of the
    // MRS → ORR → MSR → MRS → BIC → MSR round-trip. We clear I first so
    // the toggle is observable.
    nds.cpu7().state().cpsr = 0x00000013u;  // Supervisor, I clear, flags clear

    // Program layout:
    //   0:  MOV  R0, #0x10          ; R0 = 0x10
    //   1:  MOV  R1, #0x20          ; R1 = 0x20
    //   2:  MOV  R2, #3             ; R2 = 3
    //   3:  MOV  R2, R0, LSL R2     ; R2 = R0 << R2 == 0x80   (reg-shift DP)
    //   4:  MUL  R3, R0, R1         ; R3 = 0x200              (short multiply)
    //   5:  MVN  R5, #0             ; R5 = 0xFFFFFFFF (= -1)
    //   6:  MUL  R5, R3, R5         ; R5 = R3 * -1 = -0x200 (modular 32-bit)
    //   7:  MOV  R6, #2             ; R6 = 2
    //   8:  SMULL R4, R7, R5, R6    ; R7:R4 = sign-ext(-0x200 * 2) = -0x400
    //   9:  MRS  R8, CPSR           ; snapshot CPSR
    //  10:  ORR  R9, R8, #0x80      ; set I bit in copy
    //  11:  MSR  CPSR_c, R9         ; I bit now live in CPSR
    //  12:  MRS  R10, CPSR          ; read back; should see I set
    //  13:  BIC  R9, R10, #0x80     ; clear I in copy
    //  14:  MSR  CPSR_c, R9         ; I bit cleared live
    const u32 program[] = {
        mov_imm(0, 0x10),
        mov_imm(1, 0x20),
        mov_imm(2, 3),
        reg_shift_mov(2, 0, 2),
        mul_instr(3, 0, 1),
        mov_neg_one(5),
        mul_instr(5, 3, 5),
        mov_imm(6, 2),
        smull_instr(7, 4, 5, 6),
        mrs_cpsr(8),
        orr_imm(9, 8, 0x80),
        msr_cpsr_c_reg(9),
        mrs_cpsr(10),
        bic_imm(9, 10, 0x80),
        msr_cpsr_c_reg(9),
    };
    constexpr u32 kInstrCount = sizeof(program) / sizeof(program[0]);

    for (u32 i = 0; i < kInstrCount; ++i) {
        nds.arm7_bus().write32(kBase + i * 4u, program[i]);
    }

    nds.cpu7().state().pc = kBase;
    nds.cpu7().state().r[15] = kBase + 8;
    // Each instruction costs 1 ARM7 cycle = 2 ARM9 cycles.
    nds.cpu7().run_until(kInstrCount * 2);

    // --- Register-shift DP results ---
    REQUIRE(nds.cpu7().state().r[0] == 0x10u);
    REQUIRE(nds.cpu7().state().r[1] == 0x20u);
    REQUIRE(nds.cpu7().state().r[2] == 0x80u);       // 0x10 << 3

    // --- Short multiply results ---
    REQUIRE(nds.cpu7().state().r[3] == 0x200u);      // 0x10 * 0x20
    REQUIRE(nds.cpu7().state().r[5] == static_cast<u32>(-0x200));  // 0x200 * -1
    REQUIRE(nds.cpu7().state().r[6] == 2u);

    // --- Long signed multiply results ---
    // SMULL(-0x200, 2) = -0x400. In 64-bit two's complement:
    //   0xFFFFFFFFFFFFFC00
    //   RdLo (R4) = 0xFFFFFC00
    //   RdHi (R7) = 0xFFFFFFFF
    REQUIRE(nds.cpu7().state().r[4] == 0xFFFFFC00u);
    REQUIRE(nds.cpu7().state().r[7] == 0xFFFFFFFFu);

    // --- MRS/MSR round-trip ---
    // R10 is the CPSR snapshot taken AFTER the first MSR set the I bit.
    // The I bit should be observable in R10.
    REQUIRE((nds.cpu7().state().r[10] & 0x80u) != 0);

    // Final CPSR: after the second MSR cleared the I bit, it should be 0.
    // Mode bits should still be Supervisor (0x13).
    REQUIRE((nds.cpu7().state().cpsr & 0x80u) == 0);
    REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x13u);

    std::puts("arm7_multiply_psr_sequence_test: capstone passed");
    return 0;
}
