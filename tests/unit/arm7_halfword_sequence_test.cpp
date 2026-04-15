// arm7_halfword_sequence_test.cpp — end-to-end capstone for slice 3b3.
// Runs a small program that exercises all four halfword transfer opcodes
// (LDRH, LDRSB, STRH, LDRSH) against shared register / memory state,
// interleaved with a reg-shift DP and a MUL from slice 3b2. Catches
// inter-family bugs where each opcode works in isolation but the
// dispatcher drops a register another family is about to consume.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 AL_COND = 0xEu << 28;
constexpr u32 kBase = 0x0380'0000u;   // ARM7 WRAM
constexpr u32 kTarget = 0x0380'0200u; // halfword-aligned, read by LDRH/LDRSH

// --- Instruction encoders ---

u32 mov_imm(u32 rd, u32 imm8) {
    // MOV Rd, #imm8 (rot=0) — imm form DP.
    return AL_COND | 0x03A00000u | ((rd & 0xFu) << 12) | (imm8 & 0xFFu);
}

u32 reg_shift_mov_lsl(u32 rd, u32 rm, u32 rs) {
    // MOV Rd, Rm, LSL Rs — reg-shift DP, opcode=MOV(0xD), shift_type=LSL(0).
    u32 instr = AL_COND | (0xDu << 21);
    instr |= (rd & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (1u << 4); // reg-shift marker
    instr |= (rm & 0xFu);
    return instr;
}

u32 mul_instr(u32 rd, u32 rm, u32 rs) {
    return AL_COND | 0x00000090u | ((rd & 0xFu) << 16) | ((rs & 0xFu) << 8) | (rm & 0xFu);
}

// Halfword immediate form: cond|000|P|U|1|W|L|Rn|Rd|imm[7:4]|1|SH|1|imm[3:0]
u32 halfword_imm(u32 p, u32 u, u32 w, u32 l, u32 rn, u32 rd, u32 sh, u32 offset8) {
    return AL_COND | (p << 24) | (u << 23) | (1u << 22) // I = 1 (immediate)
           | (w << 21) | (l << 20) | ((rn & 0xFu) << 16) | ((rd & 0xFu) << 12) |
           ((offset8 & 0xF0u) << 4) // upper 4 bits -> bits[11:8]
           | (1u << 7) | ((sh & 0x3u) << 5) | (1u << 4) | (offset8 & 0x0Fu);
}

u32 ldrh_imm_zero_off(u32 rd, u32 rn) {
    return halfword_imm(/*P*/ 1, /*U*/ 1, /*W*/ 0, /*L*/ 1, rn, rd, /*SH*/ 1, /*off*/ 0);
}

u32 ldrsb_imm_zero_off(u32 rd, u32 rn) {
    return halfword_imm(/*P*/ 1, /*U*/ 1, /*W*/ 0, /*L*/ 1, rn, rd, /*SH*/ 2, /*off*/ 0);
}

u32 ldrsh_imm_zero_off(u32 rd, u32 rn) {
    return halfword_imm(/*P*/ 1, /*U*/ 1, /*W*/ 0, /*L*/ 1, rn, rd, /*SH*/ 3, /*off*/ 0);
}

u32 strh_imm_zero_off(u32 rd, u32 rn) {
    return halfword_imm(/*P*/ 1, /*U*/ 1, /*W*/ 0, /*L*/ 0, rn, rd, /*SH*/ 1, /*off*/ 0);
}

} // namespace

int main() {
    NDS nds;

    // Pre-seed the target halfword in main RAM. 0x80FF is deliberate:
    //   low byte 0xFF  -> sign-extends to 0xFFFFFFFF via LDRSB (flexes bit 7)
    //   halfword 0x80FF -> positive when zero-extended via LDRH (0x000080FF)
    nds.arm7_bus().write16(kTarget, 0x80FFu);

    // R0 holds the target address for the whole program.
    nds.cpu7().state().r[0] = kTarget;

    // Program:
    //  0:  MOV   R7, #0              ; baseline sentinel (verifies MOV imm still works)
    //  1:  LDRH  R1, [R0]            ; R1 = 0x000080FF  (zero-extended halfword)
    //  2:  LDRSB R2, [R0]            ; R2 = 0xFFFFFFFF  (sign-extended byte 0xFF)
    //  3:  MOV   R3, #4              ; R3 = 4
    //  4:  MOV   R3, R1, LSL R3      ; reg-shift DP: R3 = 0x80FF << 4 = 0x80FF0
    //  5:  MUL   R4, R2, R3          ; R4 = -1 * 0x80FF0 = 0xFFF7F010 (mod 2^32)
    //  6:  STRH  R4, [R0]            ; stores low 16 of R4 = 0xF010 at [R0]
    //  7:  LDRSH R5, [R0]            ; R5 = sign-ext(0xF010) = 0xFFFFF010
    const u32 program[] = {
        mov_imm(7, 0),
        ldrh_imm_zero_off(/*Rd*/ 1, /*Rn*/ 0),
        ldrsb_imm_zero_off(/*Rd*/ 2, /*Rn*/ 0),
        mov_imm(3, 4),
        reg_shift_mov_lsl(/*Rd*/ 3, /*Rm*/ 1, /*Rs*/ 3),
        mul_instr(/*Rd*/ 4, /*Rm*/ 2, /*Rs*/ 3),
        strh_imm_zero_off(/*Rd*/ 4, /*Rn*/ 0),
        ldrsh_imm_zero_off(/*Rd*/ 5, /*Rn*/ 0),
    };
    constexpr u32 kInstrCount = sizeof(program) / sizeof(program[0]);

    for (u32 i = 0; i < kInstrCount; ++i) {
        nds.arm7_bus().write32(kBase + i * 4u, program[i]);
    }

    nds.cpu7().state().pc = kBase;
    nds.cpu7().state().r[15] = kBase + 8;
    // Each instruction costs 1 ARM7 cycle = 2 ARM9 cycles.
    nds.cpu7().run_until(kInstrCount * 2);

    // --- Baseline sentinel ---
    REQUIRE(nds.cpu7().state().r[7] == 0u);

    // --- LDRH (zero-extend halfword) ---
    REQUIRE(nds.cpu7().state().r[1] == 0x0000'80FFu);

    // --- LDRSB (sign-extend byte at the halfword's low byte) ---
    REQUIRE(nds.cpu7().state().r[2] == 0xFFFF'FFFFu);

    // --- reg-shift DP consumes LDRH result ---
    REQUIRE(nds.cpu7().state().r[3] == 0x0008'0FF0u);

    // --- MUL consumes LDRSB (-1) and reg-shift result (0x80FF0) ---
    // -1 * 0x80FF0 = -0x80FF0. In 32-bit two's complement:
    //   0x00000000 - 0x00080FF0 = 0xFFF7F010
    REQUIRE(nds.cpu7().state().r[4] == 0xFFF7'F010u);

    // --- STRH wrote low 16 of R4 = 0xF010 to [R0] ---
    REQUIRE(nds.arm7_bus().read16(kTarget) == 0xF010u);

    // --- LDRSH re-read the stored halfword with sign extension ---
    // 0xF010 has bit 15 set, so sign-extends to 0xFFFFF010.
    REQUIRE(nds.cpu7().state().r[5] == 0xFFFF'F010u);

    // --- R0 (base) untouched throughout the program ---
    REQUIRE(nds.cpu7().state().r[0] == kTarget);

    // --- PC pin: guarantees run_until executed all kInstrCount instructions
    // and did not halt early. Without this, a silent dispatcher regression
    // could leave the later REQUIREs vacuously true (the state they check
    // never mutated because the instructions never ran).
    REQUIRE(nds.cpu7().state().pc == kBase + kInstrCount * 4u);

    std::puts("arm7_halfword_sequence_test: capstone passed");
    return 0;
}
