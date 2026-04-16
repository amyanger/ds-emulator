// arm7_thumb_sequence_test.cpp — slice 3c capstone sequence test.
//
// Hand-written Thumb routine that computes factorial(5) = 120 via a
// PUSH/POP-wrapped subroutine called through BL, a conditional-branch
// multiplication loop, and a literal-pool PC-relative load to seed `n`.
// The test exercises — in a single emulator run — every Thumb format
// family that matters for correctness:
//
//   - THUMB.1 / THUMB.3 / THUMB.4: MOV imm, SUB imm8, MUL reg
//   - THUMB.6:   LDR PC-rel literal pool load
//   - THUMB.14:  PUSH {LR} + POP {PC} around a subroutine call
//   - THUMB.16:  BNE conditional branch terminator
//   - THUMB.19:  BL (both halfwords) into the subroutine
//   - THUMB.5:   BX LR to return from the subroutine
//   - THUMB.18:  B self (sentinel infinite-loop exit marker)
//
// Program layout (ARM7 WRAM at kBase):
//
//   kBase + 0x00  LDR  R1, [PC, #0x3C]    ; R1 = mem[literal pool] = 5
//   kBase + 0x02  MOV  R0, #1             ; accumulator = 1
//   kBase + 0x04  PUSH {LR}               ; save caller's LR (the sentinel)
//   kBase + 0x06  BL   multiply_step (hi) ; two-halfword BL
//   kBase + 0x08  BL   multiply_step (lo)
//   kBase + 0x0A  POP  {PC}               ; return to sentinel
//
//   kBase + 0x20  MUL  R0, R1             ; R0 *= R1            (loop top)
//   kBase + 0x22  SUB  R1, #1             ; R1 -= 1
//   kBase + 0x24  BNE  loop_top           ; while R1 != 0
//   kBase + 0x26  BX   LR                 ; return to caller
//
//   kBase + 0x40  .word 5                 ; literal pool
//
//   kBase + 0x100 B    kBase + 0x100      ; sentinel self-loop (exit)
//
// Expected final state: R0 == 120, PC parked on the sentinel self-loop.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;
constexpr u32 kStackTop = 0x0380'0800u;
constexpr u32 kSentinel = kBase + 0x100u;
constexpr u32 kLiteralPool = kBase + 0x40u;
constexpr u32 kSubroutine = kBase + 0x20u;

// --- Thumb encoders (copied from arm7_thumb_block_branch_test for locality) ---

// THUMB.3: 001 op2 Rd3 imm8  (op: 0=MOV 1=CMP 2=ADD 3=SUB)
u16 thumb3(u32 op, u32 rd, u32 imm8) {
    return static_cast<u16>((0b001u << 13) | (op << 11) | ((rd & 0x7u) << 8) | (imm8 & 0xFFu));
}

// THUMB.4: 010000 op4 Rs3 Rd3
u16 thumb4(u32 op, u32 rs, u32 rd) {
    return static_cast<u16>((0b010000u << 10) | ((op & 0xFu) << 6) | ((rs & 0x7u) << 3) |
                            (rd & 0x7u));
}

// THUMB.5: 010001 op2 Hd Hs Rs3 Rd3
u16 thumb5(u32 op, u32 hd, u32 hs, u32 rs, u32 rd) {
    return static_cast<u16>((0b010001u << 10) | ((op & 0x3u) << 8) | ((hd & 0x1u) << 7) |
                            ((hs & 0x1u) << 6) | ((rs & 0x7u) << 3) | (rd & 0x7u));
}

// THUMB.6: 01001 Rd3 imm8 — LDR Rd, [PC, #imm8<<2]
u16 thumb6(u32 rd, u32 imm8) {
    return static_cast<u16>((0b01001u << 11) | ((rd & 0x7u) << 8) | (imm8 & 0xFFu));
}

// THUMB.14: 1011 op1 10 R Rlist8
u16 thumb14(u32 op, u32 r_bit, u32 rlist8) {
    return static_cast<u16>((0b1011u << 12) | ((op & 1u) << 11) | (0b10u << 9) |
                            ((r_bit & 1u) << 8) | (rlist8 & 0xFFu));
}

// THUMB.16: 1101 cond4 imm8
u16 thumb16(u32 cond, u32 imm8) {
    return static_cast<u16>((0b1101u << 12) | ((cond & 0xFu) << 8) | (imm8 & 0xFFu));
}

// THUMB.18: 11100 imm11
u16 thumb18(u32 imm11) {
    return static_cast<u16>((0b11100u << 11) | (imm11 & 0x7FFu));
}

// THUMB.19 halves.
u16 thumb19_hi(u32 imm11_hi) {
    return static_cast<u16>((0b11110u << 11) | (imm11_hi & 0x7FFu));
}
u16 thumb19_lo(u32 imm11_lo) {
    return static_cast<u16>((0b11111u << 11) | (imm11_lo & 0x7FFu));
}

} // namespace

int main() {
    NDS nds;
    auto& bus = nds.arm7_bus();

    // ==== Assemble the program ====

    // LDR R1, [PC, #0x3C]
    //   At instr_addr = kBase, pc_literal = (kBase + 4) & ~2 = kBase + 4.
    //   Effective address = pc_literal + (imm8 << 2). For kLiteralPool -
    //   (kBase + 4) = 0x3C, imm8 = 0x0F.
    bus.write16(kBase + 0x00u, thumb6(/*rd=*/1, /*imm8=*/0x0Fu));
    // MOV R0, #1
    bus.write16(kBase + 0x02u, thumb3(/*op=*/0u, /*rd=*/0u, /*imm8=*/1u));
    // PUSH {LR}
    bus.write16(kBase + 0x04u, thumb14(/*op=*/0u, /*R=*/1u, /*rlist=*/0u));
    // BL multiply_step — two halfwords.
    //   Prefix: LR = (kBase+0x06 + 4) + (sext(imm11_hi) << 12) = kBase+0x0A for imm11_hi=0.
    //   Suffix: target = LR + (imm11_lo << 1). Want target = kSubroutine = kBase + 0x20.
    //           offset = 0x20 - 0x0A = 0x16, imm11_lo = 0xB.
    bus.write16(kBase + 0x06u, thumb19_hi(/*imm11_hi=*/0u));
    bus.write16(kBase + 0x08u, thumb19_lo(/*imm11_lo=*/0xBu));
    // POP {PC}
    bus.write16(kBase + 0x0Au, thumb14(/*op=*/1u, /*R=*/1u, /*rlist=*/0u));

    // --- multiply_step subroutine at kSubroutine ---
    //   MUL R0, R1    — THUMB.4 op=0xD: Rd = Rd * Rs. Rd=R0, Rs=R1.
    bus.write16(kSubroutine + 0x00u, thumb4(/*op=*/0xDu, /*rs=*/1u, /*rd=*/0u));
    //   SUB R1, #1
    bus.write16(kSubroutine + 0x02u, thumb3(/*op=*/3u, /*rd=*/1u, /*imm8=*/1u));
    //   BNE loop_top
    //     At instr_addr = kSubroutine + 4, target = (instr_addr + 4) + imm8*2.
    //     Want target = kSubroutine. imm8*2 = kSubroutine - (kSubroutine + 8) = -8.
    //     imm8 = 0xFC (sign-extended to -4).
    bus.write16(kSubroutine + 0x04u, thumb16(/*cond=*/1u, /*imm8=*/0xFCu));
    //   BX LR — THUMB.5 op=3, BX Rs where Rs = R14. Hs=1, Rs_low=14&7=6, Hd=0, Rd=0.
    bus.write16(kSubroutine + 0x06u, thumb5(/*op=*/3u, /*hd=*/0u, /*hs=*/1u, /*rs=*/6u, /*rd=*/0u));

    // Literal pool: the initial n.
    bus.write32(kLiteralPool, 5u);

    // Sentinel: B self → infinite loop. instr at kSentinel: imm11 = 0x7FE
    // (sext -2, offset -4), target = kSentinel + 4 - 4 = kSentinel.
    bus.write16(kSentinel, thumb18(0x7FEu));

    // ==== Seed the CPU state ====

    auto& s = nds.cpu7().state();
    s.pc = kBase;
    s.cpsr |= (1u << 5);      // Thumb state
    s.r[13] = kStackTop;      // stack pointer
    s.r[14] = kSentinel | 1u; // initial LR (Thumb bit set so any BX LR stays Thumb)

    // ==== Run with a generous cycle budget ====
    //
    // Straight-line instruction count for factorial(5):
    //   setup: LDR + MOV + PUSH + BL-hi + BL-lo     = 5
    //   loop body per iteration (MUL + SUB + BNE)   = 3 * 5 = 15
    //   BX LR                                        = 1
    //   POP {PC}                                     = 1
    //   sentinel spins: a few iterations
    //
    // ~25 instructions minimum. 200 cycles (= 400 ARM9 cycles) is more
    // than enough headroom and keeps a runaway branch bounded.
    nds.cpu7().run_until(400);

    // ==== Verify ====

    REQUIRE(s.r[0] == 120u);            // 5! = 120
    REQUIRE(s.r[1] == 0u);              // loop counter exhausted
    REQUIRE(s.r[13] == kStackTop);      // SP fully restored by POP
    REQUIRE(s.pc == kSentinel);         // parked on sentinel self-loop
    REQUIRE((s.cpsr & (1u << 5)) != 0); // still in Thumb state

    std::puts("arm7_thumb_sequence_test: capstone passed");
    return 0;
}
