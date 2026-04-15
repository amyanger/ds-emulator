// arm7_block_swap_sequence_test.cpp — end-to-end capstone for slice 3b4.
// Runs a short program that exercises the full LDM/STM engine and the
// new SWP family against a shared piece of register/memory state,
// interleaved with a MUL (slice 3b2) and a LDRH (slice 3b3). The point
// is to catch inter-family bugs where each opcode works in isolation
// but the dispatcher or state invariants break when they compose —
// e.g. MUL accidentally stomping the writeback helper from block
// transfers, or SWP corrupting the register latched for a following
// LDM.
//
// Program:
//     0:  STMFD SP!, {R0-R3, LR}      ; push R0..R3 and LR onto ARM7 WRAM
//     1:  MUL   R0, R1, R2             ; clobber R0 with R1 * R2 (0xBB * 0xCC)
//     2:  LDRH  R5, [R6, #0]           ; load low halfword from the swap target
//                                        into R5 (which is NOT in the pop list
//                                        so the value survives the LDMFD)
//     3:  SWP   R7, R4, [R6]           ; atomic swap: R7 gets the pre-write word,
//                                        memory at [R6] gets R4
//     4:  LDMFD SP!, {R0-R3, LR}       ; pop R0..R3 and LR; SP writeback restores
//                                        the original stack pointer
//
// After the program runs, every register clobbered in the middle must
// be restored to its pre-STMFD value, the swap must have updated
// memory with R4, and the halfword load must have landed in R1 before
// the pop restored it. The pop-then-assert dance is the capstone's
// reason to exist: any slip in the LDM/STM engine surfaces as a
// failed register equality at the end.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;
constexpr u32 kStackTop = 0x0380'0300u; // full-descending stack grows downward
constexpr u32 kSwapBase = 0x0380'0400u; // well clear of both program and stack

// Register sentinels we push and then pop. All fit in 8 bits so they
// can be installed via direct state seeding without needing extra
// instructions in the program body.
constexpr u32 kR0 = 0x0000'00AAu;
constexpr u32 kR1 = 0x0000'00BBu;
constexpr u32 kR2 = 0x0000'00CCu;
constexpr u32 kR3 = 0x0000'00DDu;
constexpr u32 kLR = 0x0000'00EEu;

// Values used by the SWP and LDRH in the middle of the program.
constexpr u32 kR4_rm = 0xDEAD'BEEFu;         // stored into memory by SWP
constexpr u32 kSwapMemBefore = 0xCAFE'BABEu; // read by SWP and LDRH
// The LDRH at kSwapBase reads the low halfword of kSwapMemBefore, which
// in little-endian storage is 0xBABE. Asserted below as 0x0000'BABEu
// after zero-extension into R1.

} // namespace

int main() {
    NDS nds;
    auto& s = nds.cpu7().state();

    // --- Seed all register state the program relies on. ---
    s.r[0] = kR0;
    s.r[1] = kR1;
    s.r[2] = kR2;
    s.r[3] = kR3;
    s.r[4] = kR4_rm;
    s.r[6] = kSwapBase;
    s.r[13] = kStackTop;
    s.r[14] = kLR;

    // Seed the swap target word in ARM7 WRAM.
    nds.arm7_bus().write32(kSwapBase, kSwapMemBefore);

    // --- Program encoding ---
    //
    //   0xE92D400F  STMFD SP!, {R0-R3, LR}
    //               cond=AL, 100, P=1, U=0, S=0, W=1, L=0, Rn=13, list=0x400F
    //
    //   0xE0000291  MUL R0, R1, R2
    //               cond=AL, 0000000S=0, Rd=R0 (bits[19:16]), Rn=0000,
    //               Rs=R2 (bits[11:8]), 1001, Rm=R1 (bits[3:0])
    //
    //   0xE1D650B0  LDRH R5, [R6, #0]
    //               cond=AL, 000, P=1, U=1, I=1, W=0, L=1, Rn=R6, Rd=R5,
    //               imm[7:4]=0, 1, SH=01 (halfword), 1, imm[3:0]=0.
    //               R5 is deliberately outside the STMFD/LDMFD list so
    //               the load's effect is observable post-instruction.
    //
    //   0xE1067094  SWP R7, R4, [R6]
    //               cond=AL, 00010, B=0, 00, Rn=R6, Rd=R7,
    //               bits[11:4]=00001001, Rm=R4
    //
    //   0xE8BD400F  LDMFD SP!, {R0-R3, LR}
    //               cond=AL, 100, P=0, U=1, S=0, W=1, L=1, Rn=13, list=0x400F
    constexpr u32 program[] = {
        0xE92D'400Fu, // STMFD SP!, {R0-R3, LR}
        0xE000'0291u, // MUL R0, R1, R2
        0xE1D6'50B0u, // LDRH R5, [R6]
        0xE106'7094u, // SWP R7, R4, [R6]
        0xE8BD'400Fu, // LDMFD SP!, {R0-R3, LR}
    };
    constexpr u32 kInstrCount = sizeof(program) / sizeof(program[0]);

    for (u32 i = 0; i < kInstrCount; ++i) {
        nds.arm7_bus().write32(kBase + i * 4u, program[i]);
    }

    s.pc = kBase;
    s.r[15] = kBase + 8u;
    // Each instruction costs 1 ARM7 cycle = 2 ARM9 cycles.
    nds.cpu7().run_until(kInstrCount * 2);

    // --- LDMFD restored every pushed register to its original value ---
    REQUIRE(s.r[0] == kR0);
    REQUIRE(s.r[1] == kR1);
    REQUIRE(s.r[2] == kR2);
    REQUIRE(s.r[3] == kR3);
    REQUIRE(s.r[14] == kLR);

    // --- LDMFD writeback rewound SP to the original top ---
    REQUIRE(s.r[13] == kStackTop);

    // --- SWP loaded the pre-write memory word into R7 ---
    REQUIRE(s.r[7] == kSwapMemBefore);

    // --- SWP wrote the original R4 (Rm) into memory at [R6] ---
    REQUIRE(nds.arm7_bus().read32(kSwapBase) == kR4_rm);

    // --- R4 itself is unchanged by the swap ---
    REQUIRE(s.r[4] == kR4_rm);

    // --- R6 (Rn of the swap) untouched ---
    REQUIRE(s.r[6] == kSwapBase);

    // --- LDRH wrote the low halfword of kSwapMemBefore into R5,
    //     zero-extended. Little-endian: kSwapMemBefore = 0xCAFE'BABE
    //     stores bytes BE BA FE CA, so the halfword at [kSwapBase]
    //     reads as 0xBABE. R5 is outside the pop list so the value
    //     survives to the end of the program. ---
    REQUIRE(s.r[5] == 0x0000'BABEu);

    // --- The STMFD pushed words are still sitting on the stack. The
    //     LDMFD only advanced SP; it did not zero the memory, so we
    //     can inspect the pushed values to prove the push actually
    //     happened. STMFD walks low-register-to-low-address, so R0
    //     lands at the lowest address (kStackTop - 20). ---
    REQUIRE(nds.arm7_bus().read32(kStackTop - 20u) == kR0);
    REQUIRE(nds.arm7_bus().read32(kStackTop - 16u) == kR1);
    REQUIRE(nds.arm7_bus().read32(kStackTop - 12u) == kR2);
    REQUIRE(nds.arm7_bus().read32(kStackTop - 8u) == kR3);
    REQUIRE(nds.arm7_bus().read32(kStackTop - 4u) == kLR);

    // --- PC pin: guarantees run_until executed all kInstrCount
    //     instructions. Without this, a dispatcher regression could
    //     leave earlier REQUIREs vacuously true because the state
    //     they check never mutated. ---
    REQUIRE(s.pc == kBase + kInstrCount * 4u);

    std::puts("arm7_block_swap_sequence_test: capstone passed");
    return 0;
}
