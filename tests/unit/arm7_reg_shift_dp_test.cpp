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
constexpr u32 kOpMOV  = 0xDu;
constexpr u32 kOpADD  = 0x4u;
constexpr u32 kLSL    = 0x0u;
constexpr u32 kLSR    = 0x1u;
constexpr u32 kASR    = 0x2u;
constexpr u32 kROR    = 0x3u;

// Assemble a reg-shift DP instruction:
//   cond = AL (0xE), I=0, bit4=1, bit7=0
//   |31..28 cond| 00 | 0 opcode S | Rn | Rd | Rs | 0 shift 1 | Rm |
u32 encode_reg_shift_dp(u32 opcode, bool s, u32 rn, u32 rd,
                        u32 rs, u32 shift_type, u32 rm) {
    u32 instr = AL_COND;
    instr |= (opcode & 0xFu) << 21;
    if (s) instr |= (1u << 20);
    instr |= (rn & 0xFu) << 16;
    instr |= (rd & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (shift_type & 0x3u) << 5;
    instr |= (1u << 4);  // bit4 = 1 (reg-shift)
    instr |= (rm & 0xFu);
    return instr;
}

}  // namespace

int main() {
    // Test 1: MOV R0, R1, LSL R2 with R1=0x1, R2=3 → R0 == 0x8
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x1u;
        nds.cpu7().state().r[2] = 3u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0x8u);
    }

    // Test 2: MOV R0, R1, LSL R2 with R2=0 → R0 == R1, CPSR.C preserved
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xDEADBEEFu;
        nds.cpu7().state().r[2] = 0u;
        nds.cpu7().state().cpsr |= (1u << 29);  // set C before the instruction
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0xDEADBEEFu);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C preserved
    }

    // Test 3: MOV R0, R1, LSL R2 with R2=32 → R0 == 0, C == bit 0 of R1
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000001u;  // bit 0 set
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C = bit 0 of R1
    }

    // Test 4: MOV R0, R1, LSL R2 with R2=33 → R0 == 0, C == 0
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 33u;
        nds.cpu7().state().cpsr |= (1u << 29);  // pre-set C to verify it gets cleared
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) == 0);  // C cleared
    }

    // Test 5: MOV R0, R1, LSR R2 with R2=32 → R0 == 0, C == bit 31 of R1
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
    }

    // Test 6: MOV R0, R1, ASR R2 with R1=0x80000000, R2=32 → R0 == 0xFFFFFFFF
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 2, kASR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFFu);
    }

    // Test 7: MOV R0, R1, ROR R2 with R2=32 → R0 == R1, C == bit 31 of R1
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kROR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
    }

    // Test 8: MOV R0, R1, ROR R2 with R2=64 → same as R2=32
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 64u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kROR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
    }

    // Test 9: "only low byte of Rs matters" rule — ADD R0, R1, R2, LSL R3
    // with R3=0xFFFFFF04 should behave exactly like LSL #4 (low byte = 0x04).
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x10u;
        nds.cpu7().state().r[2] = 0x1u;
        nds.cpu7().state().r[3] = 0xFFFFFF04u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpADD, false, 1, 0, 3, kLSL, 2));
        REQUIRE(nds.cpu7().state().r[0] == 0x10u + (0x1u << 4));
    }

    // Test 10: ADDS R0, R1, R2, LSL R3 flag update matches imm-shift path.
    // R1 = 0xFFFFFFFF, R2 = 1, R3 = 0 (no shift). Result wraps to 0, carry out.
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 0x1u;
        nds.cpu7().state().r[3] = 0u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpADD, true, 1, 0, 3, kLSL, 2));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C set (carry out)
    }

    std::puts("arm7_reg_shift_dp_test: all 10 cases passed");
    return 0;
}
