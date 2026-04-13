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

int main() {
    mov_imm_zero_writes_register_and_advances_pc();
    mov_imm_0x42_lands_in_r1();
    condition_ne_skips_when_z_set();
    std::puts("arm7_data_processing_test OK");
    return 0;
}
