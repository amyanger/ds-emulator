// Flag-update tests for S-form data-processing instructions and the
// compare-only opcodes TST/TEQ/CMP/CMN. Each case runs exactly one
// instruction and inspects the resulting CPSR NZCV bits plus any
// register writeback.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static constexpr u32 N_BIT = 1u << 31;
static constexpr u32 Z_BIT = 1u << 30;
static constexpr u32 C_BIT = 1u << 29;
static constexpr u32 V_BIT = 1u << 28;

// Preload a single instruction word at pc and run one instruction.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1);
}

static void movs_imm_zero_sets_z_clears_n() {
    NDS nds;
    // MOVS R0, #0 -> 0xE3B0'0000 (S-bit set on MOV imm).
    run_one(nds, 0x0380'0000u, 0xE3B0'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) == 0);
}

static void movs_imm_negative_sets_n_clears_z() {
    NDS nds;
    // MVNS R0, #0 -> 0xE3F0'0000 (MVN with S-bit, result=0xFFFFFFFF).
    run_one(nds, 0x0380'0000u, 0xE3F0'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) == 0);
}

static void adds_overflow_sets_v() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x7FFF'FFFFu;
    // ADDS R0, R1, #1 -> 0xE291'0001
    run_one(nds, 0x0380'0000u, 0xE291'0001u);
    REQUIRE(nds.cpu7().state().r[0] == 0x8000'0000u);
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) == 0);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) == 0);
}

static void adds_wrap_sets_c_no_v() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFF'FFFFu;
    // ADDS R0, R1, #1 -> 0xE291'0001
    run_one(nds, 0x0380'0000u, 0xE291'0001u);
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) == 0);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) == 0);
}

static void subs_sets_c_when_no_borrow() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    // SUBS R0, R1, #5 -> 0xE251'0005
    run_one(nds, 0x0380'0000u, 0xE251'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 5u);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) != 0);  // no borrow
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) == 0);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) == 0);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) == 0);
}

static void subs_clears_c_when_borrow() {
    NDS nds;
    nds.cpu7().state().r[1] = 3;
    // SUBS R0, R1, #5 -> 0xE251'0005
    run_one(nds, 0x0380'0000u, 0xE251'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFEu);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) == 0);  // borrow
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) == 0);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) == 0);
}

static void cmp_sets_flags_but_no_register_write() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xDEAD'BEEFu;  // sentinel
    nds.cpu7().state().r[1] = 7;
    // CMP R1, #7 -> 0xE351'0007
    run_one(nds, 0x0380'0000u, 0xE351'0007u);
    REQUIRE(nds.cpu7().state().r[0] == 0xDEAD'BEEFu);  // untouched
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) != 0);   // equal → no borrow
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) == 0);
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) == 0);
}

static void tst_sets_nz_from_and_but_no_register_write() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xDEAD'BEEFu;  // sentinel
    nds.cpu7().state().r[1] = 0;
    // TST R1, #0 -> 0xE311'0000
    run_one(nds, 0x0380'0000u, 0xE311'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0xDEAD'BEEFu);  // untouched
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) == 0);
}

int main() {
    movs_imm_zero_sets_z_clears_n();
    movs_imm_negative_sets_n_clears_z();
    adds_overflow_sets_v();
    adds_wrap_sets_c_no_v();
    subs_sets_c_when_no_borrow();
    subs_clears_c_when_borrow();
    cmp_sets_flags_but_no_register_write();
    tst_sets_nz_from_and_but_no_register_write();
    std::puts("arm7_flags_test OK");
    return 0;
}
