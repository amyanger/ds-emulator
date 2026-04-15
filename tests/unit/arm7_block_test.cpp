// arm7_block_test.cpp — scaffold test for the ARMv4T block data transfer
// (LDM / STM) dispatcher. Commit 3 of slice 3b4 only wires dispatch_block
// into the primary dispatch table as a warn stub; this test verifies the
// wiring by executing a recognizable LDM/STM encoding and confirming the
// stub behaves as a no-op (PC advances 4, cycles increment by 1, register
// file otherwise unchanged). Real LDM/STM semantics land in commits 4-15
// and will be covered by dedicated tests.

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
// pipeline model, and run exactly one ARM7 cycle. Mirrors the helper used
// by every other arm7 family test.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

// Snapshot R0..R14 so we can assert the register file is unchanged by the
// stub. R15 is excluded because the pipeline model updates it on every
// step (instr_addr + 8 before dispatch, instr_addr + 4 after).
struct RegSnapshot {
    u32 r[15];
};

static RegSnapshot snapshot_regs(const Arm7State& s) {
    RegSnapshot out{};
    for (u32 i = 0; i < 15; ++i) {
        out.r[i] = s.r[i];
    }
    return out;
}

static void require_regs_unchanged(const RegSnapshot& before, const Arm7State& after) {
    for (u32 i = 0; i < 15; ++i) {
        REQUIRE(before.r[i] == after.r[i]);
    }
}

} // namespace

// LDMIA R0, {R1}  — 0xE8900002
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=0, L=1,
// Rn=0, reg_list=0x0002. This is the simplest LDM form and routes
// through the primary 0b100 dispatch slot into dispatch_block.
static void test_dispatch_block_stub_ldmia_does_not_crash() {
    NDS nds;

    // Seed a couple of GPRs so we can prove the stub leaves them alone.
    nds.cpu7().state().r[0] = 0x0380'0200u;
    nds.cpu7().state().r[1] = 0xDEAD'BEEFu;
    nds.cpu7().state().r[2] = 0xCAFE'BABEu;

    const RegSnapshot before = snapshot_regs(nds.cpu7().state());
    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE890'0002u);

    // PC advances by 4, cycles consumed == 1 (ARM7 tracks cycles in ARM7
    // units; run_one advances by one ARM9-cycle pair which nets one ARM7
    // cycle for a 1-cycle instruction).
    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);

    // No bus traffic, no register changes.
    require_regs_unchanged(before, nds.cpu7().state());
}

// STMIA R0!, {R1}  — 0xE8A00002
// Encoding check: cond=AL, bits[27:25]=100, P=0, U=1, S=0, W=1, L=0,
// Rn=0, reg_list=0x0002. Second-simplest STM form; also routes through
// the 0b100 slot. Confirms both the L=0 and L=1 cases reach the stub.
static void test_dispatch_block_stub_stmia_does_not_crash() {
    NDS nds;

    nds.cpu7().state().r[0] = 0x0380'0300u;
    nds.cpu7().state().r[1] = 0x1234'5678u;

    const RegSnapshot before = snapshot_regs(nds.cpu7().state());
    const u64 cycles_before = nds.cpu7().state().cycles;

    run_one(nds, kBase, 0xE8A0'0002u);

    REQUIRE(nds.cpu7().state().pc == kBase + 4u);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1u);
    require_regs_unchanged(before, nds.cpu7().state());
}

int main() {
    test_dispatch_block_stub_ldmia_does_not_crash();
    test_dispatch_block_stub_stmia_does_not_crash();

    std::puts("arm7_block_test: dispatch_block stub wiring verified");
    return 0;
}
