// arm7_psr_transfer_test.cpp — ARMv4T MRS and MSR tests. Slice 3b2
// Task 9 covers the MRS half (CPSR read, SPSR read with User/System
// warn path). MSR tests land across Tasks 10..13.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

constexpr u32 AL_COND = 0xEu << 28;

// MRS Rd, CPSR/SPSR: cond 00010 P 00 1111 Rd 000000000000
u32 encode_mrs(u32 rd, bool use_spsr) {
    u32 instr = AL_COND | 0x010F0000u;
    if (use_spsr) instr |= (1u << 22);
    instr |= (rd & 0xFu) << 12;
    return instr;
}

}  // namespace

static void mrs_cpsr_reads_full_status_register() {
    NDS nds;
    nds.cpu7().state().cpsr = 0x6000001Fu;  // N=0, Z=1, C=1, V=0, mode=System
    run_one(nds, kBase, encode_mrs(0, false));
    REQUIRE(nds.cpu7().state().r[0] == 0x6000001Fu);
}

static void mrs_spsr_in_system_mode_warns_and_returns_zero() {
    // System mode has no SPSR slot — spsr_slot() returns nullptr, we warn
    // and write 0 into Rd.
    NDS nds;
    nds.cpu7().state().switch_mode(Mode::System);
    run_one(nds, kBase, encode_mrs(0, true));
    REQUIRE(nds.cpu7().state().r[0] == 0u);
}

static void mrs_spsr_in_irq_mode_reads_banked_spsr() {
    // In IRQ mode, MRS SPSR reads banks.spsr_irq.
    NDS nds;
    nds.cpu7().state().switch_mode(Mode::Irq);
    nds.cpu7().state().banks.spsr_irq = 0xDEADBEEFu;
    run_one(nds, kBase, encode_mrs(0, true));
    REQUIRE(nds.cpu7().state().r[0] == 0xDEADBEEFu);
}

int main() {
    mrs_cpsr_reads_full_status_register();
    mrs_spsr_in_system_mode_warns_and_returns_zero();
    mrs_spsr_in_irq_mode_reads_banked_spsr();
    std::puts("arm7_psr_transfer_test: all 3 cases passed");
    return 0;
}
