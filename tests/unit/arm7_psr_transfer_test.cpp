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

// MSR PSR, #imm form:
//   cond 00110 P 10 mask 1111 rot imm8
u32 encode_msr_imm(bool use_spsr, u32 mask_bits, u32 imm8, u32 rot) {
    u32 instr = AL_COND | 0x0320F000u;
    if (use_spsr) instr |= (1u << 22);
    instr |= (mask_bits & 0xFu) << 16;
    instr |= (rot & 0xFu) << 8;
    instr |= imm8 & 0xFFu;
    return instr;
}

// Field mask bit positions (bits[19:16], listed LSB-first):
//   bit 0 = c (control byte)
//   bit 1 = x (extension byte, reserved on v4T)
//   bit 2 = s (status byte, reserved on v4T)
//   bit 3 = f (flags byte)
constexpr u32 kMaskF    = 0x8u;
constexpr u32 kMaskC    = 0x1u;
constexpr u32 kMaskFC   = kMaskF | kMaskC;

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

static void msr_cpsr_f_rotated_imm_writes_flag_byte() {
    // Encode 0xF0000000 via rot=4 imm8=0xF0: 0xF0 ror 8 == 0xF0000000.
    // Verify only the top byte (flags) of CPSR changes.
    NDS nds;
    nds.cpu7().state().cpsr = 0x0000001Fu;
    run_one(nds, kBase, encode_msr_imm(false, kMaskF, 0xF0u, 4));
    REQUIRE(nds.cpu7().state().cpsr == 0xF000001Fu);
}

static void msr_cpsr_f_zero_clears_flag_byte() {
    NDS nds;
    nds.cpu7().state().cpsr = 0xF000001Fu;
    run_one(nds, kBase, encode_msr_imm(false, kMaskF, 0u, 0));
    REQUIRE(nds.cpu7().state().cpsr == 0x0000001Fu);
}

static void msr_cpsr_c_sets_i_bit_without_touching_flags() {
    // Source 0x9F: 0x80 (I bit) | 0x1F (System mode bits) — keeps the
    // mode at System so Task 13's future mode-change path doesn't
    // change this test's expected value.
    NDS nds;
    nds.cpu7().state().cpsr = 0x6000001Fu;  // Z + C + System
    run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x9Fu, 0));
    REQUIRE(nds.cpu7().state().cpsr == 0x6000009Fu);
}

static void msr_cpsr_fc_writes_flags_and_control_bytes() {
    // byte_mask = 0xFF0000FF. source = 0x0000009F (rot=0, imm=0x9F).
    // Top byte written: 0x00. Bottom byte written: 0x9F. Middle bytes
    // preserved: 0x0000. Starting cpsr = 0x1000001F → final 0x0000009F.
    NDS nds;
    nds.cpu7().state().cpsr = 0x1000001Fu;
    run_one(nds, kBase, encode_msr_imm(false, kMaskFC, 0x9Fu, 0));
    REQUIRE(nds.cpu7().state().cpsr == 0x0000009Fu);
}

int main() {
    mrs_cpsr_reads_full_status_register();
    mrs_spsr_in_system_mode_warns_and_returns_zero();
    mrs_spsr_in_irq_mode_reads_banked_spsr();
    msr_cpsr_f_rotated_imm_writes_flag_byte();
    msr_cpsr_f_zero_clears_flag_byte();
    msr_cpsr_c_sets_i_bit_without_touching_flags();
    msr_cpsr_fc_writes_flags_and_control_bytes();
    std::puts("arm7_psr_transfer_test: all 7 cases passed");
    return 0;
}
