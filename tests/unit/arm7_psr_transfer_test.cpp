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

// MSR PSR, Rm: cond 00010 P 10 mask 1111 00000000 Rm
u32 encode_msr_reg(bool use_spsr, u32 mask_bits, u32 rm) {
    u32 instr = AL_COND | 0x0120F000u;
    if (use_spsr) instr |= (1u << 22);
    instr |= (mask_bits & 0xFu) << 16;
    instr |= (rm & 0xFu);
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
constexpr u32 kMaskFSXC = 0xFu;  // all four byte fields (f, s, x, c)

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

static void msr_cpsr_f_reg_form_writes_flag_byte_from_register() {
    // MSR CPSR_f, R0 with R0 = 0x60000000 → only flag byte changes.
    // R0's top byte sets Z=1 (bit30), C=1 (bit29), N=0, V=0. Other CPSR
    // bytes preserved.
    NDS nds;
    nds.cpu7().state().cpsr = 0x0000001Fu;  // System mode, flags clear
    nds.cpu7().state().r[0] = 0x60000000u;
    run_one(nds, kBase, encode_msr_reg(false, kMaskF, 0));
    REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) == 0);  // N clear
    REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
    REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C set
    REQUIRE((nds.cpu7().state().cpsr & (1u << 28)) == 0);  // V clear
    REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x1Fu);    // mode preserved
}

static void msr_cpsr_fsxc_reg_form_writes_full_register() {
    // MSR CPSR_fsxc, R0 writes every byte of CPSR from R0.
    // Keep mode bits at 0x1F so we don't trigger Task 13's mode-change
    // path. R0 = 0xDEAD'BE5F (top nibble DEAD is flag+reserved bytes,
    // low byte 0x5F = I bit clear, T=0, mode=System).
    NDS nds;
    nds.cpu7().state().cpsr = 0x0000001Fu;
    nds.cpu7().state().r[0] = 0xDEADBE5Fu;
    run_one(nds, kBase, encode_msr_reg(false, kMaskFSXC, 0));
    REQUIRE(nds.cpu7().state().cpsr == 0xDEADBE5Fu);
}

static void msr_spsr_in_system_mode_warns_no_change() {
    // System mode has no SPSR slot — spsr_slot() returns nullptr.
    // Write should warn and no-op; the banked SPSR for any other mode
    // must stay untouched.
    NDS nds;
    nds.cpu7().state().switch_mode(Mode::System);
    nds.cpu7().state().banks.spsr_svc = 0xAAAAAAAAu;  // witness value — must survive
    nds.cpu7().state().r[0] = 0x5A5A5A5Au;
    run_one(nds, kBase, encode_msr_reg(true, kMaskFC, 0));
    REQUIRE(nds.cpu7().state().banks.spsr_svc == 0xAAAAAAAAu);
}

static void msr_spsr_f_reg_form_in_irq_mode_writes_irq_bank() {
    // IRQ mode → spsr_slot() returns &banks.spsr_irq. MSR SPSR_f, R0
    // writes only the flag byte of the IRQ-bank SPSR from R0.
    // Pre-load banks.spsr_irq with a known value and verify only the
    // top byte changes.
    NDS nds;
    nds.cpu7().state().switch_mode(Mode::Irq);
    nds.cpu7().state().banks.spsr_irq = 0x00000011u;  // mode byte only
    nds.cpu7().state().r[0] = 0xF0000000u;
    run_one(nds, kBase, encode_msr_reg(true, kMaskF, 0));
    REQUIRE(nds.cpu7().state().banks.spsr_irq == 0xF0000011u);
}

static void msr_spsr_c_imm_form_in_supervisor_mode_writes_svc_bank() {
    // Supervisor mode → spsr_slot() returns &banks.spsr_svc. MSR SPSR_c, #0x80
    // writes only the control byte. imm8=0x80, rot=0 → source=0x00000080.
    // Pre-load banks.spsr_svc with 0x00000013 (mode byte only). After the
    // MSR, control byte becomes 0x80 and the rest is preserved.
    NDS nds;
    nds.cpu7().state().switch_mode(Mode::Supervisor);
    nds.cpu7().state().banks.spsr_svc = 0x00000013u;
    run_one(nds, kBase, encode_msr_imm(true, kMaskC, 0x80u, 0));
    REQUIRE(nds.cpu7().state().banks.spsr_svc == 0x00000080u);
}

static void msr_cpsr_c_mode_change_svc_to_irq_banks_registers() {
    // Start in Supervisor mode with a known R13 (SVC stack pointer).
    // Pre-load the IRQ bank's R13 so the switch_mode load brings in a
    // distinct value. Then MSR CPSR_c, #0x12 switches to IRQ mode.
    // Verify:
    //   - CPSR mode bits are now 0x12 (IRQ)
    //   - Visible R13 now holds the IRQ-bank value
    //   - The old SVC-bank R13 contains what we had in R13 before
    NDS nds;
    nds.cpu7().state().banks.irq_r13_r14[0] = 0x1EEE1EEEu;  // IRQ R13
    nds.cpu7().state().switch_mode(Mode::Supervisor);
    nds.cpu7().state().r[13] = 0x5CC5CCC5u;  // Supervisor R13

    // MSR CPSR_c, #0x12 — switch to IRQ. imm8=0x12, rot=0. mask=kMaskC.
    run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x12u, 0));

    REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x12u);
    REQUIRE(nds.cpu7().state().r[13] == 0x1EEE1EEEu);
    REQUIRE(nds.cpu7().state().banks.svc_r13_r14[0] == 0x5CC5CCC5u);
}

static void msr_cpsr_c_t_bit_change_warns_but_writes() {
    // Start in ARM state (T clear), write CPSR_c with T=1.
    // imm8 = 0x3F → 0011_1111: T=1, mode=0x1F (System).
    // Starting CPSR = 0x1F, so starting mode bits are 0x1F — no mode
    // change, just the T bit flipping. Warn fires; CPSR control byte
    // reflects the write.
    NDS nds;
    nds.cpu7().state().cpsr = 0x0000001Fu;  // System, T clear
    run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x3Fu, 0));
    REQUIRE((nds.cpu7().state().cpsr & 0x0000003Fu) == 0x0000003Fu);
}

static void msr_cpsr_c_invalid_mode_warns_and_skips_bank_swap() {
    // Try to set mode bits to 0x00 — not a valid ARMv4T mode. Warn
    // fires, switch_mode is NOT called, but the byte write still lands
    // so subsequent MRS reads see the garbage the game requested.
    NDS nds;
    nds.cpu7().state().cpsr = 0x0000001Fu;  // System
    run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x00u, 0));
    REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x00u);
}

static void msr_disable_interrupts_round_trip_idiom() {
    // The canonical "disable IRQs around a critical section" idiom:
    //   MRS R0, CPSR          ; snapshot
    //   ORR R0, R0, #0x80     ; set I bit
    //   MSR CPSR_c, R0        ; write back
    //
    // We load all three instructions into ARM7 WRAM as a mini-program
    // and run_until() them end-to-end. Starting mode is Supervisor
    // (reset default), so the mode bits don't change across the MSR.
    // Final CPSR should have the I bit set and everything else preserved.
    NDS nds;
    // Reset default is Supervisor mode (0x13), I=1, F=1. That already
    // has I set, which makes the "I bit toggled" assertion trivially
    // true. Clear I first so the test actually proves something.
    nds.cpu7().state().cpsr = 0x00000013u;  // Supervisor, I clear, flags clear

    // Assemble the 3-instruction mini-program at kBase.
    const u32 mrs_r0_cpsr      = encode_mrs(0, false);
    const u32 orr_r0_r0_0x80   = 0xE3800080u;  // ORR R0, R0, #0x80 (imm form DP)
    const u32 msr_cpsr_c_r0    = encode_msr_reg(false, kMaskC, 0);

    nds.arm7_bus().write32(kBase + 0, mrs_r0_cpsr);
    nds.arm7_bus().write32(kBase + 4, orr_r0_r0_0x80);
    nds.arm7_bus().write32(kBase + 8, msr_cpsr_c_r0);
    nds.cpu7().state().pc = kBase;
    nds.cpu7().state().r[15] = kBase + 8;
    // 3 ARM7 cycles == 6 ARM9 cycles.
    nds.cpu7().run_until(6);

    REQUIRE((nds.cpu7().state().cpsr & 0x80u) != 0);    // I bit set
    REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x13u); // Supervisor preserved
}

int main() {
    mrs_cpsr_reads_full_status_register();
    mrs_spsr_in_system_mode_warns_and_returns_zero();
    mrs_spsr_in_irq_mode_reads_banked_spsr();
    msr_cpsr_f_rotated_imm_writes_flag_byte();
    msr_cpsr_f_zero_clears_flag_byte();
    msr_cpsr_c_sets_i_bit_without_touching_flags();
    msr_cpsr_fc_writes_flags_and_control_bytes();
    msr_cpsr_f_reg_form_writes_flag_byte_from_register();
    msr_cpsr_fsxc_reg_form_writes_full_register();
    msr_spsr_in_system_mode_warns_no_change();
    msr_spsr_f_reg_form_in_irq_mode_writes_irq_bank();
    msr_spsr_c_imm_form_in_supervisor_mode_writes_svc_bank();
    msr_cpsr_c_mode_change_svc_to_irq_banks_registers();
    msr_cpsr_c_t_bit_change_warns_but_writes();
    msr_cpsr_c_invalid_mode_warns_and_skips_bank_swap();
    msr_disable_interrupts_round_trip_idiom();
    std::puts("arm7_psr_transfer_test: all 16 cases passed");
    return 0;
}
