// arm7_thumb_block_branch_test.cpp — THUMB.14 PUSH/POP tests.
//
// Covers:
//   - PUSH {Rlist}          — STMDB SP!, lowest reg at lowest addr
//   - PUSH {Rlist, LR}      — LR stored at the highest address
//   - POP  {Rlist}          — LDMIA SP! with post-increment
//   - POP  {Rlist, PC}      — PC loaded, CPSR.T unchanged (ARMv4 POP {PC})
//   - POP  {PC} with low bit set — word-aligns (ARMv4: bit 0 ignored)
//   - Round-trip PUSH/POP across a synthetic subroutine boundary
//   - Store ordering: lowest listed register goes to the lowest address

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstddef>

using namespace ds;

// THUMB.14: 1011 op1 10 R Rlist8
//   op = 0 -> PUSH, op = 1 -> POP
static u16 thumb14(u32 op, u32 r_bit, u32 rlist8) {
    return static_cast<u16>((0b1011u << 12) | (op << 11) | (0b10u << 9) | (r_bit << 8) |
                            (rlist8 & 0xFFu));
}

// THUMB.15: 1100 op1 Rb3 Rlist8
//   op = 0 -> STMIA, op = 1 -> LDMIA
static u16 thumb15(u32 op, u32 rb, u32 rlist8) {
    return static_cast<u16>((0b1100u << 12) | (op << 11) | ((rb & 0x7u) << 8) | (rlist8 & 0xFFu));
}

// THUMB.12: 1010 op1 Rd3 imm8 — ADD Rd, (PC|SP), #imm8<<2
static u16 thumb12(u32 op, u32 rd, u32 imm8) {
    return static_cast<u16>((0b1010u << 12) | (op << 11) | ((rd & 0x7u) << 8) | (imm8 & 0xFFu));
}

// THUMB.13: 10110000 S imm7 — ADD/SUB SP, #imm7<<2
static u16 thumb13(u32 s, u32 imm7) {
    return static_cast<u16>((0b10110000u << 8) | (s << 7) | (imm7 & 0x7Fu));
}

// THUMB.16: 1101 cond4 imm8 — Bcond
static u16 thumb16(u32 cond, u32 imm8) {
    return static_cast<u16>((0b1101u << 12) | ((cond & 0xFu) << 8) | (imm8 & 0xFFu));
}
// THUMB.17: 11011111 imm8  (cond = 0xF route of THUMB.16) — SWI
static u16 thumb17(u32 imm8) {
    return thumb16(0xFu, imm8);
}
// THUMB.18: 11100 imm11 — B unconditional
static u16 thumb18(u32 imm11) {
    return static_cast<u16>((0b11100u << 11) | (imm11 & 0x7FFu));
}
// THUMB.19 first halfword: 11110 imm11_hi — BL prefix
static u16 thumb19_hi(u32 imm11_hi) {
    return static_cast<u16>((0b11110u << 11) | (imm11_hi & 0x7FFu));
}
// THUMB.19 second halfword BL: 11111 imm11_lo
static u16 thumb19_lo(u32 imm11_lo) {
    return static_cast<u16>((0b11111u << 11) | (imm11_lo & 0x7FFu));
}
// THUMB.19 second halfword BLX (ARMv5): 11101 imm11_lo — UNDEF on ARMv4
static u16 thumb19_lo_blx(u32 imm11_lo) {
    return static_cast<u16>((0b11101u << 11) | (imm11_lo & 0x7FFu));
}

// CPSR flag helpers.
static constexpr u32 CPSR_N = 1u << 31;
static constexpr u32 CPSR_Z = 1u << 30;
static constexpr u32 CPSR_C = 1u << 29;
static constexpr u32 CPSR_V = 1u << 28;

// Overwrite only the NZCV bits of CPSR. Preserves T and mode bits.
static void set_flags(NDS& nds, u32 nzcv) {
    auto& s = nds.cpu7().state();
    s.cpsr = (s.cpsr & ~0xF000'0000u) | (nzcv & 0xF000'0000u);
}

// Instruction stream base — ARM7 WRAM, same region used by every other
// ARM7 test. SP_BASE lives a separate page over so the PUSH/POP
// transfers can't alias the fetched opcode.
static constexpr u32 BASE = 0x0380'0000u;
static constexpr u32 SP_BASE = 0x0380'0400u;

static void setup_thumb(NDS& nds, u32 addr) {
    nds.cpu7().state().pc = addr;
    nds.cpu7().state().cpsr |= (1u << 5);
}

static void step_one(NDS& nds) {
    const u64 before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((before + 1) * 2);
}

// ==== PUSH ====

// PUSH {R0, R1}. SP pre-decrements by 8; R0 at the lower address, R1
// at the higher one (lowest register in list -> lowest memory address).
static void push_r0_r1_full_descending() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb14(0, 0, 0b0000'0011));
    setup_thumb(nds, BASE);
    const u32 sp_before = SP_BASE + 0x100;
    nds.cpu7().state().r[0] = 0xAAAA'AAAAu;
    nds.cpu7().state().r[1] = 0xBBBB'BBBBu;
    nds.cpu7().state().r[13] = sp_before;
    step_one(nds);

    const u32 sp_after = sp_before - 8;
    REQUIRE(nds.cpu7().state().r[13] == sp_after);
    REQUIRE(nds.arm7_bus().read32(sp_after + 0) == 0xAAAA'AAAAu);
    REQUIRE(nds.arm7_bus().read32(sp_after + 4) == 0xBBBB'BBBBu);
}

// PUSH {R0, LR} (R-bit set). LR is bit 14 in the synthetic reg_list,
// so it sorts above R0 and lands at the highest address.
static void push_with_lr_bit_stores_lr_at_highest_address() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb14(0, 1, 0b0000'0001));
    setup_thumb(nds, BASE);
    const u32 sp_before = SP_BASE + 0x100;
    nds.cpu7().state().r[0] = 0x11111111u;
    nds.cpu7().state().r[14] = 0x22222222u;
    nds.cpu7().state().r[13] = sp_before;
    step_one(nds);

    const u32 sp_after = sp_before - 8;
    REQUIRE(nds.cpu7().state().r[13] == sp_after);
    REQUIRE(nds.arm7_bus().read32(sp_after + 0) == 0x11111111u);
    REQUIRE(nds.arm7_bus().read32(sp_after + 4) == 0x22222222u);
}

// PUSH {R2, R5, R7}. Memory order must be R2, R5, R7 (low reg -> low
// address) and SP moves down by 12.
static void push_lowest_register_first() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb14(0, 0, 0b1010'0100));
    setup_thumb(nds, BASE);
    const u32 sp_before = SP_BASE + 0x100;
    nds.cpu7().state().r[2] = 0x22222222u;
    nds.cpu7().state().r[5] = 0x55555555u;
    nds.cpu7().state().r[7] = 0x77777777u;
    nds.cpu7().state().r[13] = sp_before;
    step_one(nds);

    const u32 sp_after = sp_before - 12;
    REQUIRE(nds.cpu7().state().r[13] == sp_after);
    REQUIRE(nds.arm7_bus().read32(sp_after + 0) == 0x22222222u);
    REQUIRE(nds.arm7_bus().read32(sp_after + 4) == 0x55555555u);
    REQUIRE(nds.arm7_bus().read32(sp_after + 8) == 0x77777777u);
}

// ==== POP ====

static void pop_r0_r1_post_increment() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb14(1, 0, 0b0000'0011));
    setup_thumb(nds, BASE);
    const u32 sp_before = SP_BASE;
    nds.arm7_bus().write32(sp_before + 0, 0xCCCC'CCCCu);
    nds.arm7_bus().write32(sp_before + 4, 0xDDDD'DDDDu);
    nds.cpu7().state().r[13] = sp_before;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[0] == 0xCCCC'CCCCu);
    REQUIRE(nds.cpu7().state().r[1] == 0xDDDD'DDDDu);
    REQUIRE(nds.cpu7().state().r[13] == sp_before + 8);
}

// POP {R0, PC}. ARMv4 behavior: PC is word-aligned by write_rd; CPSR.T
// is NOT modified (bit-0 interworking is ARMv5-only).
static void pop_with_pc_bit_loads_pc_and_stays_in_thumb() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb14(1, 1, 0b0000'0001));
    setup_thumb(nds, BASE);
    const u32 sp_before = SP_BASE;
    nds.arm7_bus().write32(sp_before + 0, 0x33333333u);
    nds.arm7_bus().write32(sp_before + 4, 0x0380'1000u);
    nds.cpu7().state().r[13] = sp_before;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[0] == 0x33333333u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'1000u);
    REQUIRE(nds.cpu7().state().r[13] == sp_before + 8);
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);
}

// POP {PC} with a stored value whose low bit is set. ARMv4: the word is
// word-aligned by write_rd's mask; bit 0 does NOT toggle CPSR.T. (ARMv5
// would switch ARM/Thumb based on bit 0 — not our concern here.)
static void pop_pc_with_low_bit_set_word_aligns() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb14(1, 1, 0b0000'0000));
    setup_thumb(nds, BASE);
    const u32 sp_before = SP_BASE;
    nds.arm7_bus().write32(sp_before + 0, 0x0380'1003u);
    nds.cpu7().state().r[13] = sp_before;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    step_one(nds);

    REQUIRE(nds.cpu7().state().pc == 0x0380'1000u);
    REQUIRE(nds.cpu7().state().r[13] == sp_before + 4);
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);
}

// ==== Round trip ====

// Full subroutine-prologue/epilogue sequence:
//   PUSH {R0-R3, LR}
//   (clobber R0..R3 with junk in-between)
//   POP  {R0-R3, PC} into a target address
// After both steps R0..R3 must be restored, PC must match the popped
// value, and SP must return to its starting value.
static void push_then_pop_round_trips_r0_r3_and_lr_pc() {
    NDS nds;
    // Two contiguous Thumb instructions at BASE and BASE+2.
    nds.arm7_bus().write16(BASE + 0, thumb14(0, 1, 0b0000'1111)); // PUSH {R0-R3, LR}
    nds.arm7_bus().write16(BASE + 2, thumb14(1, 1, 0b0000'1111)); // POP  {R0-R3, PC}
    setup_thumb(nds, BASE);

    const u32 sp_before = SP_BASE + 0x100;
    const u32 pc_target = 0x0380'2000u;
    nds.cpu7().state().r[0] = 0xA0000000u;
    nds.cpu7().state().r[1] = 0xA1111111u;
    nds.cpu7().state().r[2] = 0xA2222222u;
    nds.cpu7().state().r[3] = 0xA3333333u;
    nds.cpu7().state().r[14] = pc_target; // LR = return target
    nds.cpu7().state().r[13] = sp_before;

    // Step PUSH.
    step_one(nds);
    const u32 sp_after_push = sp_before - 20; // 5 regs * 4
    REQUIRE(nds.cpu7().state().r[13] == sp_after_push);

    // Clobber R0..R3 so the POP is observable.
    nds.cpu7().state().r[0] = 0xDEAD'0000u;
    nds.cpu7().state().r[1] = 0xDEAD'1111u;
    nds.cpu7().state().r[2] = 0xDEAD'2222u;
    nds.cpu7().state().r[3] = 0xDEAD'3333u;

    // Step POP.
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xA0000000u);
    REQUIRE(nds.cpu7().state().r[1] == 0xA1111111u);
    REQUIRE(nds.cpu7().state().r[2] == 0xA2222222u);
    REQUIRE(nds.cpu7().state().r[3] == 0xA3333333u);
    REQUIRE(nds.cpu7().state().pc == (pc_target & ~0x3u));
    REQUIRE(nds.cpu7().state().r[13] == sp_before); // SP fully restored
}

// ==== THUMB.15 LDMIA / STMIA ====

// STMIA r3!, {r0,r1,r2}. Memory is filled low-reg-to-low-address; Rb
// auto-increments by 4 * count.
static void stmia_round_trip_low_regs() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb15(0, 3, 0b0000'0111));
    setup_thumb(nds, BASE);
    const u32 base = SP_BASE;
    nds.cpu7().state().r[0] = 0xAu;
    nds.cpu7().state().r[1] = 0xBu;
    nds.cpu7().state().r[2] = 0xCu;
    nds.cpu7().state().r[3] = base;
    step_one(nds);

    REQUIRE(nds.arm7_bus().read32(base + 0) == 0xAu);
    REQUIRE(nds.arm7_bus().read32(base + 4) == 0xBu);
    REQUIRE(nds.arm7_bus().read32(base + 8) == 0xCu);
    REQUIRE(nds.cpu7().state().r[3] == base + 12u);
}

// LDMIA r3!, {r0,r1,r2}. Loads land low-reg-from-low-address; Rb
// auto-increments by 4 * count.
static void ldmia_round_trip_low_regs() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb15(1, 3, 0b0000'0111));
    setup_thumb(nds, BASE);
    const u32 base = SP_BASE;
    nds.arm7_bus().write32(base + 0, 0x11u);
    nds.arm7_bus().write32(base + 4, 0x22u);
    nds.arm7_bus().write32(base + 8, 0x33u);
    nds.cpu7().state().r[3] = base;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[0] == 0x11u);
    REQUIRE(nds.cpu7().state().r[1] == 0x22u);
    REQUIRE(nds.cpu7().state().r[2] == 0x33u);
    REQUIRE(nds.cpu7().state().r[3] == base + 12u);
}

// STMIA r0!, {r0,r1,r2}. Rb is the LOWEST listed register: ARMv4 stores
// the OLD base on the first iteration, then writeback applies as usual.
static void stmia_rb_in_list_lowest_stores_old_base() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb15(0, 0, 0b0000'0111));
    setup_thumb(nds, BASE);
    const u32 base = SP_BASE;
    nds.cpu7().state().r[0] = base;
    nds.cpu7().state().r[1] = 0xBBu;
    nds.cpu7().state().r[2] = 0xCCu;
    step_one(nds);

    REQUIRE(nds.arm7_bus().read32(base + 0) == base); // OLD r0
    REQUIRE(nds.arm7_bus().read32(base + 4) == 0xBBu);
    REQUIRE(nds.arm7_bus().read32(base + 8) == 0xCCu);
    REQUIRE(nds.cpu7().state().r[0] == base + 12u);
}

// STMIA r2!, {r0,r1,r2}. Rb is NOT the lowest listed register: ARMv4
// applies the writeback EARLY (before the loop), so the i==Rb iteration
// stores the NEW (post-writeback) base.
static void stmia_rb_in_list_not_lowest_stores_new_base() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb15(0, 2, 0b0000'0111));
    setup_thumb(nds, BASE);
    const u32 base = SP_BASE;
    nds.cpu7().state().r[0] = 0xAAu;
    nds.cpu7().state().r[1] = 0xBBu;
    nds.cpu7().state().r[2] = base;
    step_one(nds);

    REQUIRE(nds.arm7_bus().read32(base + 0) == 0xAAu);
    REQUIRE(nds.arm7_bus().read32(base + 4) == 0xBBu);
    REQUIRE(nds.arm7_bus().read32(base + 8) == base + 12u); // NEW r2
    REQUIRE(nds.cpu7().state().r[2] == base + 12u);
}

// LDMIA r1!, {r0,r1,r2}. ARMv4 LDM with Rb in the list suppresses the
// end-of-instruction writeback — the loaded word into r1 wins.
static void ldmia_rb_in_list_no_writeback() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb15(1, 1, 0b0000'0111));
    setup_thumb(nds, BASE);
    const u32 base = SP_BASE;
    nds.arm7_bus().write32(base + 0, 0x11u);
    nds.arm7_bus().write32(base + 4, 0x77u);
    nds.arm7_bus().write32(base + 8, 0x33u);
    nds.cpu7().state().r[1] = base;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[0] == 0x11u);
    REQUIRE(nds.cpu7().state().r[1] == 0x77u); // loaded value, not base + 12
    REQUIRE(nds.cpu7().state().r[2] == 0x33u);
}

// LDMIA r0!, {} — empty Rlist. execute_block_transfer warns and no-ops;
// neither memory nor Rb should change.
static void ldmia_empty_rlist_warn_and_noop() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb15(1, 0, 0));
    setup_thumb(nds, BASE);
    const u32 base = SP_BASE;
    nds.cpu7().state().r[0] = base;
    nds.cpu7().state().r[1] = 0xDEAD'BEEFu;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[0] == base);         // unchanged
    REQUIRE(nds.cpu7().state().r[1] == 0xDEAD'BEEFu); // untouched
}

// ==== THUMB.12 ADD Rd, PC/SP, #imm8<<2 ====

// ADD r1, PC, #16 at a word-aligned instruction address.
// pc_literal = (BASE + 4) & ~2 = BASE + 4 (since BASE is 4-aligned).
// Result: r1 == BASE + 4 + 16. CPSR untouched.
static void add_pc_aligned_instr_addr() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb12(0, 1, 4));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = 0;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[1] == BASE + 4u + 16u);
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);
}

// ADD r1, PC, #16 at a misaligned instruction address (instr_addr % 4 == 2).
// The handler MUST use pc_literal = (instr_addr + 4) & ~2, NOT state.r[15]
// (which equals instr_addr + 4, unaligned). For instr_addr = BASE + 2:
//   pc_literal = (BASE + 2 + 4) & ~2 = BASE + 4
//   pc_read    = BASE + 6                (would give the WRONG result)
// Expected r1 = BASE + 4 + 16 = BASE + 20.  A bug-using-state.r[15]
// implementation would yield BASE + 22 — observable difference of 2.
static void add_pc_misaligned_instr_addr_uses_literal_align() {
    NDS nds;
    const u32 addr = BASE + 2;
    nds.arm7_bus().write16(addr, thumb12(0, 1, 4));
    setup_thumb(nds, addr);
    nds.cpu7().state().r[1] = 0;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[1] == BASE + 4u + 16u);
}

// ADD r4, SP, #32. CPSR untouched.
static void add_sp_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb12(1, 4, 8));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 0;
    nds.cpu7().state().r[13] = 0x1000'0000u;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[4] == 0x1000'0020u);
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);
}

// ==== THUMB.13 ADD/SUB SP, #imm7<<2 ====

// ADD SP, #16 (S=0). CPSR untouched.
static void add_sp_positive() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb13(0, 4));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[13] = 0x1000'0000u;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[13] == 0x1000'0010u);
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);
}

// SUB SP, #16 (S=1). CPSR untouched.
static void add_sp_negative() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb13(1, 4));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[13] = 0x1000'0010u;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    step_one(nds);

    REQUIRE(nds.cpu7().state().r[13] == 0x1000'0000u);
    REQUIRE(nds.cpu7().state().cpsr == cpsr_before);
}

// ==== THUMB.16 Bcond ====

// BEQ #+4 with Z=1: taken. imm8=2 (halfwords * 2 = +4 bytes).
static void bcond_eq_taken_when_z_set() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb16(0x0u, 2)); // BEQ +4
    setup_thumb(nds, BASE);
    set_flags(nds, CPSR_Z);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE + 4u + 4u);
}

// BEQ with Z=0: not taken — PC advances by 2 only.
static void bcond_eq_not_taken_when_z_clear() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb16(0x0u, 2));
    setup_thumb(nds, BASE);
    set_flags(nds, 0u);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE + 2u);
}

// For each cond in 0x0..0xD: exercise both "met" and "not met" flag
// states and verify the branch is (not) taken. Encodes the same "NZCV
// required to satisfy this cond" table as eval_condition without
// re-importing the function.
static void bcond_every_cond_code_0_to_d() {
    struct Case {
        u32 cond;
        u32 satisfying_flags;
        u32 violating_flags;
    };
    // Each entry picks a single flag state that satisfies the condition
    // and one that violates it. Match eval_condition (arm7_alu.hpp).
    const Case cases[] = {
        {0x0u, CPSR_Z, 0u},              // EQ:  z
        {0x1u, 0u, CPSR_Z},              // NE:  !z
        {0x2u, CPSR_C, 0u},              // CS:  c
        {0x3u, 0u, CPSR_C},              // CC:  !c
        {0x4u, CPSR_N, 0u},              // MI:  n
        {0x5u, 0u, CPSR_N},              // PL:  !n
        {0x6u, CPSR_V, 0u},              // VS:  v
        {0x7u, 0u, CPSR_V},              // VC:  !v
        {0x8u, CPSR_C, CPSR_Z},          // HI:  c && !z
        {0x9u, CPSR_Z, CPSR_C},          // LS:  !c || z
        {0xAu, CPSR_N | CPSR_V, CPSR_N}, // GE:  n == v
        {0xBu, CPSR_N, CPSR_N | CPSR_V}, // LT:  n != v
        {0xCu, CPSR_N | CPSR_V, CPSR_Z}, // GT:  !z && n == v
        {0xDu, CPSR_Z, CPSR_N | CPSR_V}, // LE:  z || n != v
    };
    for (const auto& c : cases) {
        // Taken path.
        {
            NDS nds;
            nds.arm7_bus().write16(BASE, thumb16(c.cond, 2));
            setup_thumb(nds, BASE);
            set_flags(nds, c.satisfying_flags);
            step_one(nds);
            REQUIRE(nds.cpu7().state().pc == BASE + 4u + 4u);
        }
        // Not-taken path.
        {
            NDS nds;
            nds.arm7_bus().write16(BASE, thumb16(c.cond, 2));
            setup_thumb(nds, BASE);
            set_flags(nds, c.violating_flags);
            step_one(nds);
            REQUIRE(nds.cpu7().state().pc == BASE + 2u);
        }
    }
}

// BNE with imm8 = 0xFE (sign-extend = -2). Offset = -2 * 2 = -4.
// Target = instr_addr + 4 + (-4) = instr_addr. Z=0 so taken.
static void bcond_backward_negative_offset() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb16(0x1u, 0xFEu));
    setup_thumb(nds, BASE);
    set_flags(nds, 0u);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE);
}

// imm8 = 0x7F → max positive offset = +254 bytes.
static void bcond_max_positive_offset() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb16(0x1u, 0x7Fu));
    setup_thumb(nds, BASE);
    set_flags(nds, 0u); // NE
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE + 4u + 254u);
}

// imm8 = 0x80 → max negative offset = -256 bytes.
static void bcond_max_negative_offset() {
    NDS nds;
    // Start well away from zero so the subtract doesn't wrap below the
    // bus's test region.
    const u32 here = BASE + 0x400u;
    nds.arm7_bus().write16(here, thumb16(0x1u, 0x80u));
    setup_thumb(nds, here);
    set_flags(nds, 0u); // NE
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == here + 4u - 256u);
}

// cond=0xE is the "always" slot, reserved as UNDEF in the Bcond encoding.
// Slice 3d commit 5: routes through enter_exception(Undef, instr_addr + 2).
// The faulting GPR value (r[0] here) is preserved; CPSR/mode/PC/R14 change.
static void bcond_cond_e_enters_undef() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb16(0xEu, 0x12u));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xA5A5'A5A5u;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    // Exception entry is 3 ARM7 cycles (coarse model).
    const u64 before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((before + 3) * 2);
    REQUIRE(nds.cpu7().state().current_mode() == Mode::Undefined);
    REQUIRE(nds.cpu7().state().pc == 0x00000004u);
    REQUIRE(nds.cpu7().state().r[14] == BASE + 2u);
    REQUIRE(nds.cpu7().state().banks.spsr_und == cpsr_before);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) == 0u); // T clear
    REQUIRE((nds.cpu7().state().cpsr & (1u << 7)) != 0u); // I set
    REQUIRE(nds.cpu7().state().r[0] == 0xA5A5'A5A5u);
}

// ==== THUMB.17 SWI ====

// Slice 3d commit 8: THUMB.17 enters Supervisor mode with return_addr =
// instr_addr + 2, jumps to the SWI vector (0x08), masks IRQs, and clears
// the T bit. Pre-entry CPSR is captured into SPSR_svc. Deep exception-
// entry semantics (every CPSR bit, SWI number variations) live in
// arm7_exception_swi_thumb_test.cpp; this case just confirms the dispatch
// path in the Thumb decoder hands off correctly.
static void swi_enters_supervisor() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb17(0x12u));
    setup_thumb(nds, BASE);

    auto& s = nds.cpu7().state();
    const u32 cpsr_before = s.cpsr;
    // Exception entry is 3 ARM7 cycles (coarse model in enter_exception()).
    const u64 before = s.cycles;
    nds.cpu7().run_until((before + 3) * 2);

    REQUIRE(s.current_mode() == Mode::Supervisor);
    REQUIRE(s.pc == 0x00000008u);  // SWI vector
    REQUIRE(s.r[14] == BASE + 2u); // Thumb return_addr = instr_addr + 2
    REQUIRE(s.banks.spsr_svc == cpsr_before);
    REQUIRE((s.cpsr & (1u << 5)) == 0u); // T clear
    REQUIRE((s.cpsr & (1u << 7)) != 0u); // I set
}

// ==== THUMB.18 B unconditional ====

// B #+4 forward.
static void b_uncond_forward() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb18(2));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE + 4u + 4u);
}

// B with imm11=0x7FE (sext = -2) → offset = -4 → target = instr_addr.
static void b_uncond_backward() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb18(0x7FEu));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE);
}

// imm11=0x3FF → +2046 bytes from instr_addr + 4.
static void b_uncond_max_positive() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb18(0x3FFu));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == BASE + 4u + 2046u);
}

// imm11=0x400 → sext = -1024 → -2048 bytes.
static void b_uncond_max_negative() {
    NDS nds;
    const u32 here = BASE + 0x1000u;
    nds.arm7_bus().write16(here, thumb18(0x400u));
    setup_thumb(nds, here);
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == here + 4u - 2048u);
}

// ==== THUMB.19 BL ====

// Full BL: two halfwords at instr_addr A and A+2. Second halfword records
// LR = (A+2+2) | 1, then branches to LR_from_prefix + (imm11_lo << 1).
static void bl_both_halves_target_and_return() {
    NDS nds;
    const u32 A = BASE;
    // imm11_hi = 0 → LR_pref = (A+4) + 0 = A+4.
    // imm11_lo = 4 → target = (A+4) + (4 << 1) = A + 12.
    nds.arm7_bus().write16(A + 0, thumb19_hi(0));
    nds.arm7_bus().write16(A + 2, thumb19_lo(4));
    setup_thumb(nds, A);

    // Step prefix.
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[14] == A + 4u);
    REQUIRE(nds.cpu7().state().pc == A + 2u);

    // Step suffix.
    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == A + 12u);
    // Return address = next instr after BL suffix = A + 4, with Thumb bit.
    REQUIRE(nds.cpu7().state().r[14] == ((A + 4u) | 1u));
}

// Risk 4: games emit a bare second halfword as "BL LR+imm" after
// pre-staging LR externally. Verify imm11_lo = 4 case with pre-staged LR.
static void bl_second_halfword_alone_uses_prestaged_lr() {
    NDS nds;
    const u32 A = BASE;
    const u32 prestaged_lr = 0x0380'2000u;
    nds.arm7_bus().write16(A, thumb19_lo(4));
    setup_thumb(nds, A);
    nds.cpu7().state().r[14] = prestaged_lr;

    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == prestaged_lr + 8u);
    REQUIRE(nds.cpu7().state().r[14] == ((A + 2u) | 1u));
}

// Same idiom with imm11_lo = 0 — the exact 0xF800 "BL LR+0" encoding.
static void bl_second_halfword_alone_imm_zero() {
    NDS nds;
    const u32 A = BASE;
    const u32 prestaged_lr = 0x0380'3000u;
    nds.arm7_bus().write16(A, thumb19_lo(0));
    setup_thumb(nds, A);
    nds.cpu7().state().r[14] = prestaged_lr;

    step_one(nds);
    REQUIRE(nds.cpu7().state().pc == prestaged_lr);
    REQUIRE(nds.cpu7().state().r[14] == ((A + 2u) | 1u));
}

// BLX suffix (ARMv5) — UNDEF on ARMv4.
// Slice 3d commit 5: routes through enter_exception(Undef, instr_addr + 2).
// UNDEF entry banks R14 into R14_und and writes return_addr = instr_addr + 2.
static void bl_blx_suffix_enters_undef() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb19_lo_blx(4));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[14] = 0x1234'5678u;
    const u32 cpsr_before = nds.cpu7().state().cpsr;
    // Exception entry is 3 ARM7 cycles (coarse model).
    const u64 before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((before + 3) * 2);
    REQUIRE(nds.cpu7().state().current_mode() == Mode::Undefined);
    REQUIRE(nds.cpu7().state().pc == 0x00000004u);
    REQUIRE(nds.cpu7().state().r[14] == BASE + 2u);
    REQUIRE(nds.cpu7().state().banks.spsr_und == cpsr_before);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) == 0u); // T clear
    REQUIRE((nds.cpu7().state().cpsr & (1u << 7)) != 0u); // I set
}

int main() {
    push_r0_r1_full_descending();
    push_with_lr_bit_stores_lr_at_highest_address();
    push_lowest_register_first();
    pop_r0_r1_post_increment();
    pop_with_pc_bit_loads_pc_and_stays_in_thumb();
    pop_pc_with_low_bit_set_word_aligns();
    push_then_pop_round_trips_r0_r3_and_lr_pc();
    stmia_round_trip_low_regs();
    ldmia_round_trip_low_regs();
    stmia_rb_in_list_lowest_stores_old_base();
    stmia_rb_in_list_not_lowest_stores_new_base();
    ldmia_rb_in_list_no_writeback();
    ldmia_empty_rlist_warn_and_noop();
    add_pc_aligned_instr_addr();
    add_pc_misaligned_instr_addr_uses_literal_align();
    add_sp_imm();
    add_sp_positive();
    add_sp_negative();
    bcond_eq_taken_when_z_set();
    bcond_eq_not_taken_when_z_clear();
    bcond_every_cond_code_0_to_d();
    bcond_backward_negative_offset();
    bcond_max_positive_offset();
    bcond_max_negative_offset();
    bcond_cond_e_enters_undef();
    swi_enters_supervisor();
    b_uncond_forward();
    b_uncond_backward();
    b_uncond_max_positive();
    b_uncond_max_negative();
    bl_both_halves_target_and_return();
    bl_second_halfword_alone_uses_prestaged_lr();
    bl_second_halfword_alone_imm_zero();
    bl_blx_suffix_enters_undef();
    return 0;
}
