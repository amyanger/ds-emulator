// ARM7 slice 3b1: B, BL, and BX tests. BX is still stubbed in this
// task; BX-specific test cases land in Task 4 when the BX recognizer
// is added to arm7_dp.cpp.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Load a program into ARM7 WRAM at kBase, point PC there, and run
// exactly `steps` instructions (each costs 1 ARM7 cycle in slice 3b1).
void load_and_run(NDS& nds, std::initializer_list<u32> program, u32 steps) {
    u32 i = 0;
    for (u32 word : program) {
        nds.arm7_bus().write32(kBase + i * 4, word);
        ++i;
    }
    nds.cpu7().state().pc = kBase;
    nds.cpu7().run_until(static_cast<Cycle>(steps) * 2);
}

}  // namespace

static void b_forward_takes_branch() {
    NDS nds;
    // B +8 (to kBase + 8 + 8 = kBase + 0x10, three slots ahead in words).
    // Encoding: cond=AL (E), 101, L=0, offset24.
    // target = pc + 8 + (offset24 << 2). For target = kBase + 16 and
    // pc = kBase, we need offset24 = (16 - 8) / 4 = 2. Instruction:
    //   EA 00 00 02 == 0xEA000002
    load_and_run(nds, { 0xEA00'0002u }, 1);
    REQUIRE(nds.cpu7().state().pc == kBase + 16u);
}

static void b_backward_takes_branch() {
    NDS nds;
    // Put a MOV R0,#1 at kBase + 16, then a B -16 at kBase + 20 that
    // goes back to kBase + 16. After 3 steps we should be past the MOV
    // the second time, with R0 == 1 and pc == kBase + 20 (the branch
    // instruction again) or kBase + 24 if we execute it once more.
    //
    // Simpler: run exactly two steps — the MOV, then the B. After the
    // B, pc must equal kBase + 16.
    // Layout:
    //   kBase + 0x00: MOV R0, #0x55   ; E3A0'0055
    //   kBase + 0x04: B   -4          ; pc+8+(offset<<2) = 0x04+8-16 = 0xFFFFFFF4
    //                                   we want target = kBase + 0x00, so
    //                                   offset24 = ((kBase+0)-(kBase+4+8))/4
    //                                            = -12/4 = -3 = 0xFFFFFD
    //                                   encoded as EAFF'FFFD
    load_and_run(nds, { 0xE3A0'0055u, 0xEAFF'FFFDu }, 2);
    REQUIRE(nds.cpu7().state().r[0] == 0x55u);
    REQUIRE(nds.cpu7().state().pc == kBase + 0u);
}

static void b_condition_fail_does_not_branch() {
    NDS nds;
    // MOVS R0, #0      ; sets Z=1
    // BNE  +8          ; Z=1 -> skipped; pc should advance to kBase + 8
    load_and_run(nds, { 0xE3B0'0000u, 0x1A00'0002u }, 2);
    REQUIRE(nds.cpu7().state().pc == kBase + 8u);
}

static void b_max_negative_offset_wraps() {
    NDS nds;
    // B with offset24 = 0x800000 (sign-bit set, most negative).
    // target = pc + 8 + (sign_extend(0x800000) << 2)
    //        = kBase + 8 + 0xFE000000  (after sign-extend + shift)
    //        = kBase + 0xFE000008 (mod 2^32)
    // Encoding: EA80'0000.
    //
    // We don't execute anything at the target — we only verify the pc
    // value after the branch is computed correctly. The next step()
    // would try to fetch from unmapped memory; we run exactly one step
    // and stop.
    load_and_run(nds, { 0xEA80'0000u }, 1);
    const u32 expected_pc = (kBase + 8u + 0xFE00'0000u) & ~0x3u;
    REQUIRE(nds.cpu7().state().pc == expected_pc);
}

static void bl_links_return_address_and_jumps() {
    NDS nds;
    // BL +8     ; r14 should become kBase + 4, pc should become kBase + 16
    // Encoding: cond=AL, 101, L=1, offset24=2. → 0xEB00'0002
    load_and_run(nds, { 0xEB00'0002u }, 1);
    REQUIRE(nds.cpu7().state().r[14] == kBase + 4u);
    REQUIRE(nds.cpu7().state().pc    == kBase + 16u);
}

static void bl_then_mov_pc_lr_returns() {
    NDS nds;
    // kBase + 0x00: BL +4           ; EB00'0001 (target = kBase + 8 + 4 = kBase + 12)
    // kBase + 0x04: MOV R1, #0x11   ; E3A0'1011 — should be skipped by the BL
    // kBase + 0x08: <unused hole>
    // kBase + 0x0C: MOV PC, LR      ; E1A0'F00E  (Rd=15, Rm=14, MOV)
    // After 2 steps (BL, then MOV PC,LR) pc should be kBase + 4.
    load_and_run(nds,
                 { 0xEB00'0001u,   // BL +4
                   0xE3A0'1011u,   // MOV R1, #0x11 (skipped)
                   0x0000'0000u,   // hole
                   0xE1A0'F00Eu }, // MOV PC, LR
                 2);
    REQUIRE(nds.cpu7().state().r[14] == kBase + 4u);
    REQUIRE(nds.cpu7().state().r[1]  == 0u);           // MOV R1 was skipped
    REQUIRE(nds.cpu7().state().pc    == kBase + 4u);
}

static void bx_arm_target_jumps_and_clears_t() {
    NDS nds;
    // MOV R2, #(kBase + 0x10)   ; we'll synthesize this via two instructions:
    //                             kBase + 0x10 is 0x0380'0010. Encoding a
    //                             32-bit immediate in a MOV isn't possible
    //                             in one instruction, so we use the register
    //                             file directly and set R2 from the test.
    // BX R2                      ; E12F'FF12
    nds.arm7_bus().write32(kBase + 0, 0xE12F'FF12u);  // BX R2
    nds.cpu7().state().r[2] = kBase + 0x10u;
    nds.cpu7().state().pc   = kBase;
    nds.cpu7().run_until(2);  // exactly 1 ARM7 instruction

    REQUIRE(nds.cpu7().state().pc == kBase + 0x10u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) == 0u);  // T clear
}

static void bx_thumb_target_sets_t_and_masks_bit0() {
    NDS nds;
    // BX R3 with R3 = kBase + 0x21 → pc should become kBase + 0x20
    // and T should be set. We do NOT execute another step after this;
    // the next step_arm would trip the "Thumb not implemented" assert.
    nds.arm7_bus().write32(kBase + 0, 0xE12F'FF13u);  // BX R3
    nds.cpu7().state().r[3] = kBase + 0x21u;
    nds.cpu7().state().pc   = kBase;
    nds.cpu7().run_until(2);

    REQUIRE(nds.cpu7().state().pc == kBase + 0x20u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) != 0u);  // T set
}

int main() {
    b_forward_takes_branch();
    b_backward_takes_branch();
    b_condition_fail_does_not_branch();
    b_max_negative_offset_wraps();
    bl_links_return_address_and_jumps();
    bl_then_mov_pc_lr_returns();
    bx_arm_target_jumps_and_clears_t();
    bx_thumb_target_sets_t_and_masks_bit0();
    std::puts("arm7_branch_test OK");
    return 0;
}
