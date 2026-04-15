// arm7_thumb_ls_test.cpp — Thumb load/store format tests.
// THUMB.6 LDR PC-relative, THUMB.7 LDR/STR reg offset,
// THUMB.8 STRH/LDSB/LDRH/LDSH reg offset, THUMB.9 LDR/STR imm offset,
// THUMB.10 STRH/LDRH imm offset, THUMB.11 LDR/STR SP-relative.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

// THUMB.6: 01001 Rd3 imm8
static u16 thumb6(u32 rd, u32 imm8) {
    return static_cast<u16>((0b01001u << 11) | (rd << 8) | imm8);
}

// THUMB.7: 0101 LB 0 Ro3 Rb3 Rd3
static u16 thumb7(u32 l, u32 b, u32 ro, u32 rb, u32 rd) {
    return static_cast<u16>((0b0101u << 12) | (l << 11) | (b << 10) | (ro << 6) | (rb << 3) | rd);
}

// THUMB.8: 0101 op2 1 Ro3 Rb3 Rd3 (op: 0=STRH 1=LDSB 2=LDRH 3=LDSH)
static u16 thumb8(u32 op, u32 ro, u32 rb, u32 rd) {
    return static_cast<u16>((0b0101u << 12) | (op << 10) | (1u << 9) | (ro << 6) | (rb << 3) | rd);
}

// THUMB.9: 011 BL imm5 Rb3 Rd3
static u16 thumb9(u32 b, u32 l, u32 imm5, u32 rb, u32 rd) {
    return static_cast<u16>((0b011u << 13) | (b << 12) | (l << 11) | (imm5 << 6) | (rb << 3) | rd);
}

// THUMB.10: 1000 op1 imm5 Rb3 Rd3 (op: 0=STRH 1=LDRH)
static u16 thumb10(u32 op, u32 imm5, u32 rb, u32 rd) {
    return static_cast<u16>((0b1000u << 12) | (op << 11) | (imm5 << 6) | (rb << 3) | rd);
}

// THUMB.11: 1001 L Rd3 imm8
static u16 thumb11(u32 l, u32 rd, u32 imm8) {
    return static_cast<u16>((0b1001u << 12) | (l << 11) | (rd << 8) | imm8);
}

static constexpr u32 BASE = 0x0380'0000u;

static void setup_thumb(NDS& nds, u32 addr) {
    nds.cpu7().state().pc = addr;
    nds.cpu7().state().cpsr |= (1u << 5);
}

static void step_one(NDS& nds) {
    const u64 before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((before + 1) * 2);
}

// ==== THUMB.6 — LDR PC-relative ====

static void thumb6_ldr_pc_basic() {
    // LDR R0, [PC, #4*4]. Place a known word at pc_literal + 16.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb6(0, 4)); // LDR R0, [PC, #16]
    // pc_literal = (BASE + 4) & ~2 = BASE + 4 (already word-aligned)
    const u32 pc_lit = (BASE + 4) & ~2u;
    const u32 target_addr = pc_lit + (4 << 2);
    nds.arm7_bus().write32(target_addr, 0xDEAD'BEEFu);
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xDEAD'BEEFu);
}

static void thumb6_ldr_pc_max_offset() {
    // imm8=0xFF, address = pc_literal + 0x3FC.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb6(3, 0xFF)); // LDR R3, [PC, #0x3FC]
    const u32 pc_lit = (BASE + 4) & ~2u;
    const u32 target_addr = pc_lit + 0x3FCu;
    nds.arm7_bus().write32(target_addr, 0xCAFE'BABEu);
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 0xCAFE'BABEu);
}

static void thumb6_ldr_pc_misaligned_instr() {
    // Instruction at BASE+2 (not word-aligned).
    // pc_literal = (BASE+2+4) & ~2 = BASE+4.
    const u32 addr = BASE + 2;
    NDS nds;
    nds.arm7_bus().write16(addr, thumb6(1, 2)); // LDR R1, [PC, #8]
    const u32 pc_lit = (addr + 4) & ~2u;
    REQUIRE(pc_lit == BASE + 4); // verify alignment
    const u32 target_addr = pc_lit + (2 << 2);
    nds.arm7_bus().write32(target_addr, 0x1234'5678u);
    setup_thumb(nds, addr);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0x1234'5678u);
}

static void thumb6_ldr_pc_zero_offset() {
    // imm8=0, loads from pc_literal directly.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb6(5, 0)); // LDR R5, [PC, #0]
    const u32 pc_lit = (BASE + 4) & ~2u;
    nds.arm7_bus().write32(pc_lit, 0xAAAA'BBBBu);
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 0xAAAA'BBBBu);
}

// ==== THUMB.7 — LDR/STR/LDRB/STRB register offset ====

static void thumb7_str_word() {
    // STR R0, [R1, R2]
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb7(0, 0, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xAABB'CCDDu;
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0x10;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read32(BASE + 0x110) == 0xAABB'CCDDu);
}

static void thumb7_ldr_word() {
    // LDR R3, [R4, R5]
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb7(1, 0, 5, 4, 3));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = BASE + 0x200;
    nds.cpu7().state().r[5] = 0x08;
    nds.arm7_bus().write32(BASE + 0x208, 0xDEAD'BEEFu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 0xDEAD'BEEFu);
}

static void thumb7_ldr_word_misaligned() {
    // LDR from misaligned address — should rotate.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb7(1, 0, 5, 4, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = BASE + 0x100;
    nds.cpu7().state().r[5] = 1; // addr = BASE + 0x101, misaligned by 1
    const u32 aligned = (BASE + 0x101) & ~3u;
    nds.arm7_bus().write32(aligned, 0x44332211u);
    step_one(nds);
    // rotate right by 8: 0x44332211 ROR 8 = 0x11443322
    REQUIRE(nds.cpu7().state().r[0] == 0x1144'3322u);
}

static void thumb7_strb() {
    // STRB R0, [R1, R2]
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb7(0, 1, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x12345678u;
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 5;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read8(BASE + 0x105) == 0x78u);
}

static void thumb7_ldrb() {
    // LDRB R6, [R0, R1]
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb7(1, 1, 1, 0, 6));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = BASE + 0x100;
    nds.cpu7().state().r[1] = 3;
    nds.arm7_bus().write8(BASE + 0x103, 0xABu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[6] == 0xABu);
}

static void thumb7_ldr_zero_offset() {
    // LDR R0, [R1, R2] with R2=0.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb7(1, 0, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0;
    nds.arm7_bus().write32(BASE + 0x100, 0xFEED'FACEu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xFEED'FACEu);
}

// ==== THUMB.9 — LDR/STR/LDRB/STRB immediate offset ====

static void thumb9_str_word() {
    // STR R0, [R1, #8] — b=0, l=0, imm5=2 (offset=2*4=8)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(0, 0, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xCAFE'BABEu;
    nds.cpu7().state().r[1] = BASE + 0x100;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read32(BASE + 0x108) == 0xCAFE'BABEu);
}

static void thumb9_ldr_word() {
    // LDR R2, [R3, #12] — b=0, l=1, imm5=3 (offset=3*4=12)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(0, 1, 3, 3, 2));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = BASE + 0x200;
    nds.arm7_bus().write32(BASE + 0x20C, 0xBAAD'F00Du);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[2] == 0xBAAD'F00Du);
}

static void thumb9_ldr_word_misaligned() {
    // LDR with base not word-aligned. imm5=0, base misaligned by 2.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(0, 1, 0, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x102; // misaligned by 2
    const u32 aligned = (BASE + 0x102) & ~3u;
    nds.arm7_bus().write32(aligned, 0x44332211u);
    step_one(nds);
    // rotate right by 16: 0x44332211 ROR 16 = 0x22114433
    REQUIRE(nds.cpu7().state().r[0] == 0x2211'4433u);
}

static void thumb9_strb() {
    // STRB R0, [R1, #5] — b=1, l=0, imm5=5 (offset=5)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(1, 0, 5, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xFFu;
    nds.cpu7().state().r[1] = BASE + 0x100;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read8(BASE + 0x105) == 0xFFu);
}

static void thumb9_ldrb() {
    // LDRB R4, [R5, #31] — b=1, l=1, imm5=31 (max byte offset)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(1, 1, 31, 5, 4));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[5] = BASE + 0x100;
    nds.arm7_bus().write8(BASE + 0x11F, 0x42u);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[4] == 0x42u);
}

static void thumb9_str_word_max_offset() {
    // STR R0, [R1, #124] — imm5=31 (offset=31*4=124)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(0, 0, 31, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x1111'2222u;
    nds.cpu7().state().r[1] = BASE + 0x100;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read32(BASE + 0x17C) == 0x1111'2222u);
}

static void thumb9_ldr_word_zero_offset() {
    // LDR R0, [R1, #0] — imm5=0
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb9(0, 1, 0, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.arm7_bus().write32(BASE + 0x100, 0xAAAA'BBBBu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xAAAA'BBBBu);
}

// ==== THUMB.8 — STRH/LDSB/LDRH/LDSH register offset ====

static void thumb8_strh() {
    // STRH R0, [R1, R2]
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(0, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xABCDu;
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0x10;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read16(BASE + 0x110) == 0xABCDu);
}

static void thumb8_strh_misaligned() {
    // STRH to odd address — bit 0 masked before write.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(0, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x1234u;
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 1; // addr = BASE + 0x101 (odd)
    step_one(nds);
    // Write goes to BASE + 0x100 (bit 0 masked)
    REQUIRE(nds.arm7_bus().read16(BASE + 0x100) == 0x1234u);
}

static void thumb8_ldrh() {
    // LDRH R3, [R4, R5]
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(2, 5, 4, 3));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = BASE + 0x200;
    nds.cpu7().state().r[5] = 0x08;
    nds.arm7_bus().write16(BASE + 0x208, 0xBEEFu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 0xBEEFu);
}

static void thumb8_ldrh_misaligned() {
    // LDRH from odd address — bit 0 masked, halfword zero-extended.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(2, 5, 4, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = BASE + 0x100;
    nds.cpu7().state().r[5] = 1; // addr = BASE + 0x101 (odd)
    nds.arm7_bus().write16(BASE + 0x100, 0xCAFEu);
    step_one(nds);
    // Reads from BASE + 0x100 (bit 0 masked), zero-extends.
    REQUIRE(nds.cpu7().state().r[0] == 0xCAFEu);
}

static void thumb8_ldsb() {
    // LDSB R0, [R1, R2] — load signed byte, sign-extend to 32 bits.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(1, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 3;
    nds.arm7_bus().write8(BASE + 0x103, 0x80u); // -128
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FF80u);
}

static void thumb8_ldsb_positive() {
    // LDSB with a positive byte (bit 7 clear).
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(1, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0;
    nds.arm7_bus().write8(BASE + 0x100, 0x7Fu); // +127
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0x7Fu);
}

static void thumb8_ldsh() {
    // LDSH R0, [R1, R2] — load signed halfword, sign-extend to 32 bits.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(3, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0;
    nds.arm7_bus().write16(BASE + 0x100, 0x8000u); // -32768
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'8000u);
}

static void thumb8_ldsh_positive() {
    // LDSH with a positive halfword (bit 15 clear).
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(3, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0;
    nds.arm7_bus().write16(BASE + 0x100, 0x7FFFu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0x7FFFu);
}

static void thumb8_ldrh_zero_offset() {
    // LDRH with Ro=0.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb8(2, 2, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.cpu7().state().r[2] = 0;
    nds.arm7_bus().write16(BASE + 0x100, 0xFACEu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xFACEu);
}

// ==== THUMB.10 — STRH/LDRH immediate offset ====

static void thumb10_strh() {
    // STRH R0, [R1, #6] — imm5=3 (offset=3*2=6)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb10(0, 3, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xDEADu;
    nds.cpu7().state().r[1] = BASE + 0x100;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read16(BASE + 0x106) == 0xDEADu);
}

static void thumb10_ldrh() {
    // LDRH R2, [R3, #4] — imm5=2 (offset=2*2=4)
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb10(1, 2, 3, 2));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = BASE + 0x200;
    nds.arm7_bus().write16(BASE + 0x204, 0xBEEFu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[2] == 0xBEEFu);
}

static void thumb10_ldrh_zero_offset() {
    // LDRH with imm5=0 (offset=0).
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb10(1, 0, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.arm7_bus().write16(BASE + 0x100, 0x1234u);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0x1234u);
}

static void thumb10_ldrh_max_offset() {
    // LDRH with imm5=31 (offset=31*2=62).
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb10(1, 31, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = BASE + 0x100;
    nds.arm7_bus().write16(BASE + 0x13E, 0xAAAAu);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xAAAAu);
}

static void thumb10_strh_zero_offset() {
    // STRH with imm5=0 (offset=0).
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb10(0, 0, 1, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x5678u;
    nds.cpu7().state().r[1] = BASE + 0x100;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read16(BASE + 0x100) == 0x5678u);
}

// ==== THUMB.11 — LDR/STR SP-relative ====

static void thumb11_str_sp_basic() {
    // STR R0, [SP, #4*4]. Write R0 to SP+16, verify memory.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb11(0, 0, 4)); // STR R0, [SP, #16]
    setup_thumb(nds, BASE);
    const u32 sp = BASE + 0x100;
    nds.cpu7().state().r[13] = sp;
    nds.cpu7().state().r[0] = 0xFEED'FACEu;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read32(sp + 16) == 0xFEED'FACEu);
}

static void thumb11_ldr_sp_basic() {
    // LDR R0, [SP, #4*4]. Read word from SP+16, verify Rd.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb11(1, 0, 4)); // LDR R0, [SP, #16]
    setup_thumb(nds, BASE);
    const u32 sp = BASE + 0x100;
    nds.cpu7().state().r[13] = sp;
    nds.arm7_bus().write32(sp + 16, 0xBAAD'F00Du);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0xBAAD'F00Du);
}

static void thumb11_ldr_sp_misaligned_rotate() {
    // SP misaligned by 1 byte. LDR should rotate the read result.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb11(1, 2, 0)); // LDR R2, [SP, #0]
    setup_thumb(nds, BASE);
    const u32 sp = BASE + 0x101; // misaligned by 1
    nds.cpu7().state().r[13] = sp;
    // Bus reads from sp & ~3 = BASE + 0x100
    const u32 aligned = sp & ~3u;
    nds.arm7_bus().write32(aligned, 0x44332211u);
    step_one(nds);
    // rotate_read_word: rotate right by (1 * 8) = 8 bits
    // 0x44332211 ROR 8 = 0x11443322
    REQUIRE(nds.cpu7().state().r[2] == 0x1144'3322u);
}

static void thumb11_str_sp_zero_offset() {
    // STR with imm8=0, address = SP.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb11(0, 7, 0)); // STR R7, [SP, #0]
    setup_thumb(nds, BASE);
    const u32 sp = BASE + 0x200;
    nds.cpu7().state().r[13] = sp;
    nds.cpu7().state().r[7] = 0x9876'5432u;
    step_one(nds);
    REQUIRE(nds.arm7_bus().read32(sp) == 0x9876'5432u);
}

static void thumb11_ldr_sp_max_offset() {
    // imm8=0xFF, address = SP + 0x3FC.
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb11(1, 6, 0xFF)); // LDR R6, [SP, #0x3FC]
    setup_thumb(nds, BASE);
    const u32 sp = BASE + 0x100;
    nds.cpu7().state().r[13] = sp;
    nds.arm7_bus().write32(sp + 0x3FCu, 0x1111'2222u);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[6] == 0x1111'2222u);
}

int main() {
    // THUMB.6
    thumb6_ldr_pc_basic();
    thumb6_ldr_pc_max_offset();
    thumb6_ldr_pc_misaligned_instr();
    thumb6_ldr_pc_zero_offset();

    // THUMB.7
    thumb7_str_word();
    thumb7_ldr_word();
    thumb7_ldr_word_misaligned();
    thumb7_strb();
    thumb7_ldrb();
    thumb7_ldr_zero_offset();

    // THUMB.8
    thumb8_strh();
    thumb8_strh_misaligned();
    thumb8_ldrh();
    thumb8_ldrh_misaligned();
    thumb8_ldsb();
    thumb8_ldsb_positive();
    thumb8_ldsh();
    thumb8_ldsh_positive();
    thumb8_ldrh_zero_offset();

    // THUMB.9
    thumb9_str_word();
    thumb9_ldr_word();
    thumb9_ldr_word_misaligned();
    thumb9_strb();
    thumb9_ldrb();
    thumb9_str_word_max_offset();
    thumb9_ldr_word_zero_offset();

    // THUMB.10
    thumb10_strh();
    thumb10_ldrh();
    thumb10_ldrh_zero_offset();
    thumb10_ldrh_max_offset();
    thumb10_strh_zero_offset();

    // THUMB.11
    thumb11_str_sp_basic();
    thumb11_ldr_sp_basic();
    thumb11_ldr_sp_misaligned_rotate();
    thumb11_str_sp_zero_offset();
    thumb11_ldr_sp_max_offset();

    return 0;
}
