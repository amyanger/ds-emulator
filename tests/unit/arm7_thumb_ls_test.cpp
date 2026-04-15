// arm7_thumb_ls_test.cpp — THUMB.6 LDR PC-relative + THUMB.11 LDR/STR SP-relative.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

// THUMB.6: 01001 Rd3 imm8
static u16 thumb6(u32 rd, u32 imm8) {
    return static_cast<u16>((0b01001u << 11) | (rd << 8) | imm8);
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

    // THUMB.11
    thumb11_str_sp_basic();
    thumb11_ldr_sp_basic();
    thumb11_ldr_sp_misaligned_rotate();
    thumb11_str_sp_zero_offset();
    thumb11_ldr_sp_max_offset();

    return 0;
}
