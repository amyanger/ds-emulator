// ARM7 slice 3b1: LDR / STR / LDRB / STRB tests. Each test drops a
// single encoded instruction into ARM7 WRAM at kBase, seeds the state
// it needs, and runs one step. Later tasks extend this file one
// addressing-mode dimension at a time.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;
// Data scratch area: far enough from the code block that stores into
// it cannot clobber the program.
constexpr u32 kData = 0x0380'0200u;

void run_one(NDS& nds, u32 instr) {
    nds.arm7_bus().write32(kBase, instr);
    nds.cpu7().state().pc = kBase;
    nds.cpu7().run_until(2);  // 1 ARM7 cycle = 2 ARM9 cycles
}

}  // namespace

static void ldr_word_imm_offset_loads_value() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 4, 0xCAFE'BABEu);

    // LDR R0, [R1, #4]
    // cond=AL, 01, I=0, P=1, U=1, B=0, W=0, L=1, Rn=1, Rd=0, imm12=4
    // -> 1110 0101 1001 0001 0000 0000 0000 0100 = 0xE591'0004
    run_one(nds, 0xE591'0004u);

    REQUIRE(nds.cpu7().state().r[0] == 0xCAFE'BABEu);
    REQUIRE(nds.cpu7().state().r[1] == kData);  // no writeback
}

static void str_word_imm_offset_stores_value() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x1234'5678u;
    nds.cpu7().state().r[1] = kData;

    // STR R0, [R1, #8]
    // -> 1110 0101 1000 0001 0000 0000 0000 1000 = 0xE581'0008
    run_one(nds, 0xE581'0008u);

    REQUIRE(nds.arm7_bus().read32(kData + 8) == 0x1234'5678u);
    REQUIRE(nds.cpu7().state().r[0] == 0x1234'5678u);
    REQUIRE(nds.cpu7().state().r[1] == kData);
}

static void ldrb_loads_byte_zero_extended() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData, 0x1122'33FFu);  // byte 0 = 0xFF

    // LDRB R0, [R1, #0]
    // cond=AL, 01, I=0, P=1, U=1, B=1, W=0, L=1, Rn=1, Rd=0, imm12=0
    // -> 1110 0101 1101 0001 0000 0000 0000 0000 = 0xE5D1'0000
    run_one(nds, 0xE5D1'0000u);

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'00FFu);  // zero-extended
}

static void strb_stores_only_target_byte() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xDEAD'BE42u;     // low byte = 0x42
    nds.cpu7().state().r[1] = kData;
    // Seed the whole word with a known pattern so we can verify the
    // neighboring bytes are untouched.
    nds.arm7_bus().write32(kData, 0xAABB'CCDDu);

    // STRB R0, [R1, #1]
    // -> 1110 0101 1100 0001 0000 0000 0000 0001 = 0xE5C1'0001
    run_one(nds, 0xE5C1'0001u);

    // Only byte 1 should change to 0x42. Little-endian layout:
    //   before:  AA BB CC DD
    //   after:   AA BB 42 DD
    const u32 expected = (0xAABB'CCDDu & ~(0xFFu << 8)) | (0x42u << 8);
    REQUIRE(nds.arm7_bus().read32(kData) == expected);
}

int main() {
    ldr_word_imm_offset_loads_value();
    str_word_imm_offset_stores_value();
    ldrb_loads_byte_zero_extended();
    strb_stores_only_target_byte();
    std::puts("arm7_load_store_test OK");
    return 0;
}
