// Exercises the KEYCNT per-CPU R/W path via the ARM9 and ARM7 bus
// dispatchers. KEYCNT is two independent cells: a write from ARM9 must not
// alter ARM7's cell and vice versa. Bits 10..13 are reserved and clear on
// write; bits 14 (enable) and 15 (mode AND) round-trip.

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static constexpr u32 kKeyinputAddr = 0x04000130u;
static constexpr u32 kKeycntAddr = 0x04000132u;
static constexpr u32 kKeycntHiAddr = 0x04000133u;
static constexpr u16 kResetKeyinput = 0x03FFu;
static constexpr u16 kKeycntMask = 0xC3FFu;

static void Reset_BothCellsZero() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0u);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0u);
    REQUIRE(nds.arm9_io_read32(kKeycntAddr) == 0u);
    REQUIRE(nds.arm7_io_read32(kKeycntAddr) == 0u);
    REQUIRE(nds.arm9_io_read8(kKeycntAddr) == 0u);
    REQUIRE(nds.arm9_io_read8(kKeycntHiAddr) == 0u);
    REQUIRE(nds.arm7_io_read8(kKeycntAddr) == 0u);
    REQUIRE(nds.arm7_io_read8(kKeycntHiAddr) == 0u);
}

static void Arm9Write_DoesNotAffectArm7() {
    NDS nds;
    nds.reset();

    nds.arm9_io_write16(kKeycntAddr, 0xC123u);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0xC123u);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0u);
}

static void Arm7Write_DoesNotAffectArm9() {
    NDS nds;
    nds.reset();

    nds.arm9_io_write16(kKeycntAddr, 0x4055u);
    nds.arm7_io_write16(kKeycntAddr, 0x800Au);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0x800Au);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x4055u);
}

static void ReservedBits_ClearOnWrite() {
    NDS nds;
    nds.reset();

    nds.arm9_io_write16(kKeycntAddr, 0xFFFFu);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == kKeycntMask);

    nds.arm7_io_write16(kKeycntAddr, 0xFFFFu);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == kKeycntMask);
}

static void EnableAndModeBits_RoundTrip() {
    NDS nds;
    nds.reset();

    // bit 14 only (enable, OR mode), bits 0..2 selected.
    nds.arm9_io_write16(kKeycntAddr, 0x4007u);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x4007u);
    REQUIRE((nds.arm9_io_read16(kKeycntAddr) & 0x4000u) != 0u);
    REQUIRE((nds.arm9_io_read16(kKeycntAddr) & 0x8000u) == 0u);
    REQUIRE((nds.arm9_io_read16(kKeycntAddr) & 0x03FFu) == 0x0007u);

    // bits 14 and 15 (enable, AND mode), all 10 button bits selected.
    nds.arm7_io_write16(kKeycntAddr, 0xC3FFu);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0xC3FFu);
    REQUIRE((nds.arm7_io_read16(kKeycntAddr) & 0x4000u) != 0u);
    REQUIRE((nds.arm7_io_read16(kKeycntAddr) & 0x8000u) != 0u);
    REQUIRE((nds.arm7_io_read16(kKeycntAddr) & 0x03FFu) == 0x03FFu);
}

static void ByteWrite_HighByte_PreservesLowByte() {
    NDS nds;
    nds.reset();

    nds.arm9_io_write16(kKeycntAddr, 0xC123u);
    nds.arm9_io_write8(kKeycntHiAddr, 0x80u);
    // (0xC123 & 0x00FF) | (0x80 << 8) = 0x8023; masked = 0x8023.
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x8023u);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0u);

    // High byte 0xFF: (0x8023 & 0x00FF) | (0xFF << 8) = 0xFF23; masked = 0xC323.
    nds.arm9_io_write8(kKeycntHiAddr, 0xFFu);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0xC323u);
}

static void ByteWrite_LowByte_PreservesHighByte() {
    NDS nds;
    nds.reset();

    nds.arm7_io_write16(kKeycntAddr, 0xC123u);
    nds.arm7_io_write8(kKeycntAddr, 0x55u);
    // (0xC123 & 0xFF00) | 0x55 = 0xC155; masked = 0xC155 & 0xC3FF = 0xC155.
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0xC155u);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0u);
}

static void Write32_At0130_TargetsKeycntHighHalf_DropsKeyinputLow() {
    NDS nds;
    nds.reset();

    nds.arm9_io_write32(kKeyinputAddr, 0x80AA1234u);
    // High halfword 0x80AA & 0xC3FF = 0x80AA → ARM9 KEYCNT.
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x80AAu);
    // KEYINPUT is read-only and shared; reset value preserved.
    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == kResetKeyinput);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == kResetKeyinput);
    // ARM7 KEYCNT unaffected.
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0u);

    nds.arm7_io_write32(kKeyinputAddr, 0xDEAD8005u);
    // High halfword 0xDEAD & 0xC3FF = 0xC2AD → ARM7 KEYCNT.
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0xC2ADu);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x80AAu);
    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == kResetKeyinput);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == kResetKeyinput);
}

static void Write32_At0132_TargetsKeycntLowHalf_DropsUnmappedHigh() {
    NDS nds;
    nds.reset();

    // Low halfword 0xBEEF & 0xC3FF = 0x82EF → ARM9 KEYCNT.
    // High halfword 0xDEAD lands at 0x4000134 which is unmapped; silently dropped.
    nds.arm9_io_write32(kKeycntAddr, 0xDEADBEEFu);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x82EFu);
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0u);

    nds.arm7_io_write32(kKeycntAddr, 0xCAFEFACEu);
    // 0xFACE & 0xC3FF = 0xC2CE.
    REQUIRE(nds.arm7_io_read16(kKeycntAddr) == 0xC2CEu);
    REQUIRE(nds.arm9_io_read16(kKeycntAddr) == 0x82EFu);
}

int main() {
    Reset_BothCellsZero();
    Arm9Write_DoesNotAffectArm7();
    Arm7Write_DoesNotAffectArm9();
    ReservedBits_ClearOnWrite();
    EnableAndModeBits_RoundTrip();
    ByteWrite_HighByte_PreservesLowByte();
    ByteWrite_LowByte_PreservesHighByte();
    Write32_At0130_TargetsKeycntHighHalf_DropsKeyinputLow();
    Write32_At0132_TargetsKeycntLowHalf_DropsUnmappedHigh();
    return 0;
}
