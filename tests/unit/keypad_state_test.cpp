// Exercises the KEYINPUT shared-state read path via the ARM9 and ARM7 bus
// dispatchers. KEYINPUT is one physical cell shared by both CPUs; KEYCNT is
// two independent cells. Active-LOW: a set bit = button released.

#include "input/keypad.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static constexpr u32 kKeyinputAddr = 0x04000130u;
static constexpr u32 kKeyinputHiAddr = 0x04000131u;
static constexpr u16 kResetKeyinput = 0x03FFu;

static void Reset_ReadsReturnPostFirmwareValue() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == kResetKeyinput);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == kResetKeyinput);

    REQUIRE(nds.arm9_io_read8(kKeyinputAddr) == 0xFFu);
    REQUIRE(nds.arm9_io_read8(kKeyinputHiAddr) == 0x03u);
    REQUIRE(nds.arm7_io_read8(kKeyinputAddr) == 0xFFu);
    REQUIRE(nds.arm7_io_read8(kKeyinputHiAddr) == 0x03u);

    REQUIRE(nds.arm9_io_read32(kKeyinputAddr) == 0x000003FFu);
    REQUIRE(nds.arm7_io_read32(kKeyinputAddr) == 0x000003FFu);
}

static void SetKeypadState_BusMirrorsBothCpus() {
    NDS nds;
    nds.reset();

    nds.set_keypad_state(0u);
    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == 0u);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == 0u);

    nds.set_keypad_state(kResetKeyinput);
    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == kResetKeyinput);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == kResetKeyinput);
}

static void SetKeypadState_HighBitsMasked() {
    NDS nds;
    nds.reset();

    nds.set_keypad_state(0xFFFFu);
    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == kResetKeyinput);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == kResetKeyinput);
}

static void ByteReads_SliceLowAndHighHalves() {
    NDS nds;
    nds.reset();

    nds.set_keypad_state(0x02ABu);
    REQUIRE(nds.arm9_io_read8(kKeyinputAddr) == 0xABu);
    REQUIRE(nds.arm9_io_read8(kKeyinputHiAddr) == 0x02u);
    REQUIRE(nds.arm7_io_read8(kKeyinputAddr) == 0xABu);
    REQUIRE(nds.arm7_io_read8(kKeyinputHiAddr) == 0x02u);
}

// 32-bit reads at 0x04000130 fold the issuing CPU's own KEYCNT cell into the
// high halfword. The two cells are independent: ARM9's cell pairs with the
// ARM9 read, ARM7's cell pairs with the ARM7 read.
static void Read32_PairsKeyinputWithPerCpuKeycnt() {
    NDS nds;
    nds.reset();

    constexpr u16 kArm9Cnt = 0x40A5u;
    constexpr u16 kArm7Cnt = 0xC11Au;
    nds.keypad().write_keycnt(Keypad::Side::Arm9, kArm9Cnt, nds.irq9(), nds.irq7());
    nds.keypad().write_keycnt(Keypad::Side::Arm7, kArm7Cnt, nds.irq9(), nds.irq7());

    nds.set_keypad_state(0x0155u);

    const u16 cnt9 = nds.keypad().read_keycnt(Keypad::Side::Arm9);
    const u16 cnt7 = nds.keypad().read_keycnt(Keypad::Side::Arm7);
    REQUIRE(nds.arm9_io_read32(kKeyinputAddr) == ((static_cast<u32>(cnt9) << 16) | 0x0155u));
    REQUIRE(nds.arm7_io_read32(kKeyinputAddr) == ((static_cast<u32>(cnt7) << 16) | 0x0155u));
    REQUIRE(cnt9 != cnt7);
}

// KEYINPUT is read-only on hardware. The bus has no write route for 0x4000130
// — writes must not alter the shared keyinput cell.
static void Writes_ToKeyinputAreSilentlyDropped() {
    NDS nds;
    nds.reset();

    nds.set_keypad_state(0x01F5u);
    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == 0x01F5u);

    nds.arm9_io_write16(kKeyinputAddr, 0xDEADu);
    nds.arm7_io_write16(kKeyinputAddr, 0xBEEFu);
    nds.arm9_io_write32(kKeyinputAddr, 0xCAFEBABEu);
    nds.arm7_io_write32(kKeyinputAddr, 0x12345678u);
    nds.arm9_io_write8(kKeyinputAddr, 0x00u);
    nds.arm7_io_write8(kKeyinputHiAddr, 0x00u);

    REQUIRE(nds.arm9_io_read16(kKeyinputAddr) == 0x01F5u);
    REQUIRE(nds.arm7_io_read16(kKeyinputAddr) == 0x01F5u);
}

int main() {
    Reset_ReadsReturnPostFirmwareValue();
    SetKeypadState_BusMirrorsBothCpus();
    SetKeypadState_HighBitsMasked();
    ByteReads_SliceLowAndHighHalves();
    Read32_PairsKeyinputWithPerCpuKeycnt();
    Writes_ToKeyinputAreSilentlyDropped();
    return 0;
}
