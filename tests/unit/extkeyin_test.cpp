// Exercises the EXTKEYIN (0x4000136) read path. EXTKEYIN is NDS7-only: ARM7
// reads return the register, ARM9 reads fall through to open-bus zero. Only
// bit 7 (hinge) is host-writable this slice, via set_lid_closed(); X/Y/DEBUG/
// pen bits stay released until their subsystems land. The register is R/O —
// bus writes are silently dropped. The IF.22 raise is commit 6, not here.

#include "input/lid_switch.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static constexpr u32 kExtkeyinAddr = 0x04000136u;
static constexpr u32 kExtkeyinHiAddr = 0x04000137u;
static constexpr u16 kResetExtkeyin = 0x007Fu;
static constexpr u16 kClosedExtkeyin = 0x00FFu; // reset value with hinge bit 7 set

static void Extkeyin_Reset_Is007F() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.lid_switch().read_extkeyin() == kResetExtkeyin);
}

static void Extkeyin_Arm7Bus_HalfwordReadReturnsRegister() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.arm7_io_read16(kExtkeyinAddr) == kResetExtkeyin);
}

// EXTKEYIN does not exist on the ARM9 view; the bus has no route, so reads of
// every width fall through to the open-bus zero default.
static void Extkeyin_Arm9Bus_OpenBusReturnsZero() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.arm9_io_read8(kExtkeyinAddr) == 0u);
    REQUIRE(nds.arm9_io_read8(kExtkeyinHiAddr) == 0u);
    REQUIRE(nds.arm9_io_read16(kExtkeyinAddr) == 0u);
    REQUIRE(nds.arm9_io_read32(kExtkeyinAddr) == 0u);
}

static void Extkeyin_Arm7ByteReads_HighLowSplit() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.arm7_io_read8(kExtkeyinAddr) == 0x7Fu);
    REQUIRE(nds.arm7_io_read8(kExtkeyinHiAddr) == 0x00u);
}

// The 32-bit ARM7 read dispatches through a separate path from the halfword
// read and must zero-extend the 16-bit register into the high halfword.
static void Extkeyin_Arm7Bus_WordRead_ZeroExtends() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.arm7_io_read32(kExtkeyinAddr) == static_cast<u32>(kResetExtkeyin));
    nds.set_lid_closed(true);
    REQUIRE(nds.arm7_io_read32(kExtkeyinAddr) == static_cast<u32>(kClosedExtkeyin));
}

static void Extkeyin_Bit7AfterClose_ReadsAs0x00FF() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    REQUIRE(nds.arm7_io_read16(kExtkeyinAddr) == kClosedExtkeyin);
}

static void Extkeyin_Bit7AfterReopen_ReadsAs0x007F() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE(nds.arm7_io_read16(kExtkeyinAddr) == kResetExtkeyin);
}

// Pen-down (bit 6) has no host write path this slice; it stays released (1)
// regardless of lid motion. Locks the §3 touchscreen non-goal.
static void Extkeyin_Bit6_StaysReleased_NoTouchSupportYet() {
    NDS nds;
    nds.reset();

    constexpr u16 kPenBit = 1u << 6;
    REQUIRE((nds.arm7_io_read16(kExtkeyinAddr) & kPenBit) != 0u);
    nds.set_lid_closed(true);
    REQUIRE((nds.arm7_io_read16(kExtkeyinAddr) & kPenBit) != 0u);
    nds.set_lid_closed(false);
    REQUIRE((nds.arm7_io_read16(kExtkeyinAddr) & kPenBit) != 0u);
}

// X (bit 0), Y (bit 1), DEBUG (bit 3) and the GBATEK "unknown / set" bits
// 2, 4, 5 all read as 1 and never change — only the hinge bit moves.
static void Extkeyin_Bits01345_StayReleased_NoXYDebugYet() {
    NDS nds;
    nds.reset();

    constexpr u16 kLowSix = 0x003Fu; // bits 0..5
    REQUIRE((nds.arm7_io_read16(kExtkeyinAddr) & kLowSix) == kLowSix);
    nds.set_lid_closed(true);
    REQUIRE((nds.arm7_io_read16(kExtkeyinAddr) & kLowSix) == kLowSix);
    nds.set_lid_closed(false);
    REQUIRE((nds.arm7_io_read16(kExtkeyinAddr) & kLowSix) == kLowSix);
}

// EXTKEYIN is read-only; the bus has no write route for 0x4000136, so writes
// of every width fall through and never disturb the backing register.
static void Extkeyin_Arm7BusWrite_SilentlyDropped() {
    NDS nds;
    nds.reset();

    nds.arm7_io_write8(kExtkeyinAddr, 0x00u);
    nds.arm7_io_write16(kExtkeyinAddr, 0x0000u);
    nds.arm7_io_write32(kExtkeyinAddr, 0x00000000u);
    REQUIRE(nds.lid_switch().read_extkeyin() == kResetExtkeyin);
    REQUIRE(nds.arm7_io_read16(kExtkeyinAddr) == kResetExtkeyin);
}

int main() {
    Extkeyin_Reset_Is007F();
    Extkeyin_Arm7Bus_HalfwordReadReturnsRegister();
    Extkeyin_Arm9Bus_OpenBusReturnsZero();
    Extkeyin_Arm7ByteReads_HighLowSplit();
    Extkeyin_Arm7Bus_WordRead_ZeroExtends();
    Extkeyin_Bit7AfterClose_ReadsAs0x00FF();
    Extkeyin_Bit7AfterReopen_ReadsAs0x007F();
    Extkeyin_Bit6_StaysReleased_NoTouchSupportYet();
    Extkeyin_Bits01345_StayReleased_NoXYDebugYet();
    Extkeyin_Arm7BusWrite_SilentlyDropped();
    return 0;
}
