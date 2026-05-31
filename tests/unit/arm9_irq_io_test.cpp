// Exercises the ARM9-side IrqController storage and IO routing for IME/IE/IF.
// Covers the bus-side round-trip plus the live IRQ line pushed into cpu9_ by
// update_arm9_irq_signals().

#include "bus/io_regs.hpp"
#include "interrupt/irq_controller.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Case 1: ARM9 IME round-trips through the bus, and ARM7 IME is untouched
// (cross-CPU isolation: each side owns its own IrqController).
static void arm9_io_ime_writes_and_reads_back() {
    NDS nds;

    nds.arm9_io_write32(IO_IME, 1u);
    REQUIRE(nds.arm9_io_read32(IO_IME) == 1u);
    REQUIRE(nds.arm7_io_read32(IO_IME) == 0u);
}

// Case 2: 16-bit halfword writes to IO_IE and IO_IE+2 assemble into a 32-bit
// IE correctly on the ARM9 side. Mirrors the existing ARM7 halfword test.
static void arm9_io_ie_halfword_split() {
    NDS nds;

    nds.arm9_io_write16(IO_IE, 0x1234u);
    nds.arm9_io_write16(IO_IE + 2u, 0xABCDu);
    REQUIRE(nds.arm9_io_read32(IO_IE) == 0xABCD1234u);
}

// Case 3: write-1-clear applies to ARM9 IF; bits written as 1 are cleared,
// other bits stay set, and ARM7 IF is untouched.
static void arm9_io_if_write_one_clear() {
    NDS nds;

    nds.irq9().raise(0x7u);
    REQUIRE(nds.arm9_io_read32(IO_IF) == 0x7u);

    nds.arm9_io_write32(IO_IF, 0x2u); // clear bit 1 only
    REQUIRE(nds.arm9_io_read32(IO_IF) == 0x5u);
    REQUIRE(nds.arm7_io_read32(IO_IF) == 0u);
}

// Case 4: 8-bit IE byte writes merge into the correct byte of the 32-bit
// register without disturbing the other bytes.
static void arm9_io_ie_byte_merge() {
    NDS nds;

    nds.arm9_io_write32(IO_IE, 0xAABBCCDDu);
    nds.arm9_io_write8(IO_IE + 1u, 0x11u);
    REQUIRE(nds.arm9_io_read32(IO_IE) == 0xAABB11DDu);

    nds.arm9_io_write8(IO_IE + 3u, 0x22u);
    REQUIRE(nds.arm9_io_read32(IO_IE) == 0x22BB11DDu);
}

// Case 5: 16-bit IF halfword writes are write-1-clear over the targeted half.
static void arm9_io_if_halfword_write_one_clear() {
    NDS nds;

    nds.irq9().raise(0x000F'00FFu);
    nds.arm9_io_write16(IO_IF, 0x00FFu); // clear low halfword bits
    REQUIRE(nds.arm9_io_read32(IO_IF) == 0x000F'0000u);

    nds.arm9_io_write16(IO_IF + 2u, 0x0003u); // clear bits 16,17 only
    REQUIRE(nds.arm9_io_read32(IO_IF) == 0x000C'0000u);
}

// Case 6: writes to the reserved bytes of IME (IO_IME+1..+3) are no-ops on
// hardware; verify state is unchanged after writing nonzero values to them.
static void arm9_io_ime_reserved_bytes_are_no_ops() {
    NDS nds;

    nds.arm9_io_write32(IO_IME, 1u);
    nds.arm9_io_write8(IO_IME + 1u, 0xFFu);
    nds.arm9_io_write8(IO_IME + 2u, 0xFFu);
    nds.arm9_io_write8(IO_IME + 3u, 0xFFu);
    REQUIRE(nds.arm9_io_read32(IO_IME) == 1u);
}

// Case 7: every IME/IE/IF write path calls update_arm9_irq_signals(), which
// pushes the live line into cpu9_. Set up state where the line is true and
// confirm the IF write path drives cpu9_'s IRQ line.
static void arm9_irq_line_pushed_on_writes() {
    NDS nds;

    nds.arm9_io_write32(IO_IME, 1u);
    nds.arm9_io_write32(IO_IE, 1u);
    nds.irq9().raise(1u);
    nds.arm9_io_write32(IO_IF, 0u);
    REQUIRE(nds.cpu9().irq_line() == true);
}

int main() {
    arm9_io_ime_writes_and_reads_back();
    std::puts("[PASS] arm9_io_ime_writes_and_reads_back");
    arm9_io_ie_halfword_split();
    std::puts("[PASS] arm9_io_ie_halfword_split");
    arm9_io_if_write_one_clear();
    std::puts("[PASS] arm9_io_if_write_one_clear");
    arm9_io_ie_byte_merge();
    std::puts("[PASS] arm9_io_ie_byte_merge");
    arm9_io_if_halfword_write_one_clear();
    std::puts("[PASS] arm9_io_if_halfword_write_one_clear");
    arm9_io_ime_reserved_bytes_are_no_ops();
    std::puts("[PASS] arm9_io_ime_reserved_bytes_are_no_ops");
    arm9_irq_line_pushed_on_writes();
    std::puts("[PASS] arm9_irq_line_pushed_on_writes");
    std::puts("arm9_irq_io_test: all 7 cases passed");
    return 0;
}
