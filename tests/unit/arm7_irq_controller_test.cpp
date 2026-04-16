// arm7_irq_controller_test.cpp — exercises the Arm7IrqController state
// machine (IME/IE/IF + line()) and the ARM7 I/O routing that lives in NDS.
// Slice 3d commit 3. No IRQ sampling yet — that arrives in commit 10. This
// test only proves the plumbing: writes land, write-1-clear on IF behaves,
// raise() sets bits, and NDS pushes line() into cpu7_'s irq_line_ field.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "interrupt/irq_controller.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Case 1: with IME=0 the line must never assert, no matter what IE/IF say.
static void ime_disabled_means_no_line() {
    Arm7IrqController ctrl;
    ctrl.reset();
    ctrl.write_ie(0xFFu);
    ctrl.raise(0x01u); // IF = 1
    REQUIRE(ctrl.line() == false);
}

// Case 2: with IME=1, a bit set in both IE and IF asserts the line.
static void ime_enabled_with_matching_ie_if_bit_asserts_line() {
    Arm7IrqController ctrl;
    ctrl.reset();
    ctrl.write_ime(1u);
    ctrl.write_ie(0x1u);
    ctrl.raise(0x1u);
    REQUIRE(ctrl.line() == true);
}

// Case 3: write_if is write-1-clear. Starting IF = 0b111, writing 0b010
// should clear bit 1 only, leaving IF = 0b101.
static void write_1_clears_if_bit() {
    Arm7IrqController ctrl;
    ctrl.reset();
    ctrl.write_ime(1u);
    ctrl.write_ie(0x7u);
    ctrl.raise(0x7u);
    REQUIRE(ctrl.read_if() == 0x7u);
    REQUIRE(ctrl.line() == true);

    ctrl.write_if(0x2u); // clear bit 1 only
    REQUIRE(ctrl.read_if() == 0x5u);
    REQUIRE(ctrl.line() == true); // bits 0 and 2 still pending
}

// Case 4: raise() ORs source bits into IF without any auto-clear.
static void raise_sets_if_bit_without_auto_clear() {
    Arm7IrqController ctrl;
    ctrl.reset();
    REQUIRE(ctrl.read_if() == 0u);
    ctrl.raise(0x10u);
    REQUIRE(ctrl.read_if() == 0x10u);
    ctrl.raise(0x01u);
    REQUIRE(ctrl.read_if() == 0x11u); // OR-in, does not clear prior bits
}

// Case 5: NDS I/O routing for IME/IE (word width). IF is write-1-clear, so
// we can't set bits in it via a bus write — we seed it via raise() on the
// controller directly (the path future IRQ-source subsystems will take),
// then force a line-update by re-writing IME/IE. After the bus writes,
// cpu7_.irq_line() must reflect the asserted line.
static void bus_routing_via_nds_io_write32() {
    NDS nds;

    nds.irq7().raise(0x01u); // bypasses line update — the write below fixes that
    nds.arm7_io_write32(IO_IE, 0xFFu);
    nds.arm7_io_write32(IO_IME, 0x1u);

    REQUIRE(nds.arm7_io_read32(IO_IME) == 1u);
    REQUIRE(nds.arm7_io_read32(IO_IE) == 0xFFu);
    REQUIRE(nds.arm7_io_read32(IO_IF) == 0x01u);
    REQUIRE(nds.cpu7().irq_line() == true);
}

// Case 6: write-1-clear through the bus. After setting up an asserted line,
// writing 0x01 to IF via the bus must clear bit 0 and drop the line.
// Note: write_if is write-1-clear, so the "set IF" step uses the controller
// accessor (raise) rather than a bus write — bus writes to IF only clear.
static void bus_routing_write_1_clear_if() {
    NDS nds;

    nds.arm7_io_write32(IO_IME, 0x1u);
    nds.arm7_io_write32(IO_IE, 0x01u);
    nds.irq7().raise(0x01u);
    // raise() bypasses the NDS glue; poke the line update explicitly. The
    // real subsystems that will eventually call raise() will go through an
    // NDS helper that also pushes the line — that helper doesn't exist yet.
    nds.arm7_io_write32(IO_IF, 0x00u); // no-op write (nothing to clear) but
                                       // re-runs update_arm7_irq_line()
    REQUIRE(nds.arm7_io_read32(IO_IF) == 0x01u);
    REQUIRE(nds.cpu7().irq_line() == true);

    // Write 1 to bit 0 — clears it. Line drops because (IE & IF) becomes 0.
    nds.arm7_io_write32(IO_IF, 0x01u);
    REQUIRE(nds.arm7_io_read32(IO_IF) == 0x00u);
    REQUIRE(nds.cpu7().irq_line() == false);
}

// Case 7: 16-bit access splits the 32-bit register into low/high halfword
// windows. Writing 0x1234 to IO_IE and 0xABCD to IO_IE+2 should yield a
// 32-bit IE = 0xABCD1234.
static void halfword_access_splits_low_high() {
    NDS nds;

    nds.arm7_io_write16(IO_IE, 0x1234u);
    nds.arm7_io_write16(IO_IE + 2u, 0xABCDu);
    REQUIRE(nds.arm7_io_read32(IO_IE) == 0xABCD1234u);
    REQUIRE(nds.arm7_io_read16(IO_IE) == 0x1234u);
    REQUIRE(nds.arm7_io_read16(IO_IE + 2u) == 0xABCDu);
}

// Case 8: a halfword write-1-clear to the LOW half of IF only clears bits
// within that halfword — the upper 16 bits are untouched. Start with IF =
// 0xDEADBEEF, write 0x000F as a halfword to IO_IF (low), expect low nibble
// cleared: 0xDEADBEEF & ~0x0000000F = 0xDEADBEE0.
static void halfword_if_partial_write_clear() {
    NDS nds;
    nds.irq7().raise(0xDEADBEEFu);
    REQUIRE(nds.arm7_io_read32(IO_IF) == 0xDEADBEEFu);

    nds.arm7_io_write16(IO_IF, 0x000Fu); // write-1-clear low halfword bits
    REQUIRE(nds.arm7_io_read32(IO_IF) == 0xDEADBEE0u);
}

int main() {
    ime_disabled_means_no_line();
    ime_enabled_with_matching_ie_if_bit_asserts_line();
    write_1_clears_if_bit();
    raise_sets_if_bit_without_auto_clear();
    bus_routing_via_nds_io_write32();
    bus_routing_write_1_clear_if();
    halfword_access_splits_low_high();
    halfword_if_partial_write_clear();
    std::puts("arm7_irq_controller_test: all 8 cases passed");
    return 0;
}
