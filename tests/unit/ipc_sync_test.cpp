// Exercises the IpcSync register at 0x4000180 across both bus views.

#include "bus/io_regs.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Case 1: after reset both sides read 0 — no leftover out_lo or irq_enable.
static void ipc_sync_reset_all_zero() {
    NDS nds;

    REQUIRE(nds.arm9_io_read16(IO_IPCSYNC) == 0u);
    REQUIRE(nds.arm7_io_read16(IO_IPCSYNC) == 0u);
}

// Case 2: ARM9 writes its out_lo; ARM7 sees it in bits 0-3 of its own read.
// ARM9's own read still reflects ARM9's out_lo in bits 8-11.
static void ipc_sync_arm9_out_to_arm7_in() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0x0500u); // out_lo = 5
    REQUIRE((nds.arm7_io_read16(IO_IPCSYNC) & 0xFu) == 0x5u);
    REQUIRE((nds.arm9_io_read16(IO_IPCSYNC) & 0x0F00u) == 0x0500u);
    REQUIRE((nds.arm9_io_read16(IO_IPCSYNC) & 0xFu) == 0u); // ARM7 still 0
}

// Case 3: symmetric — ARM7 writes its out_lo; ARM9 reads it in bits 0-3.
static void ipc_sync_arm7_out_to_arm9_in() {
    NDS nds;

    nds.arm7_io_write16(IO_IPCSYNC, 0x0A00u); // out_lo = A
    REQUIRE((nds.arm9_io_read16(IO_IPCSYNC) & 0xFu) == 0xAu);
    REQUIRE((nds.arm7_io_read16(IO_IPCSYNC) & 0x0F00u) == 0x0A00u);
    REQUIRE((nds.arm7_io_read16(IO_IPCSYNC) & 0xFu) == 0u);
}

// Case 4: bit 14 (irq_enable) is R/W per side and does not bleed across.
static void ipc_sync_irq_enable_per_side() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0x4000u); // bit 14 = 1
    REQUIRE((nds.arm9_io_read16(IO_IPCSYNC) & 0x4000u) == 0x4000u);
    REQUIRE((nds.arm7_io_read16(IO_IPCSYNC) & 0x4000u) == 0u);
}

// Case 5: bit 13 (send-IRQ-to-remote) is write-only one-shot — never persists
// in any side's read.
static void ipc_sync_bit13_does_not_persist() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0x2000u); // bit 13 = 1
    REQUIRE((nds.arm9_io_read16(IO_IPCSYNC) & 0x2000u) == 0u);
    REQUIRE((nds.arm7_io_read16(IO_IPCSYNC) & 0x2000u) == 0u);
}

// Case 6: reserved bits (4-7, 12, 15) read as zero regardless of write value.
static void ipc_sync_reserved_bits_read_zero() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0xFFFFu);
    const u16 v = nds.arm9_io_read16(IO_IPCSYNC);
    REQUIRE((v & 0x00F0u) == 0u); // bits 4-7
    REQUIRE((v & 0x1000u) == 0u); // bit 12
    REQUIRE((v & 0x2000u) == 0u); // bit 13 (one-shot)
    REQUIRE((v & 0x8000u) == 0u); // bit 15
}

// Case 7: 32-bit access zero-extends the low halfword on read and narrows on
// write — IPCSYNC is natively a 16-bit register.
static void ipc_sync_32bit_access_zero_extends() {
    NDS nds;

    nds.arm9_io_write32(IO_IPCSYNC, 0xCAFE'0700u); // out_lo = 7, upper ignored
    REQUIRE(nds.arm9_io_read32(IO_IPCSYNC) == 0x0000'0700u);
    REQUIRE((nds.arm7_io_read16(IO_IPCSYNC) & 0xFu) == 0x7u);
}

// Case 8: 8-bit byte-window read/write covers byte 0 (read-only on this side)
// and byte 1 (out_lo + irq_enable). Byte writes RMW through the 16-bit path.
static void ipc_sync_byte_access() {
    NDS nds;

    // Write byte 1 = 0x43 → out_lo = 3, bit 14 = 1.
    nds.arm9_io_write8(IO_IPCSYNC + 1u, 0x43u);
    REQUIRE(nds.arm9_io_read8(IO_IPCSYNC + 1u) == 0x43u);
    REQUIRE(nds.arm9_io_read8(IO_IPCSYNC) == 0u); // remote in still 0
    REQUIRE((nds.arm7_io_read8(IO_IPCSYNC) & 0xFu) == 0x3u);

    // Byte 0 writes do not change state (bits 0-3 are R-only of remote.out_lo,
    // bits 4-7 are reserved).
    nds.arm9_io_write8(IO_IPCSYNC, 0xFFu);
    REQUIRE(nds.arm9_io_read8(IO_IPCSYNC + 1u) == 0x43u);
}

// Case 9: ARM9 sets bit 13 with ARM7's irq_enable on → ARM7 IF.16 raised.
static void ipc_sync_bit13_arm9_to_arm7_raises_when_enabled() {
    NDS nds;

    nds.arm7_io_write16(IO_IPCSYNC, 0x4000u); // ARM7 enables receive
    nds.arm9_io_write16(IO_IPCSYNC, 0x2000u); // ARM9 fires bit 13
    REQUIRE((nds.irq7().read_if() & (1u << 16)) != 0u);
    REQUIRE((nds.irq9().read_if() & (1u << 16)) == 0u);
}

// Case 10: bit 13 is silently dropped if the receiver hasn't enabled.
static void ipc_sync_bit13_no_raise_when_remote_disabled() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0x2000u);
    REQUIRE(nds.irq7().read_if() == 0u);
    REQUIRE(nds.irq9().read_if() == 0u);
}

// Case 11: bit 13 never raises the *sender's* IF, regardless of the sender's
// own bit-14 state. The receiver's enable is what gates.
static void ipc_sync_bit13_never_raises_local_irq() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0x4000u); // ARM9 enables its own bit 14
    nds.arm9_io_write16(IO_IPCSYNC, 0x6000u); // bit 14 + bit 13
    REQUIRE((nds.irq9().read_if() & (1u << 16)) == 0u);
    REQUIRE((nds.irq7().read_if() & (1u << 16)) == 0u); // ARM7 didn't enable
}

// Case 12: symmetric — ARM7 sets bit 13 with ARM9's irq_enable on → ARM9 IF.16.
static void ipc_sync_bit13_arm7_to_arm9_raises_when_enabled() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCSYNC, 0x4000u);
    nds.arm7_io_write16(IO_IPCSYNC, 0x2000u);
    REQUIRE((nds.irq9().read_if() & (1u << 16)) != 0u);
    REQUIRE((nds.irq7().read_if() & (1u << 16)) == 0u);
}

int main() {
    ipc_sync_reset_all_zero();
    std::puts("[PASS] ipc_sync_reset_all_zero");
    ipc_sync_arm9_out_to_arm7_in();
    std::puts("[PASS] ipc_sync_arm9_out_to_arm7_in");
    ipc_sync_arm7_out_to_arm9_in();
    std::puts("[PASS] ipc_sync_arm7_out_to_arm9_in");
    ipc_sync_irq_enable_per_side();
    std::puts("[PASS] ipc_sync_irq_enable_per_side");
    ipc_sync_bit13_does_not_persist();
    std::puts("[PASS] ipc_sync_bit13_does_not_persist");
    ipc_sync_reserved_bits_read_zero();
    std::puts("[PASS] ipc_sync_reserved_bits_read_zero");
    ipc_sync_32bit_access_zero_extends();
    std::puts("[PASS] ipc_sync_32bit_access_zero_extends");
    ipc_sync_byte_access();
    std::puts("[PASS] ipc_sync_byte_access");
    ipc_sync_bit13_arm9_to_arm7_raises_when_enabled();
    std::puts("[PASS] ipc_sync_bit13_arm9_to_arm7_raises_when_enabled");
    ipc_sync_bit13_no_raise_when_remote_disabled();
    std::puts("[PASS] ipc_sync_bit13_no_raise_when_remote_disabled");
    ipc_sync_bit13_never_raises_local_irq();
    std::puts("[PASS] ipc_sync_bit13_never_raises_local_irq");
    ipc_sync_bit13_arm7_to_arm9_raises_when_enabled();
    std::puts("[PASS] ipc_sync_bit13_arm7_to_arm9_raises_when_enabled");
    std::puts("ipc_sync_test: all 12 cases passed");
    return 0;
}
