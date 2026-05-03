// End-to-end ARM9↔ARM7 IPC handshake. Drives the full boot-style sequence
// (IME/IE arming → CNT enable → IPCSYNC ping → IPC FIFO round trip) through
// the public NDS bus interface only — no IpcSync / IpcFifo / IrqController
// internals. Implements the scenario specified in
// docs/specs/2026-04-24-ipc-phase1-slice3i-design.md §6.5 ("Ipc_Boot_Handshake").

#include "bus/io_regs.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Bit masks used by the handshake.
static constexpr u32 kImeEnable = 1u;
static constexpr u32 kIeIpcMask = (1u << 16) | (1u << 17) | (1u << 18);

static constexpr u32 kIf16 = 1u << 16; // IPC SYNC
static constexpr u32 kIf17 = 1u << 17; // IPC send-empty
static constexpr u32 kIf18 = 1u << 18; // IPC recv-not-empty

// CNT bits: master enable | recv-not-empty IRQ enable | send-empty IRQ enable.
// Equivalent to CNT.15 | CNT.10 | CNT.2 = 0x8404.
static constexpr u16 kCntFullArm = (1u << 15) | (1u << 10) | (1u << 2);

// IPCSYNC values driven during the handshake.
//   ARM9 enables receiving SYNC IRQs from ARM7: bit 14.
//   ARM7 outputs nibble 5, enables receive, and pulses bit 13 to ping ARM9.
static constexpr u16 kSyncArm9EnableRx = 1u << 14;           // 0x4000
static constexpr u16 kSyncArm7Ping = (1u << 14) | (1u << 13) // enable + trigger
                                     | (5u << 8);            // out_lo nibble = 5

// Drive the §6.5 scenario end-to-end through the public bus surface.
static void ipc_boot_handshake() {
    NDS nds;

    // Step 1: arm both sides' master IRQ enables.
    nds.arm9_io_write32(IO_IME, kImeEnable);
    nds.arm7_io_write32(IO_IME, kImeEnable);
    REQUIRE(nds.arm9_io_read32(IO_IME) == kImeEnable);
    REQUIRE(nds.arm7_io_read32(IO_IME) == kImeEnable);

    // Step 2: enable the three IPC sources (bits 16/17/18) in IE on both sides.
    nds.arm9_io_write32(IO_IE, kIeIpcMask);
    nds.arm7_io_write32(IO_IE, kIeIpcMask);
    REQUIRE(nds.arm9_io_read32(IO_IE) == kIeIpcMask);
    REQUIRE(nds.arm7_io_read32(IO_IE) == kIeIpcMask);
    std::puts("[step] IME + IE armed for IPC bits 16/17/18");

    // Step 3: enable IPC FIFO master + both IRQ masks. Per §5.8 / commit 6, the
    // 0→1 transition of CNT.2 with a send FIFO that's already empty raises a
    // rising-edge gated send-empty value, so each side latches IF.17 here. Ack
    // those local pulses before proceeding so later edge checks start clean.
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntFullArm);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntFullArm);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf17) != 0u);
    nds.arm9_io_write32(IO_IF, kIf17);
    nds.arm7_io_write32(IO_IF, kIf17);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIeIpcMask) == 0u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIeIpcMask) == 0u);
    std::puts("[step] CNT armed, initial IF.17 self-pulses acked");

    // Step 4: ARM9 enables reception of remote IPCSYNC pings (bit 14).
    nds.arm9_io_write16(IO_IPCSYNC, kSyncArm9EnableRx);

    // Step 5: ARM7 writes out_lo=5, enables its own RX, and pulses bit 13.
    // Because ARM9 has bit 14 set, this raises IF.16 on ARM9 (one-shot).
    nds.arm7_io_write16(IO_IPCSYNC, kSyncArm7Ping);

    // Step 6: ARM9 sees IF.16 set; the SYNC view shows ARM7's nibble (5) in
    // ARM9's read bits 0-3. ARM7 IF.16 stays cleared (raise targets remote).
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf16) != 0u);
    const u16 arm9_sync_view = nds.arm9_io_read16(IO_IPCSYNC);
    REQUIRE((arm9_sync_view & 0x000Fu) == 5u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf16) == 0u);
    std::puts("[step] IPCSYNC ping delivered: ARM9 IF.16 set, nibble = 5");

    // Step 7: ARM9 acks IF.16, then pushes 4 distinct words into the FIFO. The
    // first push (count 0→1) is the rising edge of ARM7's gated recv-not-empty
    // value and raises IF.18 on ARM7. Subsequent pushes keep gated at 1, no
    // further raise. ARM9's send-empty also falls 1→0 on the first push (we
    // already acked the post-CNT pulse), no IF.17 raise.
    nds.arm9_io_write32(IO_IF, kIf16);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf16) == 0u);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x11111111u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x22222222u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x33333333u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x44444444u);

    // Step 8: ARM7 sees IF.18 from the rising edge, then drains all four words
    // in FIFO order. Each pop decreases count by 1; the 4th pop (count 1→0)
    // is the rising edge of ARM9's gated send-empty value, raising IF.17 on
    // ARM9. IF.18 on ARM7 stays latched until acked — we deliberately do not.
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x11111111u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x22222222u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x33333333u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x44444444u);
    std::puts("[step] ARM7 drained 4 words in FIFO order");

    // Step 9: ARM9's send-empty rose 0→1 on the final pop, so IF.17 is set.
    // ARM7's IF.18 is still set from step 8 (we never acked it).
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
}

int main() {
    ipc_boot_handshake();
    std::puts("[PASS] ipc_boot_handshake");
    std::puts("ipc_integration_test: all 1 cases passed");
    return 0;
}
