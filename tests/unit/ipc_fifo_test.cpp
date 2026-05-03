// Exercises the IpcFifo control / send / receive registers across both bus
// views. Edge-triggered IF.17 / IF.18 raise is covered separately.

#include "bus/io_regs.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static constexpr u16 kCntMasterEnable = 1u << 15;

// Case 1: after reset both sides report empty FIFOs, no error, master off.
static void ipc_fifo_reset_baseline() {
    NDS nds;

    const u16 a = nds.arm9_io_read16(IO_IPCFIFOCNT);
    const u16 b = nds.arm7_io_read16(IO_IPCFIFOCNT);
    REQUIRE((a & 1u) != 0u);               // send empty
    REQUIRE((a & (1u << 8)) != 0u);        // recv empty
    REQUIRE((a & (1u << 1)) == 0u);        // send not full
    REQUIRE((a & (1u << 9)) == 0u);        // recv not full
    REQUIRE((a & (1u << 14)) == 0u);       // no error
    REQUIRE((a & kCntMasterEnable) == 0u); // master off
    REQUIRE((b & 1u) != 0u);
    REQUIRE((b & (1u << 8)) != 0u);
}

// Case 2: master-enable disabled — writes to SEND drop silently. Error stays
// clear; the FIFO never fills.
static void ipc_fifo_send_while_disabled_dropped() {
    NDS nds;

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xDEAD'BEEFu);
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & 1u) != 0u);         // still empty
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & (1u << 14)) == 0u); // no error
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 8)) != 0u);  // recv empty
}

// Case 3: master-enable disabled on the *reader* — empty-FIFO RECV peeks the
// last received word without advancing or setting error.
static void ipc_fifo_recv_while_disabled_empty_returns_last_word() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xCAFE'0001u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xCAFE'0001u);

    nds.arm7_io_write16(IO_IPCFIFOCNT, 0u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xCAFE'0001u);
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 14)) == 0u);
}

// Case 3b: master-enable disabled with a non-empty FIFO — peek must return
// the *oldest unread* word (head), not the most-recently-popped one. Per
// GBATEK "return the oldest FIFO word ... without removing it from the FIFO."
static void ipc_fifo_recv_while_disabled_nonempty_peeks_head() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x1111u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x2222u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x3333u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x1111u);

    nds.arm7_io_write16(IO_IPCFIFOCNT, 0u);
    // Head is 0x2222 now; peek must return that, not the popped 0x1111.
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x2222u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x2222u); // idempotent
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 14)) == 0u);
}

// Case 4: fill to 16 then push one more — error sets, FIFO unchanged. Drain
// 16 in order to confirm FIFO ordering, observe full/empty bits.
static void ipc_fifo_fill_drain_full_error() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);

    for (u32 i = 0; i < 16; ++i) {
        nds.arm9_io_write32(IO_IPCFIFOSEND, 0x1000u + i);
    }
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & (1u << 1)) != 0u); // send full
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 9)) != 0u); // recv full

    // 17th push: error on sender, contents untouched.
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xBADBAD00u);
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & (1u << 14)) != 0u);

    for (u32 i = 0; i < 16; ++i) {
        REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0x1000u + i);
    }
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 8)) != 0u); // recv empty
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & 1u) != 0u);        // send empty
}

// Case 5: empty RECV returns last popped word and sets reader's error bit.
static void ipc_fifo_empty_recv_returns_last_word() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xDEAD'C0DEu);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xDEAD'C0DEu);

    // Now empty — peek returns last word, error sets on ARM7.
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xDEAD'C0DEu);
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 14)) != 0u);
}

// Case 6: empty RECV before anything was ever sent returns 0.
static void ipc_fifo_empty_recv_never_received_returns_zero() {
    NDS nds;

    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0u);
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 14)) != 0u);
}

// Case 7: CNT.3 clears the sender's FIFO and zeroes last_word + ever_received,
// so a subsequent empty-RECV on the receiver returns 0 (not the last word).
static void ipc_fifo_cnt3_clear_zeroes_data_and_last_word() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    for (u32 i = 0; i < 5; ++i) {
        nds.arm9_io_write32(IO_IPCFIFOSEND, 0xA0u + i);
    }

    // ARM9 clears its send FIFO via CNT.3 (master-enable kept on).
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | (1u << 3));
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & 1u) != 0u);        // send empty
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 8)) != 0u); // recv empty

    // Empty RECV now returns 0, not the previously-popped value.
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0u);
}

// Case 8: CNT.14 ack clears the error bit; CNT.3 does not persist in the read.
static void ipc_fifo_cnt14_ack_and_cnt3_not_persisted() {
    NDS nds;

    // Trigger error: send while disabled does NOT set the bit, but reading
    // empty RECV (with master enable) does. Use that path.
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    (void) nds.arm7_io_read32(IO_IPCFIFORECV);
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 14)) != 0u);

    // Ack: write CNT with bit 14 set (and master enable so we don't disable).
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | (1u << 14));
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 14)) == 0u);

    // CNT.3 doesn't persist: write it, read back, bit 3 is zero.
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | (1u << 3));
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & (1u << 3)) == 0u);
}

// Case 9: a2b and b2a are independent — sending one direction doesn't affect
// the other side's FIFO state.
static void ipc_fifo_directions_are_independent() {
    NDS nds;

    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x9999u);
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & 1u) == 0u);        // send not empty
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & (1u << 8)) == 0u); // recv not empty
    REQUIRE((nds.arm7_io_read16(IO_IPCFIFOCNT) & 1u) != 0u);        // ARM7 send empty
    REQUIRE((nds.arm9_io_read16(IO_IPCFIFOCNT) & (1u << 8)) != 0u); // ARM9 recv empty
}

int main() {
    ipc_fifo_reset_baseline();
    std::puts("[PASS] ipc_fifo_reset_baseline");
    ipc_fifo_send_while_disabled_dropped();
    std::puts("[PASS] ipc_fifo_send_while_disabled_dropped");
    ipc_fifo_recv_while_disabled_empty_returns_last_word();
    std::puts("[PASS] ipc_fifo_recv_while_disabled_empty_returns_last_word");
    ipc_fifo_recv_while_disabled_nonempty_peeks_head();
    std::puts("[PASS] ipc_fifo_recv_while_disabled_nonempty_peeks_head");
    ipc_fifo_fill_drain_full_error();
    std::puts("[PASS] ipc_fifo_fill_drain_full_error");
    ipc_fifo_empty_recv_returns_last_word();
    std::puts("[PASS] ipc_fifo_empty_recv_returns_last_word");
    ipc_fifo_empty_recv_never_received_returns_zero();
    std::puts("[PASS] ipc_fifo_empty_recv_never_received_returns_zero");
    ipc_fifo_cnt3_clear_zeroes_data_and_last_word();
    std::puts("[PASS] ipc_fifo_cnt3_clear_zeroes_data_and_last_word");
    ipc_fifo_cnt14_ack_and_cnt3_not_persisted();
    std::puts("[PASS] ipc_fifo_cnt14_ack_and_cnt3_not_persisted");
    ipc_fifo_directions_are_independent();
    std::puts("[PASS] ipc_fifo_directions_are_independent");
    std::puts("ipc_fifo_test: all 10 cases passed");
    return 0;
}
