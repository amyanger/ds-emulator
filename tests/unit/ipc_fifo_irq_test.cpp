// Edge-triggered IF.17 / IF.18 raise semantics for the IPC FIFO. IF.17 fires
// on the rising edge of (CNT.2 AND CNT.0) — gated send-empty — on the sender
// side; IF.18 fires on the rising edge of (CNT.10 AND NOT CNT.8) — gated
// recv-not-empty — on the receiver side. See spec
// docs/specs/2026-04-24-ipc-phase1-slice3i-design.md §4.5 / §6.4 and the
// verbatim GBATEK quote on lines 13050-13053 of the upstream HTML.
//
// Driven entirely through the public NDS bus interface; never touches IpcFifo
// internals. Each case constructs a fresh NDS so latches start clean.

#include "bus/io_regs.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static constexpr u16 kCntSendEmptyIrq = 1u << 2;
static constexpr u16 kCntRecvNotEmptyIrq = 1u << 10;
static constexpr u16 kCntMasterEnable = 1u << 15;

static constexpr u32 kIf17 = 1u << 17;
static constexpr u32 kIf18 = 1u << 18;

// Helper: enable both sides' master enable. No IRQ masks set here, so no
// rising-edge fires from this call (gated values stay 0→0 on both sides).
static void enable_both_masters(NDS& nds) {
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable);
}

// Helper: turn on ARM9 CNT.2 (send-empty IRQ enable) and immediately ack the
// resulting IF.17 raise. With the send FIFO empty after reset, flipping CNT.2
// on is a 0→1 rising edge of the gated send-empty value, so the recompute
// raises IF.17 on the ARM9 side. Tests that exercise CNT.2 must clear that
// initial pulse before running their actual scenario.
static void arm9_set_send_empty_irq_and_ack(NDS& nds) {
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntSendEmptyIrq);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);
    nds.arm9_io_write32(IO_IF, kIf17);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);
}

// Case a: rising edge of gated recv-not-empty raises IF.18 on the receiver.
static void fifo_recv_not_empty_rising_edge_raises_if18() {
    NDS nds;
    enable_both_masters(nds);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntRecvNotEmptyIrq);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u); // mask flip with empty FIFO: no edge

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xCAFE'0001u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf18) == 0u); // local IF untouched
}

// Case b: with the receiver's CNT.10 mask off, sending data does not raise
// IF.18 even though the FIFO becomes non-empty.
static void fifo_recv_not_empty_no_mask_no_raise() {
    NDS nds;
    enable_both_masters(nds);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0x1234u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);
}

// Case c: rising edge of gated send-empty raises IF.17 on the sender. Send a
// word (count 0→1, gated falls 1→0), then drain it (count 1→0, gated rises
// 0→1) — that drain is the rising edge.
static void fifo_send_empty_rising_edge_raises_if17() {
    NDS nds;
    enable_both_masters(nds);
    arm9_set_send_empty_irq_and_ack(nds);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xAAAA'0001u); // count 0→1, falling edge
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);

    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xAAAA'0001u); // count 1→0, rising edge
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf17) == 0u); // remote IF untouched
}

// Case d: IF.17 only fires when the gated send-empty actually returns to true
// (count == 0). Draining from N>1 to N-1>0 keeps gated at 0, so no raise.
static void fifo_send_empty_only_rises_at_true_zero() {
    NDS nds;
    enable_both_masters(nds);
    arm9_set_send_empty_irq_and_ack(nds);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xBBBB'0001u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xBBBB'0002u); // count = 2, gated stayed 0

    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xBBBB'0001u); // count 2→1, gated still 0
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);

    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xBBBB'0002u); // count 1→0, gated 0→1
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);
}

// Case e: every falling edge is silent. Cover both gated values:
//   - Recv-not-empty falling: drain a non-empty FIFO to 0; no IF.18 reraise.
//   - Send-empty falling: push a word into the now-empty FIFO; no spurious
//     IF.17 reraise (it would already have been fired by the drain-to-zero).
// Then confirm the sender-side rising edge still works on the next 0→1.
static void fifo_falling_edge_never_raises() {
    NDS nds;
    enable_both_masters(nds);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntRecvNotEmptyIrq);
    arm9_set_send_empty_irq_and_ack(nds);

    // Push 3: recv goes 0→1 (raises IF.18 on ARM7) → 2 → 3. Send-empty falls
    // 1→0 on the first push and stays 0 thereafter (no IF.17 raises).
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xCCCC'0001u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xCCCC'0002u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xCCCC'0003u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
    nds.arm7_io_write32(IO_IF, kIf18); // ack
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);

    // Drain 3: recv-not-empty falls 3→2, 2→1, 1→0. Falling edges are silent
    // for IF.18. The 1→0 transition is a *rising* edge for the sender's gated
    // send-empty value, so IF.17 raises on ARM9 at the third drain.
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xCCCC'0001u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xCCCC'0002u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xCCCC'0003u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);

    // Ack IF.17. Now push another word — that's a clean recv 0→1 rising edge
    // (so IF.18 reraises) and a *falling* edge for send-empty (so IF.17 stays
    // cleared).
    nds.arm9_io_write32(IO_IF, kIf17);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xCCCC'0004u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u);
}

// Case f: flipping the receiver's CNT.10 on while the FIFO is already
// non-empty is itself a rising edge of the gated value — IF.18 must raise.
static void fifo_toggling_cnt_mask_on_while_condition_true_raises() {
    NDS nds;
    enable_both_masters(nds);

    // Receiver mask off; sending fills the FIFO without raising (case b).
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xDEAD'0001u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);

    // Now flip mask on. Gated recv-not-empty: false → true. Rising edge.
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntRecvNotEmptyIrq);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
}

// Case g: after acking IF.18, further pushes that keep the gated value at 1
// must not reraise. Only a clean 0→1 transition (drain fully then refill)
// fires again.
static void fifo_ack_if_then_reassert() {
    NDS nds;
    enable_both_masters(nds);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntRecvNotEmptyIrq);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xBEEF'0001u); // count 0→1, raise IF.18
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
    nds.arm7_io_write32(IO_IF, kIf18);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xBEEF'0002u); // count 1→2, gated stays 1
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);

    // Drain fully, then push — that 0→1 push must reraise.
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xBEEF'0001u);
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xBEEF'0002u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u); // falls 1→0, silent

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xBEEF'0003u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
}

// Case h: per-GBATEK, CNT.3 clear is a count-to-zero transition like any
// other. With the sender's CNT.2 mask kept on across the write, the gated
// send-empty rises false→true and IF.17 fires. Counterintuitive but spec.
static void fifo_cnt3_clear_raises_if17_on_sender_side_when_mask_on() {
    NDS nds;
    enable_both_masters(nds);
    arm9_set_send_empty_irq_and_ack(nds);

    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xEEEE'0001u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xEEEE'0002u);
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xEEEE'0003u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) == 0u); // gated stayed 0 throughout

    // Master + send-empty-IRQ + CNT.3 clear. count goes 3→0, gated rises.
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntSendEmptyIrq | (1u << 3));
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf17) != 0u);
}

// Case i: a2b_ and b2a_ track edges independently. ARM9→ARM7 traffic only
// raises ARM7 IF.18 (and ARM9 IF.17 on full drain); the symmetric
// ARM7→ARM9 traffic only affects the other side.
static void fifo_both_sides_independent() {
    NDS nds;
    enable_both_masters(nds);
    nds.arm9_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntRecvNotEmptyIrq);
    nds.arm7_io_write16(IO_IPCFIFOCNT, kCntMasterEnable | kCntRecvNotEmptyIrq);

    // ARM9 → ARM7: only ARM7's IF.18 should rise.
    nds.arm9_io_write32(IO_IPCFIFOSEND, 0xAAAA'1111u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) != 0u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf18) == 0u);

    // Drain + ack so the next direction starts clean.
    REQUIRE(nds.arm7_io_read32(IO_IPCFIFORECV) == 0xAAAA'1111u);
    nds.arm7_io_write32(IO_IF, kIf18);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);

    // ARM7 → ARM9: only ARM9's IF.18 should rise.
    nds.arm7_io_write32(IO_IPCFIFOSEND, 0x5555'2222u);
    REQUIRE((nds.arm9_io_read32(IO_IF) & kIf18) != 0u);
    REQUIRE((nds.arm7_io_read32(IO_IF) & kIf18) == 0u);
}

int main() {
    fifo_recv_not_empty_rising_edge_raises_if18();
    std::puts("[PASS] fifo_recv_not_empty_rising_edge_raises_if18");
    fifo_recv_not_empty_no_mask_no_raise();
    std::puts("[PASS] fifo_recv_not_empty_no_mask_no_raise");
    fifo_send_empty_rising_edge_raises_if17();
    std::puts("[PASS] fifo_send_empty_rising_edge_raises_if17");
    fifo_send_empty_only_rises_at_true_zero();
    std::puts("[PASS] fifo_send_empty_only_rises_at_true_zero");
    fifo_falling_edge_never_raises();
    std::puts("[PASS] fifo_falling_edge_never_raises");
    fifo_toggling_cnt_mask_on_while_condition_true_raises();
    std::puts("[PASS] fifo_toggling_cnt_mask_on_while_condition_true_raises");
    fifo_ack_if_then_reassert();
    std::puts("[PASS] fifo_ack_if_then_reassert");
    fifo_cnt3_clear_raises_if17_on_sender_side_when_mask_on();
    std::puts("[PASS] fifo_cnt3_clear_raises_if17_on_sender_side_when_mask_on");
    fifo_both_sides_independent();
    std::puts("[PASS] fifo_both_sides_independent");
    std::puts("ipc_fifo_irq_test: all 9 cases passed");
    return 0;
}
