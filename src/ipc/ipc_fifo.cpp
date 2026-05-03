#include "ipc/ipc_fifo.hpp"

#include "interrupt/irq_controller.hpp"

namespace ds {

namespace {
// IPCFIFOCNT bit layout per GBATEK §IPCFIFOCNT.
constexpr u16 kCntSendEmpty = 1u << 0;
constexpr u16 kCntSendFull = 1u << 1;
constexpr u16 kCntSendEmptyIrq = 1u << 2;
constexpr u16 kCntRecvEmpty = 1u << 8;
constexpr u16 kCntRecvFull = 1u << 9;
constexpr u16 kCntRecvNotEmptyIrq = 1u << 10;
constexpr u16 kCntError = 1u << 14;
constexpr u16 kCntMasterEnable = 1u << 15;

// IF source bits raised on the per-side IRQ controller (per GBATEK §IPCFIFO).
constexpr u32 kIfSendFifoEmpty = 1u << 17;
constexpr u32 kIfRecvFifoNotEmpty = 1u << 18;
} // namespace

void IpcFifo::reset() {
    a2b_ = Direction{};
    b2a_ = Direction{};
    arm9_cnt_ = SideCnt{};
    arm7_cnt_ = SideCnt{};
}

u16 IpcFifo::read_cnt(Side side) const {
    const SideCnt& c = cnt(side);
    const Direction& s = send_dir(side);
    const Direction& r = recv_dir(side);
    u16 value = 0;
    if (s.count == 0)
        value |= kCntSendEmpty;
    if (s.count == 16)
        value |= kCntSendFull;
    if (c.send_empty_irq)
        value |= kCntSendEmptyIrq;
    if (r.count == 0)
        value |= kCntRecvEmpty;
    if (r.count == 16)
        value |= kCntRecvFull;
    if (c.recv_not_empty_irq)
        value |= kCntRecvNotEmptyIrq;
    if (c.error)
        value |= kCntError;
    if (c.master_enable)
        value |= kCntMasterEnable;
    return value;
}

void IpcFifo::write_cnt(Side side, u16 value, IrqController& arm9_irq, IrqController& arm7_irq) {
    SideCnt& c = cnt(side);

    c.send_empty_irq = ((value >> 2) & 1u) != 0;
    c.recv_not_empty_irq = ((value >> 10) & 1u) != 0;
    c.master_enable = ((value >> 15) & 1u) != 0;

    // CNT.14 is write-1-clear (ack the error flag).
    if ((value >> 14) & 1u)
        c.error = false;

    // CNT.3 is a write-only one-shot that flushes the local side's send FIFO,
    // and per GBATEK ZEROES last_word + ever_received as well (distinct from
    // a natural drain, which preserves last_word).
    if ((value >> 3) & 1u) {
        Direction& s = send_dir(side);
        s.data.fill(0);
        s.head = 0;
        s.tail = 0;
        s.count = 0;
        s.last_word = 0;
        s.ever_received = false;
    }

    recompute_irqs(arm9_irq, arm7_irq);
}

void IpcFifo::write_send(Side side, u32 value, IrqController& arm9_irq, IrqController& arm7_irq) {
    SideCnt& c = cnt(side);
    Direction& s = send_dir(side);

    // Master-enable off: writes silently dropped, error bit *not* set.
    if (!c.master_enable)
        return;

    // FIFO full: error bit set on the sender's side; data not stored.
    if (s.count == 16) {
        c.error = true;
        return;
    }

    s.data[s.tail] = value;
    s.tail = static_cast<u8>((s.tail + 1u) & 15u);
    s.count = static_cast<u8>(s.count + 1u);
    s.last_word = value;
    s.ever_received = true;

    recompute_irqs(arm9_irq, arm7_irq);
}

u32 IpcFifo::read_recv(Side side, IrqController& arm9_irq, IrqController& arm7_irq) {
    SideCnt& c = cnt(side);
    Direction& r = recv_dir(side);

    // Master-enable off: per GBATEK, peek the oldest unread word without
    // advancing the FIFO. Error bit is *not* set on this path. When empty,
    // fall back to last_word (the same value the master-enabled empty path
    // would return), but still without setting the error bit.
    if (!c.master_enable) {
        return r.count == 0 ? r.last_word : r.data[r.head];
    }

    if (r.count == 0) {
        // Empty: returns last_word (or 0 if cleared / never received). The
        // *reader's* error bit is set per GBATEK.
        c.error = true;
        return r.last_word;
    }

    const u32 value = r.data[r.head];
    r.head = static_cast<u8>((r.head + 1u) & 15u);
    r.count = static_cast<u8>(r.count - 1u);
    r.last_word = value;

    recompute_irqs(arm9_irq, arm7_irq);
    return value;
}

bool IpcFifo::send_empty(Side side) const {
    return send_dir(side).count == 0;
}

bool IpcFifo::send_full(Side side) const {
    return send_dir(side).count == 16;
}

bool IpcFifo::recv_empty(Side side) const {
    return recv_dir(side).count == 0;
}

bool IpcFifo::recv_full(Side side) const {
    return recv_dir(side).count == 16;
}

u8 IpcFifo::fill_level_send(Side side) const {
    return send_dir(side).count;
}

IpcFifo::Direction& IpcFifo::send_dir(Side side) {
    return side == Side::Arm9 ? a2b_ : b2a_;
}

IpcFifo::Direction& IpcFifo::recv_dir(Side side) {
    return side == Side::Arm9 ? b2a_ : a2b_;
}

const IpcFifo::Direction& IpcFifo::send_dir(Side side) const {
    return side == Side::Arm9 ? a2b_ : b2a_;
}

const IpcFifo::Direction& IpcFifo::recv_dir(Side side) const {
    return side == Side::Arm9 ? b2a_ : a2b_;
}

IpcFifo::SideCnt& IpcFifo::cnt(Side side) {
    return side == Side::Arm9 ? arm9_cnt_ : arm7_cnt_;
}

const IpcFifo::SideCnt& IpcFifo::cnt(Side side) const {
    return side == Side::Arm9 ? arm9_cnt_ : arm7_cnt_;
}

void IpcFifo::recompute_irqs(IrqController& arm9_irq, IrqController& arm7_irq) {
    // Per GBATEK §IPCFIFO: IF.17 raises on the rising edge of
    // (CNT.2 AND CNT.0)  — gated send-empty — and IF.18 raises on the rising
    // edge of (CNT.10 AND NOT CNT.8) — gated recv-not-empty. Each side has
    // its own pair of latches; per-side IRQs land on that side's controller.
    // Discipline: compare-and-raise FIRST, latch update SECOND. Updating the
    // latch before the compare loses the rising edge (spec §8.1 risk #1).
    constexpr std::array<Side, 2> kSides{Side::Arm9, Side::Arm7};
    for (Side side : kSides) {
        SideCnt& c = cnt(side);
        const Direction& s = send_dir(side);
        const Direction& r = recv_dir(side);

        const bool new_send_empty_gated = c.send_empty_irq && (s.count == 0);
        const bool new_recv_not_empty_gated = c.recv_not_empty_irq && (r.count > 0);

        IrqController& local_irq = (side == Side::Arm9) ? arm9_irq : arm7_irq;
        if (new_send_empty_gated && !c.prev_send_empty_gated)
            local_irq.raise(kIfSendFifoEmpty);
        if (new_recv_not_empty_gated && !c.prev_recv_not_empty_gated)
            local_irq.raise(kIfRecvFifoNotEmpty);

        c.prev_send_empty_gated = new_send_empty_gated;
        c.prev_recv_not_empty_gated = new_recv_not_empty_gated;
    }
}

} // namespace ds
