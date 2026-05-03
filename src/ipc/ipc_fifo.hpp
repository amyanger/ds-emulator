#pragma once

// IPC FIFO (CNT 0x4000184, SEND 0x4000188, RECV 0x4100000) — two 16-deep
// directional ring buffers between ARM9 and ARM7. Each side sees its own
// half of the CNT register (send-empty/full status, IRQ enables, error,
// master enable, write-only clear/ack bits) plus a 32-bit SEND port that
// pushes into the local→remote direction and a 32-bit RECV port that pops
// from the remote→local direction. Per GBATEK §IPCFIFO.

#include "ds/common.hpp"

#include <array>

namespace ds {

class IrqController;

class IpcFifo {
public:
    enum class Side { Arm9, Arm7 };

    void reset();

    u16 read_cnt(Side side) const;
    void write_cnt(Side side, u16 value, IrqController& arm9_irq, IrqController& arm7_irq);
    void write_send(Side side, u32 value, IrqController& arm9_irq, IrqController& arm7_irq);
    u32 read_recv(Side side, IrqController& arm9_irq, IrqController& arm7_irq);

    // Test introspection — not for cross-subsystem use.
    bool send_empty(Side side) const;
    bool send_full(Side side) const;
    bool recv_empty(Side side) const;
    bool recv_full(Side side) const;
    u8 fill_level_send(Side side) const;

private:
    struct Direction {
        std::array<u32, 16> data{};
        u8 head = 0;
        u8 tail = 0;
        u8 count = 0;
        u32 last_word = 0;
        bool ever_received = false;
    };

    struct SideCnt {
        bool send_empty_irq = false;     // CNT.2
        bool recv_not_empty_irq = false; // CNT.10
        bool error = false;              // CNT.14
        bool master_enable = false;      // CNT.15
        bool prev_send_empty_gated = false;
        bool prev_recv_not_empty_gated = false;
    };

    static constexpr Side other(Side side) { return side == Side::Arm9 ? Side::Arm7 : Side::Arm9; }

    // ARM9's send goes into a2b_ (ARM9→ARM7); ARM9's recv pops from b2a_.
    Direction& send_dir(Side side);
    Direction& recv_dir(Side side);
    const Direction& send_dir(Side side) const;
    const Direction& recv_dir(Side side) const;

    SideCnt& cnt(Side side);
    const SideCnt& cnt(Side side) const;

    // Edge-triggered IF.17 / IF.18 raise; gated state is latched between
    // calls so a rising edge of (mask AND condition) raises exactly once.
    void recompute_irqs(IrqController& arm9_irq, IrqController& arm7_irq);

    Direction a2b_{}; // ARM9 → ARM7
    Direction b2a_{}; // ARM7 → ARM9
    SideCnt arm9_cnt_{};
    SideCnt arm7_cnt_{};
};

} // namespace ds
