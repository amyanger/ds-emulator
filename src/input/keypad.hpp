#pragma once

// KEYINPUT (0x4000130, R) is one shared cell — physical buttons are one
// object. KEYCNT (0x4000132, R/W) is two independent cells, one per CPU:
// a write from ARM9 does NOT affect ARM7's IF.12 condition, and vice
// versa. KEYINPUT bits 0..9 are active-LOW (a press clears the bit).

#include "ds/common.hpp"

namespace ds {

class IrqController;

class Keypad {
public:
    enum class Side : u8 { Arm9, Arm7 };

    struct KeyCnt {
        u16 select_mask = 0;
        bool enable = false;
        bool mode_and = false;
    };

    void reset();

    void set_keyinput(u16 v, IrqController& irq9, IrqController& irq7);
    u16 read_keyinput() const;

    u16 read_keycnt(Side side) const;
    void write_keycnt(Side side, u16 v, IrqController& irq9, IrqController& irq7);

    bool prev_condition(Side side) const;

private:
    KeyCnt& cell(Side s) { return cnt_[static_cast<u8>(s)]; }
    const KeyCnt& cell(Side s) const { return cnt_[static_cast<u8>(s)]; }
    bool& prev_cell(Side s) { return prev_condition_[static_cast<u8>(s)]; }

    bool compute_condition(Side side) const;
    void recompute_irqs(IrqController& irq9, IrqController& irq7);

    static constexpr u16 kButtonMask = 0x03FFu;
    static constexpr u16 kKeyCntMask = 0xC3FFu;
    static constexpr u16 kKeyCntEnable = 1u << 14;
    static constexpr u16 kKeyCntModeAnd = 1u << 15;

    u16 keyinput_ = 0x03FFu;
    KeyCnt cnt_[2]{};
    bool prev_condition_[2]{};
};

} // namespace ds
