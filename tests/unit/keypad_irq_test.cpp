// Slice 3k commit 4 — keypad IF.12 rising-edge raise.
//
// Drives `NDS::set_keypad_state(...)` and per-CPU KEYCNT writes through the
// bus dispatchers, then inspects irq9_.read_if() / irq7_.read_if() to
// confirm:
//   * OR-mode raises on the first rising edge and not on idempotent calls.
//   * Falling edges never raise; a fresh rise after fall raises again.
//   * AND-mode requires every selected button pressed simultaneously.
//   * KEYCNT writes are per-CPU: ARM9 KEYCNT does not raise ARM7 IF.12.
//   * Toggling KEYCNT bit 14 (enable) while the gated condition is already
//     true counts as a rising edge.
//   * Switching OR→AND while only one of two selected buttons is held
//     drops the condition (no raise on the fall); a subsequent press
//     satisfying AND raises.
//   * selected==0 in AND mode never raises (matches melonDS resolution of
//     the GBATEK vacuous-true ambiguity).

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static constexpr u32 kKeycntAddr = 0x04000132u;
static constexpr u32 kIf12 = 1u << 12;
static constexpr u16 kAllReleased = 0x03FFu;

// KEYINPUT is active-low: clearing bit N = pressing button N.
static constexpr u16 press(u16 state, u16 button_bit) {
    return static_cast<u16>(state & ~button_bit);
}

static constexpr u16 kBtnA = 1u << 0;
static constexpr u16 kBtnB = 1u << 1;
static constexpr u16 kBtnStart = 1u << 3;

static void OrMode_RisingEdge_RaisesIf12() {
    NDS nds;
    nds.reset();

    // ARM7 KEYCNT: select A + B, enable, OR mode.
    nds.arm7_io_write16(kKeycntAddr, 0x4003u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);
}

static void OrMode_NoChange_NoReraise() {
    NDS nds;
    nds.reset();
    nds.arm7_io_write16(kKeycntAddr, 0x4003u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);

    // Acknowledge IF.12 (write-1-clear), then push the same state again.
    nds.irq7().write_if(kIf12);
    nds.set_keypad_state(press(kAllReleased, kBtnA));
    // No new rising edge, so the bit must stay clear.
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
}

static void OrMode_FallingEdge_NoRaise() {
    NDS nds;
    nds.reset();
    nds.arm7_io_write16(kKeycntAddr, 0x4003u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    nds.irq7().write_if(kIf12);

    nds.set_keypad_state(kAllReleased);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
}

static void OrMode_AfterFallAndRise_RaisesAgain() {
    NDS nds;
    nds.reset();
    nds.arm7_io_write16(kKeycntAddr, 0x4003u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    nds.irq7().write_if(kIf12);
    nds.set_keypad_state(kAllReleased);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
}

static void AndMode_AllRequired() {
    NDS nds;
    nds.reset();

    // ARM9 KEYCNT: select A + B + Start, enable, AND mode.
    nds.arm9_io_write16(kKeycntAddr, 0xC00Bu);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);

    nds.set_keypad_state(press(press(kAllReleased, kBtnA), kBtnB));
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);

    const u16 all_three = press(press(press(kAllReleased, kBtnA), kBtnB), kBtnStart);
    nds.set_keypad_state(all_three);
    REQUIRE((nds.irq9().read_if() & kIf12) != 0u);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
}

static void AndMode_ReleaseOneFallsThenAllAgainRaises() {
    NDS nds;
    nds.reset();
    nds.arm9_io_write16(kKeycntAddr, 0xC00Bu);

    const u16 all_three = press(press(press(kAllReleased, kBtnA), kBtnB), kBtnStart);
    nds.set_keypad_state(all_three);
    REQUIRE((nds.irq9().read_if() & kIf12) != 0u);
    nds.irq9().write_if(kIf12);

    // Release Start: only A + B held — AND condition falls.
    nds.set_keypad_state(press(press(kAllReleased, kBtnA), kBtnB));
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);

    nds.set_keypad_state(all_three);
    REQUIRE((nds.irq9().read_if() & kIf12) != 0u);
}

static void PerCpuIndependence_Arm9RaisesNotArm7() {
    NDS nds;
    nds.reset();

    nds.arm9_io_write16(kKeycntAddr, 0x4001u); // ARM9: select A, enable, OR.
    // ARM7 KEYCNT stays at reset (0x0000 — disabled).

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq9().read_if() & kIf12) != 0u);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
}

static void PerCpuIndependence_Arm7RaisesNotArm9() {
    NDS nds;
    nds.reset();

    nds.arm7_io_write16(kKeycntAddr, 0x4001u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);
}

static void KeycntEnableBitToggleRaises() {
    NDS nds;
    nds.reset();

    // Press A first while ARM7 KEYCNT is disabled (selected, but bit 14 = 0).
    nds.arm7_io_write16(kKeycntAddr, 0x0001u);
    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);

    // Flip bit 14 on while the button stays pressed. The gate opens and the
    // condition transitions false→true, which is the rising edge.
    nds.arm7_io_write16(kKeycntAddr, 0x4001u);
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
}

static void KeycntModeChange_OrToAndDropsCondition_ThenRaises() {
    NDS nds;
    nds.reset();

    // ARM7 OR mode, select A + B.
    nds.arm7_io_write16(kKeycntAddr, 0x4003u);

    // Press A only — OR condition true.
    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
    nds.irq7().write_if(kIf12);

    // Switch to AND mode keeping same selection. matched (0x0001) !=
    // selected (0x0003) — condition falls. No raise.
    nds.arm7_io_write16(kKeycntAddr, 0xC003u);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);

    // Press B as well — AND now satisfied → rising edge.
    nds.set_keypad_state(press(press(kAllReleased, kBtnA), kBtnB));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
}

static void SelectedZero_InAndMode_NoRaise() {
    NDS nds;
    nds.reset();

    // ARM7 KEYCNT = 0xC000: enable + AND, but no select bits.
    nds.arm7_io_write16(kKeycntAddr, 0xC000u);

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);

    nds.set_keypad_state(0x0000u); // every button pressed.
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
}

static void Arm7OnlyConfigured_RaiseDoesNotReachArm9() {
    NDS nds;
    nds.reset();

    nds.arm7_io_write16(kKeycntAddr, 0x4001u);
    // ARM9 KEYCNT stays at reset (0x0000 — disabled).

    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);
}

int main() {
    OrMode_RisingEdge_RaisesIf12();
    OrMode_NoChange_NoReraise();
    OrMode_FallingEdge_NoRaise();
    OrMode_AfterFallAndRise_RaisesAgain();
    AndMode_AllRequired();
    AndMode_ReleaseOneFallsThenAllAgainRaises();
    PerCpuIndependence_Arm9RaisesNotArm7();
    PerCpuIndependence_Arm7RaisesNotArm9();
    KeycntEnableBitToggleRaises();
    KeycntModeChange_OrToAndDropsCondition_ThenRaises();
    SelectedZero_InAndMode_NoRaise();
    Arm7OnlyConfigured_RaiseDoesNotReachArm9();
    return 0;
}
