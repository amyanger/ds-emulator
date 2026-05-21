// Slice 3k commit 6 — lid IF.22 ("screens unfolded") rising-edge raise.
//
// EXTKEYIN bit 7 is the hinge: 0 = open, 1 = closed. IF.22 raises on the
// rising edge of "unfolded" (the 1->0 transition of bit 7). The folding edge
// raises nothing. Per GBATEK the hinge has no source-side enable: IF.22
// latches unconditionally on the edge and IE.22 gates only delivery. The
// source is NDS7-only, so ARM9's IF.22 must never be set.

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static constexpr u32 kIf12 = 1u << 12;
static constexpr u32 kIf22 = 1u << 22;
static constexpr u16 kHingeBit = 1u << 7;

static void Lid_Reset_PrevUnfoldedTrue() {
    NDS nds;
    nds.reset();

    REQUIRE(nds.lid_switch().prev_unfolded() == true);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);
}

// Lid is already open at reset; opening it again is not a rising edge.
static void Lid_OpenAtReset_FirstOpenCallNoRaise() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);
}

static void Lid_Close_NoRaise() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);
    REQUIRE((nds.lid_switch().read_extkeyin() & kHingeBit) != 0u);
}

static void Lid_CloseThenOpen_RaisesIf22OnUnfold() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);
    REQUIRE((nds.lid_switch().read_extkeyin() & kHingeBit) == 0u);
}

static void Lid_OpenAgain_NoSecondRaise() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(false);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);
}

// IE.22 clear: the raise still latches IF.22, but the delivery line stays low.
static void Lid_IeBit22Clear_StillLatches() {
    NDS nds;
    nds.reset();

    nds.irq7().write_ime(1u);
    nds.irq7().write_ie(0u);

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);
    REQUIRE(nds.irq7().line() == false);
}

// The hinge wakes a HALTed ARM7 even with IME=0: halt-wake gates on (IE & IF)
// only, never on IME. Opening the lid must set halt_wake_pending while the
// delivery line stays low. This is the path games use to wake from sleep.
static void Lid_Open_SetsHaltWakePending() {
    NDS nds;
    nds.reset();

    nds.irq7().write_ie(kIf22); // enabled for wake, IME deliberately left 0

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE(nds.irq7().halt_wake_pending() == true);
    REQUIRE(nds.irq7().line() == false);
}

static void Lid_Arm9_IfBit22_NeverSet() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq9().read_if() & kIf22) == 0u);
}

static void Lid_RaiseDoesNotAffectIf12() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
}

// Folding the lid again must not clear IF.22 (only write-1-clear clears it).
static void Lid_RaiseDoesNotClearOnFold() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);

    nds.set_lid_closed(true);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);
}

static void Lid_AckThenReopen_LatchesAgain() {
    NDS nds;
    nds.reset();

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);

    nds.irq7().write_if(kIf22);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);

    nds.set_lid_closed(true);
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);
}

int main() {
    Lid_Reset_PrevUnfoldedTrue();
    Lid_OpenAtReset_FirstOpenCallNoRaise();
    Lid_Close_NoRaise();
    Lid_CloseThenOpen_RaisesIf22OnUnfold();
    Lid_OpenAgain_NoSecondRaise();
    Lid_IeBit22Clear_StillLatches();
    Lid_Open_SetsHaltWakePending();
    Lid_Arm9_IfBit22_NeverSet();
    Lid_RaiseDoesNotAffectIf12();
    Lid_RaiseDoesNotClearOnFold();
    Lid_AckThenReopen_LatchesAgain();
    return 0;
}
