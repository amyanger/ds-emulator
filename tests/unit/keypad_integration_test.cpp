// End-to-end keypad + lid integration test — slice 3k commit 7 (capstone).
//
// Drives the full input-driven IRQ flow through NDS using only public
// surfaces: the ARM bus dispatchers (for IME / IE / IF and KEYCNT) and the
// host-injection API (set_keypad_state / set_lid_closed). Each unit test in
// slice 3k has already proved one link of the chain; this test wires them
// together exactly the way a real ROM + frontend would and asserts:
//   * Reset seeds KEYINPUT = 0x03FF, both KEYCNTs disabled, lid open.
//   * Bus writes to IME / IE recompute the ARM7 line through
//     update_arm7_irq_signals().
//   * A host keypad press on the rising edge of an OR-mode condition raises
//     IF.12 and flips cpu7_.irq_line() high on the same call.
//   * Acknowledging IF.12 via the bus write-1-clear path drops the line.
//   * The lid folding edge raises nothing; the unfolding edge latches IF.22
//     (NDS7-only, no source-side enable) and raises the line.
//   * ARM9 sees none of it when only ARM7's KEYCNT is configured.
//   * A symmetric run with ARM9's KEYCNT in AND mode raises IF.12 on ARM9
//     only, leaving the unconfigured ARM7 side low.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "interrupt/irq_controller.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static constexpr u32 kIf12 = 1u << 12;
static constexpr u32 kIf22 = 1u << 22;
static constexpr u16 kAllReleased = 0x03FFu;

static constexpr u16 kBtnA = 1u << 0;
static constexpr u16 kBtnB = 1u << 1;

// KEYINPUT is active-low: clearing bit N = pressing button N.
static constexpr u16 press(u16 state, u16 button_bit) {
    return static_cast<u16>(state & ~button_bit);
}

static void Keypad_FullBoot_OrModeRaisesAndLine() {
    NDS nds;
    nds.reset();

    // Step 1: reset defaults — keypad released, lid open, line low.
    REQUIRE(nds.keypad().read_keyinput() == kAllReleased);
    REQUIRE(nds.lid_switch().prev_unfolded() == true);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 2: ARM7 enables keypad (IF.12) and hinge (IF.22) delivery via the
    // bus. No source is pending yet, so the line stays low.
    nds.arm7_io_write32(IO_IME, 1u);
    nds.arm7_io_write32(IO_IE, kIf12 | kIf22);
    REQUIRE(nds.irq7().line() == false);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 3: ARM7 selects A, enable, OR mode in its own KEYCNT cell.
    nds.arm7_io_write16(IO_KEYCNT, 0x4001u);

    // Step 4: host presses A. Rising edge of the OR condition latches IF.12
    // and update_arm7_irq_signals() flips the cached ARM7 line high.
    nds.set_keypad_state(press(kAllReleased, kBtnA));
    REQUIRE((nds.irq7().read_if() & kIf12) != 0u);
    REQUIRE(nds.cpu7().irq_line() == true);

    // Step 5: ARM7 acknowledges IF.12 with a write-1-clear store; line drops.
    nds.arm7_io_write32(IO_IF, kIf12);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 6: folding the lid (open -> closed) is the falling edge — nothing
    // raises.
    nds.set_lid_closed(true);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 7: unfolding the lid (closed -> open) is the rising edge — IF.22
    // latches and the line goes high.
    nds.set_lid_closed(false);
    REQUIRE((nds.irq7().read_if() & kIf22) != 0u);
    REQUIRE(nds.cpu7().irq_line() == true);

    // Step 8: ARM7 acknowledges IF.22; line drops.
    nds.arm7_io_write32(IO_IF, kIf22);
    REQUIRE((nds.irq7().read_if() & kIf22) == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 9: ARM9 saw nothing throughout — only ARM7's KEYCNT was
    // configured and IF.22 is NDS7-only.
    REQUIRE(nds.irq9().read_if() == 0u);
}

static void Keypad_FullBoot_Arm9KeycntConfigured_RaisesOnArm9Only() {
    NDS nds;
    nds.reset();

    // ARM9 enables IF.12 delivery and configures its own KEYCNT cell: select
    // A + B, enable, AND mode (0xC003).
    nds.arm9_io_write32(IO_IME, 1u);
    nds.arm9_io_write32(IO_IE, kIf12);
    nds.arm9_io_write16(IO_KEYCNT, 0xC003u);

    // Pushing the all-released state is a no-op — the AND condition is false.
    nds.set_keypad_state(kAllReleased);
    REQUIRE((nds.irq9().read_if() & kIf12) == 0u);

    // Press A and B together — every selected button is held, so the AND
    // condition rises and IF.12 latches on ARM9 only.
    const u16 a_and_b = press(press(kAllReleased, kBtnA), kBtnB);
    nds.set_keypad_state(a_and_b);
    REQUIRE((nds.irq9().read_if() & kIf12) != 0u);
    REQUIRE((nds.irq7().read_if() & kIf12) == 0u);

    // ARM9 delivery line is high; the unconfigured ARM7 side stays low.
    REQUIRE(nds.irq9().line() == true);
    REQUIRE(nds.irq7().line() == false);
}

int main() {
    std::puts("[Keypad_FullBoot_OrModeRaisesAndLine]");
    Keypad_FullBoot_OrModeRaisesAndLine();
    std::puts("[Keypad_FullBoot_Arm9KeycntConfigured_RaisesOnArm9Only]");
    Keypad_FullBoot_Arm9KeycntConfigured_RaisesOnArm9Only();
    std::puts("keypad_integration_test: OK");
    return 0;
}
