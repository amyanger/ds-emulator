// Exercises the RTC rising-edge IF.7 raise discipline — slice 3j commit 6.
//
// Each test constructs an isolated `Rtc` + `IrqController irq7;` and drives
// `Rtc::tick(irq7)` to verify:
//   * No raise when there is no match (gating by status2 + alarm fields).
//   * Exactly one raise on a 0→1 transition of the match condition.
//   * No re-raise while match stays true (rising-edge discipline).
//   * No raise on a 1→0 falling edge.
//   * Fresh rise after fall+rise.
//   * Independence of Alarm-1 vs Alarm-2 latches.
//   * Simultaneous double-rise coalesces into a single IF.7 bit set.
//   * Raise targets only the irq controller passed to tick() (NDS7-only).

#include "interrupt/irq_controller.hpp"
#include "require.hpp"
#include "rtc/rtc.hpp"

#include <cstdio>

using namespace ds;

// File-local BCD helper (Rtc::to_bcd is private).
static constexpr u8 bcd(u8 v) {
    return static_cast<u8>(((v / 10u) << 4) | (v % 10u));
}

// Helpers to install status2/alarm directly via the public test surface
// without round-tripping the SIO bit-bang in every test. We bit-bang only
// what's necessary to write registers; tick() and IRQ assertions read the
// observable state.
static constexpr u8 kDirAllCpuDriven = static_cast<u8>((1u << 4) | (1u << 5) | (1u << 6));

static void clock_bit_in(Rtc& rtc, u8 bit) {
    const u8 cs = static_cast<u8>(1u << 2);
    const u8 base = static_cast<u8>(kDirAllCpuDriven | cs | (bit & 1u));
    rtc.write_pins(base);
    rtc.write_pins(static_cast<u8>(base | (1u << 1)));
}

static void clock_byte_in(Rtc& rtc, u8 byte) {
    for (u8 i = 0; i < 8u; ++i) {
        clock_bit_in(rtc, static_cast<u8>((byte >> i) & 1u));
    }
}

static void assert_cs_high(Rtc& rtc) {
    rtc.write_pins(kDirAllCpuDriven);
    rtc.write_pins(static_cast<u8>(kDirAllCpuDriven | (1u << 2)));
}

static void write_status2(Rtc& rtc, u8 value) {
    assert_cs_high(rtc);
    clock_byte_in(rtc, 0x64u);
    clock_byte_in(rtc, value);
}

static void write_alarm1(Rtc& rtc, u8 dow, u8 hour, u8 min) {
    assert_cs_high(rtc);
    clock_byte_in(rtc, 0x61u);
    clock_byte_in(rtc, dow);
    clock_byte_in(rtc, hour);
    clock_byte_in(rtc, min);
}

static void write_alarm2(Rtc& rtc, u8 dow, u8 hour, u8 min) {
    assert_cs_high(rtc);
    clock_byte_in(rtc, 0x65u);
    clock_byte_in(rtc, dow);
    clock_byte_in(rtc, hour);
    clock_byte_in(rtc, min);
}

// ---- Test cases (per spec §6.5) ----

// Case 1: alarm enabled but dt is far from match — no raise.
static void Rtc_NoMatch_NoRaise() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 5, 0, 0});
    write_status2(r, 0x04u);
    // Alarm fires at 14:00; we're at 05:00:00 → tick to 05:00:01 (no match).
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);
    REQUIRE(!r.int1_latched());
}

// Case 2: alarm armed and exactly matching this tick — IF.7 raised.
static void Rtc_RisingEdge_RaisesIf7() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    write_status2(r, 0x04u);
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7); // → 14:00:00 — rising edge.
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    REQUIRE(r.int1_latched());
}

// Case 3: stay-matching window does not re-raise. Ack between ticks so we
// can tell whether the second tick re-raised IF.7.
static void Rtc_StaysMatching_DoesNotReRaise() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    write_status2(r, 0x04u);
    // Alarm matches whole minute 14:00:00..14:00:59 (only min/hour gated).
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7); // → 14:00:00 — rises.
    REQUIRE((irq7.read_if() & 0x80u) != 0u);

    // Ack IF.7 — must be 0 before next tick so we can distinguish re-raise.
    irq7.write_if(0x80u);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Tick to 14:00:01 — still matching, but prev_alarm1_match_ is true now.
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // And again at 14:00:02.
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);
}

// Case 4: 1→0 transition (match falls) does not raise. Status1 bit 4 stays
// latched until a Status1 read (we only verify "no raise this tick").
static void Rtc_FallingEdge_NoRaise() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 14, 0, 59}); // last second of the matching minute
    write_status2(r, 0x04u);
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    // First tick: 14:00:59 → 14:01:00 — match falls (min becomes 1).
    // prev_alarm1_match_ was false (just-seeded) so the body's path is:
    // a1_match = false → no raise → prev <- false. No edge either way.
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);
    REQUIRE(!r.int1_latched());

    // To exercise the explicit 1→0 fall: re-seed to mid-match and walk.
    r.seed({2026, 1, 1, 4, 14, 0, 30}); // matching second
    r.tick(irq7);                       // → 14:00:31 — rising edge → raise
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    irq7.write_if(0x80u);

    // Walk forward 29 ticks. After the rising-edge tick above, dt=14:00:31.
    // Ticks 1..28 advance sec to 32..59 — still matching minute=0 → no re-raise.
    // Tick 29 wraps sec 59→0 and bumps min 0→1 → falling edge → NO raise.
    for (u32 i = 0; i < 29u; ++i) {
        r.tick(irq7);
        REQUIRE((irq7.read_if() & 0x80u) == 0u);
    }
    REQUIRE(r.now_datetime().min == 1u);
    REQUIRE(r.now_datetime().sec == 0u);
    // Final state: 14:01:00 — the falling-edge tick (#29) did not raise.
    REQUIRE((irq7.read_if() & 0x80u) == 0u);
}

// Case 5: after a fall, a fresh rise raises again. Pattern: rise (ack), tick
// past match window into non-match, tick back into match → fresh rise.
static void Rtc_AfterFall_FreshRiseRaises() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    write_status2(r, 0x04u);
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7); // → 14:00:00 — rises.
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    irq7.write_if(0x80u);

    // Walk through the matching minute (sec 1..59) — stays matching.
    for (u32 i = 0; i < 59u; ++i) {
        r.tick(irq7);
    }
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Next tick: 14:00:59 → 14:01:00 — falling edge. prev becomes false here.
    r.tick(irq7);
    REQUIRE(r.now_datetime().min == 1u);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Now produce a fresh rise via re-seed back to 13:59:59 → tick → 14:00:00.
    // Re-seed clears prev (architect-locked behavior); the next tick rises.
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
}

// Case 6: alarm-1 fires while alarm-2 is enabled but not matching — only
// INT1 latches, IF.7 set exactly once.
static void Rtc_RaiseOnAlarm1_DoesNotAffectAlarm2_IFBit() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    // 0x44: Alarm-1 enable (0100b in low nibble) + Alarm-2 enable (bit 6).
    write_status2(r, 0x44u);
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));
    // Alarm-2 set to a different time so it doesn't match.
    write_alarm2(r, 0x00u, static_cast<u8>(0x80u | bcd(20u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7); // → 14:00:00 — alarm-1 rises, alarm-2 unchanged.
    REQUIRE(r.int1_latched());
    REQUIRE(!r.int2_latched());
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    // Bit-set is idempotent OR — only one raise happened anyway, but assert
    // no other bits got set as side-effect.
    REQUIRE(irq7.read_if() == 0x80u);
}

// Case 7: both alarms rise on the same tick — both INT bits latch, IF.7
// set (single bit; two raise() calls coalesce in OR).
static void Rtc_BothAlarms_RiseSimultaneously_BothLatch() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    write_status2(r, 0x44u);
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));
    write_alarm2(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7);
    REQUIRE(r.int1_latched());
    REQUIRE(r.int2_latched());
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    REQUIRE(irq7.read_if() == 0x80u);
}

// Case 8: raise reaches only the irq7 controller passed to tick(). A
// separately-constructed irq9 must remain untouched. (Rtc never has access
// to irq9; this confirms the architecture rule by construction.)
static void Rtc_RaiseDoesNotReachArm9() {
    Rtc r;
    r.reset();
    IrqController irq7;
    IrqController irq9; // independent
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    write_status2(r, 0x04u);
    write_alarm1(r, 0x00u, static_cast<u8>(0x80u | bcd(14u)), static_cast<u8>(0x80u | bcd(0u)));

    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    REQUIRE(irq9.read_if() == 0u); // IF.7 is NDS7-only (§5.8)
}

int main() {
    std::puts("[Rtc_NoMatch_NoRaise]");
    Rtc_NoMatch_NoRaise();
    std::puts("[Rtc_RisingEdge_RaisesIf7]");
    Rtc_RisingEdge_RaisesIf7();
    std::puts("[Rtc_StaysMatching_DoesNotReRaise]");
    Rtc_StaysMatching_DoesNotReRaise();
    std::puts("[Rtc_FallingEdge_NoRaise]");
    Rtc_FallingEdge_NoRaise();
    std::puts("[Rtc_AfterFall_FreshRiseRaises]");
    Rtc_AfterFall_FreshRiseRaises();
    std::puts("[Rtc_RaiseOnAlarm1_DoesNotAffectAlarm2_IFBit]");
    Rtc_RaiseOnAlarm1_DoesNotAffectAlarm2_IFBit();
    std::puts("[Rtc_BothAlarms_RiseSimultaneously_BothLatch]");
    Rtc_BothAlarms_RiseSimultaneously_BothLatch();
    std::puts("[Rtc_RaiseDoesNotReachArm9]");
    Rtc_RaiseDoesNotReachArm9();
    std::puts("OK");
    return 0;
}
