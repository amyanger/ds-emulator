// Exercises the RTC alarm comparison + rising-edge raise — slice 3j commit 6.
//
// Direct Rtc tests drive `Rtc::tick(irq7)` against a default-constructed
// IrqController and verify:
//   * `alarm_matches` honors the per-field compare-enable mask (bit 7).
//   * Status2 enable gating (bits 0..3 == 0100b for Alarm-1, bit 6 for
//     Alarm-2) suppresses raises when an alarm is disarmed.
//   * Status1 bits 4 (INT1) / 5 (INT2) latch on rising-edge match and
//     auto-clear when the CPU completes a Status1 read via bit-bang.
//
// IF.7 raise discipline (rising-edge, ack-then-tick) lives in
// rtc_irq_test.cpp — this file focuses on the chip-side alarm semantics.

#include "interrupt/irq_controller.hpp"
#include "require.hpp"
#include "rtc/rtc.hpp"

#include <cstdio>

using namespace ds;

// File-local BCD helper. Rtc::to_bcd is private; the encoding is trivial.
static constexpr u8 bcd(u8 v) {
    return static_cast<u8>(((v / 10u) << 4) | (v % 10u));
}

// ---- Bit-bang helpers (mirror rtc_commands_test.cpp; kept self-contained) ----

static constexpr u8 kDirAllCpuDriven = static_cast<u8>((1u << 4) | (1u << 5) | (1u << 6));
static constexpr u8 kDirChipDrivesSio = static_cast<u8>((1u << 5) | (1u << 6));

static void clock_bit_in(Rtc& rtc, u8 bit, bool dir_write_to_chip) {
    const u8 dir = dir_write_to_chip ? kDirAllCpuDriven : kDirChipDrivesSio;
    const u8 cs = static_cast<u8>(1u << 2);
    const u8 base = static_cast<u8>(dir | cs | (bit & 1u));
    rtc.write_pins(base);                              // /SCK low
    rtc.write_pins(static_cast<u8>(base | (1u << 1))); // /SCK rising
}

static void clock_byte_in(Rtc& rtc, u8 byte, bool dir_write_to_chip) {
    for (u8 i = 0; i < 8u; ++i) {
        clock_bit_in(rtc, static_cast<u8>((byte >> i) & 1u), dir_write_to_chip);
    }
}

static void assert_cs_high(Rtc& rtc) {
    rtc.write_pins(kDirAllCpuDriven);                              // /CS = 0
    rtc.write_pins(static_cast<u8>(kDirAllCpuDriven | (1u << 2))); // /CS rising
}

static u8 read_byte_out(Rtc& rtc) {
    u8 byte = 0;
    for (u8 i = 0; i < 8u; ++i) {
        rtc.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2)));
        const u8 bit = static_cast<u8>(rtc.read_pins() & 1u);
        byte = static_cast<u8>(byte | (bit << i));
        rtc.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2) | (1u << 1)));
    }
    return byte;
}

// ---- Test cases (per spec §6.3) ----

// Case 1: alarm with all three compare-enable bits 0 ("don't care") matches at
// every dt. Verified by direct calls to tick() at three unrelated dt seeds.
// alarm_matches() is private; we observe its result indirectly via the
// rising-edge raise: with alarm enabled and matches=true on every tick, only
// the FIRST tick after seed (which clears prev) produces a rising edge.
static void Rtc_AlarmCmpEnableMask_AllZero_AlwaysMatches() {
    Rtc r;
    r.reset();
    IrqController irq7;
    // status2 = 0x04 enables Alarm-1; alarm fields all zero (no compare bits).
    r.seed({2026, 1, 1, 4, 0, 0, 0});
    // Write status2 = 0x04 via bit-bang. Alarm registers stay at default (0,0,0).
    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true); // Write Status2
    clock_byte_in(r, 0x04u, true); // Alarm-1 enable

    // First tick: dt advances to {0, 0, 1}. With all-zero alarm, matches=true.
    // prev was cleared by seed() → rising edge → IF.7 raised.
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);

    // Ack and tick again at unrelated dt — re-seed clears prev.
    irq7.write_if(0x80u);
    r.seed({2026, 6, 15, 1, 14, 30, 45});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);

    // Third unrelated seed.
    irq7.write_if(0x80u);
    r.seed({2030, 12, 31, 2, 23, 59, 0});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
}

// Case 2: alarm = {0x00, 0x00, 0x80 | bcd(30)} matches once per hour.
// Tick across one full hour and count rising edges (ack-then-tick discipline).
static void Rtc_AlarmCmpEnableMask_OnlyMin_MatchesEveryHour() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 10, 0, 0});

    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true); // Write Status2
    clock_byte_in(r, 0x04u, true); // Alarm-1 enable

    assert_cs_high(r);
    clock_byte_in(r, 0x61u, true);                             // Write Alarm1
    clock_byte_in(r, 0x00u, true);                             // dow: don't care
    clock_byte_in(r, 0x00u, true);                             // hour: don't care
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(30u)), true); // min=30, cmp-enable

    // Walk one hour (3600 seconds). Count rising edges by observing IF.7
    // transitions: ack each rise immediately so a stay-matching window
    // (sec 0..59 within minute 30) cannot mask a re-raise bug.
    u32 raise_count = 0;
    for (u32 s = 0; s < 3600u; ++s) {
        r.tick(irq7);
        if ((irq7.read_if() & 0x80u) != 0u) {
            ++raise_count;
            irq7.write_if(0x80u);
        }
    }
    REQUIRE(raise_count == 1u);

    // Confirm we landed at hour 11, minute 0 (seed was 10:00:00, +3600 s).
    const auto dt = r.now_datetime();
    REQUIRE(dt.hour == 11u);
    REQUIRE(dt.min == 0u);
    REQUIRE(dt.sec == 0u);
}

// Case 3: alarm = {0x00, 0x80|bcd(14), 0x80|bcd(0)} matches once per day at
// 14:00. Full 24-hour walk would be 86,400 ticks — instead exercise the
// per-day boundary at strategic seeds: rise at 14:00, no re-raise at 14:01,
// re-seed back to 13:59:59 (clears prev) and confirm a fresh rise on the
// next tick (which advances to 14:00:00). Together these prove "exactly one
// rise per match-window per day".
static void Rtc_AlarmCmpEnableMask_HourPlusMin_MatchesOncePerDay() {
    Rtc r;
    r.reset();
    IrqController irq7;
    // Seed at 13:59:59 — next tick lands on 14:00:00 (rising edge).
    r.seed({2026, 1, 1, 4, 13, 59, 59});

    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true);
    clock_byte_in(r, 0x04u, true);

    assert_cs_high(r);
    clock_byte_in(r, 0x61u, true);
    clock_byte_in(r, 0x00u, true);                             // dow: don't care
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(14u)), true); // hour=14
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(0u)), true);  // min=0

    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
    REQUIRE(r.now_datetime().hour == 14u);
    REQUIRE(r.now_datetime().min == 0u);

    // Stay-matching: tick into 14:00:01 — still matching (min=0, hour=14),
    // but no rising edge because prev_alarm1_match_ is now true.
    irq7.write_if(0x80u);
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Tick once more — 14:00:02, still match, still no re-raise.
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Re-seed back to 13:59:59. seed() clears prev_alarm*_match_ — this is
    // the architect-locked invariant that lets the test stay reasonable
    // without simulating 86,400 ticks. Spec §4.6 + architect note.
    r.seed({2026, 1, 2, 5, 13, 59, 59});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
}

// Case 4: alarm = {0x80|3, 0x80|bcd(9), 0x80|bcd(0)} matches Wed 09:00 only.
// "Once per week" property: rise on Wed 09:00, no rise at 09:00 on other
// dows. Spans 7 days — too many ticks to simulate; assert at strategic
// dt seeds. seed() clears prev each time (architect-locked behavior).
static void Rtc_AlarmCmpEnableMask_AllThree_MatchesOncePerWeek() {
    Rtc r;
    r.reset();
    IrqController irq7;

    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true);
    clock_byte_in(r, 0x04u, true);

    assert_cs_high(r);
    clock_byte_in(r, 0x61u, true);
    clock_byte_in(r, static_cast<u8>(0x80u | 3u), true);      // dow=3 (Wed)
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(9u)), true); // hour=9
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(0u)), true); // min=0

    // Wed 08:59:59 → tick → Wed 09:00:00 (match). Note: dow=3 in DateTime
    // is preserved across the second cascade (no day rollover here).
    r.seed({2026, 1, 7, 3, 8, 59, 59});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);

    // Same time on Thursday (dow=4): no match because dow compare-enable
    // requires equality. seed() clears prev, so we'd see a rising edge if
    // the dow gate weren't enforced.
    irq7.write_if(0x80u);
    r.seed({2026, 1, 8, 4, 8, 59, 59});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Tuesday (dow=2) at 09:00 — also no match.
    r.seed({2026, 1, 6, 2, 8, 59, 59});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);

    // Following Wednesday: re-rise. Confirms the "once per week" property.
    r.seed({2026, 1, 14, 3, 8, 59, 59});
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
}

// Case 5: alarm fully matching but status2 bits 0..3 ≠ 0100b → no raise.
// Tries values 0x00 (disabled), 0x05 (non-enable code), 0x0F (all bits set
// but != 0x04). Bit 6 (alarm-2) is irrelevant for alarm-1 gating.
static void Rtc_Status2EnableOff_NoRaise_Once(u8 status2) {
    Rtc r;
    r.reset();
    IrqController irq7;
    // dt + alarm such that alarm_matches would return true (all-don't-care).
    r.seed({2026, 1, 1, 4, 0, 0, 0});

    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true);
    clock_byte_in(r, status2, true);

    // Default alarm1 (all zero — would always match if enabled).
    r.tick(irq7);
    REQUIRE((irq7.read_if() & 0x80u) == 0u);
    REQUIRE(!r.int1_latched());
}

static void Rtc_Status2EnableOff_NoRaise() {
    Rtc_Status2EnableOff_NoRaise_Once(0x00u); // disabled
    Rtc_Status2EnableOff_NoRaise_Once(0x05u); // non-enable code
    Rtc_Status2EnableOff_NoRaise_Once(0x0Fu); // all bits set but != 0x04
}

// Case 6: status1 bit 4 latches on rising-edge match. A complete bit-bang
// Status1 read clears bit 4. After fall→rise, bit 4 sets again.
static void Rtc_Status1Bit4_LatchesOnRaise_ClearsOnRead() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});

    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true);
    clock_byte_in(r, 0x04u, true);

    assert_cs_high(r);
    clock_byte_in(r, 0x61u, true);
    clock_byte_in(r, 0x00u, true);
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(14u)), true);
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(0u)), true);

    // Rising edge → bit 4 latches.
    r.tick(irq7);
    REQUIRE(r.int1_latched());
    REQUIRE((r.status1() & 0x10u) != 0u);

    // Bit-bang Status1 read — last bit out triggers auto-clear of bits 4..7.
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true); // Read Status1
    const u8 byte = read_byte_out(r);
    REQUIRE((byte & 0x10u) != 0u); // saw the latched bit before clear
    REQUIRE(!r.int1_latched());
    REQUIRE((r.status1() & 0x10u) == 0u);

    // Fall: re-seed to 14:01:30 — match=false (min=1, alarm wants min=0). prev
    // gets cleared by seed() so the next match is a fresh rising edge.
    r.seed({2026, 1, 1, 4, 14, 1, 30});
    r.tick(irq7);
    REQUIRE(!r.int1_latched());

    // Rise again: re-seed to 13:59:59, tick → 14:00:00 → bit 4 set.
    r.seed({2026, 1, 1, 4, 13, 59, 59});
    r.tick(irq7);
    REQUIRE(r.int1_latched());
    REQUIRE((r.status1() & 0x10u) != 0u);
}

// Case 7: when both alarms rise on the same tick, status1 bits 4 AND 5 both
// latch independently. The two raise() calls into IF.7 coalesce in OR.
static void Rtc_Alarm2_IndependentLatch_Status1Bits() {
    Rtc r;
    r.reset();
    IrqController irq7;
    r.seed({2026, 1, 1, 4, 13, 59, 59});

    // Status2 = 0x44: bits 0..3 = 0100b (Alarm-1) + bit 6 (Alarm-2 enable).
    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true);
    clock_byte_in(r, 0x44u, true);

    // Alarm 1 and Alarm 2 both fire on hour=14 min=0 (same trigger).
    assert_cs_high(r);
    clock_byte_in(r, 0x61u, true);
    clock_byte_in(r, 0x00u, true);
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(14u)), true);
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(0u)), true);

    assert_cs_high(r);
    clock_byte_in(r, 0x65u, true); // Write Alarm 2
    clock_byte_in(r, 0x00u, true);
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(14u)), true);
    clock_byte_in(r, static_cast<u8>(0x80u | bcd(0u)), true);

    r.tick(irq7);
    REQUIRE(r.int1_latched());
    REQUIRE(r.int2_latched());
    REQUIRE((r.status1() & 0x30u) == 0x30u); // bits 4 AND 5 set
    REQUIRE((irq7.read_if() & 0x80u) != 0u);
}

int main() {
    std::puts("[Rtc_AlarmCmpEnableMask_AllZero_AlwaysMatches]");
    Rtc_AlarmCmpEnableMask_AllZero_AlwaysMatches();
    std::puts("[Rtc_AlarmCmpEnableMask_OnlyMin_MatchesEveryHour]");
    Rtc_AlarmCmpEnableMask_OnlyMin_MatchesEveryHour();
    std::puts("[Rtc_AlarmCmpEnableMask_HourPlusMin_MatchesOncePerDay]");
    Rtc_AlarmCmpEnableMask_HourPlusMin_MatchesOncePerDay();
    std::puts("[Rtc_AlarmCmpEnableMask_AllThree_MatchesOncePerWeek]");
    Rtc_AlarmCmpEnableMask_AllThree_MatchesOncePerWeek();
    std::puts("[Rtc_Status2EnableOff_NoRaise]");
    Rtc_Status2EnableOff_NoRaise();
    std::puts("[Rtc_Status1Bit4_LatchesOnRaise_ClearsOnRead]");
    Rtc_Status1Bit4_LatchesOnRaise_ClearsOnRead();
    std::puts("[Rtc_Alarm2_IndependentLatch_Status1Bits]");
    Rtc_Alarm2_IndependentLatch_Status1Bits();
    std::puts("OK");
    return 0;
}
