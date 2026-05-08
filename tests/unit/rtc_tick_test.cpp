// Exercises the RTC 1 Hz scheduler tick + calendar advance — slice 3j commit 5.
//
// Two layers:
//   * Direct Rtc tests drive `Rtc::tick(irq7)` against a default-constructed
//     IrqController and assert calendar cascade across every boundary
//     (sec→min, min→hour, hour→day+dow, non-leap Feb→Mar, leap Feb→Feb29 and
//     Feb29→Mar, Dec→Jan with year++, plus the Gregorian leap-year rule
//     covering 4-yearly, century, and 400-yearly cases).
//   * Scheduler-integration tests construct a full NDS and verify reset()
//     seeded the first Rtc1HzTick at `now + kArm9CyclesPerSecond`, and that
//     a handler invocation reschedules the next tick.
//
// Alarm comparison + IF.7 raise is *not* exercised here (commit 6 territory).

#include "nds.hpp"
#include "require.hpp"
#include "rtc/rtc.hpp"

#include <cstdio>

using namespace ds;

// Mirror of the nds.cpp-local constant. Not exported via header — the value
// is duplicated here intentionally so the test fails loudly if production
// drifts. Comment in nds.cpp pins both sides at 67,027,964.
static constexpr Cycle kArm9CyclesPerSecond = 67'027'964;

// ---- Direct Rtc::tick tests ----

static void tick_advances_seconds() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2026, 1, 1, 4, 0, 0, 0});
    for (u32 i = 0; i < 60u; ++i) {
        rtc.tick(irq7);
    }
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.min == 1u);
    REQUIRE(dt.sec == 0u);
    REQUIRE(dt.hour == 0u);
    REQUIRE(dt.day == 1u);
    REQUIRE(dt.month == 1u);
    REQUIRE(dt.year == 2026u);
}

static void tick_rollover_minutes_to_hours() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2026, 1, 1, 4, 0, 59, 59});
    rtc.tick(irq7);
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.hour == 1u);
    REQUIRE(dt.min == 0u);
    REQUIRE(dt.sec == 0u);
    REQUIRE(dt.day == 1u);
    REQUIRE(dt.dow == 4u); // unchanged — only day rollover advances dow
}

static void tick_rollover_hours_to_days_advances_dow() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2026, 1, 1, 3, 23, 59, 59});
    rtc.tick(irq7);
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.day == 2u);
    REQUIRE(dt.dow == 4u);
    REQUIRE(dt.hour == 0u);
    REQUIRE(dt.min == 0u);
    REQUIRE(dt.sec == 0u);
    REQUIRE(dt.month == 1u);
    REQUIRE(dt.year == 2026u);
}

static void tick_rollover_non_leap_feb_to_mar() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2026, 2, 28, 0, 23, 59, 59});
    rtc.tick(irq7);
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.year == 2026u);
    REQUIRE(dt.month == 3u);
    REQUIRE(dt.day == 1u);
    REQUIRE(dt.dow == 1u);
    REQUIRE(dt.hour == 0u);
    REQUIRE(dt.min == 0u);
    REQUIRE(dt.sec == 0u);
}

static void tick_rollover_leap_feb_to_feb29() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2024, 2, 28, 3, 23, 59, 59});
    rtc.tick(irq7);
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.year == 2024u);
    REQUIRE(dt.month == 2u);
    REQUIRE(dt.day == 29u);
    REQUIRE(dt.dow == 4u);
    REQUIRE(dt.hour == 0u);
}

static void tick_rollover_leap_feb29_to_mar1() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2024, 2, 29, 4, 23, 59, 59});
    rtc.tick(irq7);
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.year == 2024u);
    REQUIRE(dt.month == 3u);
    REQUIRE(dt.day == 1u);
    REQUIRE(dt.dow == 5u);
}

static void tick_rollover_dec_to_jan_year_advances() {
    Rtc rtc;
    rtc.reset();
    IrqController irq7;
    rtc.seed({2026, 12, 31, 4, 23, 59, 59});
    rtc.tick(irq7);
    const auto dt = rtc.now_datetime();
    REQUIRE(dt.year == 2027u);
    REQUIRE(dt.month == 1u);
    REQUIRE(dt.day == 1u);
    REQUIRE(dt.dow == 5u);
    REQUIRE(dt.hour == 0u);
    REQUIRE(dt.min == 0u);
    REQUIRE(dt.sec == 0u);
}

// Indirect coverage of the Gregorian rule via the calendar cascade:
// 2025 (not leap), 2100 (century, not /400 → not leap), 2400 (/400 → leap).
// 2024 leap is covered by tick_rollover_leap_feb_to_feb29 above.
static void is_leap_gregorian_rule() {
    {
        Rtc rtc;
        rtc.reset();
        IrqController irq7;
        rtc.seed({2025, 2, 28, 0, 23, 59, 59});
        rtc.tick(irq7);
        const auto dt = rtc.now_datetime();
        REQUIRE(dt.year == 2025u);
        REQUIRE(dt.month == 3u);
        REQUIRE(dt.day == 1u);
    }
    {
        Rtc rtc;
        rtc.reset();
        IrqController irq7;
        rtc.seed({2100, 2, 28, 0, 23, 59, 59});
        rtc.tick(irq7);
        const auto dt = rtc.now_datetime();
        REQUIRE(dt.year == 2100u);
        REQUIRE(dt.month == 3u);
        REQUIRE(dt.day == 1u);
    }
    {
        Rtc rtc;
        rtc.reset();
        IrqController irq7;
        rtc.seed({2400, 2, 28, 0, 23, 59, 59});
        rtc.tick(irq7);
        const auto dt = rtc.now_datetime();
        REQUIRE(dt.year == 2400u);
        REQUIRE(dt.month == 2u);
        REQUIRE(dt.day == 29u);
    }
}

// ---- Scheduler-integration tests ----

static void first_tick_scheduled_by_reset() {
    NDS nds;
    REQUIRE(nds.scheduler().peek_next() == kArm9CyclesPerSecond);
}

static void tick_reschedules_itself() {
    // Run frames until the reset-seeded tick at kArm9CyclesPerSecond fires
    // naturally. After the handler runs, exactly one Rtc1HzTick must remain
    // in the heap, scheduled at +kArm9CyclesPerSecond from the firing time.
    // Asserting against an injected immediate tick would leave the original
    // reset-seeded tick lingering and mask a duplicate-event regression.
    NDS nds;
    const u8 before_sec = nds.rtc().now_datetime().sec;
    while (nds.scheduler().now() < kArm9CyclesPerSecond) {
        nds.run_frame();
    }
    REQUIRE(nds.rtc().now_datetime().sec == static_cast<u8>(before_sec + 1u));
    REQUIRE(nds.scheduler().peek_next() == 2u * kArm9CyclesPerSecond);
}

int main() {
    tick_advances_seconds();
    tick_rollover_minutes_to_hours();
    tick_rollover_hours_to_days_advances_dow();
    tick_rollover_non_leap_feb_to_mar();
    tick_rollover_leap_feb_to_feb29();
    tick_rollover_leap_feb29_to_mar1();
    tick_rollover_dec_to_jan_year_advances();
    is_leap_gregorian_rule();
    first_tick_scheduled_by_reset();
    tick_reschedules_itself();
    std::printf("rtc_tick_test: OK\n");
    return 0;
}
