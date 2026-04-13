// tests/unit/scheduler_test.cpp
#include "scheduler/event.hpp"
#include "scheduler/scheduler.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void test_basic_schedule_peek() {
    Scheduler s;
    s.schedule_at(100, EventKind::FrameEnd);
    s.schedule_at(50,  EventKind::FrameEnd);
    s.schedule_at(200, EventKind::FrameEnd);
    REQUIRE(s.peek_next() == 50);
}

static void test_pop_order() {
    Scheduler s;
    s.schedule_at(100, EventKind::FrameEnd);
    s.schedule_at(50,  EventKind::FrameEnd);
    s.schedule_at(200, EventKind::FrameEnd);

    Event ev{};
    REQUIRE(s.pop_due(Scheduler::kNoEvent, ev) && ev.when == 50);
    REQUIRE(s.pop_due(Scheduler::kNoEvent, ev) && ev.when == 100);
    REQUIRE(s.pop_due(Scheduler::kNoEvent, ev) && ev.when == 200);
    REQUIRE(!s.pop_due(Scheduler::kNoEvent, ev));
}

static void test_same_cycle_fifo() {
    Scheduler s;
    const EventId a = s.schedule_at(100, EventKind::FrameEnd, 0xAA);
    const EventId b = s.schedule_at(100, EventKind::FrameEnd, 0xBB);
    REQUIRE(a < b);  // ids are monotonic

    Event ev{};
    REQUIRE(s.pop_due(100, ev));
    REQUIRE(ev.id == a);
    REQUIRE(ev.payload == 0xAA);

    REQUIRE(s.pop_due(100, ev));
    REQUIRE(ev.id == b);
    REQUIRE(ev.payload == 0xBB);
}

static void test_cancel_head() {
    Scheduler s;
    const EventId a = s.schedule_at(50,  EventKind::FrameEnd, 0xAA);
    s.cancel(a);
    s.schedule_at(100, EventKind::FrameEnd, 0xBB);

    Event ev{};
    REQUIRE(s.pop_due(Scheduler::kNoEvent, ev));
    REQUIRE(ev.when == 100);
    REQUIRE(ev.payload == 0xBB);
    REQUIRE(!s.pop_due(Scheduler::kNoEvent, ev));
}

static void test_cancel_mid_heap() {
    Scheduler s;
    s.schedule_at(50,  EventKind::FrameEnd, 0xAA);
    const EventId b = s.schedule_at(100, EventKind::FrameEnd, 0xBB);
    s.schedule_at(150, EventKind::FrameEnd, 0xCC);
    s.cancel(b);

    Event ev{};
    REQUIRE(s.pop_due(Scheduler::kNoEvent, ev));
    REQUIRE(ev.payload == 0xAA);
    REQUIRE(s.pop_due(Scheduler::kNoEvent, ev));
    REQUIRE(ev.payload == 0xCC);  // B was tombstoned; skipped silently
    REQUIRE(!s.pop_due(Scheduler::kNoEvent, ev));
}

static void test_schedule_in_uses_now() {
    Scheduler s;
    s.set_now(1000);
    s.schedule_in(500, EventKind::FrameEnd);

    Event ev{};
    REQUIRE(s.pop_due(1500, ev));
    REQUIRE(ev.when == 1500);
}

static void test_pop_due_boundary() {
    Scheduler s;
    s.schedule_at(100, EventKind::FrameEnd, 0xAA);
    s.schedule_at(200, EventKind::FrameEnd, 0xBB);

    Event ev{};
    REQUIRE(s.pop_due(100, ev));
    REQUIRE(ev.payload == 0xAA);
    REQUIRE(!s.pop_due(100, ev));   // B is in the future relative to target
    REQUIRE(s.pop_due(200, ev));
    REQUIRE(ev.payload == 0xBB);
}

static void test_reentrant_scheduling() {
    // Models the "event handler schedules a new event at the same cycle"
    // case NDS::run_frame's inner pop_due loop relies on.
    Scheduler s;
    s.schedule_at(100, EventKind::FrameEnd, 0xAA);

    Event ev{};
    REQUIRE(s.pop_due(100, ev));
    REQUIRE(ev.payload == 0xAA);

    s.schedule_at(100, EventKind::FrameEnd, 0xBB);

    REQUIRE(s.pop_due(100, ev));
    REQUIRE(ev.payload == 0xBB);
    REQUIRE(!s.pop_due(100, ev));
}

static void test_stress_ordering() {
    Scheduler s;
    // Linear congruential generator — deterministic without pulling <random>.
    u64 rng = 0xC0FFEE;
    auto next = [&rng]() -> Cycle {
        rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<Cycle>((rng >> 16) & 0xFFFFF);  // 20-bit `when`
    };

    constexpr int kN = 10000;
    for (int i = 0; i < kN; ++i) {
        s.schedule_at(next(), EventKind::FrameEnd, static_cast<u64>(i));
    }

    Event ev{};
    Cycle prev = 0;
    int popped = 0;
    while (s.pop_due(Scheduler::kNoEvent, ev)) {
        REQUIRE(ev.when >= prev);
        prev = ev.when;
        ++popped;
    }
    REQUIRE(popped == kN);
}

int main() {
    test_basic_schedule_peek();
    test_pop_order();
    test_same_cycle_fifo();
    test_cancel_head();
    test_cancel_mid_heap();
    test_schedule_in_uses_now();
    test_pop_due_boundary();
    test_reentrant_scheduling();
    test_stress_ordering();
    std::printf("scheduler_test: all passed\n");
    return 0;
}
