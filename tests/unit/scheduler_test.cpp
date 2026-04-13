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

int main() {
    test_basic_schedule_peek();
    test_pop_order();
    test_same_cycle_fifo();
    test_cancel_head();
    test_cancel_mid_heap();
    std::printf("scheduler_test: all passed\n");
    return 0;
}
