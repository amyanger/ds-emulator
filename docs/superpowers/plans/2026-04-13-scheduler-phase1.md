# Scheduler (Phase 1, slice 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Phase 0 scheduler stub with a real min-heap + tombstone implementation and wire it into `NDS::run_frame` via a `peek_next → run_until → set_now → pop_due` loop.

**Architecture:** The scheduler is a pure data structure with zero knowledge of `NDS` — no callback, no dispatch. `NDS::run_frame` owns the drain loop and routes popped events through one `switch` in `NDS::on_scheduler_event`. This keeps the scheduler trivially unit-testable and preserves the "no subsystem points back at another subsystem" tree invariant from CLAUDE.md rule #3.

**Tech Stack:** C++20, `std::vector` min-heap via `std::push_heap`/`std::pop_heap`, `std::unordered_set<u64>` for tombstones, CMake + CTest for the test harness, no third-party deps.

**Spec:** `docs/superpowers/specs/2026-04-13-scheduler-phase1-design.md`
**Parent spec:** `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` §9

---

## Context for the engineer

- **Read CLAUDE.md first.** The "Architecture Rules (non-negotiable)" and "Code Style" sections are binding. Critical rules for this plan:
  - Rule 1: the scheduler is the clock.
  - Rule 3: no subsystem points back at another subsystem.
  - Rule 6: NDS core has no `<SDL.h>` in its include graph.
  - 4-space indent, pointer alignment left (`uint8_t* p`), fixed-width types (use `u32`/`u64` from `ds/common.hpp`).
  - Compiler warnings treated as errors in spirit (`-Wall -Wextra -Wpedantic -Werror`).
- **Read the spec** (`docs/superpowers/specs/2026-04-13-scheduler-phase1-design.md`). Every design decision is there. If you find yourself wanting to deviate, stop and ask — don't silently change the API shape.
- **Build from `build/`**, not the repo root:
  ```bash
  cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
  ```
- **Do not commit** `CLAUDE.md` — it is gitignored as local-only context.
- **Do not add Claude (or any AI) as a co-author** on any commit. No `Co-Authored-By` lines.
- **Do not amend previous commits.** When a step fails, fix and create a *new* commit.

---

## File structure

### Files created

| Path | Responsibility |
|---|---|
| `tests/support/require.hpp` | Tiny Release-safe `REQUIRE(cond)` assertion macro shared by all unit tests. |
| `src/scheduler/event.hpp` | `EventKind` enum, `Event` struct, `EventId` alias. Pure data. |
| `tests/unit/scheduler_test.cpp` | Unit tests for the scheduler. |

### Files modified

| Path | Reason |
|---|---|
| `tests/unit/fixed_test.cpp` | Switch bare `assert` → `REQUIRE`. |
| `tests/unit/ring_buffer_test.cpp` | Switch bare `assert` → `REQUIRE`. |
| `tests/CMakeLists.txt` | Add `tests/support` to include path, register `scheduler_test`. |
| `src/scheduler/scheduler.hpp` | Replace Phase 0 stub class with the full API. |
| `src/scheduler/scheduler.cpp` | Replace empty TU with the real heap implementation. |
| `src/nds.hpp` | Add `on_scheduler_event(const Event&)` and `#include` for `event.hpp`. |
| `src/nds.cpp` | Rewrite `run_frame` to the event-loop shape. |
| `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` | §9.1 and §9.4 amendments (drop `fire()`, relocate the dispatch switch). |

---

## Task 1: Release-safe REQUIRE macro and retrofit existing tests

Current unit tests use bare `<cassert>` which is a no-op under `-DNDEBUG`. The scheduler test cannot be trusted in Release without fixing this. Fix it first, retrofit the two existing test files in the same pass, and confirm they still pass.

**Files:**
- Create: `tests/support/require.hpp`
- Modify: `tests/CMakeLists.txt`
- Modify: `tests/unit/fixed_test.cpp`
- Modify: `tests/unit/ring_buffer_test.cpp`

- [ ] **Step 1: Create the REQUIRE macro header.**

Write `tests/support/require.hpp` exactly as follows:

```cpp
#pragma once

// Release-safe test assertion. Unlike <cassert>, this survives -DNDEBUG,
// which is the mode ctest will run tests in if the build is Release.

#include <cstdio>
#include <cstdlib>

#define REQUIRE(cond)                                                                   \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            std::fprintf(stderr,                                                        \
                         "REQUIRE failed: %s\n  at %s:%d\n",                            \
                         #cond, __FILE__, __LINE__);                                    \
            std::exit(1);                                                               \
        }                                                                               \
    } while (0)
```

- [ ] **Step 2: Wire the support dir into the test include path.**

Edit `tests/CMakeLists.txt`. Add `${CMAKE_SOURCE_DIR}/tests/support` to the `target_include_directories` call inside `add_ds_unit_test`:

```cmake
# ds_tests — one binary per test source file, each links ds_core only.
# No SDL dependency, runs in milliseconds.

function(add_ds_unit_test name)
    add_executable(${name} unit/${name}.cpp)
    target_link_libraries(${name} PRIVATE ds_core)
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/tests/support
    )
    target_enable_warnings(${name})
    target_apple_silicon(${name})
    target_enable_sanitizers(${name})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_ds_unit_test(fixed_test)
add_ds_unit_test(ring_buffer_test)
```

- [ ] **Step 3: Retrofit `fixed_test.cpp`.**

Replace the entire file contents with:

```cpp
// tests/unit/fixed_test.cpp
#include "ds/fixed.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void test_construct_and_to_int() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(5);
    REQUIRE(a.to_int() == 5);

    Fixed<20, 12> b = Fixed<20, 12>::from_raw(4096);  // 1.0 in 20.12
    REQUIRE(b.to_int() == 1);
}

static void test_addition() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(3);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(4);
    Fixed<20, 12> c = a + b;
    REQUIRE(c.to_int() == 7);
}

static void test_subtraction() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(10);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(3);
    Fixed<20, 12> c = a - b;
    REQUIRE(c.to_int() == 7);
}

static void test_multiplication() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(6);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(7);
    Fixed<20, 12> c = a * b;
    REQUIRE(c.to_int() == 42);
}

static void test_multiplication_fractional() {
    // 1.5 * 2.0 = 3.0
    Fixed<20, 12> a = Fixed<20, 12>::from_raw(1 << 12 | 1 << 11);  // 1.5
    Fixed<20, 12> b = Fixed<20, 12>::from_int(2);
    Fixed<20, 12> c = a * b;
    REQUIRE(c.to_int() == 3);
}

static void test_negative() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(-5);
    REQUIRE(a.to_int() == -5);
    Fixed<20, 12> b = -a;
    REQUIRE(b.to_int() == 5);
}

static void test_raw_roundtrip() {
    const int32_t raw = 0x12345;
    Fixed<20, 12> a = Fixed<20, 12>::from_raw(raw);
    REQUIRE(a.raw() == raw);
}

int main() {
    test_construct_and_to_int();
    test_addition();
    test_subtraction();
    test_multiplication();
    test_multiplication_fractional();
    test_negative();
    test_raw_roundtrip();
    std::printf("fixed_test: all passed\n");
    return 0;
}
```

- [ ] **Step 4: Retrofit `ring_buffer_test.cpp`.**

Replace the entire file contents with:

```cpp
// tests/unit/ring_buffer_test.cpp
#include "ds/ring_buffer.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void test_empty_new() {
    CircularBuffer<int, 4> rb;
    REQUIRE(rb.size() == 0);
    REQUIRE(rb.empty());
    REQUIRE(!rb.full());
}

static void test_push_and_size() {
    CircularBuffer<int, 4> rb;
    rb.push(10);
    REQUIRE(rb.size() == 1);
    REQUIRE(!rb.empty());
    rb.push(20);
    rb.push(30);
    REQUIRE(rb.size() == 3);
}

static void test_pop_fifo_order() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    REQUIRE(rb.pop() == 1);
    REQUIRE(rb.pop() == 2);
    REQUIRE(rb.pop() == 3);
    REQUIRE(rb.empty());
}

static void test_full() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    REQUIRE(rb.full());
    REQUIRE(rb.size() == 4);
}

static void test_wrap_around() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2); rb.push(3); rb.push(4);
    REQUIRE(rb.pop() == 1);
    REQUIRE(rb.pop() == 2);
    rb.push(5);
    rb.push(6);
    REQUIRE(rb.pop() == 3);
    REQUIRE(rb.pop() == 4);
    REQUIRE(rb.pop() == 5);
    REQUIRE(rb.pop() == 6);
    REQUIRE(rb.empty());
}

static void test_clear() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2);
    rb.clear();
    REQUIRE(rb.empty());
    REQUIRE(rb.size() == 0);
}

int main() {
    test_empty_new();
    test_push_and_size();
    test_pop_fifo_order();
    test_full();
    test_wrap_around();
    test_clear();
    std::printf("ring_buffer_test: all passed\n");
    return 0;
}
```

- [ ] **Step 5: Build and run tests in Debug to confirm parity.**

```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make fixed_test ring_buffer_test && ctest --output-on-failure -R 'fixed_test|ring_buffer_test'
```

Expected output (end of run):
```
100% tests passed, 0 tests failed out of 2
```

- [ ] **Step 6: Repeat the verification under Release to prove the macro survives `-DNDEBUG`.**

```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make fixed_test ring_buffer_test && ctest --output-on-failure -R 'fixed_test|ring_buffer_test'
```

Expected: `100% tests passed, 0 tests failed out of 2`. If either test reports "all passed" without actually running checks, the `REQUIRE` macro is wrong — go back and fix `tests/support/require.hpp` before moving on.

- [ ] **Step 7: Rebuild Debug for the rest of the plan.**

```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
```

- [ ] **Step 8: Commit.**

```bash
git add tests/support/require.hpp tests/CMakeLists.txt tests/unit/fixed_test.cpp tests/unit/ring_buffer_test.cpp
git commit -m "tests: Release-safe REQUIRE macro, retrofit existing tests

Bare <cassert> becomes a no-op under -DNDEBUG, which means any Release
build of the test suite would silently pass. Introduce a tiny REQUIRE
macro that works in both modes and retrofit fixed_test and
ring_buffer_test before scheduler_test lands on the same plumbing."
```

---

## Task 2: Scheduler core — event type, min-heap, `pop_due`, tests 1–3

Introduce `EventKind` / `Event`, replace the Phase 0 stub scheduler with the real implementation (schedule/peek/pop/set_now/reset — no cancel, no schedule_in yet), and land the first three test cases covering ordering and same-cycle FIFO.

**Files:**
- Create: `src/scheduler/event.hpp`
- Create: `tests/unit/scheduler_test.cpp`
- Modify: `src/scheduler/scheduler.hpp`
- Modify: `src/scheduler/scheduler.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Create `src/scheduler/event.hpp`.**

```cpp
#pragma once

#include "ds/common.hpp"

namespace ds {

// One enum, one switch, one handler per case in nds.cpp. Grows as
// subsystems land; Phase 1 slice 1 only has FrameEnd.
enum class EventKind : u32 {
    FrameEnd,
};

using EventId = u64;

struct Event {
    Cycle     when;    // ARM9 cycles since power-on
    EventKind kind;
    u64       payload; // event-kind-specific opaque data; unused for FrameEnd
    EventId   id;      // monotonic, assigned by Scheduler
};

}  // namespace ds
```

- [ ] **Step 2: Rewrite `src/scheduler/scheduler.hpp`.**

Replace the entire file with:

```cpp
#pragma once

#include "ds/common.hpp"
#include "scheduler/event.hpp"

#include <limits>
#include <unordered_set>
#include <vector>

namespace ds {

// Pure data structure. Holds no reference to NDS and dispatches nothing.
// NDS::run_frame drives the `peek_next -> run_until -> set_now -> pop_due`
// loop and routes popped events through NDS::on_scheduler_event.
class Scheduler {
public:
    static constexpr Cycle kNoEvent = std::numeric_limits<Cycle>::max();

    Scheduler() = default;

    void reset();

    EventId schedule_at(Cycle when, EventKind kind, u64 payload = 0);

    Cycle now() const { return now_; }
    void  set_now(Cycle t);

    // Returns the `when` of the earliest live event, or kNoEvent if none.
    // Non-const because it lazily drops tombstoned events from the heap head.
    Cycle peek_next();

    // Pops the earliest live event if its `when` is <= target. Returns true
    // and writes the event to `out` on success, false if the heap is empty
    // or the next live event is in the future.
    bool  pop_due(Cycle target, Event& out);

private:
    struct HeapCmp {
        // std::push_heap / std::pop_heap use operator() as a "less", producing
        // a max-heap of that order. Invert for a min-heap keyed on (when, id).
        bool operator()(const Event& a, const Event& b) const {
            if (a.when != b.when) return a.when > b.when;
            return a.id > b.id;
        }
    };

    std::vector<Event>          heap_;
    std::unordered_set<EventId> cancelled_;
    Cycle                       now_     = 0;
    EventId                     next_id_ = 1;

    // Drops tombstoned events from the top of the heap until heap_[0] is
    // live or the heap is empty. Shared by peek_next and pop_due.
    void drop_head_tombstones();
};

}  // namespace ds
```

- [ ] **Step 3: Rewrite `src/scheduler/scheduler.cpp` with just enough to satisfy the header.**

```cpp
#include "scheduler/scheduler.hpp"

#include <algorithm>
#include <cassert>

namespace ds {

void Scheduler::reset() {
    heap_.clear();
    cancelled_.clear();
    now_     = 0;
    next_id_ = 1;
}

EventId Scheduler::schedule_at(Cycle when, EventKind kind, u64 payload) {
    const EventId id = next_id_++;
    heap_.push_back(Event{when, kind, payload, id});
    std::push_heap(heap_.begin(), heap_.end(), HeapCmp{});
    return id;
}

void Scheduler::set_now(Cycle t) {
    assert(t >= now_ && "Scheduler::set_now is monotonic");
    now_ = t;
}

void Scheduler::drop_head_tombstones() {
    while (!heap_.empty() && cancelled_.count(heap_.front().id) != 0) {
        cancelled_.erase(heap_.front().id);
        std::pop_heap(heap_.begin(), heap_.end(), HeapCmp{});
        heap_.pop_back();
    }
}

Cycle Scheduler::peek_next() {
    drop_head_tombstones();
    return heap_.empty() ? kNoEvent : heap_.front().when;
}

bool Scheduler::pop_due(Cycle target, Event& out) {
    drop_head_tombstones();
    if (heap_.empty() || heap_.front().when > target) {
        return false;
    }
    std::pop_heap(heap_.begin(), heap_.end(), HeapCmp{});
    out = heap_.back();
    heap_.pop_back();
    return true;
}

}  // namespace ds
```

`cancel()` and `schedule_in()` land in Tasks 3 and 4 — leaving them out now keeps this commit focused.

- [ ] **Step 4: Create `tests/unit/scheduler_test.cpp` with the first three cases.**

```cpp
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

int main() {
    test_basic_schedule_peek();
    test_pop_order();
    test_same_cycle_fifo();
    std::printf("scheduler_test: all passed\n");
    return 0;
}
```

- [ ] **Step 5: Register `scheduler_test` in `tests/CMakeLists.txt`.**

Append one line after the existing `add_ds_unit_test` calls:

```cmake
add_ds_unit_test(fixed_test)
add_ds_unit_test(ring_buffer_test)
add_ds_unit_test(scheduler_test)
```

- [ ] **Step 6: Verify the failing state before implementing more.**

Before any of this compiles, `NDS::run_frame` still calls the Phase 0 `scheduler_.advance_to(target)` and the Phase 0 scheduler header had a `reset()` method but no `set_now`. Our new header has `set_now` and no `advance_to`. That means `nds.cpp` will break compilation. We'll fix `nds.cpp` in Task 5 — for this task, we rely on the fact that `scheduler_test.cpp` is a standalone binary that links only against `ds_core`, and `ds_core` still compiles because `nds.cpp` only uses `Scheduler` via `run_frame`/`reset`, both of which need updating.

Easiest path: **also update `nds.cpp` minimally in this task** so the full build stays green. Specifically, change the single line `scheduler_.advance_to(target);` to `scheduler_.set_now(target);` in `src/nds.cpp`. Leave the rest of `run_frame` alone (full rewrite happens in Task 5).

```cpp
// src/nds.cpp — minimal edit: scheduler_.advance_to(target) -> scheduler_.set_now(target)
void NDS::run_frame() {
    const Cycle target = scheduler_.now() + kFrameCycles;
    cpu9_.run_until(target);
    cpu7_.run_until(target);
    scheduler_.set_now(target);
    frame_done_ = true;
}
```

- [ ] **Step 7: Build everything.**

```bash
cd build && make
```

Expected: clean build, no warnings treated as errors.

- [ ] **Step 8: Run the scheduler test.**

```bash
cd build && ctest --output-on-failure -R scheduler_test
```

Expected:
```
scheduler_test: all passed
1/1 Test #1: scheduler_test ..................   Passed    0.00 sec
100% tests passed, 0 tests failed out of 1
```

- [ ] **Step 9: Run all tests to confirm nothing else regressed.**

```bash
cd build && ctest --output-on-failure
```

Expected: `3/3 tests passed`.

- [ ] **Step 10: Commit.**

```bash
git add src/scheduler/event.hpp src/scheduler/scheduler.hpp src/scheduler/scheduler.cpp tests/unit/scheduler_test.cpp tests/CMakeLists.txt src/nds.cpp
git commit -m "scheduler: min-heap core with schedule_at/peek_next/pop_due

Replaces the Phase 0 clock-counter stub with a real std::vector min-heap
keyed on (when, id). Ordering, reverse-order insertion, and same-cycle
FIFO are covered by scheduler_test cases 1-3. NDS::run_frame gets a
one-line edit to call set_now instead of the removed advance_to; the
full run_frame rewrite lands in Task 5."
```

---

## Task 3: `cancel()` with tombstones — tests 4 and 5

Add cancellation. Tombstoned events stay in the heap until popped; the head-drop helper already exists from Task 2, so this is mostly `cancel()` plus two test cases.

**Files:**
- Modify: `src/scheduler/scheduler.hpp`
- Modify: `src/scheduler/scheduler.cpp`
- Modify: `tests/unit/scheduler_test.cpp`

- [ ] **Step 1: Add the `cancel` declaration to `scheduler.hpp`.**

Insert after `schedule_at`:

```cpp
    EventId schedule_at(Cycle when, EventKind kind, u64 payload = 0);

    // Mark an event as cancelled. Idempotent; unknown ids are a no-op.
    // The event stays in the heap as a tombstone until popped.
    void cancel(EventId id);
```

- [ ] **Step 2: Implement `cancel` in `scheduler.cpp`.**

Add below `schedule_at`:

```cpp
void Scheduler::cancel(EventId id) {
    cancelled_.insert(id);
}
```

Single insert — the `drop_head_tombstones` helper from Task 2 already handles skipping and cleanup.

- [ ] **Step 3: Add tests 4 and 5 to `scheduler_test.cpp`.**

Insert after `test_same_cycle_fifo`:

```cpp
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
```

And wire them into `main`:

```cpp
int main() {
    test_basic_schedule_peek();
    test_pop_order();
    test_same_cycle_fifo();
    test_cancel_head();
    test_cancel_mid_heap();
    std::printf("scheduler_test: all passed\n");
    return 0;
}
```

- [ ] **Step 4: Build and run.**

```bash
cd build && make scheduler_test && ctest --output-on-failure -R scheduler_test
```

Expected: `scheduler_test: all passed`, `1/1 tests passed`.

- [ ] **Step 5: Commit.**

```bash
git add src/scheduler/scheduler.hpp src/scheduler/scheduler.cpp tests/unit/scheduler_test.cpp
git commit -m "scheduler: tombstone-based cancel()

cancel(id) marks the id; drop_head_tombstones skips and cleans lazily
when peek_next or pop_due reaches the tombstoned entry. Covers the
cancel-head and cancel-mid-heap cases from the spec test plan."
```

---

## Task 4: `schedule_in` and the remaining test cases

Add the `schedule_in` convenience method, plus the remaining four test cases: `schedule_in` base, `pop_due` target-boundary, re-entrant scheduling mid-drain, and a 10k-event deterministic stress sweep.

**Files:**
- Modify: `src/scheduler/scheduler.hpp`
- Modify: `src/scheduler/scheduler.cpp`
- Modify: `tests/unit/scheduler_test.cpp`

- [ ] **Step 1: Declare `schedule_in` in `scheduler.hpp`.**

Insert immediately after `schedule_at`:

```cpp
    EventId schedule_at(Cycle when, EventKind kind, u64 payload = 0);
    EventId schedule_in(Cycle delta, EventKind kind, u64 payload = 0);
```

- [ ] **Step 2: Implement `schedule_in` in `scheduler.cpp`.**

Add below `schedule_at`:

```cpp
EventId Scheduler::schedule_in(Cycle delta, EventKind kind, u64 payload) {
    return schedule_at(now_ + delta, kind, payload);
}
```

- [ ] **Step 3: Add the remaining test cases.**

Insert after `test_cancel_mid_heap`:

```cpp
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
```

Wire them into `main`:

```cpp
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
```

- [ ] **Step 4: Build and run.**

```bash
cd build && make scheduler_test && ctest --output-on-failure -R scheduler_test
```

Expected: `scheduler_test: all passed`, `1/1 tests passed`. If the stress test reports an ordering violation, the `HeapCmp` comparator direction is wrong — re-check Step 2 of Task 2.

- [ ] **Step 5: Commit.**

```bash
git add src/scheduler/scheduler.hpp src/scheduler/scheduler.cpp tests/unit/scheduler_test.cpp
git commit -m "scheduler: schedule_in + boundary/re-entrant/stress tests

Completes the test plan from the spec: schedule_in relative to now,
pop_due's >target cutoff, re-entrant scheduling mid-drain, and a 10k
deterministic-seed ordering sweep."
```

---

## Task 5: Wire `NDS::run_frame` to the event loop

Replace the Phase 0 stub `run_frame` with the real `peek_next → run_until → set_now → pop_due` loop, add `on_scheduler_event`, and make `FrameEnd` flip `frame_done_`.

**Files:**
- Modify: `src/nds.hpp`
- Modify: `src/nds.cpp`

- [ ] **Step 1: Update `src/nds.hpp`.**

Replace the entire file with:

```cpp
#pragma once

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm9/arm9.hpp"
#include "ds/common.hpp"
#include "scheduler/event.hpp"
#include "scheduler/scheduler.hpp"

namespace ds {

class NDS {
public:
    NDS();

    void run_frame();
    void reset();

    Scheduler& scheduler() { return scheduler_; }
    Arm9&      cpu9()      { return cpu9_; }
    Arm7&      cpu7()      { return cpu7_; }

private:
    void on_scheduler_event(const Event& ev);

    Scheduler scheduler_;
    Arm9      cpu9_;
    Arm7      cpu7_;
    bool      frame_done_ = false;
};

}  // namespace ds
```

- [ ] **Step 2: Rewrite `src/nds.cpp`.**

Replace the entire file with:

```cpp
#include "nds.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380. Real
// PPU-derived timing lands when the PPU lands.
static constexpr Cycle kFrameCycles = 1'120'380;

NDS::NDS() {
    reset();
}

void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;
}

void NDS::run_frame() {
    frame_done_ = false;
    scheduler_.schedule_in(kFrameCycles, EventKind::FrameEnd);

    while (!frame_done_) {
        const Cycle next = scheduler_.peek_next();
        cpu9_.run_until(next);
        cpu7_.run_until(next);
        scheduler_.set_now(next);

        Event ev;
        while (scheduler_.pop_due(next, ev)) {
            on_scheduler_event(ev);
        }
    }
}

void NDS::on_scheduler_event(const Event& ev) {
    switch (ev.kind) {
        case EventKind::FrameEnd:
            frame_done_ = true;
            break;
    }
}

}  // namespace ds
```

- [ ] **Step 3: Build the full binary.**

```bash
cd build && make
```

Expected: `ds_emulator`, `ds_core`, `ds_frontend`, and all three test binaries build with no warnings. Any warning-as-error means CLAUDE.md style compliance slipped — fix before moving on.

- [ ] **Step 4: Run all unit tests.**

```bash
cd build && ctest --output-on-failure
```

Expected: `3/3 tests passed`.

- [ ] **Step 5: Smoke-test the emulator binary.**

```bash
cd build && timeout 2 ./ds_emulator || true
```

The binary should open a window showing the two-screen test pattern and exit after ~2s (when `timeout` SIGTERMs it). A segfault, an abort, or an immediate exit with an error code means `run_frame` is mis-wired — the most likely failure is `peek_next` returning `kNoEvent` on the first iteration because `schedule_in` wasn't called, which leaves `next = UINT64_MAX` and `set_now` gets called with that value. If that happens, re-check Step 2: `scheduler_.schedule_in(kFrameCycles, EventKind::FrameEnd);` must come before the `while` loop inside `run_frame`.

If you are running this task from a subagent environment without a display, **skip Step 5** and note the skip in your report. `ds_emulator` requires a real SDL window; it is not meaningful to headless-test until the headless path exists in a later phase.

- [ ] **Step 6: Commit.**

```bash
git add src/nds.hpp src/nds.cpp
git commit -m "nds: drive run_frame via the event loop

Schedules FrameEnd at the start of each frame and drains pending events
via peek_next -> run_until -> set_now -> pop_due. on_scheduler_event
holds the single dispatch switch required by the design rule that no
subsystem points back at another subsystem."
```

---

## Task 6: Amend the master design spec

Update `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` §9.1 and §9.4 so the master spec and the slice spec don't drift on the `pop_due` API vs the original `fire()` sketch.

**Files:**
- Modify: `docs/superpowers/specs/2026-04-12-nds-emulator-design.md`

- [ ] **Step 1: Amend §9.1 class sketch.**

Find this block in the design doc:

```cpp
class Scheduler {
    std::vector<Event> heap;    // min-heap by `when`
    Cycle              now = 0;
    uint64_t           next_id = 1;
public:
    uint64_t schedule_in(Cycle delta, uint32_t kind, uint64_t payload = 0);
    uint64_t schedule_at(Cycle when,  uint32_t kind, uint64_t payload = 0);
    void     cancel(uint64_t id);
    Cycle    peek_next() const;
    void     advance_to(Cycle target);
    void     fire(const Event&);
};
```

Replace with:

```cpp
class Scheduler {
    std::vector<Event>          heap;       // min-heap by (when, id)
    std::unordered_set<uint64_t> cancelled; // tombstones
    Cycle                       now = 0;
    uint64_t                    next_id = 1;
public:
    uint64_t schedule_in(Cycle delta, EventKind kind, uint64_t payload = 0);
    uint64_t schedule_at(Cycle when,  EventKind kind, uint64_t payload = 0);
    void     cancel(uint64_t id);
    Cycle    now() const;
    void     set_now(Cycle t);
    Cycle    peek_next();                         // kNoEvent if empty
    bool     pop_due(Cycle target, Event& out);   // pops one due event
};
```

Immediately after the code block, add this paragraph:

> The scheduler is a pure data structure. It has no reference to `NDS`, no
> callback, and no dispatch switch. Event handling lives in
> `NDS::on_scheduler_event`, called from the inner drain loop of
> `NDS::run_frame`. This keeps the scheduler trivially unit-testable and
> preserves rule #3 (no subsystem points back at another subsystem).

- [ ] **Step 2: Amend §9.4 dispatch location.**

Find:

```
A single enum, dispatched via one switch in `scheduler.cpp`:
```

Replace with:

```
A single enum, dispatched via one switch in `nds.cpp::on_scheduler_event`.
The scheduler itself never sees `EventKind` as anything other than an
opaque ordering key.
```

- [ ] **Step 3: Commit.**

```bash
git add docs/superpowers/specs/2026-04-12-nds-emulator-design.md
git commit -m "spec: amend §9.1/§9.4 for pop_due API and NDS-side dispatch

The design sketch still showed fire(Event) and a dispatch switch in
scheduler.cpp. The implemented shape is a pure scheduler with pop_due
and the switch in nds.cpp::on_scheduler_event, per the 2026-04-13
slice design. Sync the master spec so the two docs don't drift."
```

---

## Verification checklist (run at the end)

- [ ] `cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make` is clean, no warnings.
- [ ] `cd build && ctest --output-on-failure` shows `3/3 tests passed`.
- [ ] `cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make && ctest --output-on-failure` also shows `3/3 tests passed` — proves REQUIRE works under `-DNDEBUG`.
- [ ] `git log --oneline origin/main..HEAD` shows six new commits matching Tasks 1–6, none with `Co-Authored-By` lines.
- [ ] `git status` is clean.
- [ ] `CLAUDE.md` does not appear in any commit in the branch (`git log -p origin/main..HEAD -- CLAUDE.md` produces no output).
- [ ] Running `./ds_emulator` in the build dir still opens the test-pattern window and exits on Esc.

---
