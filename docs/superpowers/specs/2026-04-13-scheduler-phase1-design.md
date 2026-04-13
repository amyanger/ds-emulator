# Scheduler (Phase 1, slice 1) — design

**Status:** approved for implementation
**Parent spec:** `2026-04-12-nds-emulator-design.md` §9
**Date:** 2026-04-13

---

## 1. Context

CLAUDE.md architecture rule #1: *the scheduler is the clock*. In this
design, time advances only through the `peek_next → cpu.run_until →
set_now → pop_due` loop owned by `NDS::run_frame`. Today's Phase 0
scaffold has a stub `Scheduler` that is just a `Cycle` counter with an
`advance_to(Cycle)` method that bumps `now_` — no heap, no events, no
dispatch. That `advance_to` goes away in this slice; the new API replaces
it with `peek_next` / `pop_due` / `set_now`. Every subsystem we build in
Phase 1 and beyond (timers, DMA, IRQs, cart DMA, GXFIFO stalls, SPU
sampling, H/VBlank) depends on a real scheduler existing first, so this is
the natural first slice of Phase 1.

This doc specifies the scheduler as a pure data structure: no cross-subsystem
coupling, no callback into `NDS`, no knowledge of what events mean. `NDS`
owns the dispatch loop and the `switch` over `EventKind`.

---

## 2. Scope

### In scope

- **Prerequisite**: a tiny Release-safe `REQUIRE(cond)` test macro (header
  under `tests/support/require.hpp`) that survives `-DNDEBUG`, plus
  retrofitting `fixed_test.cpp` and `ring_buffer_test.cpp` to use it.
  Bundled here because `scheduler_test.cpp` cannot be trusted otherwise,
  and fixing the two existing test files in the same pass avoids mixed
  styles.
- `event.hpp` — `EventKind` enum, `Event` struct, `EventId` alias.
- Rewritten `scheduler.hpp` / `scheduler.cpp` — full min-heap + tombstone
  implementation.
- `scheduler_test.cpp` — unit tests covering ordering, cancellation,
  re-entrant scheduling, and stress, registered in `tests/CMakeLists.txt`
  via `add_ds_unit_test`.
- `NDS::run_frame` rewritten to the event-loop shape from the parent spec
  §4, plus a new `NDS::on_scheduler_event(const Event&)` method holding the
  one dispatch `switch`.
- Amendment to parent spec §9.1 and §9.4 documenting the `pop_due` API and
  the fact that the switch lives in `nds.cpp`, not `scheduler.cpp`.

### Out of scope (deliberately deferred)

- **Save-state serialization of the scheduler** (parent §9.6). Lands with
  the broader save-state pass, not this slice.
- **`set_trace_sink` debug hook** (parent §9.7). Needs the X-Ray overlay,
  which is post-Phase-1.
- **All non-`FrameEnd` event kinds.** `HBlankStart/End`, `VBlankStart/End`,
  `Timer*`, `Dma*`, `CartXferDone`, `GeometryFifoReady`, `GxBufferSwapReady`,
  `SpuSample`, `RtcTick`, `IpcSyncChanged` — each of these lands in the
  phase that introduces the subsystem that owns it. Adding a new event kind
  later is adding one enum value and one `case` in `nds.cpp`.
- **ARM9/ARM7 execution.** The CPU cores remain empty `run_until` stubs;
  that's the next slice after this.

---

## 3. Data

### `event.hpp`

```cpp
#pragma once

#include "ds/common.hpp"

namespace ds {

enum class EventKind : u32 {
    FrameEnd,
};

using EventId = u64;

struct Event {
    Cycle     when;      // ARM9 cycles since power-on
    EventKind kind;
    u64       payload;   // event-kind-specific opaque data
    EventId   id;        // monotonic, assigned by Scheduler
};

}  // namespace ds
```

`payload` is present for forward compatibility — future events (e.g.
`Timer9_0..3` needing to carry the timer index, `Dma9_0..3` carrying a
channel number) will use it. `FrameEnd` does not need it; pass `0`.

### `Scheduler` state

```cpp
std::vector<Event>         heap_;       // min-heap ordered by (when, id)
std::unordered_set<EventId> cancelled_; // tombstones for cancel()
Cycle                      now_     = 0;
EventId                    next_id_ = 1;
```

Flat `std::vector` + `std::push_heap` / `std::pop_heap` with a custom
comparator. **Not `std::priority_queue`** — that hides the underlying
container and makes inspection and (future) serialization harder.

Heap comparator (logical min-heap, so `operator()` is the "greater-than" the
STL wants for a max-heap of the inverted order):

```cpp
struct HeapCmp {
    bool operator()(const Event& a, const Event& b) const {
        return std::tie(a.when, a.id) > std::tie(b.when, b.id);
    }
};
```

The `(when, id)` tiebreak guarantees deterministic FIFO ordering for events
scheduled at the same cycle: whichever was scheduled first (lower `id`) pops
first. This matters for chained DMAs, timer overflow cascades, and any case
where "event B should fire after event A" is encoded only by scheduling
order.

---

## 4. API

```cpp
class Scheduler {
public:
    Scheduler() = default;

    void    reset();

    // Schedule an event at an absolute ARM9 cycle. Returns its id.
    EventId schedule_at(Cycle when, EventKind kind, u64 payload = 0);

    // Schedule an event `delta` cycles after `now()`. Returns its id.
    EventId schedule_in(Cycle delta, EventKind kind, u64 payload = 0);

    // Mark an event as cancelled. Idempotent; unknown ids are a no-op.
    // The event stays in the heap as a tombstone until popped.
    void    cancel(EventId id);

    // Current ARM9 cycle. Monotonic.
    Cycle   now() const { return now_; }

    // Move the clock forward to `t`. Asserts `t >= now_`.
    // Does NOT pop events — pop_due is the caller's job.
    void    set_now(Cycle t);

    // Earliest `when` of a live (non-tombstoned) event. Returns UINT64_MAX
    // if no live events exist. Skips over tombstones at the head of the
    // heap as a side effect (shrinking the heap lazily is fine).
    Cycle   peek_next();

    // If the earliest live event has `when <= target`, pop it into `out`
    // and return true. Otherwise return false. Silently drops tombstones
    // encountered at the head and advances.
    bool    pop_due(Cycle target, Event& out);
};
```

### Method contracts

- **`schedule_at(when, kind, payload)`**: legal to schedule an event at or
  before `now_` — it will fire on the next `pop_due` call, treated as
  "already due." The new event gets `next_id_++`, so it will tiebreak after
  any already-queued event with the same `when`.
- **`schedule_in(delta, kind, payload)`**: equivalent to
  `schedule_at(now() + delta, kind, payload)`. No overflow check; `Cycle`
  is 64-bit and the emulator will not run for 2^64 cycles.
- **`cancel(id)`**: adds `id` to `cancelled_`. If `id` was never issued, or
  was already fired, or was already cancelled, this is a no-op. Cancelling
  the head of the heap and then calling `peek_next` or `pop_due` will cause
  the tombstone to be popped and discarded.
- **`peek_next()`**: non-const because it lazily drops tombstones from the
  heap head. After the call, either the heap is empty (returns
  `UINT64_MAX`) or `heap_[0]` is a live event.
- **`pop_due(target, out)`**: main drain method. Skips tombstones, pops
  the earliest live event if its `when <= target`, returns `true` and
  writes it to `out`. Returns `false` when the next live event is in the
  future relative to `target`, or when the heap is empty.
- **`set_now(t)`**: advances the monotonic clock. `t < now_` is a bug and
  triggers `assert`. `pop_due` does not mutate `now_` — the caller
  (`NDS::run_frame`) decides when to update the clock.

### Error handling

- `set_now(t < now_)` → `assert(false)`. Monotonic time is an invariant;
  violation is a programming error, not a runtime condition.
- `peek_next()` / `pop_due(...)` on empty or all-tombstoned heap → return
  `UINT64_MAX` / `false`. The caller must handle "no events pending."
- `cancel(unknown_id)` → no-op. Idempotent cancel simplifies callers.
- `schedule_at(when < now_)` → legal. The event fires on the next `pop_due`.

No exceptions are thrown from any scheduler method. Allocation failure from
`std::vector::push_back` or `std::unordered_set::insert` terminates — the
emulator has no recovery story for OOM and CLAUDE.md forbids exceptions in
the hot path.

---

## 5. NDS integration

`NDS::run_frame` is rewritten from the Phase 0 "advance clock by N" stub to
the event-loop shape specified in parent spec §4:

```cpp
void NDS::run_frame() {
    frame_done_ = false;
    scheduler_.schedule_in(kFrameCycles, EventKind::FrameEnd);

    while (!frame_done_) {
        const Cycle next = scheduler_.peek_next();
        if (next == Scheduler::kNoEvent) break;  // heap drained early
        cpu9_.run_until(next);
        cpu7_.run_until(next);
        scheduler_.set_now(next);

        Event ev{};
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
```

`kFrameCycles` stays at 1,120,380 (the 67.03 MHz / 59.82 Hz approximation
from Phase 0). Later phases may derive it from real PPU timing; that's
a separate change.

Because the CPU stubs are empty `run_until` no-ops, Phase 1 slice 1 still
produces the same visible behavior as Phase 0: the window opens, the test
pattern renders, `Esc` quits. The scheduler is wired through but there is
only one event kind and only the top-level `FrameEnd` fires per frame. This
is intentional — we verify the plumbing without needing any new subsystem
to exercise it.

---

## 6. Determinism

Every ordering decision in the scheduler is deterministic:

- Events are ordered by `(when, id)`. `id` is a monotonically increasing
  counter incremented on every `schedule_at` / `schedule_in` call. Two
  events with the same `when` always fire in insertion order.
- `cancelled_` is an `unordered_set`, but we only `find` and `erase`; we
  never iterate it, so its internal ordering is irrelevant to observable
  behavior.
- `peek_next` and `pop_due` are the only methods that observe the heap's
  interior, and they always look at `heap_[0]` and only pop tombstones,
  not reorder them.

Combined with every subsystem's future `save_state` / `load_state`, this
gives us the determinism foundation the parent spec calls out in §9.6:
same ROM + same input stream = same event sequence.

---

## 7. Performance

Per parent spec §9.8, target scheduler overhead is 1–3% of total runtime.
This slice makes that trivially easy to hit:

- One event per frame (`FrameEnd`) = 59.82 events/second.
- One `std::vector<Event>::push_back` + `std::push_heap` per schedule.
- One `std::pop_heap` + `pop_back` per fire.
- `cancelled_` stays empty in this slice (nothing cancels).

The hot path only becomes interesting in later phases when timers, DMA,
and H/VBlank start scheduling. For this slice, the perf shape does not
need measurement — it is dominated by the empty CPU loops.

---

## 8. Test plan

`tests/unit/scheduler_test.cpp`, registered via
`add_ds_unit_test(scheduler_test)`. Before this test file lands we switch
off bare `assert` (which is a no-op under `-DNDEBUG`) in favor of a tiny
`REQUIRE(cond)`-style macro that works in Release. This addresses the
project-memory item flagged in Phase 0 and is a prerequisite commit for
this slice — it also retrofits `fixed_test.cpp` and `ring_buffer_test.cpp`
in the same pass so all three test files use the same macro. All nine
cases below link only against `ds_core` and run in milliseconds.

### Cases

1. **Basic schedule/peek**: schedule three events at cycles 100, 50, 200.
   `peek_next()` returns 50.
2. **Pop order**: same setup, drain with `pop_due(UINT64_MAX)`. Events
   come out in (when) order: 50, 100, 200.
3. **Same-cycle FIFO**: schedule event A at cycle 100, then event B at
   cycle 100. `pop_due(100)` returns A first, then B. Verifies `(when, id)`
   tiebreak.
4. **Cancel head**: schedule event A at cycle 50, cancel A, schedule B
   at 100. `pop_due(UINT64_MAX)` returns B only.
5. **Cancel mid-heap**: schedule A@50, B@100, C@150. Cancel B.
   `pop_due(UINT64_MAX)` returns A then C. The tombstone for B is silently
   skipped when the heap advances past its position.
6. **`schedule_in` uses current `now`**: `set_now(1000)`, then
   `schedule_in(500, FrameEnd)`. `pop_due(1500)` returns an event with
   `when == 1500`.
7. **`pop_due` boundary**: schedule A@100, B@200. `pop_due(100)` returns A,
   then returns `false` (B is in the future). `pop_due(200)` returns B.
8. **Re-entrant scheduling**: schedule A@100. Call `pop_due(100)` → returns
   A. Immediately call `schedule_at(100, ...)` to add B. Call `pop_due(100)`
   again → returns B. Models the sequence `NDS::run_frame`'s inner `while
   (pop_due(...))` relies on when a fired event's handler schedules a new
   event at the same cycle: the new event lands in the heap and the next
   `pop_due` call in the same drain iteration picks it up.
9. **Stress**: push 10,000 events with pseudo-random `when` values (seeded
   `std::mt19937` for determinism), drain with `pop_due(UINT64_MAX)`,
   assert the drained sequence is non-decreasing in `when`.

Each case is a free function (`test_basic_schedule()` etc.) called from
`main()`. Failures `std::fprintf(stderr, ...)` and return non-zero from
`main`. Matching the current `fixed_test.cpp` / `ring_buffer_test.cpp`
style.

### What we are NOT testing in this slice

- `NDS::run_frame` integration (covered indirectly by the existing `main`
  smoke path: window opens, frame ticks, `Esc` quits).
- Serialization (out of scope).
- `set_trace_sink` (out of scope).

---

## 9. Spec amendments to `2026-04-12-nds-emulator-design.md`

Two small edits to §9, bundled into this slice's commit so the master spec
and the new slice doc don't drift:

**§9.1 — replace the `fire(Event)` method in the class sketch with:**

```cpp
Cycle peek_next();                 // UINT64_MAX if empty
bool  pop_due(Cycle target, Event& out);
void  set_now(Cycle t);
```

…and add a sentence: *"The scheduler is a pure data structure. It has no
knowledge of `NDS` and no dispatch callback — event handling lives in
`NDS::on_scheduler_event`, called from `NDS::run_frame`'s inner drain
loop."*

**§9.4 — replace:**

> A single enum, dispatched via one switch in `scheduler.cpp`:

**with:**

> A single enum, dispatched via one switch in `nds.cpp::on_scheduler_event`.
> The scheduler itself never sees `EventKind` as anything other than an
> opaque ordering key.

All other §9 sub-sections (tombstones, time model, latency, save state,
debug hooks, performance) remain accurate.

---

## 10. Build targets affected

- `ds_core` — picks up `scheduler/scheduler.cpp` (already in the target;
  file is being rewritten, not added) and `scheduler/event.hpp`
  (header-only, no target change).
- `tests/scheduler_test` — new binary registered in
  `tests/CMakeLists.txt` via `add_ds_unit_test(scheduler_test)`. Linked
  only against `ds_core`.
- `tests/fixed_test`, `tests/ring_buffer_test` — unchanged targets, but
  source files are edited to use the new `REQUIRE` macro.
- `ds_emulator` — unchanged. Still links `ds_frontend` + `ds_core`.

---

## 11. Follow-ups after this slice lands

Tracked outside this doc, but listed here so nothing is forgotten:

1. ARM9 bus + main RAM — next slice of Phase 1.
2. ARM9 ARMv5TE decoder — slice after that.
3. Scheduler serialization (`save_state`/`load_state`) when the broader
   save-state pass happens.
4. `set_trace_sink` + `XraySchedulerPage` when the X-Ray overlay lands.

---
