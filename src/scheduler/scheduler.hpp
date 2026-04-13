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
    EventId schedule_in(Cycle delta, EventKind kind, u64 payload = 0);

    // Mark an event as cancelled. The event stays in the heap as a tombstone
    // until popped. Only pass ids returned from schedule_at — an unknown or
    // already-fired id adds a permanent entry to cancelled_ that is cleaned
    // only by reset().
    void cancel(EventId id);

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
