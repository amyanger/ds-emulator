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
