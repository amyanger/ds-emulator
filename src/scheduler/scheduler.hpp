#pragma once

#include "ds/common.hpp"

#include <cstdint>

namespace ds {

// Placeholder event kind set for Phase 0. Real enumeration lives in Phase 1.
enum class EventKind : uint32_t {
    None = 0,
    FrameEnd,
};

// Phase 0 skeleton. Only exposes `now()` and a stubbed `advance_to` that
// updates the clock without firing events.
class Scheduler {
public:
    Scheduler() = default;

    Cycle now() const { return now_; }

    // Phase 0: just bump the clock. Phase 1 will pop the heap and fire events.
    void advance_to(Cycle target) {
        if (target > now_) {
            now_ = target;
        }
    }

    // Phase 1 will implement these. For now, stubs that do nothing so callers
    // can link.
    void reset() { now_ = 0; }

private:
    Cycle now_ = 0;
};

}  // namespace ds
