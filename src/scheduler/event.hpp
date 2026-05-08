#pragma once

#include "ds/common.hpp"

namespace ds {

// One enum, one switch, one handler per case in nds.cpp. Grows as
// subsystems land.
enum class EventKind : u32 {
    FrameEnd,
    Rtc1HzTick,
};

using EventId = u64;

struct Event {
    Cycle when; // ARM9 cycles since power-on
    EventKind kind;
    u64 payload; // event-kind-specific opaque data; unused for FrameEnd
    EventId id;  // monotonic, assigned by Scheduler
};

} // namespace ds
