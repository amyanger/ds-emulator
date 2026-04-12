#pragma once

namespace ds {

class CpuCore;

// Swappable instruction dispatcher. Phase 0 has no implementations; Phase 1
// introduces the interpreter. Phase 6+ may add cached-block or JIT dispatchers.
class Dispatcher {
public:
    virtual ~Dispatcher() = default;
    virtual void step(CpuCore& core) = 0;
};

}  // namespace ds
