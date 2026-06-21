#pragma once

#include "bus/arm9_bus.hpp"
#include "cpu/arm9/arm9.hpp"
#include "cpu/arm9/arm9_state.hpp"
#include "ds/common.hpp"
#include "nds.hpp"
#include "require.hpp"

namespace ds::test {

// Preload a single ARM instruction word at `pc` and execute exactly one
// instruction on the ARM9. Sets R15 = pc + 8 (the pipeline read-ahead value
// the core expects at execute time). Unlike the ARM7, the ARM9 runs at full
// scheduler rate, so one instruction = one ARM9 cycle and run_until's target
// is `cycles_before + 1` with NO halving. Asserts exactly one cycle elapsed.
inline void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm9_bus().write32(pc, instr);
    nds.cpu9().state().pc = pc;
    nds.cpu9().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu9().state().cycles;
    nds.cpu9().run_until(cycles_before + 1);
    REQUIRE(nds.cpu9().state().cycles == cycles_before + 1);
}

} // namespace ds::test
