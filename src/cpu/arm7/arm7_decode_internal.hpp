#pragma once

// Private header shared between the ARM7 decoder translation units
// (arm7_decode.cpp, arm7_dp.cpp, arm7_branch.cpp, arm7_loadstore.cpp).
// NOT part of any public include path. Only files under src/cpu/arm7/
// may include it.

#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7Bus;

// Single register-file writeback point used by every family handler.
// Writing to R15 also stomps state.pc so the next fetch lands on the
// target; callers relying on the pipeline model (state.r[15] ==
// instr_addr + 8) must read R15 BEFORE calling this.
inline void write_rd(Arm7State& s, u32 rd, u32 value) {
    s.r[rd] = value;
    if (rd == 15) {
        s.pc = value & ~0x3u;
    }
}

// Family dispatch helpers. Each returns the number of ARM7 cycles the
// instruction consumed. In slice 3b1 every path returns 1.
u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr);
u32 dispatch_branch(Arm7State& state, u32 instr);
u32 dispatch_single_data_transfer(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

}  // namespace ds
