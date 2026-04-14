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

// Read Rm in register-shift DP form. In reg-shift form, the ARM reads Rm
// one pipeline stage later than in imm-shift form, so Rm == 15 returns
// PC + 12 instead of PC + 8. Also used for Rn reads in reg-shift form.
inline u32 read_rm_pc12(const Arm7State& s, u32 rm) {
    return (rm == 15) ? (s.r[15] + 4) : s.r[rm];
}

// Read Rs (the register holding the shift amount) in register-shift DP
// form. Rs == 15 is UNPREDICTABLE on real ARMv4T hardware; we log a warn
// and return PC + 12 for determinism. Games do not use this form.
inline u32 read_rs_for_reg_shift(const Arm7State& s, u32 rs) {
    if (rs == 15) {
        DS_LOG_WARN("arm7: reg-shift DP with Rs == 15 (unpredictable) at PC 0x%08X",
                    s.pc);
    }
    return (rs == 15) ? (s.r[15] + 4) : s.r[rs];
}

// Family dispatch helpers. Each returns the number of ARM7 cycles the
// instruction consumed. In slice 3b1 every path returns 1.
u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr);
u32 dispatch_branch(Arm7State& state, u32 instr);
u32 dispatch_single_data_transfer(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

}  // namespace ds
