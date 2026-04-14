// arm7_branch.cpp — ARMv4T branch dispatch. Slice 3b1 implements B, BL,
// and BX (the latter is actually caught inside arm7_dp.cpp because BX
// lives in DP encoding space; this file only handles bits[27:25] == 101).

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_branch(Arm7State& /*state*/, u32 instr) {
    DS_LOG_WARN("arm7: branch form 0x%08X (stub)", instr);
    return 1;
}

}  // namespace ds
