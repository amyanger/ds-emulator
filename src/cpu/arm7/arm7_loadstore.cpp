// arm7_loadstore.cpp — ARMv4T single-data-transfer dispatch (LDR/STR
// and their byte variants). Halfword/signed forms live in DP encoding
// space and stay deferred to slice 3b2.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_single_data_transfer(Arm7State& /*state*/, Arm7Bus& /*bus*/,
                                  u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm7: single-data-transfer form 0x%08X at 0x%08X (stub)",
                instr, instr_addr);
    return 1;
}

}  // namespace ds
