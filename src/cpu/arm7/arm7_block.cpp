// arm7_block.cpp
//
// ARMv4T block data transfer instructions (LDM / STM). Dispatched from
// dispatch_arm() for the bits[27:25] == 100 primary slot.
//
// This file is the commit-3 scaffold: field decoding, addressing modes,
// Rn-in-list rules, S-bit handling, and R15-in-list semantics all land
// in later commits (4-15). For now the dispatcher logs a deterministic
// warn and returns 1 cycle so that the primary-dispatch wiring can be
// proven end-to-end without any bus traffic.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_block(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    // TODO(cycles): LDM/STM have variable cycle counts (nS+1N+1I /
    // (n-1)S+2N). Returning 1 as a placeholder until the cycle-accuracy
    // slice.
    (void) state;
    (void) bus;
    DS_LOG_WARN("arm7: LDM/STM not implemented yet 0x%08X at 0x%08X", instr, instr_addr);
    return 1;
}

} // namespace ds
