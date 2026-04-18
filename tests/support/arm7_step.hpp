#pragma once

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"
#include "nds.hpp"
#include "require.hpp"

namespace ds::test {

// Steps the ARM7 one instruction at a time until PC reaches
// `expected_return_pc`. Used for SWIs whose real handlers re-enter the
// interpreter (via the BIOS trampoline), where the cycle cost depends
// on guest-callback instructions and fixed-cycle helpers no longer apply.
// Fails the test if the handler doesn't return within 2048 steps.
inline void run_until_returns(NDS& nds, u32 pc, u32 instr, u32 expected_return_pc) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    for (u32 i = 0; i < 2048u; ++i) {
        nds.cpu7().step_one_instruction();
        if (nds.cpu7().state().pc == expected_return_pc) {
            return;
        }
    }
    REQUIRE(false && "run_until_returns: handler did not return within step budget");
}

} // namespace ds::test
