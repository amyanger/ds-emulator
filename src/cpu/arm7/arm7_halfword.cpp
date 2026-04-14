// arm7_halfword.cpp
//
// ARM7TDMI halfword and signed single-data-transfer instructions:
//   LDRH  — unsigned halfword load, zero-extended to 32
//   STRH  — halfword store (low 16 bits of Rd)
//   LDRSB — signed byte load, sign-extended to 32
//   LDRSH — signed halfword load, sign-extended to 32
//
// Encoding: bits[27:25] == 000, bit[4] == 1, bit[7] == 1. Lives in the
// ARMv4T "halfword extension" corner of the data-processing encoding
// space. Dispatched from dispatch_dp() after the BX/PSR/multiply
// recognizers.
//
// LDRD/STRD are ARMv5TE+ only and not valid on ARM7 (GBATEK verbatim:
// "STRD/LDRD supported on ARMv5TE and up only, not ARMv5, not
// ARMv5TExP"). Their encoding slots (L=0, SH=2/3) log a warn on ARM7.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_halfword(Arm7State& state, Arm7Bus& bus, u32 instr, u32 /*instr_addr*/) {
    (void)bus;  // Task 2 stub; Task 3+ wire the helpers that use the bus.

    const bool l  = (instr & (1u << 20)) != 0;
    const u32  sh = (instr >> 5) & 0x3u;

    if (!l) {
        // Store-side SH slots.
        switch (sh) {
            case 0:
                DS_LOG_WARN("arm7: SWP encoding encountered, not yet supported at 0x%08X",
                            state.pc);
                return 1;
            case 1:
                DS_LOG_WARN("arm7: STRH not yet implemented at 0x%08X", state.pc);
                return 1;
            case 2:
            case 3:
                DS_LOG_WARN("arm7: LDRD/STRD (ARMv5TE) on ARM7 at 0x%08X", state.pc);
                return 1;
        }
    } else {
        // Load-side SH slots.
        switch (sh) {
            case 0:
                DS_LOG_WARN("arm7: halfword load with SH=0 (reserved) at 0x%08X", state.pc);
                return 1;
            case 1:
                DS_LOG_WARN("arm7: LDRH not yet implemented at 0x%08X", state.pc);
                return 1;
            case 2:
                DS_LOG_WARN("arm7: LDRSB not yet implemented at 0x%08X", state.pc);
                return 1;
            case 3:
                DS_LOG_WARN("arm7: LDRSH not yet implemented at 0x%08X", state.pc);
                return 1;
        }
    }
    return 1;  // TODO(cycles): 1S+1N+1I for loads, 2N for STRH per GBATEK.
}

}  // namespace ds
