// arm9_decode_stubs.cpp — warn-stubs for every ARM9 family executor that
// has not yet landed. Each stub emits one DS_LOG_WARN naming the family and
// the commit where the real executor arrives, then consumes 1 cycle so the
// dispatch loop keeps advancing. These are replaced one-by-one in commits
// 6-14; until then the recognizer fan-out (arm9_decode_000.cpp) routes to
// these so the decoder skeleton compiles and runs end-to-end.

#include "cpu/arm9/arm9_decode_internal.hpp"

namespace ds {

u32 dispatch_dp(Arm9State& state, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: DP stub — executor unimplemented (commit 6) instr=0x%08X addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    return 1;
}

u32 dispatch_branch(Arm9State& state, u32 instr) {
    DS_LOG_WARN("arm9: branch stub — executor unimplemented (commit 7) instr=0x%08X", instr);
    (void) state;
    return 1;
}

u32 dispatch_single_data_transfer(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: LDR/STR stub — executor unimplemented (commit 8) instr=0x%08X addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    (void) bus;
    return 1;
}

u32 execute_bx(Arm9State& state, u32 rm_value) {
    DS_LOG_WARN("arm9: BX stub — executor unimplemented (commit 8) rm_value=0x%08X", rm_value);
    (void) state;
    (void) rm_value;
    return 1;
}

u32 dispatch_halfword(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: halfword stub — executor unimplemented (commit 9) instr=0x%08X addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    (void) bus;
    return 1;
}

u32 dispatch_block(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: LDM/STM stub — executor unimplemented (commit 10) instr=0x%08X addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    (void) bus;
    return 1;
}

u32 dispatch_multiply(Arm9State& state, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: multiply stub — executor unimplemented (commit 11) instr=0x%08X addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    return 1;
}

u32 dispatch_psr_transfer(Arm9State& state, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: PSR-transfer stub — executor unimplemented (commit 12) instr=0x%08X "
                "addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    return 1;
}

u32 dispatch_swap(Arm9State& state, Arm9Bus& bus, u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm9: SWP stub — executor unimplemented (commit 13) instr=0x%08X addr=0x%08X",
                instr,
                instr_addr);
    (void) state;
    (void) bus;
    return 1;
}

} // namespace ds
