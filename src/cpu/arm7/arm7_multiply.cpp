// arm7_multiply.cpp — ARMv4T multiply family dispatch.
// Handles MUL, MLA (short 32-bit forms) and UMULL/UMLAL/SMULL/SMLAL
// (long 64-bit forms) across the bits[27:22] ∈ {000000, 000001} +
// bits[7:4] == 1001 encoding pattern. All variants currently return 1
// ARM7 cycle; real Rs-dependent early-termination timing is deferred
// to a dedicated cycle-accuracy slice.
//
// Slice 3b2 Task 5 implements the two short-form variants; the long-form
// branch logs a warn and no-ops. Task 6 and Task 7 fill in the long path.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "ds/common.hpp"

namespace ds {

namespace {

// Log a warn for any UNPREDICTABLE multiply case. Single call site so
// a grep of the log finds every instance.
void log_multiply_unpredictable(u32 instr, const char* reason, u32 pc) {
    DS_LOG_WARN("arm7: multiply UNPREDICTABLE (%s) instr 0x%08X at PC 0x%08X",
                reason, instr, pc);
}

}  // namespace

u32 dispatch_multiply(Arm7State& state, u32 instr, u32 instr_addr) {
    // Pattern: cond xxxx 000 0 xxxA S xxxx xxxx xxxx 1001 xxxx
    //          (bit 23 decides short vs long; bit 22 signed; bit 21 accumulate;
    //           bit 20 set-flags)
    if ((instr & 0x0F0000F0u) != 0x00000090u) {
        return 0;  // not a multiply — caller continues with normal DP
    }

    const bool long_form  = ((instr >> 23) & 1u) != 0;
    const bool accumulate = ((instr >> 21) & 1u) != 0;
    const bool set_flags  = ((instr >> 20) & 1u) != 0;

    if (!long_form) {
        // MUL / MLA: 32-bit result in Rd at [19:16]. Rn at [15:12] is the
        // accumulate operand (MLA) — ignored when A==0.
        const u32 rm = instr & 0xFu;
        const u32 rs = (instr >> 8) & 0xFu;
        const u32 rn = (instr >> 12) & 0xFu;
        const u32 rd = (instr >> 16) & 0xFu;

        if (rd == 15) log_multiply_unpredictable(instr, "Rd==15", state.pc);
        if (rm == rd) log_multiply_unpredictable(instr, "Rm==Rd (ARMv4T)", state.pc);

        const u32 rm_v = state.r[rm];
        const u32 rs_v = state.r[rs];
        u32 result = rm_v * rs_v;
        if (accumulate) {
            result += state.r[rn];  // modular 32-bit, wraparound matches HW
        }
        write_rd(state, rd, result);

        if (set_flags) {
            // N and Z reflect the 32-bit result. C is UNPREDICTABLE on
            // ARMv4T → preserve. V unchanged.
            state.cpsr = set_nz(state.cpsr, result);
        }

        (void)instr_addr;
        // TODO(cycles): Rs-dependent early-termination table.
        return 1;
    }

    // Long-form handling is added in Tasks 6 / 7. In slice 3b2 Task 5
    // we log a warn so any ROM that uses UMULL/SMULL/UMLAL/SMLAL is
    // loudly noticed, then no-op.
    DS_LOG_WARN("arm7: long-form multiply 0x%08X at 0x%08X (not yet implemented)",
                instr, instr_addr);
    (void)accumulate;
    (void)set_flags;
    return 1;
}

}  // namespace ds
