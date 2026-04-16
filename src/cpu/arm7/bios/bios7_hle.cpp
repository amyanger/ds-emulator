#include "cpu/arm7/bios/bios7_hle.hpp"

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_state.hpp"

namespace ds {

u32 arm7_bios_hle_dispatch_swi(Arm7State& state, Arm7Bus& /*bus*/, u32 swi_number) {
    // Flat switch on the low 8 bits — every DS SWI uses only the bottom
    // byte of the comment field. Commits 5–11 convert the warn-stub cases
    // below into real implementations by adding `case 0xNN: cycles = ...`
    // arms BEFORE the default. Stubs all charge 1 cycle for the body; real
    // per-SWI costs arrive with their implementations.
    u32 cycles = 1;
    switch (swi_number & 0xFFu) {
    case 0x00:
        // SoftReset — direct-boot never invokes this (spec §4.12).
        DS_LOG_WARN(
            "arm7/bios: SoftReset (SWI 0x00) not implemented — direct-boot should never call this");
        break;
    case 0x10:
    case 0x11:
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x15:
        // Decompressor family — full implementations land in slice 3f.
        DS_LOG_WARN("arm7/bios: decompressor SWI 0x%02X not implemented (slice 3f)",
                    swi_number & 0xFFu);
        break;
    default:
        // Includes genuinely invalid SWIs (0x01, 0x02, 0x0A, 0x16–0x19, 0x1E)
        // and anything not yet implemented in later commits.
        DS_LOG_WARN("arm7/bios: SWI 0x%02X not implemented / invalid (NOP-return)",
                    swi_number & 0xFFu);
        break;
    }

    // Implicit MOVS PC, R14 — real BIOS code would run this at the end of the
    // handler; direct-boot has no ROM, so the dispatcher runs it inline.
    // GBATEK: "both PC=R14_svc, and CPSR=SPSR_svc". T bit in the restored
    // CPSR decides Thumb vs ARM PC alignment.
    //
    // TODO: share with arm7_dp.cpp (S=1 Rd=R15) and arm7_block.cpp (LDM-S)
    // via an arm7_exception_return helper in a dedicated refactor slice.
    //
    // User/System fallback (spsr == nullptr) is UNPREDICTABLE per ARMv4;
    // leave CPSR/PC untouched to match arm7_dp.cpp. Unreachable in practice
    // since SWI always enters SVC.
    u32* spsr = state.spsr_slot();
    if (spsr != nullptr) {
        const u32 new_cpsr = *spsr;
        const Mode new_mode = static_cast<Mode>(new_cpsr & 0x1Fu);
        // Capture R14 of the current (SVC) mode BEFORE switch_mode re-banks
        // the register file to the caller's mode — otherwise we'd read the
        // caller's stale R14 and end up jumping to the wrong address.
        const u32 return_r14 = state.r[14];
        state.switch_mode(new_mode);
        state.cpsr = new_cpsr;
        const bool thumb = (new_cpsr & (1u << 5)) != 0;
        state.pc = thumb ? (return_r14 & ~0x1u) : (return_r14 & ~0x3u);
    }
    return cycles;
}

} // namespace ds
