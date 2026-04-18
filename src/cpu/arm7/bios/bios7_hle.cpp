#include "cpu/arm7/bios/bios7_hle.hpp"

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "cpu/arm7/bios/bios7_halt.hpp"
#include "cpu/arm7/bios/bios7_math.hpp"
#include "cpu/arm7/bios/bios7_memcpy.hpp"
#include "cpu/arm7/bios/bios7_tables.hpp"

namespace ds {

u32 arm7_bios_hle_dispatch_swi(Arm7& cpu, u32 swi_number) {
    Arm7State& state = cpu.state();
    Arm7Bus& bus = cpu.bus();

    // DS SWIs use only the low 8 bits of the comment field.
    u32 cycles = 1;
    switch (swi_number & 0xFFu) {
    case 0x00:
        // SoftReset — direct-boot never invokes this (spec §4.12).
        DS_LOG_WARN(
            "arm7/bios: SoftReset (SWI 0x00) not implemented — direct-boot should never call this");
        break;
    case 0x03:
        cycles = bios7_wait_by_loop(state, bus);
        break;
    case 0x04:
        cycles = bios7_intr_wait(state, bus);
        break;
    case 0x05:
        cycles = bios7_vblank_intr_wait(state, bus);
        break;
    case 0x06:
        cycles = bios7_halt(state, bus);
        break;
    case 0x07:
        cycles = bios7_sleep(state, bus);
        break;
    case 0x08:
        cycles = bios7_sound_bias(state, bus);
        break;
    case 0x09:
        cycles = bios7_div(state, bus);
        break;
    case 0x0B:
        cycles = bios7_cpu_set(state, bus);
        break;
    case 0x0C:
        cycles = bios7_cpu_fast_set(state, bus);
        break;
    case 0x0D:
        cycles = bios7_sqrt(state, bus);
        break;
    case 0x0E:
        cycles = bios7_get_crc16(state, bus);
        break;
    case 0x0F:
        cycles = bios7_is_debugger(state, bus);
        break;
    case 0x10:
        cycles = bios7_bit_unpack(state, bus);
        break;
    case 0x11:
        cycles = bios7_lz77_uncomp_wram(state, bus);
        break;
    case 0x12:
        cycles = bios7_lz77_uncomp_vram(cpu);
        break;
    case 0x13:
        cycles = bios7_huff_callback_stub(state, bus);
        break;
    case 0x14:
        cycles = bios7_rl_uncomp_wram(state, bus);
        break;
    case 0x15:
        cycles = bios7_rl_callback_stub(state, bus);
        break;
    case 0x1D:
        cycles = bios7_get_boot_procs(state, bus);
        break;
    case 0x1F:
        cycles = bios7_custom_halt(state, bus);
        break;
    default:
        // Invalid SWIs per GBATEK: 0x01, 0x02, 0x0A, 0x16–0x19, 0x1E.
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
