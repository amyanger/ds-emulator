#include "cpu/arm7/bios/bios7_hle.hpp"

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_state.hpp"

namespace ds {

u32 arm7_bios_hle_dispatch_swi(Arm7State& /*state*/, Arm7Bus& /*bus*/, u32 swi_number) {
    // Slice 3d: mechanism-only. Every SWI number is a stub until the
    // BIOS-HLE slice lands the real table (SoftReset, Halt, CpuSet, etc.).
    // Returning 0 here means the game's MOVS PC, R14 at the end of the
    // handler consumes the "cycle budget" for the SWI itself; this will
    // become a real per-SWI cost in the BIOS-HLE slice.
    DS_LOG_WARN("arm7/bios: SWI 0x%02X not implemented (slice 3d stub)", swi_number & 0xFFu);
    return 0;
}

} // namespace ds
