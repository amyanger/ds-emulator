#include "cpu/arm7/bios/bios7_tables.hpp"

#include "bus/arm7_bus.hpp"
#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7_state.hpp"

namespace ds {

u32 bios7_sound_bias(Arm7State& state, Arm7Bus& bus) {
    const u16 target = (state.r[0] != 0) ? u16{0x200} : u16{0x000};
    bus.write16(IO_SOUNDBIAS, target);
    return 1;
}

u32 bios7_is_debugger(Arm7State& state, Arm7Bus& /*bus*/) {
    state.r[0] = 0;
    return 1;
}

// Outputs land in R0/R1/R3 (R2 is an input per GBATEK's "incoming r2" XOR
// note). HG/SS does not consume the values post-direct-boot, so zeros
// suffice in place of DeSmuME's constants.
u32 bios7_get_boot_procs(Arm7State& state, Arm7Bus& /*bus*/) {
    state.r[0] = 0;
    state.r[1] = 0;
    state.r[3] = 0;
    return 1;
}

} // namespace ds
