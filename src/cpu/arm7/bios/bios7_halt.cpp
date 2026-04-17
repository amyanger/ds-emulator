#include "cpu/arm7/bios/bios7_halt.hpp"

#include "bus/arm7_bus.hpp"
#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7_state.hpp"

namespace ds {

// Hardware runs `LOP: SUB R0,1 / BGT LOP` in BIOS ROM; exit leaves R0 == 0.
// 4 cycles/iter matches GBATEK's calibration table (r0=0x20BA ≈ 1ms @ 33.51MHz).
u32 bios7_wait_by_loop(Arm7State& state, Arm7Bus& /*bus*/) {
    state.cycles += 4ull * static_cast<u64>(state.r[0]);
    state.r[0] = 0;
    return 1;
}

// GBATEK: "On GBA and NDS7/DSi7, Halt is implemented by writing to HALTCNT,
// Port 4000301h." All registers unchanged on NDS7. The halt state machine
// lives in the HALTCNT write handler (nds.cpp) + run_until fast-path.
u32 bios7_halt(Arm7State& /*state*/, Arm7Bus& bus) {
    bus.write8(IO_HALTCNT, 0x80u);
    return 1;
}

// GBATEK is vague on Sleep ("probably similar as GBA SWI 03h Stop"). By
// analogy with CustomHalt's 0xC0 = Sleep encoding, write 0xC0 to HALTCNT.
// Slice 3e §3 deliberately treats Sleep as Halt (no wake-source restrictions
// modeled); HALTCNT's mode-decode handler maps both to halted_ = true.
u32 bios7_sleep(Arm7State& /*state*/, Arm7Bus& bus) {
    bus.write8(IO_HALTCNT, 0xC0u);
    return 1;
}

// GBATEK: "Writes the 8bit parameter value to HALTCNT" — the full byte from
// R2 is written verbatim, including reserved bits. HALTCNT's mode-decode
// handler picks out bits 6-7. Only low 8 bits of R2 are meaningful because
// HALTCNT is a 1-byte register.
u32 bios7_custom_halt(Arm7State& state, Arm7Bus& bus) {
    bus.write8(IO_HALTCNT, static_cast<u8>(state.r[2] & 0xFFu));
    return 1;
}

} // namespace ds
