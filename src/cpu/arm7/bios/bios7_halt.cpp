#include "cpu/arm7/bios/bios7_halt.hpp"

#include "cpu/arm7/arm7_state.hpp"

namespace ds {

// Hardware runs `LOP: SUB R0,1 / BGT LOP` in BIOS ROM; exit leaves R0 == 0.
// 4 cycles/iter matches GBATEK's calibration table (r0=0x20BA ≈ 1ms @ 33.51MHz).
u32 bios7_wait_by_loop(Arm7State& state, Arm7Bus& /*bus*/) {
    state.cycles += 4ull * static_cast<u64>(state.r[0]);
    state.r[0] = 0;
    return 1;
}

} // namespace ds
