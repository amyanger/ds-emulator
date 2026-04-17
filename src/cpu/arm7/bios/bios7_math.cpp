#include "cpu/arm7/bios/bios7_math.hpp"

#include "cpu/arm7/arm7_state.hpp"

namespace ds {

// Iteration state is widened to u64 so the initial guess `(x + 1) / 2` does
// not overflow to zero when v == 0xFFFFFFFF. The final result is masked to
// 16 bits per GBATEK.
u32 bios7_sqrt(Arm7State& state, Arm7Bus& /*bus*/) {
    const u32 v = state.r[0];
    if (v == 0) {
        state.r[0] = 0;
        return 1;
    }
    u64 x = v;
    u64 y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + v / x) / 2;
    }
    state.r[0] = static_cast<u32>(x) & 0xFFFFu;
    return 1;
}

} // namespace ds
