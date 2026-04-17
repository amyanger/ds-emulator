#include "cpu/arm7/bios/bios7_math.hpp"

#include "cpu/arm7/arm7_state.hpp"

namespace ds {

// C++11+ signed integer division truncates toward zero, matching ARM's
// semantics. INT32_MIN / -1 is implementation-defined but wraps to INT32_MIN
// on every compiler/target we build for (clang and gcc, arm64 and x86_64).
// R3 uses unsigned negation to compute |q| so INT32_MIN stays strictly defined.
u32 bios7_div(Arm7State& state, Arm7Bus& /*bus*/) {
    const i32 num = static_cast<i32>(state.r[0]);
    const i32 den = static_cast<i32>(state.r[1]);
    if (den == 0) {
        DS_LOG_WARN("arm7/bios: Div by zero at PC=0x%08X (returning 0/0)", state.pc);
        state.r[1] = state.r[0]; // dividend preserved in R1
        state.r[0] = 0;
        state.r[3] = 0;
        return 1;
    }
    const i32 q = num / den;
    const i32 r = num % den;
    const u32 uq = static_cast<u32>(q);
    state.r[0] = uq;
    state.r[1] = static_cast<u32>(r);
    state.r[3] = (q < 0) ? (0u - uq) : uq;
    return 1;
}

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
