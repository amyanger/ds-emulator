#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_tables.hpp"

#include <atomic>

namespace ds {
namespace {

// First-warn gate prevents log flooding from a guest polling a bad index.
// Sine and Pitch have independent warned-flags so the first misuse of each
// SWI surfaces exactly once per process.
u32 clamp_or_warn(u32 raw) {
    if (raw >= kPitchTableSize) [[unlikely]] {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            DS_LOG_WARN("arm7/bios: GetPitchTable index 0x%X out of range "
                        "(table size 0x%X), clamping to 0x%X",
                        raw,
                        kPitchTableSize,
                        kPitchTableSize - 1u);
        }
        return kPitchTableSize - 1u;
    }
    return raw;
}

} // namespace

u32 bios7_get_pitch_table(Arm7State& state, Arm7Bus& /*bus*/) {
    state.r[0] = pitch_table_lookup(clamp_or_warn(state.r[0]));
    return 1;
}

} // namespace ds
