#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_tables.hpp"

#include <atomic>

namespace ds {
namespace {

// First-warn gate prevents log flooding from a guest polling a bad index.
// Each sibling SWI (Sine/Pitch/Volume) keeps its own warned flag so the first
// misuse of each surfaces exactly once per process.
u32 clamp_or_warn(u32 raw) {
    if (raw >= kVolumeTableSize) [[unlikely]] {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            DS_LOG_WARN("arm7/bios: GetVolumeTable index 0x%X out of range "
                        "(table size 0x%X), clamping to 0x%X",
                        raw,
                        kVolumeTableSize,
                        kVolumeTableSize - 1u);
        }
        return kVolumeTableSize - 1u;
    }
    return raw;
}

} // namespace

u32 bios7_get_volume_table(Arm7State& state, Arm7Bus& /*bus*/) {
    state.r[0] = volume_table_lookup(clamp_or_warn(state.r[0]));
    return 1;
}

} // namespace ds
