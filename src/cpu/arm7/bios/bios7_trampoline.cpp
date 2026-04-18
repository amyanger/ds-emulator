#include "cpu/arm7/bios/bios7_trampoline.hpp"

#include "cpu/arm7/arm7.hpp"

namespace ds {

u32 arm7_bios_call_guest(Arm7& /*cpu*/, u32 target_pc, std::span<const u32, 4> /*args*/) {
    DS_LOG_WARN("arm7/bios: trampoline scaffold — call to 0x%08X returning 0", target_pc);
    return 0u;
}

} // namespace ds
