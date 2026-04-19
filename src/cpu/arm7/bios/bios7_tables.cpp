#include "cpu/arm7/bios/bios7_tables.hpp"

#include "bus/arm7_bus.hpp"
#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7_state.hpp"

#include <array>
#include <cmath>
#include <numbers>

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

// SineTable: SWI 0x1A GetSineTable. 64 entries of `lround(sin(i * pi / 128)
// * 32768)`. Closed-form formula; no BIOS bytes in the repo. std::sin and
// std::lround are not constexpr in C++20, so the table is built once at
// static-init via a regular function. See spec §4.5.
namespace {

std::array<u16, kSineTableSize> compute_sine_table() {
    std::array<u16, kSineTableSize> t{};
    for (u32 i = 0; i < kSineTableSize; ++i) {
        const double angle = (static_cast<double>(i) * std::numbers::pi) / 128.0;
        const double scaled = std::sin(angle) * 32768.0;
        t[i] = static_cast<u16>(std::lround(scaled));
    }
    return t;
}

const std::array<u16, kSineTableSize> kSineTable = compute_sine_table();

// PitchTable: SWI 0x1B GetPitchTable. 768 entries of `lround((2^(i/768) - 1)
// * 65536)` — the 12-tone equal-temperament fractional ratio table NitroSDK's
// sound driver reads to pitch-shift samples. Closed-form formula; no BIOS
// bytes in the repo. Built once at static-init via the same regular-function
// pattern as kSineTable (std::pow / std::lround are not constexpr in C++20).
// Our top entry lands at 0xFF6A rather than GBATEK's 0xFF8A; see spec §4.6
// and §5.3 for the acknowledged ~0x20 delta.
std::array<u16, kPitchTableSize> compute_pitch_table() {
    std::array<u16, kPitchTableSize> t{};
    const double size = static_cast<double>(kPitchTableSize);
    for (u32 i = 0; i < kPitchTableSize; ++i) {
        const double ratio = std::pow(2.0, static_cast<double>(i) / size);
        const double scaled = (ratio - 1.0) * 65536.0;
        t[i] = static_cast<u16>(std::lround(scaled));
    }
    return t;
}

const std::array<u16, kPitchTableSize> kPitchTable = compute_pitch_table();

} // namespace

// Precondition: index < kSineTableSize. The SWI 0x1A handler clamps via
// clamp_or_warn; this internal accessor trusts the caller for hot-path speed.
u16 sine_table_lookup(u32 index) {
    return kSineTable[index];
}

// Precondition: index < kPitchTableSize. The SWI 0x1B handler clamps via
// clamp_or_warn; this internal accessor trusts the caller for hot-path speed.
u16 pitch_table_lookup(u32 index) {
    return kPitchTable[index];
}

} // namespace ds
