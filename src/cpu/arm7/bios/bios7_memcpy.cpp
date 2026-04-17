#include "cpu/arm7/bios/bios7_memcpy.hpp"

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_state.hpp"

#include <array>

namespace ds {

// Control-word layout shared by CpuSet and CpuFastSet (GBATEK).
constexpr u32 CPUSET_UNIT_COUNT_MASK = 0x001FFFFFu; // bits 20:0
constexpr u32 CPUSET_FILL_BIT = 1u << 24;           // 0 = copy, 1 = fill
constexpr u32 CPUSET_WIDE_BIT = 1u << 26;           // 0 = 16-bit, 1 = 32-bit (CpuSet only)

// Spec §5.6 — no BIOS-region check; we have no BIOS ROM in direct-boot.
u32 bios7_cpu_set(Arm7State& state, Arm7Bus& bus) {
    u32 src = state.r[0];
    u32 dst = state.r[1];
    const u32 ctrl = state.r[2];
    const u32 len = ctrl & CPUSET_UNIT_COUNT_MASK;
    const bool fill = (ctrl & CPUSET_FILL_BIT) != 0;
    const bool wide = (ctrl & CPUSET_WIDE_BIT) != 0;

    if (len == 0)
        return 1;

    if (wide) {
        src &= ~3u;
        dst &= ~3u;
        const u32 seed = fill ? bus.read32(src) : 0u;
        for (u32 i = 0; i < len; ++i) {
            const u32 v = fill ? seed : bus.read32(src + i * 4u);
            bus.write32(dst + i * 4u, v);
        }
    } else {
        src &= ~1u;
        dst &= ~1u;
        const u16 seed = fill ? bus.read16(src) : u16{0};
        for (u32 i = 0; i < len; ++i) {
            const u16 v = fill ? seed : bus.read16(src + i * 2u);
            bus.write16(dst + i * 2u, v);
        }
    }
    return 1;
}

u32 bios7_cpu_fast_set(Arm7State& state, Arm7Bus& bus) {
    u32 src = state.r[0] & ~3u;
    u32 dst = state.r[1] & ~3u;
    const u32 ctrl = state.r[2];
    u32 len = ctrl & CPUSET_UNIT_COUNT_MASK;
    const bool fill = (ctrl & CPUSET_FILL_BIT) != 0;
    if (len == 0)
        return 1;
    // Real hardware processes 8-word blocks, so trailing words past the
    // requested end are clobbered. Matches GBATEK.
    len = (len + 7u) & ~7u;

    const u32 seed = fill ? bus.read32(src) : 0u;
    for (u32 i = 0; i < len; ++i) {
        const u32 v = fill ? seed : bus.read32(src + i * 4u);
        bus.write32(dst + i * 4u, v);
    }
    return 1;
}

// Polynomial table per GBATEK §"BIOS GetCRC16 Function". Byte-by-byte,
// LSB-first bit processing matches spec §5.9 pseudocode exactly.
u32 bios7_get_crc16(Arm7State& state, Arm7Bus& bus) {
    static constexpr std::array<u16, 8> CRC16_TABLE = {
        0xC0C1,
        0xC181,
        0xC301,
        0xC601,
        0xCC01,
        0xD801,
        0xF001,
        0xA001,
    };

    u16 crc = static_cast<u16>(state.r[0]);
    const u32 ptr = state.r[1] & ~1u;
    const u32 len = state.r[2] & ~1u; // even-byte length
    u16 last = 0;

    for (u32 off = 0; off < len; off += 2) {
        const u16 word = bus.read16(ptr + off);
        last = word;
        for (u32 bit = 0; bit < 8; ++bit) {
            const bool xor_now = ((crc ^ (word >> bit)) & 1u) != 0;
            crc >>= 1;
            if (xor_now)
                crc ^= CRC16_TABLE[bit];
        }
        for (u32 bit = 0; bit < 8; ++bit) {
            const bool xor_now = ((crc ^ (word >> (bit + 8))) & 1u) != 0;
            crc >>= 1;
            if (xor_now)
                crc ^= CRC16_TABLE[bit];
        }
    }
    state.r[0] = crc;
    state.r[3] = last;
    return 1;
}

} // namespace ds
