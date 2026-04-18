#include "cpu/arm7/bios/bios7_decomp.hpp"

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_state.hpp"

namespace ds {

// Per spec §4.3: a single 32-bit bus read at the source address yields the
// shared LZ77/RLE header. Reserved bits 0-3 are not validated (matches
// hardware). Type validation is the algorithm's job — a mismatch warns and
// decompresses anyway, same as real BIOS.
DecompHeader bios7_decomp_parse_header(Arm7Bus& bus, u32 src_addr) {
    const u32 hdr = bus.read32(src_addr);
    return DecompHeader{
        static_cast<u8>(bits<u32>(hdr, 7, 4)),
        bits<u32>(hdr, 31, 8),
    };
}

// Slice 3f commit 1 scaffolds the three ReadNormal entry points. Each warns
// once per call and returns 1 cycle to match every other slice-3e SWI body.
// Commits 2/3/4 replace each scaffold with the real algorithm.

u32 bios7_bit_unpack(Arm7State& state, Arm7Bus& bus) {
    const u32 src = state.r[0];
    const u32 dest = state.r[1];
    const u32 info = state.r[2];

    const u16 src_len_bytes = bus.read16(info + 0u);
    const u8 src_width = bus.read8(info + 2u);
    const u8 dst_width = bus.read8(info + 3u);
    const u32 offset_word = bus.read32(info + 4u);
    const u32 data_offset = offset_word & 0x7FFF'FFFFu;
    const bool zero_flag = (offset_word & 0x8000'0000u) != 0u;

    // 64-bit mask construction so dst_width == 32 doesn't trip UB on `1u << 32`.
    const u32 src_mask = static_cast<u32>((1ull << src_width) - 1ull);
    const u32 dst_mask = static_cast<u32>((1ull << dst_width) - 1ull);

    u32 out_buffer = 0u;
    u32 out_bits = 0u;
    u32 dest_addr = dest;

    // Source chunks are walked LSB-first per byte (GBATEK omits this; matches
    // melonDS and real BIOS behavior verified by every known game).
    for (u32 i = 0; i < src_len_bytes; ++i) {
        const u8 byte = bus.read8(src + i);
        for (u32 bit = 0; bit < 8u; bit += src_width) {
            const u32 chunk = (static_cast<u32>(byte) >> bit) & src_mask;
            const u32 value = (chunk != 0u || zero_flag) ? (chunk + data_offset) : 0u;

            out_buffer |= (value & dst_mask) << out_bits;
            out_bits += dst_width;

            if (out_bits == 32u) {
                bus.write32(dest_addr, out_buffer);
                dest_addr += 4u;
                out_buffer = 0u;
                out_bits = 0u;
            }
        }
    }

    // Flush partial trailing word so malformed input is deterministic.
    if (out_bits != 0u) {
        bus.write32(dest_addr, out_buffer);
    }
    return 1;
}

u32 bios7_lz77_uncomp_wram(Arm7State& /*state*/, Arm7Bus& /*bus*/) {
    DS_LOG_WARN("arm7/bios: SWI 0x11 LZ77UnComp(Wram) not yet implemented (slice 3f scaffold)");
    return 1;
}

u32 bios7_rl_uncomp_wram(Arm7State& /*state*/, Arm7Bus& /*bus*/) {
    DS_LOG_WARN("arm7/bios: SWI 0x14 RLUnComp(Wram) not yet implemented (slice 3f scaffold)");
    return 1;
}

// Per spec §4.7: the three ReadByCallback variants ship as distinct warn-stubs
// so runtime triage can tell which callback path a game expects. R0 and the
// callback structure in R3 are deliberately untouched — slice 3g fills in the
// re-entrant interpreter trampoline that actually drives the callbacks.

u32 bios7_lz77_callback_stub(Arm7State& /*state*/, Arm7Bus& /*bus*/) {
    DS_LOG_WARN("arm7/bios: SWI 0x12 LZ77UnComp(callback,Vram) not implemented (slice 3g)");
    return 1;
}

u32 bios7_huff_callback_stub(Arm7State& /*state*/, Arm7Bus& /*bus*/) {
    DS_LOG_WARN("arm7/bios: SWI 0x13 HuffUnComp(callback) not implemented (slice 3g)");
    return 1;
}

u32 bios7_rl_callback_stub(Arm7State& /*state*/, Arm7Bus& /*bus*/) {
    DS_LOG_WARN("arm7/bios: SWI 0x15 RLUnComp(callback,Vram) not implemented (slice 3g)");
    return 1;
}

} // namespace ds
