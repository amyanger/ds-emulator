#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "cpu/arm7/bios/bios7_trampoline.hpp"

#include <array>
#include <span>

namespace ds {

namespace {

// GBATEK ABI: entry-R0 is an opaque handle (typically a game-maintained
// callback state pointer) that the BIOS passes as R0 to every callback.
// Open additionally receives entry-R1 (dest) and entry-R2 (user param).
struct CallbackBinding {
    Arm7& cpu;
    DecompCallbacks cb;
    u32 r0_handle;

    u32 open(u32 dest, u32 user_param) {
        const std::array<u32, 4> args = {r0_handle, dest, user_param, 0u};
        return arm7_bios_call_guest(cpu, cb.open_and_get_32bit, std::span<const u32, 4>(args));
    }

    u32 get8() {
        const std::array<u32, 4> args = {r0_handle, 0u, 0u, 0u};
        return arm7_bios_call_guest(cpu, cb.get_8bit, std::span<const u32, 4>(args));
    }

    void close_if_any() {
        // GBATEK: Close pointer == 0 means no Close callback.
        if (cb.close == 0u) {
            return;
        }
        const std::array<u32, 4> args = {r0_handle, 0u, 0u, 0u};
        (void) arm7_bios_call_guest(cpu, cb.close, std::span<const u32, 4>(args));
    }
};

} // namespace

// SWI 0x15 — RLUnCompReadByCallbackWrite16bit.
//
// Algorithm mirrors SWI 0x14 (RLE Wram) with two substitutions:
//   1. Source bytes come from `Get_8bit` callbacks, not direct bus reads.
//   2. Destination writes are 16-bit halfwords — each emit_byte appends to a
//      local buffer that flushes to `bus.write16` once two bytes accumulate.
//
// Unlike LZ77 Vram there is no `disp` caveat — RLE has no back-references,
// so the pending halfword buffer never leaks stale memory into the output.
u32 bios7_rl_uncomp_vram(Arm7& cpu) {
    Arm7State& state = cpu.state();
    Arm7Bus& bus = cpu.bus();

    CallbackBinding binding{cpu, read_decomp_callbacks(bus, state.r[3]), state.r[0]};

    const u32 dest_start = state.r[1];
    const u32 user_param = state.r[2];

    const auto hdr = bios7_decomp_split_header(binding.open(dest_start, user_param));
    if (hdr.type != 3u) {
        DS_LOG_WARN("arm7/bios: RLE SWI 0x15 with header type %u (expected 3)", hdr.type);
    }

    u32 dest = dest_start;
    u32 written = 0u;
    u16 halfword_buffer = 0u;
    u32 halfword_bits = 0u;

    auto emit_byte = [&](u8 byte) {
        halfword_buffer =
            static_cast<u16>(halfword_buffer | (static_cast<u16>(byte) << halfword_bits));
        halfword_bits += 8u;
        if (halfword_bits == 16u) {
            bus.write16(dest, halfword_buffer);
            dest += 2u;
            halfword_buffer = 0u;
            halfword_bits = 0u;
        }
    };

    // Truncate at hdr.size; real BIOS does the same when the last block overshoots.
    while (written < hdr.size) {
        const u8 flag = static_cast<u8>(binding.get8());
        const bool compressed = (flag & 0x80u) != 0u;
        const u32 length_field = flag & 0x7Fu;
        const u32 len = length_field + (compressed ? 3u : 1u);

        if (compressed) {
            const u8 byte = static_cast<u8>(binding.get8());
            for (u32 i = 0; i < len && written < hdr.size; ++i) {
                emit_byte(byte);
                ++written;
            }
        } else {
            for (u32 i = 0; i < len && written < hdr.size; ++i) {
                const u8 byte = static_cast<u8>(binding.get8());
                emit_byte(byte);
                ++written;
            }
        }
    }

    // Flush any dangling byte so malformed input (odd decompressed size) is
    // deterministic. Well-formed RLE Vram payloads always end on a halfword
    // boundary, in which case halfword_bits == 0 and this is a no-op.
    if (halfword_bits != 0u) {
        bus.write16(dest, halfword_buffer);
    }

    binding.close_if_any();
    return 1;
}

} // namespace ds
