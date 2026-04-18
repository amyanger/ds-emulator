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

// SWI 0x12 — LZ77UnCompReadByCallbackWrite16bit.
//
// Algorithm mirrors SWI 0x11 (LZ77 Wram) with two substitutions:
//   1. Source bytes come from `Get_8bit` callbacks, not direct bus reads.
//   2. Destination writes are 16-bit halfwords — each emit_byte appends to a
//      local buffer that flushes to `bus.write16` once two bytes accumulate.
//
// disp=0 caveat (spec §5.3, GBATEK): the Vram variant reads back the most
// recently emitted bytes from dest to resolve back-references. Because dest
// only advances on halfword flush, bytes sitting in the pending halfword
// buffer have not yet been written to memory. A back-ref with actual_disp=1
// therefore reads stale dest memory instead of the byte just emitted — this
// is documented hardware behavior ("Vram works only with disp=001h..FFFh,
// not with disp=000h"). We preserve it exactly.
u32 bios7_lz77_uncomp_vram(Arm7& cpu) {
    Arm7State& state = cpu.state();
    Arm7Bus& bus = cpu.bus();

    CallbackBinding binding{cpu, read_decomp_callbacks(bus, state.r[3]), state.r[0]};

    const u32 dest_start = state.r[1];
    const u32 user_param = state.r[2];

    const auto hdr = bios7_decomp_split_header(binding.open(dest_start, user_param));
    if (hdr.type != 1u) {
        DS_LOG_WARN("arm7/bios: LZ77 SWI 0x12 with header type %u (expected 1)", hdr.type);
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

    while (written < hdr.size) {
        const u8 flags = static_cast<u8>(binding.get8());
        for (int bit = 7; bit >= 0 && written < hdr.size; --bit) {
            const bool compressed = ((flags >> bit) & 1u) != 0u;
            if (!compressed) {
                const u8 byte = static_cast<u8>(binding.get8());
                emit_byte(byte);
                ++written;
            } else {
                const u8 b0 = static_cast<u8>(binding.get8());
                const u8 b1 = static_cast<u8>(binding.get8());
                const u32 len = ((static_cast<u32>(b0) >> 4) & 0x0Fu) + 3u;
                const u32 disp = (((static_cast<u32>(b0) & 0x0Fu) << 8) | b1) + 1u;
                // disp-read uses the CURRENT effective byte-dest (dest +
                // pending-byte count). Per spec §4.6 / §5.3 we do NOT
                // special-case disp=1; the bus read on unflushed bytes
                // returns stale memory, matching hardware.
                for (u32 i = 0; i < len && written < hdr.size; ++i) {
                    const u32 read_addr = dest + (halfword_bits / 8u) - disp;
                    const u8 byte = bus.read8(read_addr);
                    emit_byte(byte);
                    ++written;
                }
            }
        }
    }

    // Flush any dangling byte so malformed input (odd decompressed size) is
    // deterministic. Well-formed LZ77 Vram payloads always end on a halfword
    // boundary, in which case halfword_bits == 0 and this is a no-op.
    if (halfword_bits != 0u) {
        bus.write16(dest, halfword_buffer);
    }

    binding.close_if_any();
    return 1;
}

} // namespace ds
