#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "cpu/arm7/bios/bios7_trampoline.hpp"

#include <array>
#include <span>

namespace ds {

namespace {

// Maximum Huffman tree table size. tree_size_byte is u8, so the largest
// representable table is (255 + 1) * 2 = 512 bytes. The fixed bound also
// pins malformed-tree reads to a known range — see the OOB branch in the
// decoder below.
constexpr u32 kHuffMaxTreeBytes = 512u;

// Tree-node bit fields, per GBATEK Huffman tree layout.
constexpr u8 kHuffOffsetMask = 0x3Fu;   // bits 0-5 of an internal node
constexpr u8 kHuffNode1EndFlag = 0x40u; // bit 6 — child 1 is a leaf
constexpr u8 kHuffNode0EndFlag = 0x80u; // bit 7 — child 0 is a leaf

// GBATEK ABI: entry-R0 is an opaque handle (typically a game-maintained
// callback state pointer) that the BIOS passes as R0 to every callback.
// Open additionally receives entry-R1 (dest) and entry-R2 (user param);
// Huffman uses R2 for the scratch-buffer pointer that real BIOS uses to
// copy the tree into. Our impl holds the tree in a fixed-size array
// instead and forwards R2 to Open unchanged — the callback may ignore it.
struct HuffCallbackBinding {
    Arm7& cpu;
    DecompCallbacks cb;
    u32 r0_handle;

    u32 open(u32 dest, u32 user_param) {
        const std::array<u32, 4> args = {r0_handle, dest, user_param, 0u};
        return arm7_bios_call_guest(cpu, cb.open_and_get_32bit, std::span<const u32, 4>(args));
    }

    u8 get8() {
        const std::array<u32, 4> args = {r0_handle, 0u, 0u, 0u};
        return static_cast<u8>(
            arm7_bios_call_guest(cpu, cb.get_8bit, std::span<const u32, 4>(args)));
    }

    u32 get32() {
        const std::array<u32, 4> args = {r0_handle, 0u, 0u, 0u};
        return arm7_bios_call_guest(cpu, cb.get_32bit, std::span<const u32, 4>(args));
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

// Huffman-specific header split. Distinct from `DecompHeader` because that
// shared type does not carry `data_size_bits` (LZ77 / RLE do not use it).
struct HuffHeader {
    u8 data_size_bits; // bits 0-3  (typically 4 or 8)
    u8 type;           // bits 4-7  (must be 2 for Huffman)
    u32 size;          // bits 8-31 (decompressed size in bytes)
};

HuffHeader split_huff_header(u32 hdr_word) {
    return HuffHeader{
        static_cast<u8>(hdr_word & 0x0Fu),
        static_cast<u8>((hdr_word >> 4) & 0x0Fu),
        (hdr_word >> 8) & 0x00FF'FFFFu,
    };
}

} // namespace

// SWI 0x13 — HuffUnCompReadByCallback.
//
// GBATEK Huffman layout (BIOS Decompression Functions, Huffman section):
//   Data Header (32 bits, fetched via Open_and_get_32bit):
//     Bit 0-3   Data size in bit units (4 or 8)
//     Bit 4-7   Compressed type (must be 2 for Huffman)
//     Bit 8-31  Decompressed size in bytes
//   Tree Size (8 bits, fetched via Get_8bit):
//     Bit 0-7   tree_table_size/2 - 1 → tree occupies (value+1)*2 bytes
//   Tree Nodes (non-data):
//     Bit 0-5   Offset to next child node pair
//     Bit 6     Node1 end flag (1 = child 1 is a leaf)
//     Bit 7     Node0 end flag (1 = child 0 is a leaf)
//     Next child node0 at (CurrentAddr AND NOT 1) + Offset*2 + 2
//     Next child node1 at (CurrentAddr AND NOT 1) + Offset*2 + 3
//   Data Nodes:
//     Bit 0-7   Data value (upper bits zero when data_size_bits < 8)
//   Compressed Bitstream (fetched via Get_32bit, 32 bits at a time):
//     Bit 31 = First Bit (MSB-first). 0 = go to node0, 1 = node1.
//   Output:
//     Assembled into 32-bit words, LSB-first packing of data_size_bits
//     chunks, written via bus.write32.
//
// The `AND NOT 1` masking on the current node's address is critical: tree
// nodes are byte-packed, so a leaf descriptor ends up at an odd address
// whenever its parent sits at an even index but its sibling came first.
// The walker's next-address formula must ignore that low bit, else a
// whole subtree of codes decodes to the wrong leaves (Risk #6 in the
// slice 3g spec).
u32 bios7_huff_uncomp(Arm7& cpu) {
    Arm7State& state = cpu.state();
    Arm7Bus& bus = cpu.bus();

    HuffCallbackBinding binding{cpu, read_decomp_callbacks(bus, state.r[3]), state.r[0]};

    const u32 dest_start = state.r[1];
    const u32 user_param = state.r[2];

    const HuffHeader hdr = split_huff_header(binding.open(dest_start, user_param));
    if (hdr.type != 2u) {
        DS_LOG_WARN("arm7/bios: huff_uncomp SWI 0x13 with header type %u (expected 2)", hdr.type);
    }
    if (hdr.data_size_bits != 4u && hdr.data_size_bits != 8u) {
        DS_LOG_WARN("arm7/bios: huff_uncomp SWI 0x13 with data_size_bits=%u (expected 4 or 8)",
                    hdr.data_size_bits);
    }
    // The decode loop's termination depends on each leaf-emit advancing
    // written_bits by hdr.data_size_bits. A header with data_size_bits=0
    // would never satisfy `written_bits < total_bits` once total_bits > 0,
    // hanging the entire emulation thread. Bail before the loop instead.
    if (hdr.data_size_bits == 0u) {
        binding.close_if_any();
        return 1;
    }

    // Fetch tree-size byte and the entire tree table up front. Real BIOS
    // copies the tree into the scratch buffer (R2) before walking for the
    // same reason: a callback per node would be prohibitively slow.
    const u8 tree_size_byte = binding.get8();
    const u32 tree_bytes_total = (static_cast<u32>(tree_size_byte) + 1u) * 2u;
    std::array<u8, kHuffMaxTreeBytes> tree{};
    for (u32 i = 0; i < tree_bytes_total; ++i) {
        tree[i] = binding.get8();
    }

    u32 dest = dest_start;
    // Output progress is counted in BITS so data_size_bits=4 works correctly
    // (two decoded leaves per output byte). A 32-bit multiply is wide enough
    // for the 24-bit hdr.size * 8 worst case.
    const u32 total_bits = hdr.size * 8u;
    u32 written_bits = 0u;
    u32 out_word = 0u;
    u32 out_bits = 0u;
    u32 bit_buf = 0u;
    u32 bits_left = 0u;
    const u32 data_mask =
        hdr.data_size_bits >= 32u ? 0xFFFF'FFFFu : ((1u << hdr.data_size_bits) - 1u);

    while (written_bits < total_bits) {
        // Walk the tree from the root (tree[0]) until we hit a leaf.
        u32 node_idx = 0u;
        while (true) {
            if (bits_left == 0u) {
                bit_buf = binding.get32();
                bits_left = 32u;
            }
            const u32 bit = (bit_buf >> 31) & 1u;
            bit_buf <<= 1;
            --bits_left;

            // Malformed-tree guard: if a previous step routed past the end of
            // the actual table, treat this iteration as a deterministic zero
            // leaf so the outer loop still makes progress. Without the
            // emit-and-advance, a tree whose internal nodes route off the end
            // would consume bits forever without ever satisfying the outer
            // termination condition (hang risk on malformed input).
            if (node_idx >= tree_bytes_total) {
                out_bits += hdr.data_size_bits;
                written_bits += hdr.data_size_bits;
                break;
            }
            const u8 node_byte = tree[node_idx];
            const u32 offset = node_byte & kHuffOffsetMask;
            const u32 child_base = (node_idx & ~1u) + offset * 2u + 2u;
            const u32 child_idx = child_base + bit;
            const u8 end_flag_mask = bit ? kHuffNode1EndFlag : kHuffNode0EndFlag;
            const bool is_leaf = (node_byte & end_flag_mask) != 0u;
            node_idx = child_idx;
            if (is_leaf) {
                // child_idx can exceed kHuffMaxTreeBytes on a malformed tree
                // whose offset routes past the end; bound the leaf read so
                // the array access stays in-range and the outer loop still
                // makes progress (zero leaf on overflow).
                const u8 data = node_idx < kHuffMaxTreeBytes
                                    ? (tree[node_idx] & static_cast<u8>(data_mask))
                                    : 0u;
                out_word |= static_cast<u32>(data) << out_bits;
                out_bits += hdr.data_size_bits;
                written_bits += hdr.data_size_bits;
                break;
            }
        }

        if (out_bits == 32u) {
            bus.write32(dest, out_word);
            dest += 4u;
            out_word = 0u;
            out_bits = 0u;
        }
    }

    // Flush any trailing partial word so malformed input (decompressed size
    // not aligned to 32 bits) is deterministic. Well-formed payloads end on
    // a word boundary, in which case out_bits == 0 and this is a no-op.
    if (out_bits != 0u) {
        bus.write32(dest, out_word);
    }

    binding.close_if_any();
    return 1;
}

} // namespace ds
