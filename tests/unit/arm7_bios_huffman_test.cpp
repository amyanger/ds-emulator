// Invokes bios7_huff_uncomp directly (bypassing the SWI dispatcher) so each
// case can seed a minimal callback state and assert byte-level output.
// Structural mirror of arm7_bios_lz77_vram_test.cpp. Huffman adds Get_32bit
// (for the bitstream) and a non-null Close (to verify close_if_any fires).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>
#include <initializer_list>

using namespace ds;

namespace {

// Guest-code layout in main RAM. Callback stubs, state struct, and payload
// buffers are placed far enough apart that bus writes to dest never overlap
// callback instructions or stream data.
constexpr u32 kCbStruct = 0x0200'0000u;       // 5 × u32 callback pointer table
constexpr u32 kOpenCbAddr = 0x0200'0020u;     // Open_and_get_32bit stub
constexpr u32 kGet8CbAddr = 0x0200'0040u;     // Get_8bit stub
constexpr u32 kGet32CbAddr = 0x0200'0080u;    // Get_32bit stub
constexpr u32 kCloseCbAddr = 0x0200'00C0u;    // Close stub
constexpr u32 kHandleAddr = 0x0200'0100u;     // 4 × u32: header, byte_ptr, word_ptr, close_count
constexpr u32 kTreeAddr = 0x0200'0180u;       // tree_size byte + tree bytes
constexpr u32 kWordStreamAddr = 0x0200'0200u; // bitstream words
constexpr u32 kDstAddr = 0x0200'0300u;        // decompression destination
constexpr u32 kCpsrSvcMode = 0x0000'0013u;
constexpr u32 kSvcSp = 0x0300'7FE0u;

// ARM-state instruction encodings. Each u32 is a full ARM instruction.
constexpr u32 kInstrBxLr = 0xE12F'FF1Eu;       // BX LR
constexpr u32 kInstrOpenLdrHdr = 0xE590'0000u; // LDR  R0, [R0]       — header = *handle
constexpr u32 kInstrMovR0Zero = 0xE3A0'0000u;  // MOV  R0, #0

constexpr u32 kInstrGet8LoadPtr = 0xE590'1004u;    // LDR  R1, [R0, #4]   — byte_ptr
constexpr u32 kInstrGet8LoadByte = 0xE5D1'2000u;   // LDRB R2, [R1]
constexpr u32 kInstrGet8AdvancePtr = 0xE281'1001u; // ADD R1, R1, #1
constexpr u32 kInstrGet8StorePtr = 0xE580'1004u;   // STR  R1, [R0, #4]
constexpr u32 kInstrGet8MovR0R2 = 0xE1A0'0002u;    // MOV  R0, R2

constexpr u32 kInstrGet32LoadPtr = 0xE590'1008u;    // LDR  R1, [R0, #8]   — word_ptr
constexpr u32 kInstrGet32LoadWord = 0xE591'2000u;   // LDR  R2, [R1]
constexpr u32 kInstrGet32AdvancePtr = 0xE281'1004u; // ADD R1, R1, #4
constexpr u32 kInstrGet32StorePtr = 0xE580'1008u;   // STR  R1, [R0, #8]
constexpr u32 kInstrGet32MovR0R2 = 0xE1A0'0002u;    // MOV  R0, R2

// Close stub: increment *(handle + 12) and return 0. Lets tests assert Close
// actually fired (non-null pointer exercised by binding.close_if_any).
constexpr u32 kInstrCloseLoadCnt = 0xE590'100Cu; // LDR  R1, [R0, #12]
constexpr u32 kInstrCloseIncr = 0xE281'1001u;    // ADD  R1, R1, #1
constexpr u32 kInstrCloseStore = 0xE580'100Cu;   // STR  R1, [R0, #12]

// Header packing for SWI 0x13: bit 0-3 data_size_bits, bit 4-7 type, bit 8-31 size.
static constexpr u32 make_header(u8 data_size_bits, u8 type, u32 size) {
    return (static_cast<u32>(data_size_bits & 0x0Fu)) | (static_cast<u32>(type & 0x0Fu) << 4) |
           (size << 8);
}

static void setup_svc(NDS& nds) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = kCpsrSvcMode;
    state.r[13] = kSvcSp;
}

// Installs Open / Get_8bit / Get_32bit / Close stubs + the callback struct.
// Close is non-null (increments handle[3] each invocation) so each test can
// assert the close callback actually fired.
static void install_callbacks(Arm7Bus& bus) {
    // Open_and_get_32bit:
    //   LDR R0, [R0]     ; header word from handle[0]
    //   BX  LR
    bus.write32(kOpenCbAddr + 0u, kInstrOpenLdrHdr);
    bus.write32(kOpenCbAddr + 4u, kInstrBxLr);

    // Get_8bit:
    //   LDR  R1, [R0, #4]   ; byte_ptr
    //   LDRB R2, [R1]
    //   ADD  R1, R1, #1
    //   STR  R1, [R0, #4]
    //   MOV  R0, R2
    //   BX   LR
    bus.write32(kGet8CbAddr + 0u, kInstrGet8LoadPtr);
    bus.write32(kGet8CbAddr + 4u, kInstrGet8LoadByte);
    bus.write32(kGet8CbAddr + 8u, kInstrGet8AdvancePtr);
    bus.write32(kGet8CbAddr + 12u, kInstrGet8StorePtr);
    bus.write32(kGet8CbAddr + 16u, kInstrGet8MovR0R2);
    bus.write32(kGet8CbAddr + 20u, kInstrBxLr);

    // Get_32bit:
    //   LDR R1, [R0, #8]   ; word_ptr
    //   LDR R2, [R1]
    //   ADD R1, R1, #4
    //   STR R1, [R0, #8]
    //   MOV R0, R2
    //   BX  LR
    bus.write32(kGet32CbAddr + 0u, kInstrGet32LoadPtr);
    bus.write32(kGet32CbAddr + 4u, kInstrGet32LoadWord);
    bus.write32(kGet32CbAddr + 8u, kInstrGet32AdvancePtr);
    bus.write32(kGet32CbAddr + 12u, kInstrGet32StorePtr);
    bus.write32(kGet32CbAddr + 16u, kInstrGet32MovR0R2);
    bus.write32(kGet32CbAddr + 20u, kInstrBxLr);

    // Close:
    //   LDR R1, [R0, #12]  ; close_count
    //   ADD R1, R1, #1
    //   STR R1, [R0, #12]
    //   MOV R0, #0
    //   BX  LR
    bus.write32(kCloseCbAddr + 0u, kInstrCloseLoadCnt);
    bus.write32(kCloseCbAddr + 4u, kInstrCloseIncr);
    bus.write32(kCloseCbAddr + 8u, kInstrCloseStore);
    bus.write32(kCloseCbAddr + 12u, kInstrMovR0Zero);
    bus.write32(kCloseCbAddr + 16u, kInstrBxLr);

    // Callback struct: [Open, Close, Get_8bit, Get_16bit, Get_32bit].
    bus.write32(kCbStruct + 0x00u, kOpenCbAddr);
    bus.write32(kCbStruct + 0x04u, kCloseCbAddr);
    bus.write32(kCbStruct + 0x08u, kGet8CbAddr);
    bus.write32(kCbStruct + 0x0Cu, 0u);
    bus.write32(kCbStruct + 0x10u, kGet32CbAddr);
}

static void write_tree(Arm7Bus& bus, u8 tree_size_byte, std::initializer_list<u8> tree_bytes) {
    bus.write8(kTreeAddr, tree_size_byte);
    u32 offset = 1u;
    for (u8 b : tree_bytes) {
        bus.write8(kTreeAddr + offset, b);
        ++offset;
    }
}

static void write_bitstream(Arm7Bus& bus, std::initializer_list<u32> words) {
    u32 offset = 0u;
    for (u32 w : words) {
        bus.write32(kWordStreamAddr + offset, w);
        offset += 4u;
    }
}

// Sets up handle[0..3]: header_word, byte_ptr (tree stream), word_ptr
// (bitstream), close_count. Returns after dispatching bios7_huff_uncomp.
static void invoke(NDS& nds, u32 header_word) {
    auto& bus = nds.arm7_bus();
    bus.write32(kHandleAddr + 0u, header_word);
    bus.write32(kHandleAddr + 4u, kTreeAddr);
    bus.write32(kHandleAddr + 8u, kWordStreamAddr);
    bus.write32(kHandleAddr + 12u, 0u); // close_count

    auto& state = nds.cpu7().state();
    setup_svc(nds);
    state.r[0] = kHandleAddr;
    state.r[1] = kDstAddr;
    state.r[2] = 0u; // user param / scratch-buffer pointer (unused by our stubs)
    state.r[3] = kCbStruct;
    bios7_huff_uncomp(nds.cpu7());
}

} // namespace

// Case 5 (ordered first — simplest): hdr.size=0 validates that the outer
// decode loop short-circuits, close still fires, dest is untouched, and the
// tree_size + 2-byte tree fetch still advances the Get_8bit cursor (matches
// real BIOS which unconditionally copies the tree before checking size).
static void huffman_zero_size() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    // Minimal tree: tree_size_byte=0 → 2-byte tree (contents irrelevant).
    write_tree(bus, 0x00u, {0x00u, 0x00u});

    constexpr u8 kSentinel = 0xC3u;
    for (u32 i = 0; i < 16u; ++i) {
        bus.write8(kDstAddr + i, kSentinel);
    }

    invoke(nds, make_header(8u, 2u, 0u));

    // Dest untouched.
    for (u32 i = 0; i < 16u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == kSentinel);
    }
    // byte_ptr advanced by 3 (1 tree_size byte + 2 tree bytes).
    REQUIRE(bus.read32(kHandleAddr + 4u) == kTreeAddr + 3u);
    // word_ptr untouched — no bitstream consumed.
    REQUIRE(bus.read32(kHandleAddr + 8u) == kWordStreamAddr);
    // Close fired once.
    REQUIRE(bus.read32(kHandleAddr + 12u) == 1u);
}

// Case 1: trivial two-leaf tree, data_size_bits=8.
//   tree[0] = 0xC0 (root: offset=0, both end flags set → leaves at 2,3)
//   tree[1] = pad
//   tree[2] = 0xAA (leaf for node0 → bit 0)
//   tree[3] = 0xBB (leaf for node1 → bit 1)
// Bitstream word = 0xF0F0F0F0 → MSB-first bits =
//   1111 0000 1111 0000 1111 0000 1111 0000 → codes 1,1,1,1,0,0,0,0, ...
// hdr.size=4 → first 4 codes = 1,1,1,1 → output = {BB,BB,BB,BB}.
// Packed LSB-first into 32-bit out_word, flushed as 0xBBBBBBBB.
static void huffman_trivial_two_leaf_data8() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    write_tree(bus, 0x01u, {0xC0u, 0x00u, 0xAAu, 0xBBu}); // 4-byte tree
    write_bitstream(bus, {0xF0F0'F0F0u});

    invoke(nds, make_header(8u, 2u, 4u));

    REQUIRE(bus.read32(kDstAddr) == 0xBBBB'BBBBu);
    // Close fired once.
    REQUIRE(bus.read32(kHandleAddr + 12u) == 1u);
}

// Case 2: balanced 8-leaf tree (3-bit codes) with internal nodes at ODD
// indices. Pins the AND-NOT-1 address masking: if the walker drops the
// NOT-1, level-3 internal at addr 3 resolves its children to the wrong
// base and decoding collapses immediately.
//
// Tree layout (16 bytes, tree_size_byte = 7):
//   [0] = 0x00  root         (children internal at 2,3)
//   [1] = 0x00  pad
//   [2] = 0x00  L  internal  (children at 4,5)
//   [3] = 0x01  R  internal  (children at 6,7 via AND-NOT-1 + offset=1)
//   [4] = 0xC1  LL internal  (leaves at 8,9)
//   [5] = 0xC2  LR internal  (leaves at 10,11 via AND-NOT-1=4, offset=2)
//   [6] = 0xC2  RL internal  (leaves at 12,13 via AND-NOT-1=6, offset=2)
//   [7] = 0xC3  RR internal  (leaves at 14,15 via AND-NOT-1=6, offset=3)
//   [8..15] = 8 distinct leaves: 0x11..0x88
//
// Codes (MSB of code first in bitstream):
//   000 → 0x11, 001 → 0x22, 010 → 0x33, 011 → 0x44,
//   100 → 0x55, 101 → 0x66, 110 → 0x77, 111 → 0x88.
//
// Payload = 16 codes = sequence 1..8 twice = 16 bytes = 48 bitstream bits
// (fits across 2 bitstream words with 16 unused trailing bits).
static void huffman_balanced_8leaf_data8() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    write_tree(bus,
               0x07u,
               {0x00u,
                0x00u,
                0x00u,
                0x01u,
                0xC1u,
                0xC2u,
                0xC2u,
                0xC3u,
                0x11u,
                0x22u,
                0x33u,
                0x44u,
                0x55u,
                0x66u,
                0x77u,
                0x88u});
    // Word 1 = seq bits 0..31 packed MSB-first (bit 31 consumed first).
    //   Codes: 000 001 010 011 100 101 110 111 000 001 01
    //   word1 = 0x0539'7705 (derived bit-by-bit; see test notes).
    // Word 2 = seq bits 32..63; only 32..47 meaningful, rest zero.
    //   Codes continuation: _ _ 010 011 100 101 110 111
    //   word2 = 0x3977'0000.
    write_bitstream(bus, {0x0539'7705u, 0x3977'0000u});

    invoke(nds, make_header(8u, 2u, 16u));

    // Expected output bytes 0..15.
    constexpr u8 kExpected[16] = {
        0x11u,
        0x22u,
        0x33u,
        0x44u,
        0x55u,
        0x66u,
        0x77u,
        0x88u,
        0x11u,
        0x22u,
        0x33u,
        0x44u,
        0x55u,
        0x66u,
        0x77u,
        0x88u,
    };
    for (u32 i = 0; i < 16u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == kExpected[i]);
    }
    REQUIRE(bus.read32(kHandleAddr + 12u) == 1u);
}

// Case 3: data_size_bits = 4 with leaf bytes carrying non-zero upper nibbles,
// to verify (a) nibble LSB-first packing into 32-bit output words and
// (b) data_mask correctly strips upper bits.
//
// Tree layout (8 bytes, tree_size_byte = 3):
//   [0] = 0x00 root         (children internal at 2,3)
//   [1] = 0x00 pad
//   [2] = 0xC0 L internal   (leaves at 4,5)
//   [3] = 0xC1 R internal   (leaves at 6,7 via AND-NOT-1=2, offset=1)
//   [4..7] = {0xF1, 0xF2, 0xF3, 0xF4}  — upper 0xF must be masked off
//
// Codes (2 bits each):
//   00 → nibble 0x1, 01 → 0x2, 10 → 0x3, 11 → 0x4.
// hdr.size = 4 bytes = 8 nibbles. Payload codes: 1,2,3,4,1,2,3,4.
//   Sequence bits: 00 01 10 11 00 01 10 11 → 16 bits.
//   word1 bits 31..16 = 0001'1011'0001'1011 = 0x1B1B; bits 15..0 = 0.
//   word1 = 0x1B1B0000.
// Expected output, packed LSB-first into one 32-bit dest word:
//   (0x1<<0)|(0x2<<4)|(0x3<<8)|(0x4<<12)|(0x1<<16)|(0x2<<20)|(0x3<<24)|(0x4<<28)
//   = 0x43214321.
static void huffman_data_size_4_nibbles() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    write_tree(bus, 0x03u, {0x00u, 0x00u, 0xC0u, 0xC1u, 0xF1u, 0xF2u, 0xF3u, 0xF4u});
    write_bitstream(bus, {0x1B1B'0000u});

    invoke(nds, make_header(4u, 2u, 4u));

    REQUIRE(bus.read32(kDstAddr) == 0x4321'4321u);
    REQUIRE(bus.read32(kHandleAddr + 12u) == 1u);
}

// Case 4: bitstream word boundary. 2-leaf tree (1-bit codes), hdr.size=64,
// data_size_bits=8 → 64 output bytes = 512 output bits = 64 codes × 1 bit
// = exactly 64 bitstream bits = 2 full 32-bit words. Pins the
// bits_left == 0 refill branch at the word boundary (after the 32nd code
// the bit buffer must refill before code 33 is decoded).
//
// Word 1 = 0xAAAA'AAAA. MSB-first bits = 1,0,1,0,...,1,0 → codes
// BB,AA,BB,AA,... → 32 alternating bytes starting with BB.
// Word 2 = 0x5555'5555. MSB-first bits = 0,1,0,1,...,0,1 → codes
// AA,BB,AA,BB,... → 32 alternating bytes starting with AA.
static void huffman_bitstream_word_boundary() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    write_tree(bus, 0x01u, {0xC0u, 0x00u, 0xAAu, 0xBBu});
    write_bitstream(bus, {0xAAAA'AAAAu, 0x5555'5555u});

    invoke(nds, make_header(8u, 2u, 64u));

    for (u32 i = 0; i < 32u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == (i & 1u ? 0xAAu : 0xBBu));
    }
    for (u32 i = 32u; i < 64u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == (i & 1u ? 0xBBu : 0xAAu));
    }
    REQUIRE(bus.read32(kHandleAddr + 12u) == 1u);
}

int main() {
    huffman_zero_size();
    huffman_trivial_two_leaf_data8();
    huffman_balanced_8leaf_data8();
    huffman_data_size_4_nibbles();
    huffman_bitstream_word_boundary();
    std::puts("arm7_bios_huffman_test: all 5 cases passed");
    return 0;
}
