// Invokes bios7_rl_uncomp_vram directly (bypassing the SWI dispatcher) so
// each case can seed a minimal callback state and assert byte-level output.
// Structural mirror of arm7_bios_lz77_vram_test.cpp.

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

// Guest-code layout in main RAM. Callback stubs are placed well clear of the
// payload buffers so bus writes to dest never overlap callback instructions.
constexpr u32 kCbStruct = 0x0200'0000u;   // 5 × u32 callback pointer table
constexpr u32 kOpenCbAddr = 0x0200'0020u; // Open_and_get_32bit stub
constexpr u32 kGet8CbAddr = 0x0200'0040u; // Get_8bit stub
constexpr u32 kHandleAddr = 0x0200'0080u; // 2 × u32: stream_ptr, header_word
constexpr u32 kStreamAddr = 0x0200'0100u; // compressed source stream
constexpr u32 kDstAddr = 0x0200'0400u;    // decompression destination
constexpr u32 kCpsrSvcMode = 0x0000'0013u;
constexpr u32 kSvcSp = 0x0300'7FE0u;

// ARM-state instruction encodings used by the two callback stubs. Each
// u32 is a full ARM instruction — comments name the instruction.
constexpr u32 kInstrOpenLdrHeader = 0xE590'0004u;  // LDR R0, [R0, #4]  — header = handle[1]
constexpr u32 kInstrBxLr = 0xE12F'FF1Eu;           // BX LR
constexpr u32 kInstrGet8LoadPtr = 0xE590'1000u;    // LDR R1, [R0]      — R1 = stream_ptr
constexpr u32 kInstrGet8LoadByte = 0xE5D1'2000u;   // LDRB R2, [R1]     — R2 = byte
constexpr u32 kInstrGet8AdvancePtr = 0xE281'1001u; // ADD R1, R1, #1    — advance
constexpr u32 kInstrGet8StorePtr = 0xE580'1000u;   // STR R1, [R0]      — write back
constexpr u32 kInstrGet8MovR0R2 = 0xE1A0'0002u;    // MOV R0, R2        — return byte

// Header packing matches SWI 0x14: bit 0-3 reserved, bit 4-7 type, bit 8-31 size.
static constexpr u32 make_header(u8 type, u32 size) {
    return (static_cast<u32>(type & 0x0Fu) << 4) | (size << 8);
}

static void setup_svc(NDS& nds) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = kCpsrSvcMode;
    state.r[13] = kSvcSp;
}

static void install_callbacks(Arm7Bus& bus) {
    // Open_and_get_32bit (ARM):
    //   LDR R0, [R0, #4]   ; handle[1] = header word
    //   BX  LR
    bus.write32(kOpenCbAddr + 0u, kInstrOpenLdrHeader);
    bus.write32(kOpenCbAddr + 4u, kInstrBxLr);

    // Get_8bit (ARM):
    //   LDR  R1, [R0]       ; R1 = *handle = stream_ptr
    //   LDRB R2, [R1]       ; R2 = byte at stream_ptr
    //   ADD  R1, R1, #1     ; advance stream_ptr
    //   STR  R1, [R0]       ; write back to handle
    //   MOV  R0, R2         ; return byte in R0
    //   BX   LR
    bus.write32(kGet8CbAddr + 0u, kInstrGet8LoadPtr);
    bus.write32(kGet8CbAddr + 4u, kInstrGet8LoadByte);
    bus.write32(kGet8CbAddr + 8u, kInstrGet8AdvancePtr);
    bus.write32(kGet8CbAddr + 12u, kInstrGet8StorePtr);
    bus.write32(kGet8CbAddr + 16u, kInstrGet8MovR0R2);
    bus.write32(kGet8CbAddr + 20u, kInstrBxLr);

    // Callback struct: [Open, Close, Get_8bit, Get_16bit, Get_32bit].
    // Close = 0 → no Close callback (per GBATEK).
    bus.write32(kCbStruct + 0x00u, kOpenCbAddr);
    bus.write32(kCbStruct + 0x04u, 0u);
    bus.write32(kCbStruct + 0x08u, kGet8CbAddr);
    bus.write32(kCbStruct + 0x0Cu, 0u);
    bus.write32(kCbStruct + 0x10u, 0u);
}

static void write_stream(Arm7Bus& bus, std::initializer_list<u8> bytes) {
    u32 offset = 0u;
    for (u8 b : bytes) {
        bus.write8(kStreamAddr + offset, b);
        ++offset;
    }
}

static void invoke(NDS& nds, u32 header_word, u32 dest = kDstAddr) {
    auto& bus = nds.arm7_bus();
    // Handle layout: [0]=stream_ptr (incremented by Get_8bit), [1]=header_word.
    bus.write32(kHandleAddr + 0u, kStreamAddr);
    bus.write32(kHandleAddr + 4u, header_word);

    auto& state = nds.cpu7().state();
    setup_svc(nds);
    state.r[0] = kHandleAddr; // opaque handle, forwarded to every callback
    state.r[1] = dest;
    state.r[2] = 0u; // unused user param
    state.r[3] = kCbStruct;
    bios7_rl_uncomp_vram(nds.cpu7());
}

} // namespace

// Case 1: uncompressed length 1 (flag 0x00 → length_field=0, raw len=1).
// Only one output byte, so the final halfword-flush path emits a trailing
// 0x00 (buffer zero-initialized; halfword_bits == 8 at loop exit). This is
// deterministic malformed-input behavior; well-formed payloads end on a
// halfword boundary.
static void rle_vram_uncompressed_length_1() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0x00u, 0xABu});

    invoke(nds, make_header(3u, 1u));

    REQUIRE(bus.read8(kDstAddr + 0u) == 0xABu);
    REQUIRE(bus.read8(kDstAddr + 1u) == 0x00u); // trailing zero from partial flush
}

// Case 2: compressed length 3 (flag 0x80 → length_field=0, run len=3 of 0xCD).
// Three output bytes; trailing halfword flush emits 0x00 at D+3.
static void rle_vram_compressed_length_3() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0x80u, 0xCDu});

    invoke(nds, make_header(3u, 3u));

    REQUIRE(bus.read8(kDstAddr + 0u) == 0xCDu);
    REQUIRE(bus.read8(kDstAddr + 1u) == 0xCDu);
    REQUIRE(bus.read8(kDstAddr + 2u) == 0xCDu);
    REQUIRE(bus.read8(kDstAddr + 3u) == 0x00u); // trailing zero from partial flush
}

// Case 3: compressed length 130 (flag 0xFF → length_field=0x7F, run len=130
// of 0xEF). Even byte count so no trailing flush. Sentinel at D+130 verifies
// we don't overrun hdr.size.
static void rle_vram_compressed_length_130() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0xFFu, 0xEFu});

    // Seed sentinel just past the expected output.
    constexpr u8 kSentinel = 0x5Au;
    bus.write8(kDstAddr + 130u, kSentinel);

    invoke(nds, make_header(3u, 130u));

    for (u32 i = 0; i < 130u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == 0xEFu);
    }
    REQUIRE(bus.read8(kDstAddr + 130u) == kSentinel);
}

// Case 4: mixed stream with run / raw / run. Stream:
//   {0x80, 0xAA}             -> run-3 of 0xAA              (3 bytes)
//   {0x02, 0x11, 0x22, 0x33} -> raw-3 of 0x11,0x22,0x33    (3 bytes)
//   {0x82, 0xBB}             -> run-5 of 0xBB              (5 bytes)
// Total output = 11 bytes. Odd count, so trailing halfword-flush emits 0x00
// at D+11.
static void rle_vram_mixed_stream() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0x80u, 0xAAu, 0x02u, 0x11u, 0x22u, 0x33u, 0x82u, 0xBBu});

    invoke(nds, make_header(3u, 11u));

    REQUIRE(bus.read8(kDstAddr + 0u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 1u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 2u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 3u) == 0x11u);
    REQUIRE(bus.read8(kDstAddr + 4u) == 0x22u);
    REQUIRE(bus.read8(kDstAddr + 5u) == 0x33u);
    REQUIRE(bus.read8(kDstAddr + 6u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 7u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 8u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 9u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 10u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 11u) == 0x00u); // trailing zero from partial flush
}

// Case 5: zero decompressed size. Open returns header with size=0. Inner
// loop never runs; Get_8bit is never called; Close is skipped (cb.close=0).
// Dest must be untouched.
static void rle_vram_zero_size_no_writes() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    constexpr u8 kSentinel = 0xC3u;
    for (u32 i = 0; i < 16u; ++i) {
        bus.write8(kDstAddr + i, kSentinel);
    }

    invoke(nds, make_header(3u, 0u));

    for (u32 i = 0; i < 16u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == kSentinel);
    }
}

int main() {
    rle_vram_uncompressed_length_1();
    rle_vram_compressed_length_3();
    rle_vram_compressed_length_130();
    rle_vram_mixed_stream();
    rle_vram_zero_size_no_writes();
    std::puts("arm7_bios_rle_vram_test: all 5 cases passed");
    return 0;
}
