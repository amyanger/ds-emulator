// Invokes bios7_lz77_uncomp_vram directly (bypassing the SWI dispatcher) so
// each case can seed a minimal callback state and assert byte-level output.

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
constexpr u32 kDstAddr = 0x0200'0200u;    // decompression destination
constexpr u32 kCpsrSvcMode = 0x0000'0013u;
constexpr u32 kSvcSp = 0x0300'7FE0u;

// ARM-state instruction encodings used by the two callback stubs. Each
// u32 is a full ARM instruction — comments name the instruction.
constexpr u32 kInstrOpenLdrHeader = 0xE590'0004u;  // LDR R0, [R0, #4]  — header = handle[1]
constexpr u32 kInstrBxLr = 0xE12F'FF1Eu;           // BX LR
constexpr u32 kInstrGet8LoadPtr = 0xE590'1000u;    // LDR R1, [R0]      — R1 = stream_ptr
constexpr u32 kInstrGet8LoadByte = 0xE5D1'2000u;   // LDRB R2, [R1]     — R2 = byte
constexpr u32 kInstrGet8AdvancePtr = 0xE281'1001u; // ADD R1, R1, #1   — advance
constexpr u32 kInstrGet8StorePtr = 0xE580'1000u;   // STR R1, [R0]      — write back
constexpr u32 kInstrGet8MovR0R2 = 0xE1A0'0002u;    // MOV R0, R2        — return byte

// Header packing matches SWI 0x11: bit 0-3 reserved, bit 4-7 type, bit 8-31 size.
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
    bios7_lz77_uncomp_vram(nds.cpu7());
}

} // namespace

// Case 1: single uncompressed block (1 raw byte). The final halfword-flush
// path emits a trailing 0x00 because the buffer is zero-initialized and
// halfword_bits == 8 when the inner loop exits. This is deterministic
// malformed-input behavior; well-formed payloads always end on a halfword
// boundary.
static void lz77_vram_single_uncompressed_byte() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0x00u, 0xABu}); // flag=0x00 (all raw), raw byte=0xAB

    invoke(nds, make_header(1u, 1u));

    REQUIRE(bus.read8(kDstAddr + 0u) == 0xABu);
    REQUIRE(bus.read8(kDstAddr + 1u) == 0x00u); // trailing zero from partial flush
}

// Two raw bytes flush a clean halfword, then a disp=2 len=3 back-ref reads
// the flushed halfword (hardware-correct path). Encoded disp=1 → actual=2;
// encoded len=0 → actual=3. Trailing 0x00 comes from the final partial-flush.
static void lz77_vram_raw_then_backref_disp2() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0x20u, 0xAAu, 0xBBu, 0x00u, 0x01u});

    invoke(nds, make_header(1u, 5u));

    REQUIRE(bus.read8(kDstAddr + 0u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 1u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 2u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 3u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 4u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 5u) == 0x00u);
}

// Case 3: len=8 disp=2 back-ref after a two-byte seed. Exercises the
// post-halfword-boundary read path — the effective read address alternates
// between (dest - 2) at halfword_bits=0 and (dest - 1) at halfword_bits=8.
//
// With disp=2 and the self-overlap pattern, the back-ref emits an infinite
// AA,BB alternation driven by the flushed seed. Total output = 2 (seed) +
// 8 (back-ref) = 10 bytes, which is exactly 5 halfwords.
//
// Encoded len=8 → raw=5 → b0 high nibble = 0x5. Encoded disp=2 → raw=1 →
// b0 low nibble = 0x0, b1 = 0x01. So b0=0x50, b1=0x01.
static void lz77_vram_len8_disp2_post_boundary() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);
    write_stream(bus, {0x20u, 0xAAu, 0xBBu, 0x50u, 0x01u});

    invoke(nds, make_header(1u, 10u));

    for (u32 i = 0; i < 10u; i += 2u) {
        REQUIRE(bus.read8(kDstAddr + i + 0u) == 0xAAu);
        REQUIRE(bus.read8(kDstAddr + i + 1u) == 0xBBu);
    }
}

// Case 4: disp=0 (actual_disp=1) — the GBATEK-documented Vram disp=0
// caveat. The back-ref reads from bytes sitting in the unflushed halfword
// buffer, which haven't been written to dest yet. The bus.read8 returns
// whatever pre-seeded sentinel memory lives there, so output is
// hardware-accurate garbage.
//
// Seed: raw 0xAA, raw 0xBB, then len=3 disp=1 (b0=0x00, b1=0x00). The Wram
// variant (SWI 0x11) with this encoding would produce
// [0xAA, 0xBB, 0xBB, 0xBB, 0xBB]. The Vram variant MUST diverge at D+3,
// where stale sentinel memory leaks into the output instead of 0xBB.
static void lz77_vram_disp0_hardware_garbage() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    // Pre-seed dest with a distinctive sentinel so garbage is detectable.
    constexpr u8 kSentinel = 0x5Au;
    for (u32 i = 0; i < 16u; ++i) {
        bus.write8(kDstAddr + i, kSentinel);
    }

    write_stream(bus, {0x20u, 0xAAu, 0xBBu, 0x00u, 0x00u});

    invoke(nds, make_header(1u, 5u));

    // First four bytes (seed + first back-ref byte reading D+1) match Wram.
    REQUIRE(bus.read8(kDstAddr + 0u) == 0xAAu);
    REQUIRE(bus.read8(kDstAddr + 1u) == 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 2u) == 0xBBu);

    // D+3: Wram would produce 0xBB (read dest-1 synchronously). Vram reads
    // unflushed dest memory (pending halfword buffer), which returns the
    // sentinel that was sitting at D+2 BEFORE we flushed the back-ref's
    // first byte. Assert the Vram output is NOT the Wram output.
    REQUIRE(bus.read8(kDstAddr + 3u) != 0xBBu);
    REQUIRE(bus.read8(kDstAddr + 3u) == kSentinel);
}

// Case 5: zero decompressed size. Open returns header with size=0. Inner
// loop never runs; Get_8bit is never called; Close is skipped (cb.close=0).
// Dest must be untouched.
static void lz77_vram_zero_size_no_writes() {
    NDS nds;
    auto& bus = nds.arm7_bus();
    install_callbacks(bus);

    constexpr u8 kSentinel = 0xC3u;
    for (u32 i = 0; i < 16u; ++i) {
        bus.write8(kDstAddr + i, kSentinel);
    }

    invoke(nds, make_header(1u, 0u));

    for (u32 i = 0; i < 16u; ++i) {
        REQUIRE(bus.read8(kDstAddr + i) == kSentinel);
    }
}

int main() {
    lz77_vram_single_uncompressed_byte();
    lz77_vram_raw_then_backref_disp2();
    lz77_vram_len8_disp2_post_boundary();
    lz77_vram_disp0_hardware_garbage();
    lz77_vram_zero_size_no_writes();
    std::puts("arm7_bios_lz77_vram_test: all 5 cases passed");
    return 0;
}
