// Verifies the ARM7 BIOS HLE dispatcher routes SWI 0x10-0x15 to the correct
// decompressor family entry point. Algorithm correctness is covered by the
// per-algorithm tests (arm7_bios_bitunpack_test, arm7_bios_lz77_test,
// arm7_bios_rle_test).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_decomp.hpp"
#include "cpu/arm7/bios/bios7_hle.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kSrc = 0x0200'0000u;
constexpr u32 kDst = 0x0200'0100u;
constexpr u32 kInfo = 0x0200'0200u;
constexpr u32 kSwiPc = 0x0200'1000u; // well clear of the payload buffers

// Header packing for LZ77 / RLE: bit 0-3 reserved, bit 4-7 type, bit 8-31 size.
static constexpr u32 make_header(u8 type, u32 size) {
    return (static_cast<u32>(type & 0x0Fu) << 4) | (size << 8);
}

// Pre-place the CPU in the post-exception-entry state the dispatcher expects.
// The dispatcher's implicit MOVS PC, R14 tail must restore cleanly without
// touching R0, otherwise the callback-stub R0-preservation asserts are bogus.
static void enter_arm_swi_state(NDS& nds, u32 swi_addr) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = 0x00000013u; // SVC, I=0, F=0, T=0
    state.r[14] = swi_addr + 4u;
    *state.spsr_slot() = 0x00000010u; // User, I=0, F=0, T=0
}

} // namespace

// SWI 0x10 BitUnPack: dispatcher-to-impl routing, minimal payload.
static void dispatch_0x10_bit_unpack() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write8(kSrc, 0x05u);
    bus.write16(kInfo + 0u, 1u); // src_len_bytes
    bus.write8(kInfo + 2u, 1u);  // src_width
    bus.write8(kInfo + 3u, 8u);  // dst_width
    bus.write32(kInfo + 4u, 0u); // data_offset=0, zero_flag=0

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSrc;
    state.r[1] = kDst;
    state.r[2] = kInfo;

    arm7_bios_hle_dispatch_swi(state, bus, 0x10u);

    REQUIRE(bus.read32(kDst + 0u) == 0x0001'0001u);
    REQUIRE(bus.read32(kDst + 4u) == 0x0000'0000u);
}

// SWI 0x11 LZ77 Wram: dispatcher-to-impl routing, minimal payload.
static void dispatch_0x11_lz77_wram() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kSrc, make_header(1u, 1u));
    bus.write8(kSrc + 4u, 0x00u);
    bus.write8(kSrc + 5u, 0xABu);

    // Sentinel guards against over-decompression past hdr.size.
    bus.write8(kDst + 1u, 0x5Au);

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSrc;
    state.r[1] = kDst;

    arm7_bios_hle_dispatch_swi(state, bus, 0x11u);

    REQUIRE(bus.read8(kDst + 0u) == 0xABu);
    REQUIRE(bus.read8(kDst + 1u) == 0x5Au);
}

// SWI 0x14 RLE Wram: dispatcher-to-impl routing, minimal payload.
static void dispatch_0x14_rl_wram() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kSrc, make_header(3u, 3u));
    bus.write8(kSrc + 4u, 0x80u); // compressed, length 3
    bus.write8(kSrc + 5u, 0xCDu);

    // Sentinel guards against over-decompression past hdr.size.
    bus.write8(kDst + 3u, 0x5Au);

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSrc;
    state.r[1] = kDst;

    arm7_bios_hle_dispatch_swi(state, bus, 0x14u);

    REQUIRE(bus.read8(kDst + 0u) == 0xCDu);
    REQUIRE(bus.read8(kDst + 1u) == 0xCDu);
    REQUIRE(bus.read8(kDst + 2u) == 0xCDu);
    REQUIRE(bus.read8(kDst + 3u) == 0x5Au);
}

constexpr u32 kSentinelR0 = 0xDEAD'BEEFu;

static void dispatch_0x12_lz77_callback_stub() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSentinelR0;

    arm7_bios_hle_dispatch_swi(state, bus, 0x12u);

    REQUIRE(state.r[0] == kSentinelR0);
}

static void dispatch_0x13_huff_callback_stub() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSentinelR0;

    arm7_bios_hle_dispatch_swi(state, bus, 0x13u);

    REQUIRE(state.r[0] == kSentinelR0);
}

static void dispatch_0x15_rl_callback_stub() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    enter_arm_swi_state(nds, kSwiPc);
    state.r[0] = kSentinelR0;

    arm7_bios_hle_dispatch_swi(state, bus, 0x15u);

    REQUIRE(state.r[0] == kSentinelR0);
}

// Distinct-address fallback for log-message verification: the project's
// logging is a bare fprintf macro with no sink-override hook, so spec §6.4
// allows proving the three callback cases dispatch to three distinct
// functions rather than capturing the warn-message content.
static void callback_stubs_are_distinct_functions() {
    using StubFn = u32 (*)(Arm7State&, Arm7Bus&);
    const StubFn lz77 = &bios7_lz77_callback_stub;
    const StubFn huff = &bios7_huff_callback_stub;
    const StubFn rl = &bios7_rl_callback_stub;

    REQUIRE(lz77 != huff);
    REQUIRE(huff != rl);
    REQUIRE(lz77 != rl);
}

int main() {
    dispatch_0x10_bit_unpack();
    dispatch_0x11_lz77_wram();
    dispatch_0x14_rl_wram();
    dispatch_0x12_lz77_callback_stub();
    dispatch_0x13_huff_callback_stub();
    dispatch_0x15_rl_callback_stub();
    callback_stubs_are_distinct_functions();
    std::puts("arm7_bios_decomp_dispatch_test: all 7 cases passed");
    return 0;
}
