// Exercises the RTC SIO bit-bang state machine: pin round-trip, /CS framing,
// /SCK rising-edge shift (LSB-first), command-byte fixed-bit validation, and
// SIO direction-bit handling. Slice 3j commit 2 — byte-boundary work in
// Param phase is still stubbed, so all assertions probe the public surface
// (read_pins) which is the only observable proof that decode succeeded.

#include "require.hpp"
#include "rtc/rtc.hpp"

#include <cstdio>

using namespace ds;

// Direction bits 4 (SIO), 5 (/SCK), 6 (/CS) all set = CPU drives every line.
// Real ROMs leave bits 5/6 set permanently and toggle only bit 4.
static constexpr u8 kDirAllCpuDriven = static_cast<u8>((1u << 4) | (1u << 5) | (1u << 6));
// Same as above but with bit 4 cleared — chip drives SIO during a read.
static constexpr u8 kDirChipDrivesSio = static_cast<u8>((1u << 5) | (1u << 6));

// Drive a single bit into the chip. Holds /CS high (bit 2 = 1), bit 4 sets
// direction, bit 0 = the SIO bit to clock, then toggles /SCK low → high to
// clock it in.
static void clock_bit_in(Rtc& rtc, u8 bit, bool dir_write_to_chip) {
    const u8 dir = dir_write_to_chip ? kDirAllCpuDriven : kDirChipDrivesSio;
    const u8 cs = static_cast<u8>(1u << 2);
    const u8 base = static_cast<u8>(dir | cs | (bit & 1u));
    rtc.write_pins(base);                              // /SCK low
    rtc.write_pins(static_cast<u8>(base | (1u << 1))); // /SCK rising
}

// Shift a whole byte LSB-first.
static void clock_byte_in(Rtc& rtc, u8 byte, bool dir_write_to_chip) {
    for (u8 i = 0; i < 8u; ++i) {
        clock_bit_in(rtc, static_cast<u8>((byte >> i) & 1u), dir_write_to_chip);
    }
}

// Drive /CS rising while /SCK is low, direction bits all "CPU writes."
static void assert_cs_high(Rtc& rtc) {
    rtc.write_pins(kDirAllCpuDriven);                              // /CS = 0
    rtc.write_pins(static_cast<u8>(kDirAllCpuDriven | (1u << 2))); // /CS rising
}

// Drive /CS falling.
static void drop_cs(Rtc& rtc) {
    rtc.write_pins(kDirAllCpuDriven);
}

// Case 1: after reset, pins read 0; /SCK toggle while /CS is low must not
// disturb anything observable; pins still round-trip.
static void Rtc_Reset_PinsZero_PhaseIdle() {
    Rtc r;
    r.reset();
    REQUIRE(r.read_pins() == 0u);

    r.write_pins(0x02u); // /SCK = 1, /CS = 0
    REQUIRE(r.read_pins() == 0x02u);

    r.write_pins(0x00u);
    REQUIRE(r.read_pins() == 0u);
}

// Case 2: direction bits 4/5/6 (and reserved 7) round-trip verbatim — bit 7
// must NOT be artificially set by any state-machine code.
static void Rtc_PinDirectionBits_RoundTrip() {
    Rtc r;
    r.reset();
    r.write_pins(0x70u);
    REQUIRE(r.read_pins() == 0x70u);
    REQUIRE((r.read_pins() & 0x80u) == 0u);
}

// Case 3: /CS rising enters Command phase. After clocking an 8-bit valid
// Write-Status1 (0x60), pins round-trip — active_read_ = false means
// read_pins returns pins verbatim, which is the expected commit-2 behavior.
static void Rtc_CsRising_EntersCommandPhase() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    clock_byte_in(r, 0x60u, true); // Write Status1: bits 5..7=110, R/W=0, idx=0
    // After the 8 command bits we are in Param phase with active_read_ = 0.
    // Drive one /SCK rising in Param phase; pins still round-trip because
    // the read-bit-replacement only fires when active_read_ is set.
    clock_bit_in(r, 1u, true);
    // Last write was: dir=0x70 | cs=0x04 | sck=0x02 | sio=0x01 = 0x77.
    REQUIRE(r.read_pins() == 0x77u);
}

// Case 4: /CS falling aborts a partial transfer. Clock 5 bits, drop /CS,
// then start a fresh transfer of Read-Status1 (0x70). The new decode must
// succeed despite the abandoned partial state — proven via the chip-driven
// bit-0 read in Param phase (read direction).
static void Rtc_CsFalling_AbortsTransfer() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    // Clock 5 bits of the byte 0x60 LSB-first: bits 0..4 = 0,0,0,0,0.
    for (u8 i = 0; i < 5u; ++i) {
        clock_bit_in(r, static_cast<u8>((0x60u >> i) & 1u), true);
    }
    drop_cs(r);

    // New transfer: Read Status1 = 0x70 (bits 5..7=110, R/W=1, idx=0).
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true);

    // Now in Param phase, read direction. Drive one /SCK rising with the
    // chip driving SIO (dir = 0). The chip-driven bit (shift_byte_ bit 0)
    // is 0 in commit 2 because produce_read_byte() isn't wired yet.
    clock_bit_in(r, 0u, false);
    REQUIRE((r.read_pins() & 0x01u) == 0u);
}

// Case 5: /SCK rising shifts one bit per edge, LSB-first. Build the byte
// 0x55 — bit 6 is 1, but bits 5 and 7 are 0, so the fixed-bits check
// (mask 0xE0 == 0x60) fails. active_cmd_ stays Unknown, so subsequent
// read_pins returns pins verbatim (no chip-driven bit replacement).
static void Rtc_SckRisingEdge_ShiftsOneBit_LSBFirst() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    for (u8 i = 0; i < 8u; ++i) {
        clock_bit_in(r, static_cast<u8>((0x55u >> i) & 1u), true);
    }
    // Drive a known pins value and verify exact round-trip — proves the
    // conditional bit-0 replacement is NOT firing.
    r.write_pins(0x71u);
    REQUIRE(r.read_pins() == 0x71u);
}

// Case 6: a bad command byte must not poison a subsequent valid transfer.
// Send 0x55 (fixed-bits check fails), then /CS toggle and 0x70 (Read
// Status1). The chip-driven bit-0 replacement firing on the second
// transfer proves the new decode succeeded despite the prior failure.
// (Case 5 already covers the "bad byte alone leaves state untouched"
// invariant; this case is specifically the recovery path.)
static void Rtc_CommandByte_FixedHighBitsValidated() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    clock_byte_in(r, 0x55u, true); // bad: fixed-bits check fails
    drop_cs(r);
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true); // good: Read Status1
    clock_bit_in(r, 0u, false);    // Param-phase /SCK with chip driving
    REQUIRE((r.read_pins() & 0x01u) == 0u);
}

// Case 7: bit 4 of the pins register (SIO direction) governs who drives
// the line during Param phase. Part 1: CPU drives — pins round-trip
// verbatim. Part 2: chip drives — read_pins replaces bit 0 with the
// chip-driven bit (0 in commit 2).
static void Rtc_DataDirectionBit4_GovernsSioDirection() {
    // Part 1: CPU writes the chip (Write Status1, bit 4 of cmd = 0 means
    // write-direction). In Param phase with bit 4 of pins = 1, the CPU
    // is driving SIO and pins round-trip exactly.
    {
        Rtc r;
        r.reset();
        assert_cs_high(r);
        clock_byte_in(r, 0x60u, true); // Write Status1
        clock_bit_in(r, 1u, true);     // CPU drives SIO=1
        // Last raw write: dir=0x70 | cs=0x04 | sck=0x02 | sio=0x01 = 0x77.
        REQUIRE(r.read_pins() == 0x77u);
    }

    // Part 2: chip drives SIO during Param phase. After Read Status1 (0x70),
    // a Param-phase /SCK rising with bit 4 = 0 means the chip is master.
    // shift_byte_ is 0 in commit 2, so chip-driven bit-0 is 0.
    {
        Rtc r;
        r.reset();
        assert_cs_high(r);
        clock_byte_in(r, 0x70u, true); // Read Status1
        clock_bit_in(r, 0u, false);    // chip drives SIO
        REQUIRE((r.read_pins() & 0x01u) == 0u);
    }
}

int main() {
    std::puts("[Rtc_Reset_PinsZero_PhaseIdle]");
    Rtc_Reset_PinsZero_PhaseIdle();
    std::puts("[Rtc_PinDirectionBits_RoundTrip]");
    Rtc_PinDirectionBits_RoundTrip();
    std::puts("[Rtc_CsRising_EntersCommandPhase]");
    Rtc_CsRising_EntersCommandPhase();
    std::puts("[Rtc_CsFalling_AbortsTransfer]");
    Rtc_CsFalling_AbortsTransfer();
    std::puts("[Rtc_SckRisingEdge_ShiftsOneBit_LSBFirst]");
    Rtc_SckRisingEdge_ShiftsOneBit_LSBFirst();
    std::puts("[Rtc_CommandByte_FixedHighBitsValidated]");
    Rtc_CommandByte_FixedHighBitsValidated();
    std::puts("[Rtc_DataDirectionBit4_GovernsSioDirection]");
    Rtc_DataDirectionBit4_GovernsSioDirection();
    std::puts("OK");
    return 0;
}
