// Exercises the RTC read-side command dispatcher: Status1/Status2 reads,
// DateTime/Date/Time BCD encoding, Status1 auto-clear gating on a complete
// byte read, and the saturating param_byte_ advance that prevents over-clock
// from wrapping back to byte 0. Slice 3j commit 3 — write-direction byte
// boundaries are still stubbed (covered in commit 4) and alarm/tick paths
// are still stubbed (covered in commit 6).

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

// Bit-bang 8 /SCK pulses with chip driving SIO; assemble the LSB-first
// byte the chip exposes. Sample order per bit: drive /SCK low (chip
// state idle on falling — no-op in our state machine), read pins.bit_0,
// drive /SCK rising (chip advances bit_idx_).
static u8 read_byte_out(Rtc& rtc) {
    u8 byte = 0;
    for (u8 i = 0; i < 8u; ++i) {
        rtc.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2)));
        const u8 bit = static_cast<u8>(rtc.read_pins() & 1u);
        byte = static_cast<u8>(byte | (bit << i));
        rtc.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2) | (1u << 1)));
    }
    return byte;
}

// Case 1: post-reset Status1 = 0x02 (24-hour mode, bit 1 set; bits 4..7 clear).
static void Rtc_ReadStatus1_DefaultIs24HourMode() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true); // Read Status1
    const u8 b = read_byte_out(r);
    REQUIRE(b == 0x02u);
    REQUIRE((b & 0xF0u) == 0u); // upper nibble (alarm/power flags) clear
    REQUIRE((b & 0x02u) != 0u); // bit 1 = 24h mode
}

// Case 2: structural test — a complete Status1 read exercises the auto-clear
// path without corrupting the register. NOT a strong test of the auto-clear
// semantics: post-reset no flags are set in bits 4..7, so the assertion
// `status1() == 0x02` is satisfied by both "auto-clear works" and "auto-
// clear never fires." The spec §6.2 prescribes pre-seeding status1_ to 0xF2
// and verifying the clear, but commit 3 has no path to set those flags
// (tick() is stubbed until commit 6). The strong assertion lands in commit
// 6's `Rtc_Status1Bit4_LatchesOnRaise_ClearsOnRead` (spec §6.3).
static void Rtc_ReadStatus1_ClearsBits47_OnRead() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true);
    const u8 read = read_byte_out(r);
    REQUIRE(read == 0x02u);
    REQUIRE(r.status1() == 0x02u); // unchanged (no flags to clear)
}

// Case 3: post-reset Status2 = 0.
static void Rtc_ReadStatus2_DefaultZero() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    clock_byte_in(r, 0x74u, true); // Read Status2 = 0x60 | 0x10 | 0x4
    REQUIRE(read_byte_out(r) == 0x00u);
}

// Case 4: DateTime read returns 7 BCD-encoded bytes in order
// {year%100, month, day, dow, hour, min, sec}.
static void Rtc_ReadDateTime_BcdEncoding() {
    Rtc r;
    r.reset();
    r.seed({2026, 5, 4, 1, 14, 30, 45});
    assert_cs_high(r);
    clock_byte_in(r, 0x72u, true); // Read DateTime = 0x60 | 0x10 | 0x2

    const u8 expected[7] = {0x26u, 0x05u, 0x04u, 0x01u, 0x14u, 0x30u, 0x45u};
    for (u8 i = 0; i < 7u; ++i) {
        const u8 got = read_byte_out(r);
        REQUIRE(got == expected[i]);
    }
}

// Case 5: Date read returns the first 3 of the DateTime sequence.
static void Rtc_ReadDate_FirstThreeBytes() {
    Rtc r;
    r.reset();
    r.seed({2026, 5, 4, 1, 14, 30, 45});
    assert_cs_high(r);
    clock_byte_in(r, 0x76u, true); // Read Date = 0x60 | 0x10 | 0x6

    const u8 expected[3] = {0x26u, 0x05u, 0x04u};
    for (u8 i = 0; i < 3u; ++i) {
        const u8 got = read_byte_out(r);
        REQUIRE(got == expected[i]);
    }
}

// Case 6: Time read returns the last 3 of the DateTime sequence (hour, min,
// sec — DateTime indices 4..6).
static void Rtc_ReadTime_LastThreeBytes() {
    Rtc r;
    r.reset();
    r.seed({2026, 5, 4, 1, 14, 30, 45});
    assert_cs_high(r);
    clock_byte_in(r, 0x7Au, true); // Read Time = 0x60 | 0x10 | 0xA

    const u8 expected[3] = {0x14u, 0x30u, 0x45u};
    for (u8 i = 0; i < 3u; ++i) {
        const u8 got = read_byte_out(r);
        REQUIRE(got == expected[i]);
    }
}

// Case 7: structural test — a partial Status1 read (4 of 8 bits + /CS abort)
// followed by a complete read must not crash or corrupt state. Same caveat
// as case 2: post-reset there are no flags in bits 4..7 to clear, so this
// cannot enforce the gating-on-complete-byte invariant — it can only
// confirm the partial-read code path doesn't crash. The strong invariant
// (auto-clear fires only on a complete byte, never on bits 1..7) lands in
// commit 6's alarm test where flags can actually be set.
static void Rtc_ReadStatus1_NoAutoClear_OnPartialRead() {
    Rtc r;
    r.reset();
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true); // Read Status1
    // Clock 4 of 8 bits in Param phase (chip driving), then abort with /CS.
    for (u8 i = 0; i < 4u; ++i) {
        clock_bit_in(r, 0u, false);
    }
    drop_cs(r);
    REQUIRE(r.status1() == 0x02u); // unchanged

    // Now perform a complete read. Still no flags set, so no real clear
    // occurs — but this exercises the full path without crashing or
    // corrupting state.
    assert_cs_high(r);
    clock_byte_in(r, 0x70u, true);
    (void) read_byte_out(r);
    REQUIRE(r.status1() == 0x02u);
}

// Case 8: over-clocking past the per-command max-byte-count must NOT wrap
// param_byte_ back to 0. With the saturating advance, shift_byte_ keeps the
// last byte's value (open-bus-equivalent), so the over-clocked 8th byte of
// a 7-byte DateTime read still echoes byte 6 (= seconds = 0x45).
static void Rtc_ReadDateTime_OverClockReturnsLastByte() {
    Rtc r;
    r.reset();
    r.seed({2026, 5, 4, 1, 14, 30, 45});
    assert_cs_high(r);
    clock_byte_in(r, 0x72u, true);

    // Clock 7 valid bytes; capture #7.
    u8 last = 0;
    for (u8 i = 0; i < 7u; ++i) {
        last = read_byte_out(r);
    }
    REQUIRE(last == 0x45u);

    // Over-clock the 8th byte — must NOT wrap to byte 0 (year = 0x26).
    REQUIRE(read_byte_out(r) == 0x45u);
}

// Case 9: bits are presented LSB-first with NO off-by-one skew.
// Seed so DateTime byte 0 = to_bcd(26) = 0x26 = 0b0010_0110.
//   Bit 0 of 0x26 = 0
//   Bit 1 of 0x26 = 1
// Manually clock the first 2 bits of byte 0 and verify each.
static void Rtc_ReadDateTime_BitIdxNoSkew_FirstBitIsLsb() {
    Rtc r;
    r.reset();
    r.seed({2026, 1, 1, 4, 0, 0, 0});
    assert_cs_high(r);
    clock_byte_in(r, 0x72u, true); // Read DateTime — byte 0 now in shift_byte_

    // Bit 0 of 0x26 = 0. Drive /SCK low with chip driving, sample pins bit 0.
    r.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2)));
    REQUIRE((r.read_pins() & 1u) == 0u);
    r.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2) | (1u << 1))); // /SCK rising

    // Bit 1 of 0x26 = 1.
    r.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2)));
    REQUIRE((r.read_pins() & 1u) == 1u);
    r.write_pins(static_cast<u8>(kDirChipDrivesSio | (1u << 2) | (1u << 1)));
}

// Case 10: Status1 writes affect bits 0..3 only; bits 4..7 (R-only flags)
// are preserved across the write. Post-reset status1_ = 0x02 (bit 1 set).
// Writing 0xFF stores 0x0F in the lower nibble and keeps the upper nibble
// at 0 (no flags set in commit 4 — flag-raise is commit 6).
static void Rtc_WriteStatus1_PreservesReservedBits() {
    Rtc r;
    r.reset();
    REQUIRE(r.status1() == 0x02u);

    assert_cs_high(r);
    clock_byte_in(r, 0x60u, true); // Write Status1 = 0x60 | 0x00 R/W=W | 0x0 cmd
    clock_byte_in(r, 0xFFu, true); // Param byte: write 0xFF
    REQUIRE(r.status1() == 0x0Fu); // bits 0..3 = 0xF, bits 4..7 = 0 (preserved)
    drop_cs(r);
}

// Case 11: Status2 stores all 8 bits verbatim, including bit 7.
static void Rtc_WriteStatus2_StoresAllBits() {
    Rtc r;
    r.reset();
    REQUIRE(r.status2() == 0x00u);

    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true); // Write Status2 = 0x60 | cmd 0x4
    clock_byte_in(r, 0x44u, true); // Alarm-1 enable (bits 0..3 = 0100b) + INT2 enable (bit 6)
    REQUIRE(r.status2() == 0x44u);
    drop_cs(r);

    // Second write to confirm bit 7 also stores.
    assert_cs_high(r);
    clock_byte_in(r, 0x64u, true);
    clock_byte_in(r, 0x80u, true);
    REQUIRE(r.status2() == 0x80u);
}

// Case 12: Alarm1 stores all 3 bytes (dow, hour, min) verbatim including
// bit 7 (compare-enable).
static void Rtc_WriteAlarm1_StoresAllThreeBytes() {
    Rtc r;
    r.reset();

    assert_cs_high(r);
    clock_byte_in(r, 0x61u, true); // Write Alarm1 = 0x60 | cmd 0x1
    clock_byte_in(r, 0x82u, true); // dow byte: bit 7 set + dow=2
    clock_byte_in(r, 0x95u, true); // hour byte: bit 7 set + 0x15 BCD
    clock_byte_in(r, 0xB0u, true); // min byte: bit 7 set + bit 5 set + 0x10 BCD
    drop_cs(r);

    REQUIRE(r.alarm1().dow == 0x82u);
    REQUIRE(r.alarm1().hour == 0x95u);
    REQUIRE(r.alarm1().min == 0xB0u);
}

// Case 13: Alarm2 writes do not bleed into Alarm1 state.
static void Rtc_WriteAlarm2_Independent() {
    Rtc r;
    r.reset();

    assert_cs_high(r);
    clock_byte_in(r, 0x65u, true); // Write Alarm2 = 0x60 | cmd 0x5
    clock_byte_in(r, 0x91u, true); // bit 7 set (compare-enable) — symmetry with case 12
    clock_byte_in(r, 0x22u, true);
    clock_byte_in(r, 0x33u, true);
    drop_cs(r);

    REQUIRE(r.alarm2().dow == 0x91u);
    REQUIRE(r.alarm2().hour == 0x22u);
    REQUIRE(r.alarm2().min == 0x33u);
    REQUIRE(r.alarm1().dow == 0u); // default-init untouched
    REQUIRE(r.alarm1().hour == 0u);
    REQUIRE(r.alarm1().min == 0u);
}

// Case 14: Write-direction DateTime command must NOT corrupt dt_. The slice
// spec lists DateTime/Date/Time only in R-only rows; a write attempt is a
// ROM bug, but the chip-side state must remain intact (just warn-log and
// discard the bytes).
static void Rtc_WriteDateTime_ReadOnly_Discarded() {
    Rtc r;
    r.reset();
    r.seed({2026, 5, 4, 1, 14, 30, 45});

    assert_cs_high(r);
    clock_byte_in(r, 0x62u, true); // Write DateTime = 0x60 | cmd 0x2 (read-only!)
    for (u8 i = 0; i < 7u; ++i) {
        clock_byte_in(r, 0xAAu, true);
    }
    drop_cs(r);

    const Rtc::DateTime dt = r.now_datetime();
    REQUIRE(dt.year == 2026u);
    REQUIRE(dt.month == 5u);
    REQUIRE(dt.day == 4u);
    REQUIRE(dt.dow == 1u);
    REQUIRE(dt.hour == 14u);
    REQUIRE(dt.min == 30u);
    REQUIRE(dt.sec == 45u);
}

int main() {
    std::puts("[Rtc_ReadStatus1_DefaultIs24HourMode]");
    Rtc_ReadStatus1_DefaultIs24HourMode();
    std::puts("[Rtc_ReadStatus1_ClearsBits47_OnRead]");
    Rtc_ReadStatus1_ClearsBits47_OnRead();
    std::puts("[Rtc_ReadStatus2_DefaultZero]");
    Rtc_ReadStatus2_DefaultZero();
    std::puts("[Rtc_ReadDateTime_BcdEncoding]");
    Rtc_ReadDateTime_BcdEncoding();
    std::puts("[Rtc_ReadDate_FirstThreeBytes]");
    Rtc_ReadDate_FirstThreeBytes();
    std::puts("[Rtc_ReadTime_LastThreeBytes]");
    Rtc_ReadTime_LastThreeBytes();
    std::puts("[Rtc_ReadStatus1_NoAutoClear_OnPartialRead]");
    Rtc_ReadStatus1_NoAutoClear_OnPartialRead();
    std::puts("[Rtc_ReadDateTime_OverClockReturnsLastByte]");
    Rtc_ReadDateTime_OverClockReturnsLastByte();
    std::puts("[Rtc_ReadDateTime_BitIdxNoSkew_FirstBitIsLsb]");
    Rtc_ReadDateTime_BitIdxNoSkew_FirstBitIsLsb();
    std::puts("[Rtc_WriteStatus1_PreservesReservedBits]");
    Rtc_WriteStatus1_PreservesReservedBits();
    std::puts("[Rtc_WriteStatus2_StoresAllBits]");
    Rtc_WriteStatus2_StoresAllBits();
    std::puts("[Rtc_WriteAlarm1_StoresAllThreeBytes]");
    Rtc_WriteAlarm1_StoresAllThreeBytes();
    std::puts("[Rtc_WriteAlarm2_Independent]");
    Rtc_WriteAlarm2_Independent();
    std::puts("[Rtc_WriteDateTime_ReadOnly_Discarded]");
    Rtc_WriteDateTime_ReadOnly_Discarded();
    std::puts("OK");
    return 0;
}
