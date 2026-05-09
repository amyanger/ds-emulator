// End-to-end RTC integration test — slice 3j commit 7 (capstone).
//
// Drives the full alarm-raise pipeline through NDS using only public
// surfaces: the ARM7 bus (for IME/IE/IF and the RTC bit-bang at 0x4000138),
// the scheduler (for the 1 Hz Rtc1HzTick event), and run_frame() for the
// pop-and-dispatch loop. Each unit test in slice 3j has already proved one
// link of the chain; this test wires them together exactly the way a real
// ROM would and asserts:
//   * Reset seeds the default DateTime and arms the recurring Rtc1HzTick.
//   * Bus writes to status2 / alarm1 round-trip into chip state via the
//     SIO state machine.
//   * Bus writes to IME/IE recompute the ARM7 line through
//     update_arm7_irq_signals().
//   * A scheduler-driven Rtc1HzTick fires through on_scheduler_event,
//     advances dt by one second, raises IF.7 on the rising edge, and the
//     post-handler line update flips cpu7_.irq_line() to true on the same
//     scheduler tick that set the IF bit.
//   * Bit-bang reading Status1 sees bit 4 latched, then auto-cleared.
//   * Acknowledging IF.7 via the bus IO_IF write-1-clear path drops the
//     ARM7 line.
//   * Subsequent ticks within the match window do NOT re-raise, and the
//     falling-edge tick (min wraps from 0 to 1) does NOT raise either —
//     the rising-edge guard holds in the production code path.

#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7.hpp"
#include "interrupt/irq_controller.hpp"
#include "nds.hpp"
#include "require.hpp"
#include "rtc/rtc.hpp"
#include "scheduler/event.hpp"

#include <cstdio>

using namespace ds;

// LSB-first BCD: tens in the high nibble, units in the low. Mirrors the
// file-local helper in rtc_irq_test.cpp; Rtc::to_bcd is private.
static constexpr u8 bcd(u8 v) {
    return static_cast<u8>(((v / 10u) << 4) | (v % 10u));
}

// Direction bits: bit 4 (SIO), bit 5 (/SCK), bit 6 (/CS). All set = CPU
// drives every line — the configuration real ROMs use during writes.
static constexpr u8 kDirAllCpuDriven = static_cast<u8>((1u << 4) | (1u << 5) | (1u << 6));
// Bit 4 cleared = chip drives SIO; used while reading.
static constexpr u8 kDirChipDrivesSio = static_cast<u8>((1u << 5) | (1u << 6));

// Bus-driven bit-bang helpers. Routing through arm7_io_write8(IO_RTC, ...)
// — rather than nds.rtc().write_pins() directly — exercises the bus path
// the way a real ARM7 store would.
static void rtc_write(NDS& nds, u8 v) {
    nds.arm7_io_write8(IO_RTC, v);
}

static u8 rtc_read(NDS& nds) {
    return nds.arm7_io_read8(IO_RTC);
}

static void clock_bit_in(NDS& nds, u8 bit) {
    const u8 cs = static_cast<u8>(1u << 2);
    const u8 base = static_cast<u8>(kDirAllCpuDriven | cs | (bit & 1u));
    rtc_write(nds, base);                              // /SCK low
    rtc_write(nds, static_cast<u8>(base | (1u << 1))); // /SCK rising
}

static void clock_byte_in(NDS& nds, u8 byte) {
    for (u8 i = 0; i < 8u; ++i) {
        clock_bit_in(nds, static_cast<u8>((byte >> i) & 1u));
    }
}

// Frame each transfer with a /CS low→high transition while /SCK is low.
static void assert_cs_high(NDS& nds) {
    rtc_write(nds, kDirAllCpuDriven);                              // /CS low
    rtc_write(nds, static_cast<u8>(kDirAllCpuDriven | (1u << 2))); // /CS rising
}

static void write_status2(NDS& nds, u8 value) {
    assert_cs_high(nds);
    clock_byte_in(nds, 0x64u); // 110b | W | cmd=4
    clock_byte_in(nds, value);
}

static void write_alarm1(NDS& nds, u8 dow, u8 hour, u8 min) {
    assert_cs_high(nds);
    clock_byte_in(nds, 0x61u); // 110b | W | cmd=1
    clock_byte_in(nds, dow);
    clock_byte_in(nds, hour);
    clock_byte_in(nds, min);
}

// Read one chip-output byte while SIO direction = chip-drives. Toggle /SCK
// low → high; sample SIO on the high half. LSB-first.
static u8 read_byte_out(NDS& nds) {
    u8 byte = 0;
    for (u8 i = 0; i < 8u; ++i) {
        rtc_write(nds, static_cast<u8>(kDirChipDrivesSio | (1u << 2)));
        const u8 bit = static_cast<u8>(rtc_read(nds) & 1u);
        byte = static_cast<u8>(byte | (bit << i));
        rtc_write(nds, static_cast<u8>(kDirChipDrivesSio | (1u << 2) | (1u << 1)));
    }
    return byte;
}

// Issue a Status1 read command (0x70 = 110b | R | cmd=0) and consume the
// one-byte chip response. The §5.4 auto-clear of status1 bits 4..7 fires at
// the end of this call.
static u8 read_status1(NDS& nds) {
    assert_cs_high(nds);
    clock_byte_in(nds, 0x70u);
    return read_byte_out(nds);
}

static void Rtc_FullBootHandshake() {
    NDS nds;

    // Step 1: implicit reset() seeded {2026,1,1,4,0,0,0} and queued the
    // first Rtc1HzTick at scheduler.now()+kArm9CyclesPerSecond.
    REQUIRE(nds.rtc().now_datetime().year == 2026u);
    REQUIRE(nds.rtc().now_datetime().sec == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 2: ARM7 enables IF.7 via IME and IE through the bus.
    nds.arm7_io_write32(IO_IME, 1u);
    nds.arm7_io_write32(IO_IE, 0x80u);
    REQUIRE(nds.irq7().read_ime() == 1u);
    REQUIRE(nds.irq7().read_ie() == 0x80u);
    REQUIRE(nds.cpu7().irq_line() == false); // no IF bit set yet

    // Step 3: ARM7 bit-bangs the alarm setup. Status2=0x04 enables Alarm-1;
    // alarm fires at any time whose minute equals 0 (only the minute field
    // has the compare-enable bit 7 set).
    write_status2(nds, 0x04u);
    write_alarm1(nds, 0x00u, 0x00u, static_cast<u8>(0x80u | bcd(0u)));
    REQUIRE(nds.rtc().status2() == 0x04u);
    REQUIRE(nds.rtc().alarm1().min == static_cast<u8>(0x80u | bcd(0u)));

    // Step 4: seed dt to the second before the rising edge. seed() also
    // clears the prev_alarm*_match latches so the next tick is guaranteed
    // to be a 0→1 transition.
    nds.rtc().seed({2026, 1, 1, 4, 0, 59, 59});

    // Step 5: poke an immediate Rtc1HzTick into the scheduler and let
    // run_frame drain it. The handler advances dt to 1:00:00, the alarm
    // matches on a rising edge, and update_arm7_irq_signals() is called
    // before the loop returns — flipping cpu7_.irq_line() to true on the
    // same scheduler step.
    nds.scheduler().schedule_in(0, EventKind::Rtc1HzTick);
    nds.run_frame();

    // Step 6: end-to-end assertions.
    REQUIRE(nds.rtc().now_datetime().hour == 1u);
    REQUIRE(nds.rtc().now_datetime().min == 0u);
    REQUIRE(nds.rtc().now_datetime().sec == 0u);
    REQUIRE((nds.irq7().read_if() & 0x80u) != 0u);
    REQUIRE(nds.rtc().int1_latched()); // status1 bit 4
    REQUIRE(nds.cpu7().irq_line() == true);

    // Step 7: bit-bang Status1 — bit 4 reads as 1 then auto-clears. Bit 1
    // (24-hour mode) is set on reset and is preserved across the read.
    const u8 s1 = read_status1(nds);
    REQUIRE((s1 & 0x10u) != 0u);
    REQUIRE((s1 & 0x02u) != 0u);
    REQUIRE(!nds.rtc().int1_latched()); // auto-cleared on read
    REQUIRE((nds.rtc().status1() & 0x10u) == 0u);

    // Step 8: ARM7 acknowledges IF.7 via a write-1-clear store. The bus
    // path runs update_arm7_irq_signals() and the line drops.
    nds.arm7_io_write32(IO_IF, 0x80u);
    REQUIRE((nds.irq7().read_if() & 0x80u) == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);

    // Step 9: walk dt past the match window. Direct rtc.tick() calls are
    // sufficient — step 5 already proved the scheduler dispatch path. The
    // assertions below cover (a) stay-matching ticks in 1:00:01..1:00:59
    // and (b) the falling-edge tick at 1:00:59→1:01:00. None must raise.
    for (u32 i = 0; i < 60u; ++i) {
        nds.rtc().tick(nds.irq7());
        REQUIRE((nds.irq7().read_if() & 0x80u) == 0u);
    }
    REQUIRE(nds.rtc().now_datetime().min == 1u);
    REQUIRE(nds.rtc().now_datetime().sec == 0u);
    REQUIRE(nds.cpu7().irq_line() == false);
}

int main() {
    std::puts("[Rtc_FullBootHandshake]");
    Rtc_FullBootHandshake();
    std::puts("rtc_integration_test: OK");
    return 0;
}
