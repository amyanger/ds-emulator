#pragma once

// RTC (0x4000138, NDS7-only) — Seiko S-35190A-compatible real-time clock on
// a bit-banged 3-wire SIO bus. The CPU drives one 8-bit register that maps
// directly onto the chip's pins:
//
//   bit 0 = SIO data (CPU↔chip serial line)
//   bit 1 = /SCK     (serial clock, active-low strobe)
//   bit 2 = /CS      (chip select / transfer frame, active-low)
//   bit 4 = SIO direction (1 = CPU drives, 0 = chip drives)
//   bit 5 = /SCK direction (always 1 in practice)
//   bit 6 = /CS  direction (always 1 in practice)
//   bits 3, 7 = reserved (round-trip whatever was written)
//
// ARM9 has no mapping for 0x4000138 — reads are open-bus on real hardware,
// and Arm9Bus deliberately does not route this address.
//
// The full public surface is declared up front so the SIO state machine,
// per-command handlers, calendar advance, alarm rising-edge raise, and
// scheduler-driven 1 Hz tick can be filled in without churning headers
// or callers.

#include "ds/common.hpp"

namespace ds {

class IrqController;

class Rtc {
public:
    struct DateTime {
        u16 year = 2026; // full 4-digit year; chip stores year-2000 in BCD
        u8 month = 1;    // 1..12
        u8 day = 1;      // 1..31
        u8 dow = 4;      // 0..6 (0=Sunday by DS firmware convention; default is Thursday)
        u8 hour = 0;     // 0..23 (24-hour mode; bit 1 of status1)
        u8 min = 0;      // 0..59
        u8 sec = 0;      // 0..59
    };

    struct Alarm {
        u8 dow = 0;  // bits 0-2 valid; bit 7 = compare enable
        u8 hour = 0; // bits 0-5 valid; bit 7 = compare enable
        u8 min = 0;  // bits 0-6 valid; bit 7 = compare enable
    };

    void reset();

    // Inject wall-clock seed. Frontend calls once at startup; reset() also
    // installs a default value so libds_core never depends on host time.
    // Tests pass fixed timestamps for determinism.
    void seed(const DateTime& dt);

    // Bus-side access — drives the SIO bit-bang state machine.
    u8 read_pins() const;
    void write_pins(u8 value);

    // Scheduler-side: advance one second and re-run alarm comparison. Raises
    // ARM7 IF.7 via irq7.raise() on rising edge of an enabled alarm match.
    // IrqController is taken by reference at call time (rule 3: Rtc holds
    // no pointer to another subsystem); NDS dispatches `rtc_.tick(irq7_)`.
    void tick(IrqController& irq7);

    // Test introspection — not for cross-subsystem callers.
    DateTime now_datetime() const;
    u8 status1() const;
    u8 status2() const;
    Alarm alarm1() const;
    Alarm alarm2() const;
    bool int1_latched() const; // status1 bit 4
    bool int2_latched() const; // status1 bit 5

private:
    // ---- SIO bit-bang state ----
    // Direction bits round-trip verbatim so reads see what was written. The
    // /SCK rising edge shifts a bit; the in-flight transfer is tracked in
    // bit_idx_ + xfer_phase_ + active_cmd_.
    enum class Phase : u8 { Idle, Command, Param };
    enum class Cmd : u8 {
        Status1,
        Status2,
        DateTime,
        Date,
        Time,
        Alarm1,
        Alarm2,
        Free,    // unimplemented — warn-log; data dropped
        FreqSel, // unimplemented — warn-log; data dropped
        Unknown
    };

    u8 pins_ = 0;
    bool prev_sck_high_ = false;
    bool prev_cs_high_ = false;
    Phase xfer_phase_ = Phase::Idle;
    u8 shift_byte_ = 0; // LSB-first accumulator (write) or queue (read)
    u8 bit_idx_ = 0;    // 0..7 within current byte
    u8 param_byte_ = 0; // index of the current parameter byte
    Cmd active_cmd_ = Cmd::Unknown;
    bool active_read_ = false; // bit 4 of the command byte (1=read)

    // ---- Real-time state ----
    DateTime dt_{};
    u8 status1_ = 0;
    u8 status2_ = 0;
    Alarm alarm1_{};
    Alarm alarm2_{};

    // Edge-trigger latches. Set true once an enabled alarm condition becomes
    // true; cleared when the condition goes false again. tick() raises IF.7
    // only on a 0→1 transition.
    bool prev_alarm1_match_ = false;
    bool prev_alarm2_match_ = false;

    // ---- helpers ----
    static u8 to_bcd(u8 binary);
    static u8 from_bcd(u8 bcd);
    static u8 days_in_month(u16 year, u8 month);
    static bool is_leap(u16 year);

    void apply_write_byte(u8 byte);
    u8 produce_read_byte();
    bool alarm_matches(const Alarm& a) const;
};

} // namespace ds
