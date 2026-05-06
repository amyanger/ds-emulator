#include "rtc/rtc.hpp"

#include "interrupt/irq_controller.hpp"

namespace ds {

void Rtc::reset() {
    pins_ = 0;
    prev_sck_high_ = false;
    prev_cs_high_ = false;
    xfer_phase_ = Phase::Idle;
    shift_byte_ = 0;
    bit_idx_ = 0;
    param_byte_ = 0;
    active_cmd_ = Cmd::Unknown;
    active_read_ = false;
    // Status1 bit 1 = 24-hour mode (set on reset; Pokemon never flips it).
    // Bit 7 (power-off flag) is intentionally not modeled — real hardware
    // sets it on cold boot and clears it after a few ms; we skip the latency.
    status1_ = 0x02u;
    status2_ = 0;
    alarm1_ = Alarm{};
    alarm2_ = Alarm{};
    prev_alarm1_match_ = false;
    prev_alarm2_match_ = false;
    // Default-init dt_ so libds_core has a deterministic baseline; host
    // time is injected via seed() and is never read from inside the core.
    dt_ = DateTime{};
}

void Rtc::seed(const DateTime& dt) {
    dt_ = dt;
}

u8 Rtc::read_pins() const {
    return pins_;
}

void Rtc::write_pins(u8 value) {
    // Pins round-trip so direction-bit reads see what was written; the SIO
    // transfer state machine latches onto /SCK edges from here.
    pins_ = value;
}

void Rtc::tick(IrqController& /*irq7*/) {}

Rtc::DateTime Rtc::now_datetime() const {
    return dt_;
}

u8 Rtc::status1() const {
    return status1_;
}

u8 Rtc::status2() const {
    return status2_;
}

Rtc::Alarm Rtc::alarm1() const {
    return alarm1_;
}

Rtc::Alarm Rtc::alarm2() const {
    return alarm2_;
}

bool Rtc::int1_latched() const {
    return (status1_ & 0x10u) != 0u;
}

bool Rtc::int2_latched() const {
    return (status1_ & 0x20u) != 0u;
}

// ---- Static helpers ----

u8 Rtc::to_bcd(u8 binary) {
    return static_cast<u8>(((binary / 10u) << 4) | (binary % 10u));
}

u8 Rtc::from_bcd(u8 bcd) {
    return static_cast<u8>(((bcd >> 4) & 0x0Fu) * 10u + (bcd & 0x0Fu));
}

bool Rtc::is_leap(u16 year) {
    if ((year % 4u) != 0u)
        return false;
    if ((year % 100u) != 0u)
        return true;
    return (year % 400u) == 0u;
}

u8 Rtc::days_in_month(u16 year, u8 month) {
    static constexpr u8 table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    // Returns 0 for an out-of-range month (treat as a sentinel: callers must
    // never compare `day > days_in_month(...)` without a non-zero guard, or a
    // bad month would make the comparison true on every tick and silently
    // walk the calendar forward).
    if (month < 1u || month > 12u)
        return 0u;
    if (month == 2u && is_leap(year))
        return 29u;
    return table[month - 1u];
}

// ---- Stubs ----

void Rtc::apply_write_byte(u8 /*byte*/) {}

u8 Rtc::produce_read_byte() {
    return 0u;
}

bool Rtc::alarm_matches(const Alarm& /*a*/) const {
    return false;
}

} // namespace ds
