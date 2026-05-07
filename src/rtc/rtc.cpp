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
    // When the chip is driving SIO during a Param-phase read, replace bit 0
    // with the current chip-driven bit (LSB-first out of shift_byte_). All
    // other bits round-trip the last value the CPU wrote — the direction
    // bits, /SCK, /CS, and reserved bits 3/7 must all read back verbatim so
    // the firmware's read-direction-bit checks pass.
    if (xfer_phase_ == Phase::Param && active_read_ && active_cmd_ != Cmd::Unknown) {
        const u8 chip_bit = static_cast<u8>((shift_byte_ >> bit_idx_) & 1u);
        return static_cast<u8>((pins_ & 0xFEu) | chip_bit);
    }
    return pins_;
}

void Rtc::write_pins(u8 value) {
    // Bit-bang SIO state machine. /CS framing rules:
    //   * /CS falling  → abort any in-flight transfer (back to Idle).
    //   * /CS rising   → start a new Command-phase transfer.
    //   * /SCK rising while /CS is high and we're not Idle → shift one bit.
    // Bit 4 (SIO direction) selects who drives SIO during Param phase: 1 =
    // CPU writes the chip, 0 = chip drives the line. In Command phase the
    // CPU is always master and we sample regardless of bit 4.
    const u8 new_cs = static_cast<u8>((value >> 2) & 1u);
    const u8 new_sck = static_cast<u8>((value >> 1) & 1u);
    const u8 new_sio = static_cast<u8>(value & 1u);
    const u8 new_dir_write = static_cast<u8>((value >> 4) & 1u);

    const bool cs_falling = prev_cs_high_ && !new_cs;
    const bool cs_rising = !prev_cs_high_ && new_cs;
    const bool sck_rising = !prev_sck_high_ && new_sck;

    if (cs_falling) {
        xfer_phase_ = Phase::Idle;
        bit_idx_ = 0;
        shift_byte_ = 0;
        param_byte_ = 0;
        active_cmd_ = Cmd::Unknown;
        active_read_ = false;
    } else if (cs_rising) {
        xfer_phase_ = Phase::Command;
        bit_idx_ = 0;
        shift_byte_ = 0;
        param_byte_ = 0;
        active_cmd_ = Cmd::Unknown;
    } else if (new_cs && sck_rising && xfer_phase_ != Phase::Idle) {
        // Sample SIO when the CPU is driving the line. In Command phase the
        // CPU is always master; in Param phase the direction bit picks who
        // drives.
        const bool sample_input = (xfer_phase_ == Phase::Command) || (new_dir_write != 0u);
        if (sample_input) {
            shift_byte_ = static_cast<u8>(shift_byte_ | (new_sio << bit_idx_));
        }
        ++bit_idx_;

        if (bit_idx_ == 8u) {
            bit_idx_ = 0;
            if (xfer_phase_ == Phase::Command) {
                // Command byte layout per slice 3j spec §5.3: bits 5..7 = fixed
                // pattern 110b (0x60), bit 4 = R/W (1 = read), bits 0..3 = 4-bit
                // command index (Time = 0xA). Mask 0xE0 isolates the fixed
                // pattern only.
                //
                // TODO(slice-3j-real-game-integration): GBATEK's "Fwd / Rev"
                // table admits a second interpretation where fixed bits sit in
                // bits 0..3 (low nibble = 0x06) and command is a 3-bit field
                // in bits 4..6 with R/W in bit 7. melonDS handles both via a
                // bit-reverse lookup keyed on `(val & 0xF0) == 0x60`. Real DS
                // firmware may send command bytes MSB-first per the original
                // Seiko S-35190A datasheet — this decoder will need a melonDS-
                // style auto-detect before HG/SS can complete an RTC handshake.
                // Tests today drive the spec's chosen convention directly.
                if ((shift_byte_ & 0xE0u) != 0x60u) {
                    DS_LOG_WARN(
                        "rtc: command byte 0x%02X has invalid fixed bits 5..7 (expected 110b)",
                        shift_byte_);
                    active_cmd_ = Cmd::Unknown;
                    // Do NOT advance to Param; wait for /CS toggle to retry.
                } else {
                    active_read_ = (shift_byte_ & 0x10u) != 0u;
                    switch (shift_byte_ & 0x0Fu) {
                    case 0x0:
                        active_cmd_ = Cmd::Status1;
                        break;
                    case 0x1:
                        active_cmd_ = Cmd::Alarm1;
                        break;
                    case 0x2:
                        active_cmd_ = Cmd::DateTime;
                        break;
                    case 0x3:
                        active_cmd_ = Cmd::Free;
                        break;
                    case 0x4:
                        active_cmd_ = Cmd::Status2;
                        break;
                    case 0x5:
                        active_cmd_ = Cmd::Alarm2;
                        break;
                    case 0x6:
                        active_cmd_ = Cmd::Date;
                        break;
                    case 0x7:
                        active_cmd_ = Cmd::FreqSel;
                        break;
                    case 0xA:
                        active_cmd_ = Cmd::Time;
                        break;
                    default:
                        active_cmd_ = Cmd::Unknown;
                        break;
                    }
                    xfer_phase_ = Phase::Param;
                    param_byte_ = 0;
                }
                shift_byte_ = 0;
            } else { // Phase::Param
                // Commit-2 stub: byte-boundary work belongs in commits 3/4.
                // TODO(slice-3j-commit-3): wire produce_read_byte()
                // TODO(slice-3j-commit-4): wire apply_write_byte(shift_byte_)
                //
                // Wiring hazard for commits 3/4 — bit_idx_ ordering on read:
                // bit_idx_ is incremented on every /SCK rising (above) before
                // the CPU has a chance to sample the line. read_pins() reads
                // (shift_byte_ >> bit_idx_) & 1, which means the CPU sees the
                // bit at the post-increment index, not the just-clocked bit.
                // For LSB-first read commands, produce_read_byte() must either
                // (a) preload shift_byte_ such that bit (bit_idx_) is the next
                // outgoing bit at all times (i.e. shift down by 1 after every
                // /SCK rising), or (b) drive the chip-output bit on /SCK
                // falling rather than rising, so the CPU samples a stable
                // value during the high half-cycle. Per GBATEK §5.2 ("Output
                // /SCK = HIGH ... then read SIO"), real CPU timing samples
                // after the rising edge — option (a) is the simpler fit.
                //
                // Wiring hazard for commit 3 — param_byte_ wrap: this counter
                // is u8 and increments unbounded if a buggy/malicious ROM
                // holds /CS high and clocks past the per-command max. The
                // commit-3 dispatcher must clamp at max_bytes_for(active_cmd_)
                // rather than wrapping at 255.
                shift_byte_ = 0;
                ++param_byte_;
            }
        }
    }

    // Always commit pins and edge memory at the end so direction-bit reads
    // round-trip and the next call sees the correct prev_* edges.
    pins_ = value;
    prev_cs_high_ = (new_cs != 0u);
    prev_sck_high_ = (new_sck != 0u);
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
