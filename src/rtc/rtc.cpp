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
                //
                // The bit-numbering question also implicates the opcode→Cmd
                // mapping below: under GBATEK's literal Fwd table, opcode 6 =
                // Time-only (hh,mm,ss), opcode 7 = Free, opcode 3 = clock-
                // adjustment, and the "Date-only" command does not exist.
                // The slice spec's mapping (opcode 6 = Date, opcode 0xA =
                // Time, etc.) follows the S-35190A datasheet's MSB-first
                // byte view. Both decisions resolve together — auto-detect
                // bit-numbering + reconcile opcode table — before HG/SS.
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
                // Read direction: load the first parameter byte for the
                // chip-output path. Write direction: shift_byte_ accumulates
                // the incoming byte from bit 0 up.
                shift_byte_ =
                    (active_read_ && active_cmd_ != Cmd::Unknown) ? produce_read_byte() : 0u;
            } else { // Phase::Param
                if (active_read_) {
                    // Status1 auto-clear: bits 4..7 are read-only flags that
                    // clear when the CPU completes a Status1 read. Spec §5.4
                    // places the clear "at the moment the last bit of the
                    // byte is shifted out" — exactly here, after the 8th
                    // /SCK rising for the byte the CPU just read. Only fires
                    // on a *complete* read of byte 0; a /CS abort mid-byte
                    // never reaches this code path.
                    if (active_cmd_ == Cmd::Status1 && param_byte_ == 0u) {
                        status1_ &= 0x0Fu;
                    }
                } else {
                    // Write direction: dispatch the freshly-shifted byte to the active
                    // command's field. Must run BEFORE the saturating param_byte_ advance
                    // below so apply_write_byte() sees the index of the byte that was
                    // just received (0 = first param byte, 1 = second, ...).
                    apply_write_byte(shift_byte_);
                }

                // Saturating advance — clamp at max so a buggy ROM clocking
                // past max-byte-count cannot wrap u8 back to 0 and silently
                // replay byte 0.
                const u8 max = max_bytes_for(active_cmd_);
                if (param_byte_ < max) {
                    ++param_byte_;
                }

                // Read direction: load the next byte's chip-output value if
                // more bytes remain in this command. Over-clock past max
                // leaves shift_byte_ unchanged (last byte's bits keep
                // echoing — open-bus-equivalent).
                if (active_read_ && active_cmd_ != Cmd::Unknown && param_byte_ < max) {
                    shift_byte_ = produce_read_byte();
                } else if (!active_read_) {
                    // Write direction: clear accumulator for the next byte.
                    shift_byte_ = 0;
                }
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

void Rtc::apply_write_byte(u8 byte) {
    switch (active_cmd_) {
    case Cmd::Status1:
        // Bits 0..3 are R/W (Reset W-only, 12/24h R/W, GP1/GP2 R/W). Bits
        // 4..7 are R-only flags set by tick() and cleared by a complete
        // read. Preserve the upper nibble verbatim so a Status1 write
        // cannot stomp an alarm-fired flag the CPU has not yet seen.
        status1_ = static_cast<u8>((status1_ & 0xF0u) | (byte & 0x0Fu));
        return;
    case Cmd::Status2:
        // All 8 bits R/W per GBATEK (including bit 7 "Test Mode, don't use").
        // Stored verbatim; we don't model test-mode behavior.
        status2_ = byte;
        return;
    case Cmd::Alarm1:
        switch (param_byte_) {
        case 0u:
            alarm1_.dow = byte;
            return;
        case 1u:
            alarm1_.hour = byte;
            return;
        case 2u:
            alarm1_.min = byte;
            return;
        default:
            return; // over-clock guarded by saturating advance
        }
    case Cmd::Alarm2:
        switch (param_byte_) {
        case 0u:
            alarm2_.dow = byte;
            return;
        case 1u:
            alarm2_.hour = byte;
            return;
        case 2u:
            alarm2_.min = byte;
            return;
        default:
            return;
        }
    case Cmd::DateTime:
    case Cmd::Date:
    case Cmd::Time:
        DS_LOG_WARN("rtc: write to read-only command (cmd=%d)", static_cast<int>(active_cmd_));
        return;
    case Cmd::Free:
    case Cmd::FreqSel:
        DS_LOG_WARN("rtc: write to unimplemented register (cmd=%d)", static_cast<int>(active_cmd_));
        return;
    case Cmd::Unknown:
    default:
        // Reachable: a write-direction command byte with valid fixed bits
        // 5..7 but an unrecognized low nibble (opcodes 0x8/0x9/0xB-0xF)
        // sets active_cmd_=Unknown AND advances to Param phase, so write
        // bytes land here. Read-side preload is guarded by
        // `active_cmd_ != Unknown`; the write-side intentionally lets the
        // ROM finish its transfer to /CS deassert without stalling. Bytes
        // are silently discarded with a warn-log.
        DS_LOG_WARN("rtc: apply_write_byte() with Cmd::Unknown — discarding "
                    "byte from write to unrecognized opcode");
        return;
    }
}

u8 Rtc::datetime_byte(u8 idx) const {
    // Per spec §5.6: 7-byte BCD-encoded sequence Year, Month, Day, DoW, Hour,
    // Min, Sec. Year stores year-2000 in BCD, so it wraps at 99→00 — past
    // year 2099 the chip silently rolls over, matching real-hardware
    // S-35190A behavior.
    switch (idx) {
    case 0u:
        return to_bcd(static_cast<u8>(dt_.year % 100u)); // year-2000 BCD; wraps past 2099
    case 1u:
        return to_bcd(dt_.month);
    case 2u:
        return to_bcd(dt_.day);
    case 3u:
        return dt_.dow; // raw 0..6, single nibble — no BCD encoding
    case 4u:
        return to_bcd(dt_.hour); // 24-hour mode (status1 bit 1 default 1)
    case 5u:
        return to_bcd(dt_.min);
    case 6u:
        return to_bcd(dt_.sec);
    default:
        return 0u;
    }
}

u8 Rtc::produce_read_byte() {
    // Pure function — must not mutate. Status1's auto-clear of bits 4..7 on
    // read is performed in write_pins()'s byte-end block, gated on a
    // *complete* byte read, not here.
    switch (active_cmd_) {
    case Cmd::Status1:
        return status1_;
    case Cmd::Status2:
        return status2_;
    case Cmd::DateTime:
        return datetime_byte(param_byte_); // indices 0..6
    case Cmd::Date:
        return datetime_byte(param_byte_); // indices 0..2
    case Cmd::Time:
        return datetime_byte(static_cast<u8>(param_byte_ + 4u)); // indices 4..6
    // Alarm reads: param_byte_ is 0..2 by the call-site guard
    // (`param_byte_ < max_bytes_for(Alarm*) == 3`). Use `% 3u` to encode
    // the bound directly so a future call-site change cannot OOB.
    case Cmd::Alarm1: {
        const u8 fields[3] = {alarm1_.dow, alarm1_.hour, alarm1_.min};
        return fields[param_byte_ % 3u];
    }
    case Cmd::Alarm2: {
        const u8 fields[3] = {alarm2_.dow, alarm2_.hour, alarm2_.min};
        return fields[param_byte_ % 3u];
    }
    case Cmd::Free:
    case Cmd::FreqSel:
        DS_LOG_WARN("rtc: read of unimplemented register (cmd=%d)", static_cast<int>(active_cmd_));
        return 0u;
    case Cmd::Unknown:
    default:
        DS_LOG_WARN("rtc: produce_read_byte() invoked with Cmd::Unknown — "
                    "state machine invariant violated");
        return 0u;
    }
}

bool Rtc::alarm_matches(const Alarm& /*a*/) const {
    return false;
}

} // namespace ds
