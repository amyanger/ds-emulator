# RTC + Scheduler-Driven IRQ — Phase 1, Slice 3j Design

**Date:** 2026-05-05
**Slice:** ARM7 RTC subsystem at `0x4000138`, the first scheduler-driven IRQ
source on the project (1 Hz tick that advances internal date/time and, when
an enabled alarm matches, raises NDS7 IF.7 via the existing edge-triggered
discipline introduced in slice 3i).
**Status:** closed (2026-05-16) — landed through `f42b0d8`; 71 CTest binaries
green. All §9 completion criteria ticked. No deviations from the design — the
seven planned commits shipped in the documented order, the six new test
binaries registered as planned, and the file-size estimates held (all under
the 500-line soft cap; `rtc.cpp` 461 lines vs the ~430 projection, largest
new test `rtc_commands_test.cpp` 354 lines).
**Prior slice:** 3i (IPC + ARM9 IRQ plumbing) — landed through `0e564e6`; 65
CTest binaries green.
**Next slice (proposed):** 3k (keypad + KEYINPUT/KEYCNT + EXTKEYIN + lid
switch — second/third scheduler-driven IRQ sources, ARM7 IF.12 keypad and
ARM7 IF.22 screens-unfolded). Scoping in progress.

---

## 1. Summary

Slice 3i wired three IPC interrupt sources into the new dual-CPU IRQ
controller: IF.16 (IPC SYNC bit-13 ping), IF.17 (send-FIFO empty edge), and
IF.18 (recv-FIFO not-empty edge). All three of those raise *synchronously*
inside the bus write that caused them — no scheduler involvement, no time
component, no reason to ever reorder a raise relative to another event.

Slice 3j adds the **first IRQ source where the raise is decoupled from a
bus write**: the Real-Time Clock. The DS RTC is a Seiko S-35180 / S-35190A-
compatible chip on the NDS7 SPI-like bit-banged bus at `0x4000138`. It
keeps date / time in BCD, supports two programmable alarms, and asserts
its INT1 / INT2 lines into the SIO unit, which surface on **NDS7 IF.7
("SIO/RCNT/RTC")**. There is no dedicated RTC bit in IE/IF.

The chip's seconds counter advances *whether or not* the CPU is reading
its registers — that is the piece of behavior that makes RTC the right
slice to introduce scheduler-driven interrupts. We schedule a recurring
event (`EventKind::Rtc1HzTick`) at 1-second intervals. NDS routes the
event to a single method on the RTC subsystem, which advances the date /
time, runs the alarm comparison, and asks the controller to raise IF.7
on the rising edge of an enabled alarm match. The infrastructure that
lands here — recurring scheduler events, alarm rising-edge latching,
IF.7 raise pathway — is the template for the keypad + lid switch in
slice 3k and for every later scheduler-driven IRQ source (PPU VBlank /
HBlank / VCount, timers, DMA completion, cart DMA, GXFIFO drain).

### Plain-language summary

The Nintendo DS has a clock chip that ticks once per second whether the
console is doing anything else or not. The CPU talks to that chip through
a bit-banged 3-wire serial bus — every command is sent one bit at a time
by toggling pins through a single 8-bit I/O register. The chip can keep
the current date and time, store two alarms, and ping the CPU when the
current time matches an alarm.

For the emulator, this means three things, and we have to do all three at
once because they don't separate cleanly:

1. **Bit-bang the protocol.** The game writes "set serial-clock low,
   put bit 0 of the next byte on the data line", "set serial-clock high",
   "read the data line", over and over. We model the chip as a state
   machine that consumes / produces one bit per `/SCK` rising edge.
2. **Keep time on a separate clock from the CPU.** A scheduler event
   fires once per emulated second (not real wall-clock second) and bumps
   the internal date / time forward.
3. **Compare alarms and raise IF.7 on a rising edge.** The chip has
   two alarm registers; when the current time matches an enabled alarm,
   the chip pulls its INT line high. Translating that to our world: we
   raise IF.7 on the ARM7 IRQ controller, but only on the *transition*
   into match (the same edge-trigger discipline slice 3i introduced for
   IPC FIFO IF.17 / IF.18). Alarms that stay matched do not re-raise;
   alarms that fire and are acknowledged do not re-raise until the
   match condition has gone false and come back true.

The hard parts — where bugs hide — are not the data structure. They are:

- **Two clocks need to disagree exactly.** Inside the emulator the
  scheduler is the only authority on emulator time, but RTC time has to
  feel like wall-clock time to the game. Resolution: emulator time is
  ARM9 cycles, RTC time advances 1 emulated second per
  `kArm9FrequencyHz` cycles, and the *initial* wall-clock time is
  injected by the frontend at startup. Tests inject a fixed initial
  time so determinism holds.

- **Bit-bang state machine is fiddly.** The protocol is
  `/SCK ↑ → write_one_bit; /SCK ↓ → next bit position`, with `/CS = high`
  framing the whole transfer, and `LSB-first` ordering for both command
  and data. Get the edge polarity backwards or the bit ordering wrong
  and the chip looks dead. The state machine has to track: are we in a
  transfer? Are we sending the command byte or a parameter byte? How
  many bits into the byte are we? Read or write?

- **Alarm semantics are partially undocumented.** GBATEK gives field
  layouts but not the exact "every minute" vs "every hour" vs "exactly
  once per day" timing of alarm fire. We resolve the ambiguity by
  picking concrete behavior and documenting it (§5.7).

The architectural decisions in this slice are larger than slice 3i's
because we are introducing a new subsystem class, a new event kind on the
scheduler, a new IRQ source bit on the ARM7 controller, and a new
host-injection point on NDS. None of them is individually big; the value
comes from doing them all to the same standard.

**What this slice builds:**

- `class Rtc` in `src/rtc/rtc.{hpp,cpp}`. State: bit-bang pin state,
  SIO transfer state machine, internal date/time + alarms in BCD-decoded
  form, status register 1 (flags) + status register 2 (INT enables),
  edge-trigger latches for INT1/INT2.
- `EventKind::Rtc1HzTick` enumerator on `Scheduler::EventKind`.
- `NDS::on_scheduler_event` switch arm that routes the tick to RTC, then
  calls `update_arm7_irq_signals()` if the tick raised an alarm.
- ARM7 bus route for `IO_RTC` (`0x4000138`, byte-wide).
- Constant `IO_RTC` in `src/bus/io_regs.hpp`.
- `NDS::seed_rtc_from_host_time(...)` accessor that the frontend (Phase 1
  later) calls before `run_frame()` to inject wall-clock time. Tests
  call the same accessor with a fixed timestamp.
- Six new test binaries: `rtc_protocol_test`, `rtc_commands_test`,
  `rtc_alarm_test`, `rtc_tick_test`, `rtc_irq_test`, `rtc_integration_test`.

**What this slice deliberately does NOT build:**

- **Other IF.7 sources.** SIO transmission and RCNT pin polling also
  surface as IF.7 on NDS7. RTC is the only IF.7 source raised this
  slice. The full SIO unit lands when wifi / GBA-cart slot work begins
  (slice 3l+). When SIO is added, the IF.7 raise stays an OR of every
  contributing source — slice 3j just owns the first source.
- **Keypad (NDS7 + NDS9 IF.12), lid switch (NDS7 IF.22), screens-unfolded.**
  Slice 3k.
- **DSi RTC extensions.** FOUT pin, alarm-date register, century
  register. Out of emulator scope (NDS-only mandate, see CLAUDE.md
  §"Implementation Phases").
- **Free register / frequency-select register / 32 kHz output enable.**
  Peripheral; no Pokemon game touches them. Deferable. Routed as
  no-op-with-DEBUG-warn.
- **ARM9-side RTC access.** NDS7-only on real hardware; ARM9 reads of
  `0x4000138` are open-bus. Already the case in our `Arm9Bus` (no route
  exists for `0x4000138`). No change needed; called out for the GBATEK
  reviewer.
- **Save state serialization.** Per CLAUDE.md rule-5 carve-out, `reset()`
  only. RTC date/time is reseeded from host time on every `reset()` (and
  later, on every `load_state()` when the serialization pass lands).
- **Modeling chip startup time / power-on reset latency.** Real
  hardware has a power-on flag (status1 bit 7) that takes a few
  milliseconds to clear. Our `reset()` zeros the flag immediately.
  Documented; revisit only on real-game divergence.

### Scope boundary

**In scope:** the new `Rtc` class, the bit-bang state machine for
commands listed in §5.5, BCD-encoded date/time + alarm registers,
scheduler-driven 1 Hz tick, alarm rising-edge IF.7 raise, ARM7 bus
route for `0x4000138`, one new I/O register constant, one new event
kind, the host-time injection accessor on NDS, six new test binaries.

**Out of scope:** see §3.

---

## 2. Goals

1. **`0x4000138` R/W behaves per S-35190A datasheet on the ARM7 bus
   only.** ARM9 access is open-bus (already true). Byte writes drive
   the bit-bang state machine; byte reads return the current pin state
   with the data-input bit (bit 0) reflecting whatever the chip is
   currently shifting out. The four direction bits (bits 4, 5, 6, plus
   reserved bit 7) round-trip exactly.
2. **The SIO bit-bang state machine receives 8-bit commands LSB-first,
   then receives or transmits the parameter bytes appropriate to that
   command.** Status1 (1 byte read), Status1 (1 byte write), Status2
   (1 byte read), Status2 (1 byte write), Date+Time (7 bytes read),
   Date (3 bytes read), Time (3 bytes read), Alarm1 (3 bytes
   read/write), Alarm2 (3 bytes read/write). Each byte is itself
   transmitted LSB-first. The `/CS` line frames the whole multi-byte
   exchange; lowering `/CS` mid-transfer aborts.
3. **Internal date/time advances by one second per
   `kArm9CyclesPerSecond` ARM9 cycles.** The scheduler reschedules the
   `Rtc1HzTick` event after every fire. Wraps follow real-calendar
   rules: 60 sec → minute, 60 min → hour, 24 hr → day, day-of-week
   advances mod 7, days-in-month respects leap years (Gregorian rule:
   year mod 4 == 0 AND (year mod 100 != 0 OR year mod 400 == 0)).
4. **Alarms compare current time against the alarm register fields,
   gated by the per-field "compare enable" bits and the per-alarm INT
   enable bit in status2.** On rising edge of (enabled AND match), the
   appropriate INT1 or INT2 bit in status1 latches AND IF.7 is raised
   via `irq7_.raise(1u << 7)`. On falling edge, no raise; the latched
   status1 bit stays set until the CPU reads status1 (auto-clears bits
   4-7 on read per GBATEK).
5. **NDS exposes `seed_rtc_from_host_time(year, month, day, dow, hh, mm, ss)`.**
   Frontend uses real wall clock; tests pass fixed timestamps. Called
   inside `reset()` with a default value (2026-01-01 00:00:00, day-of-
   week = 4) when the frontend does not seed first. Determinism holds:
   no `std::chrono::now()` is ever called from inside `libds_core`.
6. **Seven commits, each shippable individually, each ending green.**
   Same discipline as slice 3i. CTest count 65 → 71 (six new binaries).
7. **No new architectural debt.** `Rtc` holds no pointer to any other
   subsystem (rule 3). The 1 Hz tick is dispatched by NDS, which then
   passes `irq7_` by reference into a single `Rtc::tick(IrqController&)`
   call. No cross-subsystem includes (rule 8): `rtc/*.cpp` includes
   `interrupt/irq_controller.hpp` only, the same leaf header that
   `ipc/*.cpp` includes.
8. **All new files stay well under the 500-line soft cap.** Estimated
   sizes: `rtc.hpp` ~110 lines, `rtc.cpp` ~430 lines (the soft cap is
   500; we will split if drafting blows past 480). Largest test file
   estimated 280 lines.

### 2.1 IRQ source coverage matrix (post-slice)

| Bit | Source                       | This slice? | Notes                                |
|-----|------------------------------|-------------|--------------------------------------|
| 7   | SIO/RCNT/RTC (NDS7 only)     | **YES**     | RTC only this slice; SIO / RCNT later.|
| 12  | Keypad (NDS9 / NDS7)         | NO          | Slice 3k.                            |
| 16  | IPC Sync                     | done (3i)   | Already raised.                      |
| 17  | IPC Send FIFO Empty          | done (3i)   | Already raised.                      |
| 18  | IPC Recv FIFO Not Empty      | done (3i)   | Already raised.                      |
| 22  | Screens unfolded (NDS7 only) | NO          | Slice 3k.                            |
| (rest) | (other sources)           | NO          | Future slices.                       |

**1 IRQ source goes from "no implementation" to "raised by a scheduler-
driven event" + 6 new test binaries. CTest count 65 → 71.**

---

## 3. Non-goals

- **Other IF.7 sources (SIO transmission, RCNT pin sample).** GBATEK is
  explicit that IF.7 is the OR of three distinct sources; we are
  implementing one. The raise pathway is a simple `irq7_.raise(1u << 7)`,
  so adding SIO / RCNT later is purely additive. Out of scope so this
  slice ships small.
- **Keypad / lid switch.** Their *existence* in this codebase is what
  motivates introducing scheduler-driven IRQs in this slice — the keypad
  also fires on rising-edge against KEYCNT — but they don't share data
  with RTC, so they go in slice 3k.
- **DSi extensions.** Out of emulator scope.
- **Free / frequency-select / 32 kHz-out / general-purpose registers.**
  Peripheral; no game-side test for them. Routed as no-op-with-DEBUG-warn
  so they're discoverable in trace if a Pokemon game ever touches them.
- **Modelling actual SIO chip-select / clock waitstates.** GBATEK gives
  ≥5 µs minimum hold times for `/SCK` transitions, but at our level of
  fidelity an entire byte exchange completes inside one bus write. We
  model the protocol as state-machine-correct, not timing-accurate.
- **Save state serialization.** CLAUDE.md rule-5 carve-out.
- **ARM9 RTC access.** Not on real hardware; nothing to do.
- **Sub-byte write to `0x4000138`.** It's a 1-byte register; halfword and
  word writes alias byte 0 + ignore the upper bytes. Routed via the
  existing 8-bit slow-path pattern at `nds.cpp:468..539`. No new code
  needed for this in §4.
- **Treating `std::chrono::system_clock::now()` as the source of RTC
  time inside `libds_core`.** The frontend reads host time and passes
  the seed via `seed_rtc_from_host_time(...)`. `libds_core` itself
  contains no calls into `<chrono>`. This keeps tests deterministic and
  preserves the no-SDL / no-platform discipline (rule 6 in spirit).

---

## 4. Architecture

### 4.1 File layout

```
src/rtc/                                NEW directory.
  rtc.hpp                               NEW. ~110 lines. class Rtc declaration:
                                          struct DateTime { u8 year, month, day,
                                                            dow, hour, min, sec; };
                                          struct Alarm    { u8 dow, hour, min;
                                                            u8 cmp_enable_mask; };
                                          enum class Cmd : u8 { … 8 commands … };
                                          void reset();
                                          void seed(const DateTime&);
                                          DateTime now_datetime() const;
                                          u8   read_pins() const;
                                          void write_pins(u8 value);
                                          void tick(IrqController& irq7);
                                          // test introspection only
                                          u8 status1() const;
                                          u8 status2() const;
                                          const Alarm& alarm1() const;
                                          const Alarm& alarm2() const;
                                          bool int1_latched() const;
                                          bool int2_latched() const;
                                        Internal: bit-bang transfer state + the
                                        9 register banks (1+1+7+3+3+3+3+1+1).
  rtc.cpp                               NEW. ~430 lines. SIO state machine,
                                        per-command read/write handlers, BCD
                                        helpers, calendar advance, alarm
                                        comparison, edge-trigger latch.
                                        Includes interrupt/irq_controller.hpp
                                        only — no cpu/ or bus/ includes.

src/bus/
  io_regs.hpp                           MODIFIED. Add:
                                          constexpr u32 IO_RTC = 0x04000138u;

src/scheduler/
  event.hpp                             MODIFIED. Add Rtc1HzTick to EventKind:
                                          enum class EventKind : u32 {
                                              FrameEnd,
                                              Rtc1HzTick,    // NEW
                                          };

src/
  nds.hpp                               MODIFIED. Add:
                                          Rtc rtc_{};
                                          Rtc& rtc() { return rtc_; }   // accessor
                                          void seed_rtc_from_host_time(
                                              u16 year, u8 month, u8 day, u8 dow,
                                              u8 hh, u8 mm, u8 ss);
                                        Include "rtc/rtc.hpp".
  nds.cpp                               MODIFIED. ~+40 lines:
                                          - reset() seeds default time + schedules
                                            first Rtc1HzTick.
                                          - on_scheduler_event() dispatches the
                                            new kind to rtc_.tick(irq7_) +
                                            update_arm7_irq_signals(); reschedule.
                                          - arm7_io_read8 / write8 route IO_RTC
                                            to rtc_.read_pins() / write_pins().
                                          - seed_rtc_from_host_time() body.

tests/
  rtc_protocol_test.cpp                 NEW. Bit-bang state machine: pin level
                                        tracking, /CS framing, command + parameter
                                        shift order. ~220 lines.
  rtc_commands_test.cpp                 NEW. Each command's read or write
                                        semantics (status1, status2, datetime,
                                        date, time, alarm1, alarm2). ~280 lines.
  rtc_alarm_test.cpp                    NEW. Alarm comparison + status1 INT
                                        flag latching + status1 read-clears
                                        bits 4-7. ~200 lines.
  rtc_tick_test.cpp                     NEW. 1 Hz event scheduling and
                                        calendar advance (mins, hours, days,
                                        leap years, dow rollover). ~180 lines.
  rtc_irq_test.cpp                      NEW. IF.7 raised on rising edge of
                                        enabled alarm match; gated by status2
                                        INT enables; not re-raised while still
                                        matching; re-raises after match-fall
                                        + match-rise. ~200 lines.
  rtc_integration_test.cpp              NEW. End-to-end: seed wall clock, run
                                        N frames worth of ticks, set alarm,
                                        confirm IF.7 + status1 latching from
                                        ARM7's perspective via bus IO. ~140
                                        lines.
```

### 4.2 Class designs

#### `Rtc`

Plain-language note: most of the surface is read-only test introspection.
The two hot methods are `read_pins` / `write_pins` (which the bus calls
on every `0x4000138` access) and `tick` (which NDS calls on every 1 Hz
event). Everything else exists so tests can verify state without
faking up bit-bang sequences.

```cpp
namespace ds {
class IrqController;

class Rtc {
public:
    struct DateTime {
        u16 year = 2026;   // full 4-digit year; chip stores year-2000 in BCD
        u8  month = 1;     // 1..12
        u8  day   = 1;     // 1..31
        u8  dow   = 4;     // 0..6 (0=Sunday per S-35190A datasheet)
        u8  hour  = 0;     // 0..23 (24-hour mode; bit 1 of status1)
        u8  min   = 0;     // 0..59
        u8  sec   = 0;     // 0..59
    };

    struct Alarm {
        u8 dow  = 0;             // bits 0-2 valid; bit 7 = compare enable
        u8 hour = 0;             // bits 0-5 valid; bit 7 = compare enable
        u8 min  = 0;             // bits 0-6 valid; bit 7 = compare enable
    };

    void reset();
    // Inject wall-clock seed. Frontend calls once at startup; reset()
    // calls with a default value if the frontend never did. Tests pass
    // fixed timestamps for determinism.
    void seed(const DateTime& dt);

    // Bus-side access — drives the SIO bit-bang state machine.
    u8   read_pins() const;
    void write_pins(u8 value);

    // Scheduler-side: advance one second + re-run alarm comparison.
    // Raises ARM7 IF.7 via irq7.raise() on rising edge of an enabled
    // alarm match.
    void tick(IrqController& irq7);

    // Test introspection — not for cross-subsystem callers.
    DateTime now_datetime() const;
    u8       status1() const;
    u8       status2() const;
    Alarm    alarm1() const;
    Alarm    alarm2() const;
    bool     int1_latched() const;  // reflects status1 bit 4
    bool     int2_latched() const;  // reflects status1 bit 5

private:
    // ---- SIO bit-bang state ----
    // Direction bits stored verbatim so reads see what was written. The
    // /SCK rising edge is what shifts a bit; the in-flight transfer is
    // tracked in bit_idx_ + cmd_received_ + xfer_phase_.
    u8   pins_           = 0;     // last byte written by CPU
    bool prev_sck_high_  = false; // edge detect helper
    bool prev_cs_high_   = false; // /CS edge
    enum class Phase : u8 { Idle, Command, Param };
    Phase   xfer_phase_  = Phase::Idle;
    u8      shift_byte_  = 0;     // LSB-first accumulator (write) or queue (read)
    u8      bit_idx_     = 0;     // 0..7 within current byte
    u8      param_byte_  = 0;     // index of the current parameter byte
    enum class Cmd : u8 {
        Status1, Status2, DateTime, Date, Time,
        Alarm1, Alarm2,
        Free, FreqSel,             // unimplemented (warn-log; data dropped)
        Unknown
    };
    Cmd     active_cmd_  = Cmd::Unknown;
    bool    active_read_ = false; // bit 4 of the command byte (1=read)

    // ---- Real-time state ----
    DateTime dt_{};
    u8       status1_ = 0;
    u8       status2_ = 0;
    Alarm    alarm1_{};
    Alarm    alarm2_{};

    // Edge-trigger latches. Set true once an enabled alarm condition
    // becomes true; cleared when the condition becomes false again.
    // tick() raises IF.7 only on a 0→1 transition.
    bool prev_alarm1_match_ = false;
    bool prev_alarm2_match_ = false;

    // ---- helpers ----
    static u8  to_bcd(u8 binary);   // 0..99 → packed BCD
    static u8  from_bcd(u8 bcd);    // packed BCD → 0..99
    static u8  days_in_month(u16 year, u8 month);
    static bool is_leap(u16 year);

    // Called at the end of every byte received in a write transfer.
    void apply_write_byte(u8 byte);
    // Called at the start of every byte transmitted in a read transfer.
    u8   produce_read_byte();

    bool alarm_matches(const Alarm& a) const;
};
} // namespace ds
```

The `pins_` byte and the SIO state are stored separately so that bus
reads at any moment return the last-written byte XORed with whatever bit
the chip is shifting onto SIO. That mirrors the chip's behavior (the
direction bits read back literally; the data line reads back what the
chip is asserting) without trying to model picosecond-level edge timing.

### 4.3 State ownership and dependency graph

```
NDS owns:
  Scheduler         scheduler_;
  IrqController     irq9_, irq7_;       // existing
  IpcSync           ipc_sync_;          // existing
  IpcFifo           ipc_fifo_;          // existing
  Rtc               rtc_;                // NEW
  Arm7              cpu7_;               // samples irq7_.line()
  Arm9              cpu9_;               // stub; no sampling
  Arm7Bus / Arm9Bus

NDS adds:
  void seed_rtc_from_host_time(u16, u8, u8, u8, u8, u8, u8);

Call graph (rule 3 — no inter-subsystem pointers):
  Arm7Bus::slow_*          → NDS::arm7_io_{read,write}8(0x4000138)
                           → rtc_.{read_pins, write_pins}
  Scheduler pop_due loop   → NDS::on_scheduler_event(Rtc1HzTick)
                           → rtc_.tick(irq7_)
                           → update_arm7_irq_signals()
                           → scheduler_.schedule_in(kArm9CyclesPerSecond, …)
  Frontend (Phase 1 later) → NDS::seed_rtc_from_host_time(...)
                           → rtc_.seed(dt)

Rtc holds no pointers to anything. IrqController is passed by reference
into the only path that can raise (tick).
```

`update_arm7_irq_signals()` body is unchanged from slice 3i:

```cpp
void NDS::update_arm7_irq_signals() {
    cpu7_.set_irq_line(irq7_.line());
    cpu7_.set_halt_wake_pending(irq7_.halt_wake_pending());
}
```

`on_scheduler_event` gains a single new switch arm (full body shown so
the ordering is unambiguous):

```cpp
void NDS::on_scheduler_event(const Event& ev) {
    switch (ev.kind) {
    case EventKind::FrameEnd:
        frame_done_ = true;
        break;
    case EventKind::Rtc1HzTick:
        rtc_.tick(irq7_);
        update_arm7_irq_signals();
        scheduler_.schedule_in(kArm9CyclesPerSecond, EventKind::Rtc1HzTick);
        break;
    }
}
```

`kArm9CyclesPerSecond` is a `constexpr Cycle` in `nds.cpp` set to
`67'027'964` (matches the existing `kFrameCycles = 1'120'380` ratio
× 59.82 Hz; documented as the canonical ARM9-clock figure in the master
design spec §9). Tests that exercise rollover do not depend on the exact
constant — they assert relative behavior across two consecutive `tick()`
calls.

Header-include rule check (rule 8):
- `rtc/rtc.hpp` includes `ds/common.hpp` only; forward-declares
  `IrqController`. Compliant.
- `rtc/rtc.cpp` includes `interrupt/irq_controller.hpp`. Same leaf
  header that `ipc/*.cpp` already includes. Compliant.
- `nds.hpp` adds `#include "rtc/rtc.hpp"` (NDS is the integration point;
  it is allowed to include from any subsystem).
- `nds.cpp` adds no new includes — `interrupt/irq_controller.hpp` is
  already pulled in transitively via `nds.hpp`.
- `scheduler/event.hpp` is already a leaf and gains one enumerator —
  no new includes needed.

### 4.4 IO routing (the new bus path)

**ARM7 bus** (`NDS::arm7_io_read*/write*`, existing pattern at
`src/nds.cpp:300..539`):

| Address      | Width  | Read                    | Write                       |
|--------------|--------|-------------------------|-----------------------------|
| `0x4000138`  | 8-bit  | `rtc_.read_pins()`      | `rtc_.write_pins(value)`    |
| `0x4000138`  | 16-bit | `rtc_.read_pins()` (low byte; high byte = 0) | low byte → `write_pins(value & 0xFF)` |
| `0x4000138`  | 32-bit | low byte = `read_pins()`, upper 24 bits = 0 | low byte → `write_pins(value & 0xFF)` |

GBATEK lists the register as 8-bit only; halfword / word access aliases
byte 0 with the upper bytes reading as zero and writes to the upper
bytes ignored. Identical pattern to the existing `IO_HALTCNT` 8-bit
register at `src/nds.cpp:469..478`.

**ARM9 bus**: not routed. ARM9 reads of `0x4000138` return 0 from the
default fall-through in `arm9_io_read{8,16,32}` and writes are silently
dropped; this matches GBATEK ("NDS7 only — ARM9 access is open-bus").

### 4.5 SIO state machine

Plain-language note: the chip is wired up like a 3-pin SPI device:
`/CS` (chip-select, active high — i.e. high = "we're talking now"),
`/SCK` (serial clock; the chip latches data on its rising edge), and
`SIO` (one bit of data, direction set by bit 4 of the I/O register).
We model it as four states, transitioning on `/CS` and `/SCK` edges.

```text
                       +--------+
                       |  Idle  |  pins.cs == 0
                       +---+----+
                           | /CS rising (cs 0→1)
                           v
                  +--------+--------+
                  | AwaitingCommand |  collecting 8 bits LSB-first
                  +--------+--------+
                  /SCK rising = take pins.sio, append at bit_idx_,
                  bit_idx_++. After 8 bits, decode:
                    - bits 5..7 fixed pattern (verify; warn on mismatch)
                    - bit 4 = read/write direction
                    - bits 0..3 = command code → Cmd enum
                  Transition into one of:
                           |
                           v
                  +--------+--------+
                  |   ParamWrite    | (active_read_ == false)
                  |    or           |
                  |   ParamRead     | (active_read_ == true)
                  +--------+--------+
                  Each /SCK rising shifts one bit. Every 8 bits =
                  one parameter byte processed:
                    write: apply_write_byte(byte) updates the chip
                    read:  produce_read_byte() loads the next byte
                  After param_count(cmd) bytes, return to Idle on /CS
                  falling. If /CS falls early, abort transfer.
                           |
                           v
                       +--------+
                       |  Idle  |  pins.cs == 0 (CS falling)
                       +--------+
```

Transitions, in code-form (the implementation is one method,
`Rtc::write_pins`, with branches on edges; `read_pins` is pure):

```text
write_pins(new_pins):
    cs   = bit(new_pins, 2)
    sck  = bit(new_pins, 1)
    sio  = bit(new_pins, 0)
    cs_rising  = cs && !prev_cs_high_
    cs_falling = !cs && prev_cs_high_
    sck_rising = sck && !prev_sck_high_

    if cs_falling:
        xfer_phase_ = Idle
        bit_idx_ = 0
        param_byte_ = 0
        active_cmd_ = Unknown
    elif cs_rising:
        xfer_phase_ = Command
        bit_idx_ = 0
        shift_byte_ = 0
    elif cs && sck_rising && xfer_phase_ != Idle:
        // shift one bit
        if direction is write (cpu→chip on this byte):
            shift_byte_ |= (sio << bit_idx_)   // LSB-first
        bit_idx_++
        if bit_idx_ == 8:
            handle byte boundary (decode command or apply param byte)
            bit_idx_ = 0

    pins_ = new_pins
    prev_cs_high_  = cs
    prev_sck_high_ = sck

read_pins():
    // Direction bits 4-7 read back literally from pins_.
    // Bit 0 (SIO) reads back: pins_ bit 0 if direction is "data input
    // from CPU" (bit 4 == 1), else the chip-driven bit pulled from
    // shift_byte_ on read commands, else pins_ bit 0.
    return pins_ with bit 0 conditionally replaced by shift_byte_'s
           current bit during a read transfer.
```

### 4.6 1 Hz tick + alarm comparison algorithm

Plain-language note: every 67,027,964 ARM9 cycles (≈ 1 s of emulator
time), NDS pops a `Rtc1HzTick` event. The handler advances `dt_.sec`,
cascades through the calendar, then compares both alarms. The
comparison computes a single boolean per alarm: is this alarm enabled
in status2 *and* do all of its compare-enabled fields match the current
time? If that boolean transitioned from false to true since the last
tick, raise IF.7 and latch the corresponding INT bit in status1.

```text
Rtc::tick(IrqController& irq7):
    // 1. Advance time by one second.
    if ++dt_.sec == 60:
        dt_.sec = 0
        if ++dt_.min == 60:
            dt_.min = 0
            if ++dt_.hour == 24:
                dt_.hour = 0
                dt_.dow = (dt_.dow + 1) % 7
                if ++dt_.day > days_in_month(dt_.year, dt_.month):
                    dt_.day = 1
                    if ++dt_.month > 12:
                        dt_.month = 1
                        ++dt_.year

    // 2. Compute current match state per alarm.
    alarm1_enabled = (status2_ & 0x0F) == 0x04   // bits 0-3 == 0100b
    alarm2_enabled = (status2_ & 0x40) != 0      // bit 6
    a1_match = alarm1_enabled && alarm_matches(alarm1_)
    a2_match = alarm2_enabled && alarm_matches(alarm2_)

    // 3. Rising-edge raise + latch.
    if a1_match && !prev_alarm1_match_:
        status1_ |= (1 << 4)        // INT1 flag
        irq7.raise(1u << 7)
    if a2_match && !prev_alarm2_match_:
        status1_ |= (1 << 5)        // INT2 flag
        irq7.raise(1u << 7)

    prev_alarm1_match_ = a1_match
    prev_alarm2_match_ = a2_match


Rtc::alarm_matches(a):
    // Each alarm field's bit 7 is the "compare enable". If 0, that
    // field matches anything. If 1, the field must equal the
    // corresponding current-time field (BCD-decoded vs raw uint8).
    // Per S-35190A datasheet table 6 / GBATEK §RTC-Alarm.
    dow_ok  = !(a.dow  & 0x80) || ((a.dow  & 0x07) == dt_.dow)
    hour_ok = !(a.hour & 0x80) || ((a.hour & 0x3F) == dt_.hour)
    min_ok  = !(a.min  & 0x80) || ((a.min  & 0x7F) == dt_.min)
    return dow_ok && hour_ok && min_ok
```

The "alarm fires every minute / every hour / once per day" question
resolves naturally: when only the `min` field has its compare-enable
set, the alarm matches once per hour (every minute matching). When
only `hour` has compare-enable, it matches once per day for one full
minute. When all three have compare-enable, it matches once per week.
**Per-second rising-edge discipline guarantees one IF.7 raise per
match-window**, even though the match-condition stays true for a full
minute (the gating ensures we don't re-raise on every second of that
minute).

§5.7 documents the GBATEK ambiguity; this algorithm is the chosen
resolution.

---

## 5. Hardware details

These are taken directly from GBATEK and verified against the page
fetched 2026-05-04 / 2026-05-05.

### 5.1 RTC bus register (`0x4000138`, NDS7 R/W, 8-bit)

> The DS uses Seiko S-35180 (compatible with S-35190A) RTC chip on NDS,
> S-35199A01 RTC chip on DSi. The RTC uses three signal lines: data
> input/output (SIO), serial clock (SCK), and chip select (CS).
>
> | Bit | Dir | Meaning                                                |
> |-----|-----|--------------------------------------------------------|
> | 0   | R/W | Data I/O (SIO line — direction depends on bit 4)       |
> | 1   | R/W | Clock (/SCK line — should be driven by CPU)            |
> | 2   | R/W | Chip-Select (/CS line — active high)                   |
> | 3   | -   | Not used                                                |
> | 4   | R/W | Data direction (0 = Read from chip, 1 = Write to chip)  |
> | 5   | R/W | Clock direction (should be 1 = Write)                   |
> | 6   | R/W | Select direction (should be 1 = Write)                  |
> | 7   | -   | Not used (reads as 0)                                  |

[gbatek-2026-05-04 — https://problemkaputt.de/gbatek-ds-real-time-clock-rtc.htm]

### 5.2 SIO bit-transfer protocol (verbatim from GBATEK)

> Output `/SCK` = LOW, output `/SIO` = data bit (when writing). Wait
> approximately ≥ 5 µs. Output `/SCK` = HIGH. Wait approximately
> ≥ 5 µs. Then read `/SIO` = data bit (when reading).

> Chipselect initialization: output `/CS` = LOW + `/SCK` = HIGH. Wait
> approximately ≥ 1 µs. Then output `/CS` = HIGH + `/SCK` = HIGH.
> Wait approximately ≥ 1 µs. Then send commands.

[gbatek-2026-05-04 — same URL]

Plain-language: the CPU is the master, the chip is the slave, and the
chip latches data on the rising edge of `/SCK`. The `/CS` line stays
high for the whole transfer; lowering `/CS` aborts. Both commands and
data are sent **LSB first**.

### 5.3 Command byte format

> A command byte is 8 bits with bits 5..7 fixed (110b = `0x60`) and
> bits 0..3 selecting the command. Bit 4 selects direction:
> 0 = write to chip, 1 = read from chip.

Parsed into the table we use to decode the byte:

| Bits 7..5 | Bit 4   | Bits 3..0 (cmd code) | Meaning                  |
|-----------|---------|----------------------|--------------------------|
| 110       | 0 = W   | 0000                 | Write Status Register 1  |
| 110       | 1 = R   | 0000                 | Read Status Register 1   |
| 110       | 0 = W   | 0100                 | Write Status Register 2  |
| 110       | 1 = R   | 0100                 | Read Status Register 2   |
| 110       | 1 = R   | 0010                 | Read Date + Time (7B)    |
| 110       | 1 = R   | 0110                 | Read Date (3B)           |
| 110       | 1 = R   | 1010                 | Read Time (3B)           |
| 110       | 0/1     | 0001                 | Alarm 1 (Write/Read 3B)  |
| 110       | 0/1     | 0101                 | Alarm 2 (Write/Read 3B)  |
| 110       | 0/1     | 0011                 | Free register (1B)       |
| 110       | 0/1     | 0111                 | Frequency-select (1B)    |

Commands not listed (e.g. INT2-related on DSi) are out of scope this
slice.

[gbatek-2026-05-04]

**Open question — bit-numbering convention (added 2026-05-07 during
commit-2 review):** GBATEK's "Command Register" entry presents the byte
layout in two side-by-side columns ("Fwd" = LSB-first wire order, "Rev" =
MSB-first wire order). Read literally under "Fwd", the fixed pattern
sits in bits 0..3 of the received byte (low nibble = `0x06`), the command
index is a 3-bit field in bits 4..6, and R/W is bit 7. The table above
uses the Seiko S-35190A datasheet's MSB-first byte layout — fixed pattern
in bits 5..7, R/W in bit 4, 4-bit command index in bits 0..3 (allowing
`0xA` for Time-read).

Both layouts can encode the same set of valid commands; they differ only
in where the fields sit within the u8 representation. melonDS handles
both wire forms by detecting an MSB-first command (`(val & 0xF0) == 0x60`
matches the slice-spec layout exactly) and bit-reversing into a canonical
LSB-first form before its switch. The slice-3j commits 2-7 implement the
spec's MSB-first layout literally; before HG/SS can complete an RTC
handshake, the decoder will need either:

1. A melonDS-style auto-detect + bit-reverse step (handles both wire
   conventions), OR
2. A confirmation via instruction trace that real Pokemon ROMs send the
   spec's MSB-first byte layout directly, in which case no change is
   needed.

Tracked in the implementation via a `TODO(slice-3j-real-game-integration)`
marker at `src/rtc/rtc.cpp` adjacent to the fixed-bits check. Resolution
deferred to whichever later slice first integrates an HG/SS RTC trace.

### 5.4 Status Register 1 (1 byte, both read and write commands)

| Bit | Meaning                                                  | Read clears? |
|-----|----------------------------------------------------------|--------------|
| 0   | Reset (write 1 to soft-reset the chip)                   | -            |
| 1   | 12/24-hour mode (0 = 12-hour, 1 = 24-hour)               | -            |
| 2   | General-purpose                                           | -            |
| 3   | General-purpose                                           | -            |
| 4   | INT1 flag (1 = alarm 1 fired)                             | **YES**      |
| 5   | INT2 flag (1 = alarm 2 fired)                             | **YES**      |
| 6   | Power-low flag                                            | **YES**      |
| 7   | Power-off flag (set on cold boot, clears on first read)   | **YES**      |

Bits 4-7 auto-clear when the CPU reads status1. We model this in
`produce_read_byte()` for the Status1 read command: at the moment the
last bit of the byte is shifted out, mask off bits 4-7.

[gbatek-2026-05-04]

### 5.5 Status Register 2 (1 byte, both read and write commands)

| Bits  | Meaning                                                |
|-------|--------------------------------------------------------|
| 0..3  | INT1 mode/enable. `0100b` (=0x4) = enable Alarm-1 IRQ. |
| 4..5  | Reserved.                                              |
| 6     | INT2 enable (0 = Disable, 1 = Alarm 2 IRQ enable).     |
| 7     | INT1 frequency-select / steady-output (out of scope).  |

We treat bits 0..3 as a single field per the datasheet. Only
the value `0100b` enables Alarm 1; any other value disables. (Other
values configure non-alarm modes — frequency steady output etc. —
which are out of scope.)

[gbatek-2026-05-04]

### 5.6 Date+Time (7 bytes, read only at the chip level)

> Data order: year, month, day, day-of-week, hour, minute, second.
> All fields are packed BCD per S-35190A datasheet table 4.

| Byte | Field        | Range (BCD) | Notes                              |
|------|--------------|-------------|------------------------------------|
| 0    | Year         | 00..99      | year - 2000 (we store full year)   |
| 1    | Month        | 01..12      | bits 0..4 valid                    |
| 2    | Day          | 01..31      | bits 0..5 valid                    |
| 3    | Day-of-week  | 0..6        | 0 = Sunday per datasheet           |
| 4    | Hour         | 00..23      | 24-hour mode (status1 bit 1 = 1)   |
| 5    | Minute       | 00..59      | bits 0..6 valid                    |
| 6    | Second       | 00..59      | bits 0..6 valid                    |

Date (3-byte) and Time (3-byte) commands return bytes 0..2 / 4..6
respectively — same encodings.

The 12-hour mode (status1 bit 1 = 0) sets bit 5 of the hour byte to
indicate AM/PM. We always run 24-hour mode (status1 bit 1 defaults to
1 on reset). Pokemon uses 24-hour. If a game flips to 12-hour we still
store / advance the time correctly internally; the read-out byte
encodes 12-hour with the AM/PM bit. Out of scope to switch internal
representation.

[gbatek-2026-05-04]

### 5.7 Alarm 1 / Alarm 2 (3 bytes each, read/write)

> Alarm registers store day-of-week, hour, and minute. Each field has
> a "compare enable" bit at bit 7. If compare-enable = 0, the field is
> a "don't care" (matches anything). If compare-enable = 1, the field
> must equal the current value for the alarm to fire.

| Byte | Field        | Bit 7    | Other bits                           |
|------|--------------|----------|--------------------------------------|
| 0    | Day-of-week  | Cmp-en   | bits 0..2 = dow; bits 3..6 unused    |
| 1    | Hour         | Cmp-en   | bits 0..5 = hour (BCD encoded for 24)|
| 2    | Minute       | Cmp-en   | bits 0..6 = minute (BCD encoded)     |

[gbatek-2026-05-04 / S-35190A datasheet table 6]

**GBATEK ambiguity flag** — the spec is **silent** on the exact
condition under which IF.7 fires:
1. Once per match-window (rising edge of "all enabled fields match")?
2. Once per second while matching?
3. Once at the start, then re-arm only after explicit acknowledge?

We pick **option 1 (rising edge)**. Justification:
- Real hardware ties INT1/INT2 into the SIO unit's SI input. SI is
  edge-sensitive in every documented SIO mode.
- melonDS source (rtc.cpp, fetched 2026-05-04) implements rising-edge.
- The status1 bit-4/5 latch is explicitly described as auto-clearing
  on read — i.e. the CPU is expected to acknowledge once and the chip
  is expected to re-arm only when the match condition has fallen.

This is the design choice; flagged in §8 risk register as the most
likely point of real-game divergence.

### 5.8 NDS7 IF bit map (relevant subset)

> | Bit | Source                       | NDS9 | NDS7 |
> |-----|------------------------------|------|------|
> | 7   | SIO/RCNT/RTC                 | -    | yes  |
> | 12  | Keypad                       | yes  | yes  |
> | 16  | IPC Sync                     | yes  | yes  |
> | 17  | IPC Send FIFO Empty          | yes  | yes  |
> | 18  | IPC Recv FIFO Not Empty      | yes  | yes  |
> | 22  | Screens unfolded             | -    | yes  |

[gbatek-2026-05-04 — https://problemkaputt.de/gbatek.htm#dsinterrupts]

IF.7 is **NDS7 only** — there is no ARM9-side raise for any of the
three sources that share this bit. `irq9_.raise()` is never called
from RTC.

### 5.9 Behavior walkthroughs

These six walkthroughs are the test specifications. Each enumerates
the bus-and-event sequence, the resulting state delta, and the
visible side effects.

#### 5.9.1 ARM7 sends a "Read Status1" command via bit-bang

Plain-language: the canonical "is the chip alive?" command. CPU
toggles `/CS` high, shifts in the 8-bit command (`0xD0` after LSB-first
re-ordering of `0b110 1 0000`), then toggles `/SCK` 8 more times to
read out status1 one bit at a time.

Precondition: `reset()` ran. `status1_ == 0x02` (24-hour mode bit set,
power-off flag cleared because we don't model cold-boot latency).

Bus calls (each `arm7_io_write8(0x4000138, …)`):
1. `0x70` — direction bits 4..6 set, /CS high, /SCK low. (7 = `0b0111` →
   bit 0..2 = 110 → /CS=1, /SCK=1, SIO=1?  Wait — re-decode: a typical
   init sequence is /CS rising while /SCK is high, *then* /SCK starts
   pulsing.) On rising of /CS the state machine enters Command phase.
2. Sequence of 16 toggles per bit: low /SCK + new SIO data, then high
   /SCK. After 8 such pairs the chip has the byte.
3. Decode: command code = `0x0`, direction = read → `Cmd::Status1` read.
   `xfer_phase_ = ParamRead`. `produce_read_byte()` loads
   `shift_byte_ = status1_; status1_ &= ~0xF0` (auto-clear of
   bits 4-7).
4. CPU pulses /SCK 8 more times. On each rising edge, `read_pins()`
   returns `pins_` with bit 0 replaced by `(shift_byte_ >> bit_idx_) & 1`
   (LSB-first). After 8 bits, `bit_idx_` wraps; if /CS stays high we
   either accept another read byte (multi-byte commands) or wait for
   /CS falling.
5. CPU brings /CS low. State machine returns to Idle.

Expected: status1_ bits 4-7 cleared in `produce_read_byte`. Bits 0-3
preserved.

#### 5.9.2 ARM7 sets up Alarm 1 to fire at every :30 of every hour

Precondition: `reset()` ran. `dt_ = {2026, 1, 1, 4, 12, 29, 55}`.

Bus sequence:
1. Send "Write Status2" command (`0x40`). Param byte: `0x04`. State:
   `status2_ = 0x04` → Alarm 1 enabled.
2. Send "Write Alarm1" command (`0x10`). Param bytes:
   - dow byte = `0x00` (compare-enable = 0; don't-care).
   - hour byte = `0x00` (compare-enable = 0; don't-care).
   - min byte = `0x80 | bcd(30)` = `0xB0` (compare-enable = 1, value 30).
3. State: `alarm1_ = {dow=0x00, hour=0x00, min=0xB0}`.

Now five `Rtc1HzTick` events fire (NDS pops them as the scheduler
advances). Each tick:

- Tick 1: dt_.sec 55 → 56. `alarm_matches(alarm1_)`: dow_ok = true,
  hour_ok = true, min_ok = `(0xB0 & 0x7F) == 29` → false. No match.
- Tick 2: 56 → 57. No match.
- Tick 3: 57 → 58. No match.
- Tick 4: 58 → 59. No match.
- Tick 5: 59 → 0; min cascades 29 → 30. Match: min_ok =
  `(0xB0 & 0x7F) == 30` → true. **Rising edge of (enabled AND match).**
  status1_ bit 4 set. `irq7.raise(1u << 7)`. `prev_alarm1_match_ = true`.

Subsequent ticks 6-64 (sec 0..59 with min still 30): match stays true.
**No re-raise** — gating is rising-edge.

Tick 65 (min cascades to 31): alarm_matches false. `prev_alarm1_match_`
falls back to false. No raise.

Sometime later, when min comes around to 30 again (an hour later, if
nothing else changed): rising edge fires another raise.

Expected over the entire sequence: exactly one `irq7.raise(1u << 7)` per
:30 minute, regardless of how many seconds match.

#### 5.9.3 IF.7 raises but the CPU never had IE.7 set

Precondition: alarm-1 setup as above. Also: `irq7_.write_ie(0)` —
no IRQ sources enabled in IE.

When the alarm fires:
- `status1_ |= 0x10` is applied unconditionally (chip-level latch).
- `irq7.raise(1u << 7)` is called → `irq7_.if_ |= 0x80`.
- `update_arm7_irq_signals()` is called. `irq7_.line()` evaluates
  `(IME & 1) && (IE & IF)`. With `IE == 0`, the line is false. ARM7
  CPU stays in whatever state it was in.
- A later CPU read of status1 still observes bit 4 set (the chip-level
  latch persisted) and reading clears it. The IF.7 bit also persists
  until the CPU acknowledges via write-1-clear to IF.

Expected: `irq7_.read_if() & 0x80` is set; ARM7 line is false; ARM7
neither halts-wakes nor enters the IRQ vector. This matches real
hardware: IE controls *delivery*, not raise.

#### 5.9.4 Rolling over Feb 28 → Mar 1 in a leap year

Precondition: `dt_ = {2024, 2, 28, 3, 23, 59, 59}` (2024 is leap;
day-of-week = 3 = Wednesday). No alarms enabled.

Tick 1: sec 59 → 0; min 59 → 0; hour 23 → 0; dow 3 → 4; day 28 → 29.
  `days_in_month(2024, 2)` = 29 (leap). 29 ≤ 29, so day stays 29.
Tick 2 (one second later — many ticks elapsed but only the rollover
matters): eventually `dt_.sec` rolls over again with `day == 29`.
The next day-rollover from Feb 29 → Mar 1: 29+1 = 30 > 29, so day = 1,
month = 3. dow advances accordingly.

Expected: `is_leap(2024) == true`; `days_in_month(2024, 2) == 29`;
day rolls from 29 → 1, month from 2 → 3.

For 2026 (non-leap): same scenario from Feb 28 23:59:59. Tick:
`days_in_month(2026, 2) == 28`. day 28+1 = 29 > 28. day = 1, month = 3.
Skip Feb 29 entirely.

#### 5.9.5 Aborting a transfer mid-way by lowering /CS

Precondition: CPU has shifted in 5 bits of a command. `xfer_phase_ ==
Command`, `bit_idx_ = 5`, `shift_byte_ = partial`.

Bus call: `write_pins(value with /CS = 0)`.

Steps:
1. cs_falling detected.
2. `xfer_phase_ = Idle`, `bit_idx_ = 0`, `param_byte_ = 0`,
   `active_cmd_ = Unknown`.
3. The 5 partial bits are discarded; chip-side state is unchanged
   (nothing was applied — `apply_write_byte` is only called at byte
   boundaries).

Expected: `dt_`, `status1_`, `status2_`, `alarm1_`, `alarm2_` all
unchanged. The next /CS rising starts fresh.

#### 5.9.6 Acknowledging IF.7 then re-firing

Precondition: alarm-1 fired. `status1_ & 0x10 == 0x10`,
`irq7_.if_ & 0x80 == 0x80`, `prev_alarm1_match_ = true`. CPU has IE.7
set and is in the IRQ vector.

CPU sequence (the ARM7 IRQ handler):
1. Reads status1 via SIO bit-bang. `produce_read_byte()` returns the
   stored byte and clears bits 4-7. `status1_ & 0x10 == 0` after.
2. Writes 0x80 to IF (`arm7_io_write32(IO_IF, 0x80)`). `irq7_.if_ &
   0x80 == 0` after.
3. Calls `update_arm7_irq_signals()`. line() is now false. ARM7
   `subs pc, lr, #4` returns from vector.

Now: time keeps advancing. `prev_alarm1_match_` is still true (the
match condition didn't fall — only the latches were cleared). Until
the minute rolls over, no re-raise. After min 30 → 31, `a1_match` is
false; `prev_alarm1_match_` = false. The next time min hits 30, a
fresh rising edge → raise + latch.

Expected: status1 bit 4 cleared by the read, IF.7 cleared by the
write, no spurious second raise during the minute the match holds,
fresh raise on the next match window.

---

## 6. Testing strategy

CTest count goes from 65 to 71 (six new test binaries). Each binary
links `ds_core` only, uses `REQUIRE` from `tests/support/require.hpp`,
and runs in milliseconds.

### 6.1 `rtc_protocol_test.cpp` (commit 2)

- `Rtc_Reset_PinsZero_PhaseIdle`: REQUIRE `rtc_.read_pins() == 0` and
  active-cmd is Unknown after `reset()`.
- `Rtc_PinDirectionBits_RoundTrip`: write `0x70` (dirs all "write"),
  REQUIRE bit 4..6 read back; bit 0..3 reflect /CS, /SCK, SIO, reserved.
- `Rtc_CsRising_EntersCommandPhase`: drive /CS 0 → 1, REQUIRE phase
  becomes Command, bit_idx 0.
- `Rtc_CsFalling_AbortsTransfer`: shift 5 bits in, drop /CS, REQUIRE
  phase returns to Idle and the partial byte is discarded (no chip
  state changed).
- `Rtc_SckRisingEdge_ShiftsOneBit_LSBFirst`: shift `0x55` LSB-first,
  REQUIRE bit_idx advances 0→1→…→7 then wraps.
- `Rtc_CommandByte_FixedHighBitsValidated`: send command byte with
  bits 5..7 != `110b`, REQUIRE `active_cmd_` is `Unknown` and a
  DEBUG warn-log was emitted (no chip state changed).
- `Rtc_DataDirectionBit4_GovernsSioDirection`: with bit 4 = 1 (write
  to chip), REQUIRE chip latches SIO from pins; with bit 4 = 0,
  REQUIRE `read_pins()` produces the chip's current shift bit on
  bit 0 of the returned byte.

### 6.2 `rtc_commands_test.cpp` (commit 3 + commit 4)

Read-side commands (commit 3):

- `Rtc_ReadStatus1_DefaultIs24HourMode`: post-reset, REQUIRE the
  read-back byte has bit 1 set, bits 4..7 zero.
- `Rtc_ReadStatus1_ClearsBits47_OnRead`: pre-set status1 to 0xF2
  (all flags + 24h-mode), read it, REQUIRE the returned byte is 0xF2,
  REQUIRE post-read internal status1 is 0x02 (bits 4-7 cleared).
- `Rtc_ReadStatus2_DefaultZero`: post-reset, REQUIRE 0x00.
- `Rtc_ReadDateTime_BcdEncoding`: seed `dt = {2026, 5, 4, 1, 14, 30, 45}`,
  REQUIRE the 7-byte read sequence is
  `{0x26, 0x05, 0x04, 0x01, 0x14, 0x30, 0x45}` (BCD).
- `Rtc_ReadDate_FirstThreeBytes`: REQUIRE Read-Date returns the same
  first three bytes as Read-DateTime.
- `Rtc_ReadTime_LastThreeBytes`: REQUIRE Read-Time returns bytes 4..6
  of Read-DateTime.

Write-side commands (commit 4):

- `Rtc_WriteStatus1_PreservesReservedBits`: write `0x02`, REQUIRE
  internal status1 is `0x02`. Write `0xFF`, REQUIRE bits 4-7 are
  rejected (chip-side: bits 4-7 are flags, not writable; only bits
  0-3 of status1 are writable per datasheet).
- `Rtc_WriteStatus2_StoresAllBits`: write `0x44`, REQUIRE status2 is
  `0x44`.
- `Rtc_WriteAlarm1_StoresAllThreeBytes`: write `{0x82, 0x95, 0xB0}`,
  REQUIRE `alarm1_ = {0x82, 0x95, 0xB0}`.
- `Rtc_WriteAlarm2_Independent`: write only Alarm2, REQUIRE alarm1_
  unchanged.

### 6.3 `rtc_alarm_test.cpp` (commit 6)

- `Rtc_AlarmCmpEnableMask_AllZero_AlwaysMatches`: alarm with all
  three compare-enable bits 0, REQUIRE `alarm_matches` is true at
  every dt.
- `Rtc_AlarmCmpEnableMask_OnlyMin_MatchesEveryHour`: enable status2
  Alarm-1, alarm = `{0x00, 0x00, 0x80 | bcd(30)}`. Tick across an
  hour, REQUIRE exactly one rising-edge raise (at min == 30).
- `Rtc_AlarmCmpEnableMask_HourPlusMin_MatchesOncePerDay`: enable,
  alarm = `{0x00, 0x80|bcd(14), 0x80|bcd(0)}`. REQUIRE exactly one
  raise per 24 hr.
- `Rtc_AlarmCmpEnableMask_AllThree_MatchesOncePerWeek`: enable,
  alarm = `{0x80|3, 0x80|bcd(9), 0x80|bcd(0)}`. REQUIRE one raise on
  Wednesday 09:00, none on the same time other days.
- `Rtc_Status2EnableOff_NoRaise`: alarm fully matching but status2
  bits 0..3 ≠ `0100b`, REQUIRE no IF.7 raise.
- `Rtc_Status1Bit4_LatchesOnRaise_ClearsOnRead`: trigger raise,
  REQUIRE status1 bit 4 set; read status1 via bit-bang, REQUIRE bit 4
  clears; trigger another raise (after match-fall + match-rise),
  REQUIRE bit 4 set again.
- `Rtc_Alarm2_IndependentLatch`: alarm-1 and alarm-2 both armed and
  both match, REQUIRE both INT1 and INT2 latch and IF.7 is raised
  exactly once per rising edge per alarm.

### 6.4 `rtc_tick_test.cpp` (commit 5)

- `Rtc_TickAdvancesSeconds`: call `tick()` 60 times, REQUIRE
  `dt_.min` advanced by 1 and `dt_.sec` is 0.
- `Rtc_TickRollover_MinutesToHours`: dt at `{… 0, 59, 59}`, tick once,
  REQUIRE `{… 1, 0, 0}`.
- `Rtc_TickRollover_HoursToDays_AdvancesDow`: dt at `{… 3, 23, 59, 59}`,
  tick once, REQUIRE day++, dow becomes 4, hour=0.
- `Rtc_TickRollover_NonLeapFebToMar`: dt at `{2026, 2, 28, 0, 23, 59, 59}`,
  tick once, REQUIRE `{2026, 3, 1, 1, 0, 0, 0}`.
- `Rtc_TickRollover_LeapFebToFeb29`: dt at `{2024, 2, 28, 3, 23, 59, 59}`,
  tick once, REQUIRE day=29, month=2.
- `Rtc_TickRollover_LeapFeb29ToMar1`: dt at `{2024, 2, 29, 4, 23, 59, 59}`,
  tick once, REQUIRE day=1, month=3.
- `Rtc_TickRollover_DecToJan_YearAdvances`: dt at
  `{2026, 12, 31, 4, 23, 59, 59}`, tick once, REQUIRE
  `{2027, 1, 1, 5, 0, 0, 0}`.
- `Rtc_IsLeap_GregorianRule`: REQUIRE `is_leap(2024)` true,
  `is_leap(2025)` false, `is_leap(2100)` false (century non-leap),
  `is_leap(2400)` true (400-year leap).
- `Rtc_FirstTickScheduledByReset`: post-NDS-reset, REQUIRE the
  scheduler heap contains exactly one `Rtc1HzTick` event at
  `now + kArm9CyclesPerSecond`.
- `Rtc_TickReschedulesItself`: drive NDS through one tick event,
  REQUIRE the scheduler heap contains one new `Rtc1HzTick` at
  `now + kArm9CyclesPerSecond`.

### 6.5 `rtc_irq_test.cpp` (commit 6)

- `Rtc_NoMatch_NoRaise`: enabled alarm 1, dt nowhere near match,
  tick once, REQUIRE `irq7_.read_if() & 0x80 == 0`.
- `Rtc_RisingEdge_RaisesIf7`: alarm armed and exactly matching at
  this tick, REQUIRE `irq7_.read_if() & 0x80 != 0`.
- `Rtc_StaysMatching_DoesNotReRaise`: rising-edge raise, ack via
  IF write-1-clear, do NOT change dt, tick again with match still
  true, REQUIRE IF.7 stays 0 (no rising edge).
- `Rtc_FallingEdge_NoRaise`: was matching, tick into non-match,
  REQUIRE no raise.
- `Rtc_AfterFall_FreshRiseRaises`: was matching, tick to non-match,
  tick to match again, REQUIRE rising-edge raise fires.
- `Rtc_RaiseOnAlarm1_DoesNotAffectAlarm2`: alarm-1 fires,
  alarm-2 enabled but not matching, REQUIRE only INT1 latches and
  raise count is 1 per the rising edge.
- `Rtc_BothAlarms_RiseSimultaneously_BothLatch`: tick produces both
  rising edges, REQUIRE INT1 and INT2 both latch and IF.7 raised
  (single bit set, two raise calls coalesce in OR).
- `Rtc_RaiseDoesNotReachArm9`: same setup; REQUIRE
  `irq9_.read_if() & 0x80 == 0` (IF.7 is NDS7-only).

### 6.6 `rtc_integration_test.cpp` (commit 7)

End-to-end test driving NDS through actual frame execution:

- `Rtc_FullBootHandshake`:
  1. NDS construction. Implicit `reset()` seeded
     `dt = {2026, 1, 1, 4, 0, 0, 0}` and scheduled the first
     `Rtc1HzTick` at `now + kArm9CyclesPerSecond`.
  2. ARM7 writes IME=1, IE=`0x80` (enable IF.7).
  3. ARM7 sends bit-bang command sequence to write status2 = `0x04`,
     alarm1 = `{0x00, 0x00, 0x80|bcd(0)}` (fire every hour at :00).
  4. Drive `NDS::run_frame()` repeatedly until enough emulator-time
     elapses for the alarm to fire. Each frame is `kFrameCycles`
     ARM9 cycles ≈ 1/60 s; we need ~60 minutes of emulator time
     ≈ 3600 frames. Since this is too slow for unit-test wall time,
     the test instead pokes the scheduler directly: `nds.scheduler()
     .schedule_in(0, EventKind::Rtc1HzTick)` and runs `run_frame()`
     once after seeding `dt = {2026, 1, 1, 4, 0, 59, 59}` so the
     next tick is the rising edge.
  5. REQUIRE `irq7_.read_if() & 0x80 != 0`.
  6. REQUIRE the ARM7 line is asserted (`cpu7_` halt-wake or vector
     entry depending on CPU state).
  7. ARM7 reads status1 via bit-bang. REQUIRE returned byte has bit 4
     set; REQUIRE post-read status1 has bit 4 cleared.
  8. ARM7 writes `0x80` to IF. REQUIRE `irq7_.read_if() & 0x80 == 0`.
  9. Tick advances `dt` past the match window. REQUIRE no spurious
     re-raise.

### 6.7 What stays green from prior slices

All 65 existing CTest binaries must remain green at every commit. The
highest-churn change is commit 5 (adds the `Rtc1HzTick` enumerator and
inserts a new arm into `on_scheduler_event`). Verify after commit 5
that `nds_integration_test`, `arm7_halt_test`, `arm7_bios_intrwait_test`,
and the IPC tests (which run frames or schedule events) still pass.

---

## 7. Cross-references

- **Project design spec:** `docs/specs/2026-04-12-nds-emulator-design.md`
  §3.5 (interrupt model — confirms IF.7 is NDS7-only and groups three
  sources), §9 (scheduler internals — recurring events use
  `schedule_in` from inside the dispatch handler), §13 Phase 1
  deliverables (RTC listed under Phase 1, after IPC, before Phase 2
  PPU).
- **Prior slice spec:** `docs/specs/2026-04-24-ipc-phase1-slice3i-design.md`
  §1 (handoff line proposed scheduler-driven IRQ sources for slice 3j),
  §4.5 (edge-trigger algorithm — same discipline applies here), §8.1
  point 1 (latch-placement risk applies identically).
- **CLAUDE.md:** rule 1 (scheduler is the clock), rule 3 (no
  inter-subsystem pointers), rule 4 (per-CPU IO tables — RTC is ARM7
  only), rule 5 (`reset()` mandatory; save_state deferred), rule 7
  (file size caps), rule 8 (no cross-subsystem includes), rule 6 (no
  SDL include path — `<chrono>` in libds_core would be similar
  contamination, see §3 non-goal).
- **Existing IRQ scaffolding:**
  - `src/interrupt/irq_controller.hpp` lines 12-44 (the class RTC
    will pass by reference into `tick()`).
  - `src/cpu/arm7/arm7.hpp` (line/halt-wake interface, unchanged).
  - `src/nds.cpp` lines 289-298 (existing
    `update_arm7_irq_signals()`, the function called after every RTC
    tick that raised).
  - `src/nds.cpp` lines 469-478 (the `IO_HALTCNT` 8-bit byte-write
    pattern that the RTC route mirrors).
- **Existing bus pattern:** `src/bus/arm7_bus.cpp` lines 1-171
  (slow-path IO dispatch — RTC adds one more case to the existing
  routing table on the ARM7 side).
- **Existing scheduler pattern:** `src/scheduler/scheduler.cpp` lines
  15-24 (`schedule_at`, `schedule_in` — the RTC tick reschedules
  itself from inside `on_scheduler_event` using `schedule_in`).
- **GBATEK references** (fetched 2026-05-04 / 2026-05-05):
  - https://problemkaputt.de/gbatek-ds-real-time-clock-rtc.htm
    (RTC bus register, SIO protocol, command set, status1/status2
    layouts, alarm field layouts, BCD encoding).
  - https://problemkaputt.de/gbatek.htm#dsinterrupts (NDS7 IF bit
    map; IF.7 = SIO/RCNT/RTC).
  - https://mgba-emu.github.io/gbatek/ (markdown mirror; cross-checked
    against the canonical site for the same fetch date).
- **External cross-check:** melonDS source `src/RTC.cpp` (commit hash
  pulled 2026-05-04) for rising-edge alarm semantics. Confirms the
  GBATEK ambiguity resolution in §5.7. Used as a reference, not
  copied.

---

## 8. Risk and rollback

### 8.1 Highest-risk pitfalls

1. **Edge-trigger latch placement (recurrence of slice 3i risk #1).**
   `prev_alarm{1,2}_match_` must be updated *after* the comparison-and-
   raise, not before. The same ordering bug that would lose IPC FIFO
   rising edges loses RTC alarm rising edges. Mitigation: §4.6
   pseudocode shows the order explicitly + the
   `Rtc_StaysMatching_DoesNotReRaise` and `Rtc_AfterFall_FreshRiseRaises`
   tests exercise both directions of the latch.
2. **Bit ordering: LSB-first vs MSB-first.** GBATEK is explicit ("LSB
   first") but easy to misread when implementing. Mitigation: the
   `Rtc_SckRisingEdge_ShiftsOneBit_LSBFirst` test sends `0x55` and
   `0xAA` and asserts the resulting `shift_byte_` matches LSB-first
   ordering. The `Rtc_ReadDateTime_BcdEncoding` test seeds a known
   datetime and reads back specific BCD bytes, which would also fail
   under MSB-first.
3. **GBATEK ambiguity on alarm fire condition (§5.7).** We picked
   rising-edge per melonDS. If a Pokemon game disagrees (e.g.
   expects continuous re-fire while matching), the fix is one-line
   inside `tick()`. Mitigation: documented in §5.7 + test names
   make the chosen semantics explicit so divergence is obvious.
4. **`<chrono>` leaking into `libds_core`.** Easy to "just call
   `system_clock::now()` for the seed". Doing so leaks platform
   non-determinism into tests (every CI run has a different seed)
   and violates the spirit of rule 6. Mitigation: `Rtc::reset()`
   uses a hard-coded default; the only path to wall-clock time is
   `NDS::seed_rtc_from_host_time(...)` which the frontend calls. Tests
   never call it; they call `nds.rtc().seed(fixed_dt)` directly. Static
   audit: grep `src/rtc/` and `src/nds.{hpp,cpp}` for `chrono` after
   commit 5 must return zero hits.
5. **BCD off-by-ones.** `to_bcd(60) == 0x60`, but `bcd(60)` is not a
   valid time field; the boundary is always *less than* 60. Helpers
   are defined for binary 0..99 only; any input outside that range
   triggers a DEBUG-only `assert`. Mitigation: tests for hour 23 → 24
   rollover, min 59 → 60 rollover, sec 59 → 60 rollover at the
   exact boundary.
6. **Scheduler reschedule-from-handler race.** `on_scheduler_event`
   calls `scheduler_.schedule_in(...)` while inside the
   `pop_due` loop. The scheduler is documented (rule 1, see also
   `scheduler.cpp:35-41` tombstone handling) as reentrant-safe for
   schedule-from-handler — newly scheduled events sit in the heap
   and are picked up in subsequent iterations. Mitigation:
   `Rtc_TickReschedulesItself` test verifies the reschedule lands and
   the next `peek_next()` returns the expected `when`.
7. **Status1 auto-clear timing.** GBATEK says bits 4-7 clear "on read".
   We clear them inside `produce_read_byte()` *before* the byte is
   shifted out to the CPU — the byte that gets shifted is the
   pre-clear value, but `status1_` itself is post-clear immediately.
   This matches melonDS. If a game reads status1 mid-bit, the half-
   shifted-out byte still reflects the pre-clear value (we cached it
   in `shift_byte_`). Tested by
   `Rtc_ReadStatus1_ClearsBits47_OnRead`.
8. **`IO_RTC` constant collision with `IO_IPCSYNC` family.** Address
   `0x4000138` is well-separated from the IPC range (`0x4000180+`)
   and from IRQ (`0x4000208+`), so no collision risk. Static check:
   `grep IO_RTC src/bus/io_regs.hpp` after commit 1 returns exactly
   one definition.

### 8.2 Rollback strategy

Each commit is independently revertable:

- Commits 6-7 (alarm raise + integration test) revert cleanly without
  touching the bit-bang protocol or the calendar advance. Reverting
  them restores RTC to "advances time but never raises IF.7".
- Commit 5 (1 Hz tick + scheduler integration) reverts cleanly without
  touching commits 2-4. Reverting it restores RTC to "responds to
  bit-bang commands but the time never advances unless `seed()` is
  called". Acceptable interim state for emergency revert.
- Commits 3-4 (read- and write-side commands) revert as a pair
  cleanly. Reverting them restores the chip to "bit-bang state
  machine works but every command is a no-op".
- Commit 2 (bit-bang state machine) reverts cleanly without
  touching commit 1. Reverting it restores `Rtc::write_pins` to a
  stub that just stores `pins_`.
- Commit 1 (scaffold + IO route) is the lowest-risk single-file group.
  Reverting it requires reverting commits 2-7 first.

### 8.3 What this slice does NOT break

Confirmed unchanged behaviors after each commit:
- ARM7 instruction execution and IRQ vector entry. (RTC raise is
  additive into the existing `irq7_.raise()` path.)
- ARM9 IO routing — RTC is NDS7-only, so no `arm9_io_*` change.
- IPC SYNC, IPC FIFO, IRQ controller registers — untouched.
- Scheduler `FrameEnd` event semantics — `on_scheduler_event` gains
  one new arm and the `FrameEnd` arm's body is preserved verbatim.
- All 65 existing CTest binaries must remain green at every commit.

---

## 9. Slice completion criteria

- [x] `class Rtc` exists in `src/rtc/rtc.{hpp,cpp}` with the public
      surface declared in §4.2 and `reset()` implemented.
      (Commit `c257af4` scaffold; final shape across `c257af4..f42b0d8`.
      `rtc.hpp` 150 lines, `rtc.cpp` 461 lines.)
- [x] `IO_RTC` constant in `src/bus/io_regs.hpp`. Exactly one
      definition; no other constant changed. Verified at
      `src/bus/io_regs.hpp:32` (`IO_RTC = 0x04000138u`).
- [x] `EventKind::Rtc1HzTick` enumerator added to
      `src/scheduler/event.hpp`. No other enumerator changed.
      Verified at `src/scheduler/event.hpp:11`. (Commit `17dd7ef`.)
- [x] `Rtc rtc_` member on `NDS`. Accessor `rtc()`. Include
      `rtc/rtc.hpp` in `nds.hpp`. Verified at `src/nds.hpp:58` (accessor)
      and `src/nds.hpp:125` (member). (Commit `c257af4`.)
- [x] ARM7 IO routes for `0x4000138` wired in `arm7_io_read8`,
      `arm7_io_write8`, plus the byte-aliased halfword/word paths
      per §4.4. Verified at `src/nds.cpp:339, 373, 406` (read 8/16/32)
      and `src/nds.cpp:446, 506, 583` (write 8/16/32). (Commit `c257af4`.)
- [x] `NDS::reset()` calls `rtc_.reset()` and schedules the first
      `Rtc1HzTick` at `now + kArm9CyclesPerSecond` (after `scheduler_.reset()`).
      Verified at `src/nds.cpp:44, 46`. (Commit `17dd7ef`.)
- [x] `NDS::on_scheduler_event` switch dispatches `Rtc1HzTick` to
      `rtc_.tick(irq7_)`, then `update_arm7_irq_signals()`, then
      reschedules. Verified at `src/nds.cpp:78..84`. (Commits
      `17dd7ef` dispatch, `3fe1916` adds the IRQ-raise path inside
      `Rtc::tick`.)
- [x] `NDS::seed_rtc_from_host_time(...)` body forwards to
      `rtc_.seed({year, month, day, dow, hh, mm, ss})`. Verified at
      `src/nds.cpp:315..317`. (Commit `c257af4`.)
- [x] Bit-bang protocol per §4.5 implemented: /CS framing, /SCK
      rising-edge shifting, LSB-first byte ordering, command-byte
      decode validating bits 5..7 == `110b`. (Commit `af9a749`,
      covered by `rtc_protocol_test`.)
- [x] All read commands per §5 implemented: Status1 (with auto-clear),
      Status2, DateTime, Date, Time, Alarm1, Alarm2. (Commit `823b672`,
      covered by `rtc_commands_test`.)
- [x] All write commands per §5 implemented: Status1, Status2,
      Alarm1, Alarm2. (Commit `1308b96`, write-side REQUIRE blocks
      appended to `rtc_commands_test`.)
- [x] Free / FreqSel commands routed as no-op-with-DEBUG-warn.
      (Commit `af9a749`, default branch of the command dispatch.)
- [x] Calendar advance per §4.6 with leap-year rule per §8.1 #5.
      (Commit `17dd7ef`, covered by `rtc_tick_test`.)
- [x] Alarm comparison + rising-edge raise per §4.6, §5.7.
      (Commit `3fe1916`, covered by `rtc_alarm_test` and
      `rtc_irq_test`.)
- [x] `status1_` bits 4-7 auto-clear on read per §5.4. (Commit
      `823b672`, dedicated REQUIRE block in `rtc_commands_test`.)
- [x] Six new test binaries: `rtc_protocol_test`, `rtc_commands_test`,
      `rtc_alarm_test`, `rtc_tick_test`, `rtc_irq_test`,
      `rtc_integration_test`. All registered via `add_ds_unit_test()`
      in `tests/CMakeLists.txt:83..88`. Verified via `ctest -N`
      (test indices 66..71).
- [x] `ctest --output-on-failure` reports 71/71 passing in Debug.
      Verified 2026-05-16.
- [x] `clang-format` clean on all new files (enforced by the
      `PostToolUse` hook on every Edit/Write).
- [x] `ds-architecture-rule-checker` and `gbatek-reviewer` ran clean
      on every commit's uncommitted diff.
- [x] `quality-reviewer` ran clean on every commit.
- [x] No file exceeds the 500-line soft cap. Largest new production
      file `rtc.cpp` at 461 lines; largest new test
      `rtc_commands_test.cpp` at 354 lines.
- [x] No new SDL include path. No new `<chrono>` include in
      `libds_core` (frontend-only). Static grep over `src/rtc/`,
      `src/nds.cpp`, and `src/nds.hpp` returns only a documentation
      comment at `src/nds.hpp:31`.
- [x] No new pointers held across subsystems. No `cpu/` or `bus/`
      includes from `rtc/` — confirmed by grep over `src/rtc/`.
- [x] No save-state code added (deferred per CLAUDE.md rule-5
      carve-out). `reset()` implemented on `Rtc`.

---

## Appendix A. Commit sequence

Seven commits, shippable individually. Each ends with `ctest` green.

### Commit 1 — `rtc: scaffold class + 0x4000138 routing + control-bit storage`

- Create `src/rtc/rtc.{hpp,cpp}` with the public surface from §4.2.
  `reset()` zeroes `pins_`, the SIO state, and seeds a default
  `dt = {2026, 1, 1, 4, 0, 0, 0}`. `tick()` is a stub that does
  nothing. `read_pins()` returns `pins_`. `write_pins()` stores `pins_`
  with no state-machine logic yet.
- `src/CMakeLists.txt`: add `rtc/rtc.cpp` to the `ds_core` target.
- `src/bus/io_regs.hpp`: add `IO_RTC = 0x04000138u`.
- `src/nds.hpp`: include `rtc/rtc.hpp`, add `Rtc rtc_{}` member,
  `Rtc& rtc()` accessor, `seed_rtc_from_host_time()` declaration.
- `src/nds.cpp`: `reset()` calls `rtc_.reset()`. ARM7 IO routes for
  `0x4000138` in `arm7_io_read8 / write8` (and the byte-aliased
  halfword/word paths). `seed_rtc_from_host_time()` body.
- No new tests this commit — the routing is trivially exercised
  later. Rationale: keep the scaffold commit small and dependency-free.

**Diff size estimate:** ~140 lines added across 5 files. CTest count
65 → 65 (no new tests).

### Commit 2 — `rtc: implement SIO bit-bang state machine (CS/SCK/SIO transfer)`

- Implement the §4.5 state machine inside `Rtc::write_pins` /
  `Rtc::read_pins`. Decode the command byte; validate bits 5..7 ==
  `110b`; route to a stub handler that records `active_cmd_` but
  does not yet apply data.
- New test binary `rtc_protocol_test.cpp` per §6.1.
- `tests/CMakeLists.txt`: `add_ds_unit_test(rtc_protocol_test)`.

**Diff size estimate:** ~120 lines in `rtc.cpp`, ~220 lines new test.
CTest count 65 → 66.

### Commit 3 — `rtc: implement read-side commands (status1, status2, datetime, date, time)`

- Implement `produce_read_byte()` for the five read commands.
  Status1 read auto-clears bits 4-7. DateTime / Date / Time return
  BCD-encoded bytes from `dt_`.
- New test binary `rtc_commands_test.cpp` covering the read-side
  REQUIRE blocks from §6.2.
- `tests/CMakeLists.txt`: `add_ds_unit_test(rtc_commands_test)`.

**Diff size estimate:** ~80 lines in `rtc.cpp` (read handlers + BCD
helpers), ~140 lines test. CTest count 66 → 67.

### Commit 4 — `rtc: implement write-side commands (status1, status2, alarm1, alarm2)`

- Implement `apply_write_byte()` for the four write commands.
  Status1 writes preserve bits 4-7 (chip-side: those bits are flags,
  not writable).
- Append the write-side REQUIRE blocks from §6.2 to
  `rtc_commands_test.cpp` (no new test binary).

**Diff size estimate:** ~70 lines in `rtc.cpp`, ~140 lines appended
to test. CTest count 67 → 67.

### Commit 5 — `rtc: schedule 1 Hz tick + calendar advance`

- Add `EventKind::Rtc1HzTick` to `src/scheduler/event.hpp`.
- Implement `Rtc::tick()` calendar advance per §4.6 (no alarm
  comparison or raise yet — just the time math). Implement
  `is_leap` and `days_in_month`.
- `nds.cpp`: introduce `kArm9CyclesPerSecond`. Schedule the first
  `Rtc1HzTick` in `reset()`. Add the new arm to `on_scheduler_event`
  that calls `rtc_.tick(irq7_)` (irq7 unused this commit but the
  signature is stable), then `update_arm7_irq_signals()` (no-op
  this commit since tick() doesn't raise), then reschedules.
- New test binary `rtc_tick_test.cpp` per §6.4.
- `tests/CMakeLists.txt`: `add_ds_unit_test(rtc_tick_test)`.

**Diff size estimate:** ~90 lines in `rtc.cpp` + `nds.cpp`, ~180
lines test. CTest count 67 → 68.

### Commit 6 — `rtc: implement alarm comparison + IF.7 raise on rising edge`

- Implement `alarm_matches` and the rising-edge raise in `Rtc::tick`
  per §4.6.
- New test binaries `rtc_alarm_test.cpp` and `rtc_irq_test.cpp` per
  §6.3 and §6.5.
- `tests/CMakeLists.txt`: `add_ds_unit_test(rtc_alarm_test)`,
  `add_ds_unit_test(rtc_irq_test)`.

**Diff size estimate:** ~50 lines in `rtc.cpp`, ~400 lines across
two new tests. CTest count 68 → 70.

### Commit 7 — `rtc: end-to-end alarm integration test`

- No production code changes (or only any small fixes that surface
  during writing the integration test, in which case fold them back
  into commits 2-6 and reorder).
- New test binary `rtc_integration_test.cpp` per §6.6.
- `tests/CMakeLists.txt`: `add_ds_unit_test(rtc_integration_test)`.
- After this commit, `ctest` reports 71/71 passing.

**Diff size estimate:** ~140 lines test, no production change.
CTest count 70 → 71.

---

## Appendix B. Provenance audit

- RTC bus register layout in §5.1, §5.2: verbatim from GBATEK fetched
  2026-05-04 from
  `https://problemkaputt.de/gbatek-ds-real-time-clock-rtc.htm`.
  No interpretation, no rephrasing.
- Command byte format in §5.3: GBATEK same URL + S-35190A datasheet
  table 5 (cited but not bundled — datasheet is publicly available;
  the design references it for the exact bit-encoded patterns).
- Status1 / Status2 / Alarm field layouts in §5.4-§5.7: GBATEK same
  URL, fetched 2026-05-05 (re-verified the day of writing the spec).
- IRQ source bit map in §5.8: GBATEK fetched 2026-05-04 from
  `https://problemkaputt.de/gbatek.htm#dsinterrupts`. Filtered to
  bit 7 only; full table in upstream.
- Alarm fire-condition resolution in §5.7: GBATEK is silent;
  resolution chosen from melonDS source (`src/RTC.cpp`, fetched
  2026-05-04). Used as a reference, not copied. Decision and
  alternatives documented in §5.7.
- BCD encoding in §5.6: standard 4-bit packed BCD, two digits per
  byte, high nibble = tens. Trivial; cross-checked against the
  S-35190A datasheet table 4.
- Calendar / leap-year rule in §4.6 + §8.1 #5: standard Gregorian
  rule. Not chip-specific; reused from `<ctime>` semantics but
  reimplemented because we cannot use `<chrono>` in `libds_core`.
- No BIOS dump, no firmware dump, no proprietary data, no
  copyrighted code referenced by this design.
