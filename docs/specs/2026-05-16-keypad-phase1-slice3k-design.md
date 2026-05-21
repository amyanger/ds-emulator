# Keypad + Lid Switch + Input-Driven IRQ — Phase 1, Slice 3k Design

**Date:** 2026-05-16
**Slice:** Keypad (KEYINPUT / KEYCNT) and lid switch (EXTKEYIN.7) subsystems,
plus the second and third input-driven (rising-edge) IRQ sources on the
project. KEYINPUT is a shared physical state visible to both CPUs at
`0x4000130`. KEYCNT at `0x4000132` has two independent storage cells (one
per CPU); each cell gates its own ARM-side IF.12 raise independently.
EXTKEYIN at `0x4000136` is NDS7-only — ARM9 access returns open-bus — and
its hinge bit (EXTKEYIN.7) drives the second new source, NDS7 IF.22
"Screens unfolded", on the rising edge of "unfolded" (i.e. on the 1→0
transition of the hinge bit).
**Status:** closed (2026-05-21) — landed through `14d3cac`; 77 CTest binaries
green. All §9 completion criteria ticked. No deviations from the design — the
seven planned commits shipped in the documented order (`44662f2..14d3cac`),
the six new test binaries registered as planned (CTest indices 72..77), and
every new file came in well under the 500-line soft cap (largest production
file `keypad.cpp` at 82 lines; largest test `keypad_irq_test.cpp` at 226
lines).
**Prior slice:** 3j (ARM7 RTC + scheduler-driven IRQ source) — landed
through `f42b0d8`; 71 CTest binaries green.
**Next slice (proposed):** TBD. Candidates: timers (4 per CPU, IF.3..6),
DMA (4 per CPU, IF.8..11), or PPU scaffolding for first pixels. Choice
informed by what blocks the next concrete boot milestone.

---

## 1. Summary

Slice 3i wired three IPC-driven IRQ sources (IF.16 SYNC, IF.17 send-empty,
IF.18 recv-not-empty); each raises *synchronously inside the bus write
that caused the state change*. Slice 3j added the first source where the
raise is *decoupled from a bus write* — the RTC alarm fires from a 1 Hz
scheduler tick, on rising edge of (enabled AND match).

Slice 3k introduces the third class of raise: **input-driven**. The
keypad and lid sources do not fire on a bus write, and they do not fire
on a scheduler tick. They fire when the **host** (the frontend, or a
test) pushes new input state into the core. The signal flow is:

```
Frontend (SDL) ──set_keypad_state(0x03FE)──▶ NDS::set_keypad_state
                                              ─▶ Keypad::set_keyinput
                                                  ─▶ recompute IF.12 on both CPUs
                                                  ─▶ update_armN_irq_signals
```

A second, structurally identical path exists for the lid switch:

```
Frontend ──set_lid_closed(true)──▶ NDS::set_lid_closed
                                    ─▶ LidSwitch::set_closed
                                        ─▶ on rising-edge of "unfolded":
                                           irq7.raise(1u << 22)
                                        ─▶ update_arm7_irq_signals
```

Beyond pushing input from outside, the **other** callsite that recomputes
the keypad condition is **every KEYCNT write**. Toggling enable bit 14
from 0→1 while the condition is already true is a rising edge per
GBATEK's edge-trigger model (same discipline as slice 3i's IF.17 / IF.18
where toggling the per-side IRQ-enable on while the gated condition is
already true *does* raise).

Two new IRQ sources land: IF.12 (keypad — both CPUs, independently
gated) and IF.22 (screens-unfolded — NDS7 only, no source-side enable).
A small new public surface lands on `class NDS`: `set_keypad_state(...)`
and `set_lid_closed(...)`. A new subsystem directory lands: `src/input/`,
holding two files (`keypad.{hpp,cpp}` and `lid_switch.{hpp,cpp}`).

### Plain-language summary

The DS has two physical input panels and one mechanical switch:

1. **The main keypad** — A, B, Select, Start, D-pad (R/L/U/D), R, L. Ten
   buttons. Their state lives in one 16-bit register at `0x04000130`,
   readable from both CPUs, **active-low** (a 0 bit means *pressed* — a
   classic embedded convention because pulled-up inputs idle high). A
   second register at `0x04000132`, `KEYCNT`, holds the IRQ configuration.
   Critically, **each CPU has its own KEYCNT cell**: ARM9's KEYCNT and
   ARM7's KEYCNT live at the *same address* but in *different storage*.
   Each CPU's KEYCNT bits select which buttons the *that* CPU's keypad
   IRQ cares about, in what combination (OR / AND), and whether it's
   enabled at all.

2. **The X / Y buttons + pen + hinge** — bundled into `EXTKEYIN`, an
   NDS7-only register at `0x04000136`. ARM9 reads of this address are
   open-bus and return zero on real hardware. GBATEK is explicit that
   "Interrupts are reportedly not supported for X,Y buttons" — only the
   hinge surfaces as an IRQ source (bit 22, NDS7 only).

3. **The hinge** — EXTKEYIN bit 7 reads **0 when the lid is open** and
   **1 when the lid is closed** (folded — the screens facing each
   other). When the lid goes from closed to open ("unfolded" — 1→0
   transition), IF.22 raises on NDS7. GBATEK says this raise *has no
   source-side enable*: the only gate is the IE.22 bit. Pokemon and other
   games use this to wake the console from "sleep mode" (the ARM7 has
   entered HALTCNT-Sleep, which is just a deeper halt for our purposes).

For the emulator this is straightforward — these are not bit-bang
protocols, not periodic ticks, not anything timing-sensitive. The CPU
just reads three 16-bit registers and the frontend writes their backing
state. The interesting part is the IRQ semantics:

- **IF.12 is rising-edge** of `condition(KEYINPUT, KEYCNT)`, where
  `condition` is bit-14-enable-gated and bit-15-OR/AND-mode-selected over
  the bit-0..9 mask. Computed *per CPU* against *that CPU's KEYCNT*.
  Recomputed on every input change and every KEYCNT write.
- **IF.22 is rising-edge of "unfolded"** — i.e. on the EXTKEYIN.7
  transition 1→0. Falling edge (lid closing) raises nothing. There is no
  per-source enable bit; the only gate is IE.22 on the NDS7 controller.

The architectural decisions in this slice are smaller than slice 3j's
(no SIO protocol, no calendar math, no scheduler event), but the
*structural* shape is the same: introduce a new subsystem class, route
its registers through both bus tables (ARM7 only for EXTKEYIN; ARM7 + ARM9
for KEYINPUT and KEYCNT), wire the IRQ raises into the existing IRQ
controllers by-reference, add a host-injection entry point on NDS, and
add a focused test family.

The hard parts — where bugs hide — are not the data structure. They are:

- **KEYCNT has two cells at one address.** ARM9 writes `0x4000132` and
  ARM7 writes `0x4000132` and they do not see each other's writes. This
  has to be modeled explicitly — one `Keypad` instance, two `KeyCnt`
  storage cells, accessed by a `Side` enum (mirroring `IpcSync::Side` and
  `IpcFifo::Side`).
- **Rising-edge recompute must happen at four callsites, not one.** ARM9
  KEYCNT write; ARM7 KEYCNT write; KEYINPUT change from the host; lid
  change from the host. Forgetting any one of them loses a rising edge
  forever (the "previous condition" latch desyncs from physical state).
- **Active-low everywhere.** The user-facing model is "this button is
  pressed". The register-facing model is "this bit is 0". KEYCNT bit-0..9
  selects buttons against the *active-low* KEYINPUT directly — i.e. a
  selected button "matches" when its KEYINPUT bit is 0. Easy to flip if
  the helper isn't named carefully.
- **EXTKEYIN is NDS7-only.** ARM9 reads must fall through to the default
  open-bus zero in `arm9_io_read*`. This matches RTC's pattern but for a
  *halfword* register, not a byte.
- **Bit 14 vs bit 15 of KEYCNT.** Bit 14 is the *enable* (we treat 0 as
  "entire source disabled"); bit 15 is the *condition* (OR vs AND of the
  selected bits). These are easy to swap if you're reading GBATEK on a
  small screen — they sit adjacent and look symmetric. The condition is
  computed only when bit 14 is set; with bit 14 clear, the source's
  condition is forced false regardless of which buttons are pressed or
  what bit 15 says.

The architectural decisions in this slice are deliberately small.

**What this slice builds:**

- `class Keypad` in `src/input/keypad.{hpp,cpp}`. State: shared
  `keyinput_` u16 (active-low), two `KeyCnt` cells indexed by
  `Side::Arm9` / `Side::Arm7`, two `prev_condition_` rising-edge latches
  (one per CPU).
- `class LidSwitch` in `src/input/lid_switch.{hpp,cpp}`. State: shared
  `extkeyin_` u16 (with bit 7 = "closed" flag), one
  `prev_unfolded_` rising-edge latch. Separate file from `Keypad` for the
  reasons stated in §4.0 ("Where does the lid live").
- Three new I/O register constants in `src/bus/io_regs.hpp`:
  `IO_KEYINPUT`, `IO_KEYCNT`, `IO_EXTKEYIN`.
- New host-side public methods on `NDS`: `set_keypad_state(u16
  keyinput)` and `set_lid_closed(bool closed)`.
- ARM9 bus routes for `0x4000130` and `0x4000132` (read + write, all
  three widths). ARM7 bus routes for `0x4000130`, `0x4000132`, and
  `0x4000136` (read + write, all three widths). ARM9 access to
  `0x4000136` falls through to open-bus zero (no route added — the
  default fall-through covers it; documented for the GBATEK reviewer).
- Six new test binaries: `keypad_state_test`, `keypad_keycnt_test`,
  `keypad_irq_test`, `extkeyin_test`, `lid_irq_test`,
  `keypad_integration_test`.

**What this slice deliberately does NOT build:**

- Touchscreen / pen-down (`EXTKEYIN.6`). Constant 1 (= released /
  disabled) until the TSC2046 subsystem lands. Documented in §3.
- DEBUG button (`EXTKEYIN.3`). No retail game uses it.
- IRQ generation from X / Y buttons (`EXTKEYIN.0`, `EXTKEYIN.1`).
  GBATEK: "Interrupts are reportedly not supported for X,Y buttons."
- The "RCNT should be set to `0x80xx` before accessing EXTKEYIN" quirk.
  Every emulator ignores it; documented in §3.
- DSi extensions. Out of emulator scope.
- Save-state serialization. Per CLAUDE.md rule-5 carve-out, `reset()`
  only this slice.
- Lid IRQ on the *closing* edge (0→1 of EXTKEYIN.7). GBATEK and every
  reference emulator agree: only the *unfolding* edge raises.
- Frontend SDL key-binding logic. The frontend stays a separate target
  (slice for input wiring lands in Phase 3); this slice exposes the
  injection API only, with tests as the primary consumer.

### Scope boundary

**In scope:** two new classes (`Keypad`, `LidSwitch`), three new I/O
constants, two new host-injection methods on NDS, ARM9 and ARM7 bus
routes for the three addresses (with ARM9 EXTKEYIN open-bus), rising-edge
raise into the existing `IrqController`s by reference, six new test
binaries.

**Out of scope:** see §3.

---

## 2. Goals

1. **`0x4000130` (KEYINPUT) is a shared 16-bit R register, active-low,
   visible identically from both CPUs.** Reset value `0x03FF`. Bits 10..15
   read as 0. Read on either bus returns the current value. Writes are
   ignored on both buses (read-only register; documented as drop-with-
   DEBUG-warn for sub-16-bit writes).
2. **`0x4000132` (KEYCNT) is a 16-bit R/W register with two independent
   storage cells**, one per CPU. ARM9 read / write touches the ARM9 cell;
   ARM7 read / write touches the ARM7 cell. Bits 0..9 select buttons
   against the bit-0..9 of KEYINPUT. Bits 10..13 reserved (R/W as zero).
   Bit 14 = source enable (0 disables this CPU's keypad IRQ entirely).
   Bit 15 = condition (0 = OR, 1 = AND). Reset value `0x0000` per cell.
3. **`0x4000136` (EXTKEYIN) is a 16-bit R register on the ARM7 bus only.**
   ARM9 access returns the default-fall-through zero (open-bus). Reset
   value `0x007F` (X / Y / DEBUG / unknowns released; pen-down = 1
   "disabled"; hinge bit 7 = 0 "open"; bits 8..15 = 0). Bits 0..5 and
   bits 8..15 are read-only in this slice (touchscreen, X / Y, DEBUG are
   out of scope; the host never writes them). Bit 7 toggles via
   `set_lid_closed(...)`.
4. **IF.12 raises on the rising edge of `condition(side)` for that
   side's KEYCNT.** `condition(side)` = `bit 14 enabled` AND `mode-mask`
   over `selected_pressed_mask`, where `selected_pressed_mask` =
   `KEYCNT[0..9] AND NOT KEYINPUT[0..9]` (active-low inversion). Mode-OR:
   `selected_pressed_mask != 0`. Mode-AND: `selected_pressed_mask ==
   KEYCNT[0..9]` (every selected button pressed). The condition is
   recomputed at exactly four callsites — three Keypad-internal:
   `set_keyinput` (host input change), `write_keycnt(Arm9, …)`,
   `write_keycnt(Arm7, …)` — and once when the slice does not need to
   recompute on read. Each rising edge from per-side `prev_condition_` →
   `condition` calls `irqN.raise(1u << 12)` on that side.
5. **IF.22 raises on the rising edge of "unfolded"**, i.e. on the
   EXTKEYIN.7 transition 1→0. Recomputed at exactly one callsite:
   `LidSwitch::set_closed`. No source-side enable bit; only `IE.22`
   gates delivery. NDS7 only — `irq9_` is never called from `LidSwitch`.
6. **`class NDS` exposes `set_keypad_state(u16 keyinput)` and
   `set_lid_closed(bool closed)`.** Both are platform-free; the frontend
   wires them up to SDL events in a later input slice. Tests call them
   directly. Both methods invoke the subsystem method, then
   `update_arm7_irq_signals()` and (for keypad only)
   `update_arm9_irq_signals()`.
7. **Seven commits, each shippable individually, each ending green.**
   Same discipline as slice 3j. CTest count 71 → 77 (six new binaries).
8. **No new architectural debt.** `Keypad` and `LidSwitch` hold no
   pointer to any other subsystem (rule 3). The raise pathway is by
   reference: `Keypad::write_keycnt(Side, u16, IrqController& irq9,
   IrqController& irq7)` and `LidSwitch::set_closed(bool,
   IrqController& irq7)`. No cross-subsystem includes (rule 8): `input/
   *.cpp` includes `interrupt/irq_controller.hpp` only, the same leaf
   header that `rtc/`, `ipc/` already use.
9. **All new files stay well under the 500-line soft cap.** Estimated
   sizes: `keypad.hpp` ~80 lines, `keypad.cpp` ~180 lines, `lid_switch.
   hpp` ~50 lines, `lid_switch.cpp` ~70 lines. Largest test file
   estimated 230 lines (`keypad_irq_test.cpp`).

### 2.1 IRQ source coverage matrix (post-slice)

| Bit | Source                       | This slice? | Notes                                |
|-----|------------------------------|-------------|--------------------------------------|
| 7   | SIO/RCNT/RTC (NDS7 only)     | done (3j)   | RTC only; SIO / RCNT later.          |
| 12  | Keypad (NDS9 / NDS7)         | **YES**     | Per-CPU KEYCNT; both ARM9 + ARM7.    |
| 16  | IPC Sync                     | done (3i)   | Already raised.                      |
| 17  | IPC Send FIFO Empty          | done (3i)   | Already raised.                      |
| 18  | IPC Recv FIFO Not Empty      | done (3i)   | Already raised.                      |
| 22  | Screens unfolded (NDS7 only) | **YES**     | No source-side enable; only IE.22.   |
| (rest) | (other sources)           | NO          | Future slices.                       |

**2 IRQ sources (IF.12 + IF.22) go from "no implementation" to "fully
wired" + 6 new test binaries. CTest count 71 → 77.**

---

## 3. Non-goals

- **Touchscreen / pen-down (EXTKEYIN bit 6).** Slice 3k holds bit 6 at
  `1` (released / disabled) for the lifetime of the slice. The TSC2046
  SPI touchscreen subsystem lands later (Phase 3, input slice). The
  X-Ray overlay shows `pen_down=0` as a literal until that slice.
- **DEBUG button (EXTKEYIN bit 3).** No retail game queries it. Bit
  held at `1`.
- **X / Y IRQ generation.** GBATEK: "Interrupts are reportedly not
  supported for X,Y buttons." We hold their EXTKEYIN bits R/O (the host
  writes them via a Phase 3 X / Y wiring method that this slice does
  not introduce) and do not gate any IRQ source on them.
- **The "RCNT should be set to `0x80xx` before accessing EXTKEYIN"
  quirk.** GBATEK mentions it; every reference emulator (melonDS,
  DeSmuME) ignores it. So do we; documented for the reviewer.
- **DSi keypad extensions** (EXTKEYIN bits beyond bit 7 with DSi
  semantics). Out of emulator scope.
- **Save-state serialization** for `Keypad` and `LidSwitch`. Per
  CLAUDE.md rule-5 carve-out, `reset()` only this slice. The rising-
  edge latches `prev_condition_` and `prev_unfolded_` will need to be
  in the serialized state — flagged in §8.
- **Lid IRQ on the folding edge (0→1).** Only the unfolding edge
  (1→0) raises IF.22. The folding edge changes EXTKEYIN.7 to 1 and
  updates `prev_unfolded_` but raises nothing.
- **Frontend SDL key-binding logic and the keybinds.cfg loader.** That
  is `src/frontend/input_config.cpp`, in the Phase 3 input slice.
  Slice 3k exposes the injection API only; the frontend stays SDL-free
  by definition (`libds_core` rule 6).
- **Sub-16-bit access to KEYINPUT / KEYCNT / EXTKEYIN that doesn't
  follow GBATEK's normal aliasing.** Byte reads return the appropriate
  byte; halfword reads return the full register. Byte and halfword
  writes to KEYINPUT and EXTKEYIN are silently dropped (read-only
  registers). Byte writes to KEYCNT update the corresponding byte of
  the targeted side's cell.
- **Coalescing multiple input changes from one frame into a single
  raise.** Each `set_keypad_state` call computes one rising-edge
  check. Pokemon-style games never push more than one input change per
  emulated frame; if a single host event would somehow change KEYINPUT
  twice in zero ARM9 cycles, the second call's "previous" is the first
  call's "current" — exactly what real hardware would do. Documented
  rather than carved out.

---

## 4. Architecture

### 4.0 Open design questions — resolutions

#### Q1. KEYCNT as one class with two cells, or two separate small structs?

**Decision: one `class Keypad` with a `KeyCnt cnt_[2]` array indexed by
`Side::Arm9` / `Side::Arm7`.** Same idiom as `IpcSync` and `IpcFifo`,
which both use a `Side` enum and per-side state. The alternative — two
separate `ArmKeyCnt` / `Arm7KeyCnt` classes — duplicates the rising-
edge latch logic without benefit, since the *condition* function is
identical and only the *storage cell* differs per side.

Trade-off accepted: callers pass a `Side` argument on every KEYCNT
read / write. Acceptable; same pattern as IPC and already proven
unambiguous in tests.

#### Q2. Lid as a separate class or a field inside `Keypad`?

**Decision: separate `class LidSwitch` in `src/input/lid_switch.{hpp,
cpp}`.** Rationale:

- The slice 3j precedent: RTC is a small subsystem (a single u8 pin-
  register would have fit in `ipc_sync.cpp` if minimum byte count
  decided architecture) but it lives in its own file because it's a
  distinct GBATEK source with its own raise semantics. Same applies to
  EXTKEYIN / IF.22.
- The lid raise is **NDS7-only** and **has no source-side enable**,
  whereas the keypad raise is **per-CPU** and **gated by KEYCNT bit 14**.
  Folding lid into `Keypad` couples two semantically different raise
  models into one class.
- The two subsystems share no state. `keyinput_` is in `Keypad`;
  `extkeyin_` is in `LidSwitch`. Co-locating them would require a
  one-class-two-roles design that hides the distinction.
- File sizes are well within the 500-line soft cap either way; this
  is a clarity decision, not a size decision.

Trade-off accepted: two small files instead of one. Both are far below
even the loosest soft cap.

#### Q3. Public surface on `class NDS`.

**Decision:** two methods:

```cpp
// In nds.hpp, alongside seed_rtc_from_host_time.
void set_keypad_state(u16 keyinput);     // full 16-bit KEYINPUT value
void set_lid_closed(bool closed);
```

Alternatives considered:

- **Per-button setters** (`set_button_a(bool pressed)`, …). Rejected:
  ten setters instead of one; ten potential callsites for "did you
  remember to recompute IRQ" instead of one; the SDL frontend will
  build the 16-bit mask from per-button keys anyway, so the bitmap is
  the natural API boundary.
- **`InputState` struct** wrapping `keyinput`, `extkeyin`, `lid_closed`.
  Rejected: hides which fields the host can write (most of EXTKEYIN is
  not host-writable this slice) and bundles two unrelated raise paths
  behind one entry point.
- **Direct accessors** (`nds.keypad().set_keyinput(...)`). Rejected:
  the host should not have to know which subsystem owns which piece of
  input. Going through NDS preserves rule 3 (no inter-subsystem
  pointers) at the host boundary too.

Trade-off accepted: tests construct a u16 by hand. Acceptable; the bit
layout is small (10 buttons, well-documented) and the helpers
`KeyMask::a()`, etc., can be added later if it becomes friction.

#### Q4. Where does the rising-edge recompute fire?

**Decision: pure event-driven, recomputed at exactly four call sites:**

- `Keypad::set_keyinput(u16 v, IrqController& irq9, IrqController& irq7)`
  — called from `NDS::set_keypad_state`.
- `Keypad::write_keycnt(Side::Arm9, u16 v, IrqController& irq9,
  IrqController& irq7)` — called from `NDS::arm9_io_write*` at the
  KEYCNT route.
- `Keypad::write_keycnt(Side::Arm7, u16 v, IrqController& irq9,
  IrqController& irq7)` — called from `NDS::arm7_io_write*` at the
  KEYCNT route.
- `LidSwitch::set_closed(bool, IrqController& irq7)` — called from
  `NDS::set_lid_closed`.

The alternative — polling via a scheduler event — is wasteful (Pokemon
games change input at ≤60 Hz; a polling tick would have to be at least
1 ms granularity to avoid input lag, which is a 1000 Hz event vs ≤60 Hz
worth of actual changes — two orders of magnitude of scheduler overhead
for nothing).

Same model as IPC FIFO / IPC SYNC — every state change goes through
`recompute_irqs`. The keypad's `recompute_irqs` runs per-side because
each side has its own previous-condition latch.

#### Q5. Open-bus ARM9 access to `0x4000136`.

**Decision: add no ARM9 route.** The default fall-through in
`arm9_io_read{8,16,32}` returns 0 for any address not covered by a
case (see `src/nds.cpp:107, 132, 160`). 0 is the value real hardware
exposes as open-bus on this address. ARM9 writes to `0x4000136` fall
through to `arm9_io_write{8,16,32}` which silently drop unmapped
writes — also matches hardware (the register doesn't exist on the
ARM9 view).

This mirrors slice 3j's handling of `0x4000138` (ARM9 RTC access is
open-bus by virtue of "no ARM9 route exists"). The pattern is
documented for the `gbatek-reviewer` checklist in §7.

#### Q6. Access widths supported.

KEYINPUT and KEYCNT and EXTKEYIN are 16-bit registers. Per GBATEK they
support 8-bit and 16-bit access on both buses (with the standard
"upper-byte / lower-byte" aliasing for byte access). 32-bit access
aliases the 16-bit register into the low halfword with the upper half
reading as the next register up.

Concretely, a 32-bit read at `0x4000130` returns `(KEYCNT_thisCPU << 16)
| KEYINPUT` — two adjacent 16-bit registers in one word. Tests must
exercise this aliasing (see §6.1).

#### Q7. Does ARM9 actually have KEYCNT?

**Yes.** GBATEK lists KEYCNT in *both* the NDS9 and NDS7 I/O maps. The
specification footnote on `0x04000132` reads: "The Key IRQ Control
Register is present on both ARM7 and ARM9; both controllers see their
own copy of the register at the same address." This slice models that
literally: two independent cells, one per side. A write from ARM9 to
`0x4000132` does **not** change what ARM7's IF.12 raise condition
observes, and vice versa. This is the central reason we keep one
shared `keyinput_` (physical button state is one object) but two
`cnt_` cells (each CPU's IRQ mask is its own).

### 4.1 File layout

```
src/input/                              NEW directory.
  keypad.hpp                            NEW. ~80 lines. class Keypad declaration:
                                          enum class Side { Arm9, Arm7 };
                                          struct KeyCnt {
                                              u16 select_mask = 0;     // bits 0..9
                                              bool enable = false;     // bit 14
                                              bool mode_and = false;   // bit 15 (0=OR)
                                          };
                                          void reset();
                                          void set_keyinput(u16 v,
                                              IrqController& irq9,
                                              IrqController& irq7);
                                          u16  read_keyinput() const;
                                          u16  read_keycnt(Side) const;
                                          void write_keycnt(Side, u16 v,
                                              IrqController& irq9,
                                              IrqController& irq7);
                                          // test introspection only
                                          bool prev_condition(Side) const;
                                        Internal: u16 keyinput_ = 0x03FF;
                                                  KeyCnt cnt_[2]{};
                                                  bool prev_condition_[2]{false,false};
  keypad.cpp                            NEW. ~180 lines. set_keyinput / read /
                                        write_keycnt bodies; recompute_irqs(Side)
                                        helper; mode-OR / mode-AND match; rising-
                                        edge latch per side. Includes
                                        interrupt/irq_controller.hpp only —
                                        no cpu/ or bus/ includes.
  lid_switch.hpp                        NEW. ~50 lines. class LidSwitch declaration:
                                          void reset();
                                          void set_closed(bool closed,
                                                          IrqController& irq7);
                                          u16  read_extkeyin() const;
                                          // test introspection only
                                          bool prev_unfolded() const;
                                        Internal: u16 extkeyin_ = 0x007F;
                                                  bool prev_unfolded_ = true;
  lid_switch.cpp                        NEW. ~70 lines. set_closed body; rising-
                                        edge latch on "unfolded" (1→0 of bit 7);
                                        irq7.raise(1u << 22) on edge.
                                        Same include rule as keypad.cpp.

src/bus/
  io_regs.hpp                           MODIFIED. Add:
                                          constexpr u32 IO_KEYINPUT  = 0x04000130u;
                                          constexpr u32 IO_KEYCNT    = 0x04000132u;
                                          constexpr u32 IO_EXTKEYIN  = 0x04000136u;

src/
  nds.hpp                               MODIFIED. Add:
                                          Keypad keypad_{};                 // member
                                          LidSwitch lid_switch_{};           // member
                                          Keypad& keypad() { return keypad_; }
                                          LidSwitch& lid_switch() { return lid_switch_; }
                                          void set_keypad_state(u16 keyinput);
                                          void set_lid_closed(bool closed);
                                        Include "input/keypad.hpp" and
                                        "input/lid_switch.hpp".
  nds.cpp                               MODIFIED. ~+80 lines:
                                          - reset() calls keypad_.reset() and
                                            lid_switch_.reset().
                                          - arm9_io_read{8,16,32} route IO_KEYINPUT
                                            and IO_KEYCNT.
                                          - arm9_io_write{8,16,32} route IO_KEYCNT
                                            (KEYINPUT writes dropped).
                                          - arm7_io_read{8,16,32} route IO_KEYINPUT,
                                            IO_KEYCNT, IO_EXTKEYIN.
                                          - arm7_io_write{8,16,32} route IO_KEYCNT
                                            (KEYINPUT and EXTKEYIN writes dropped).
                                          - set_keypad_state body forwards to
                                            keypad_.set_keyinput then
                                            update_armN_irq_signals on both sides.
                                          - set_lid_closed body forwards to
                                            lid_switch_.set_closed then
                                            update_arm7_irq_signals.

tests/
  keypad_state_test.cpp                 NEW. KEYINPUT shared-state read on both
                                        buses; reset value; active-low encoding;
                                        ARM9 / ARM7 see the same value after
                                        a host change. ~150 lines.
  keypad_keycnt_test.cpp                NEW. Per-CPU independent storage cells;
                                        ARM9 and ARM7 writes do not collide;
                                        bit-14 / bit-15 / bit-0..9 round trip;
                                        sub-width access. ~190 lines.
  keypad_irq_test.cpp                   NEW. Rising-edge IF.12 raise; OR vs AND
                                        mode; enable bit gating; per-CPU
                                        independence; ARM9 KEYCNT change must
                                        recompute ARM9 IF.12 independent of
                                        ARM7. ~230 lines.
  extkeyin_test.cpp                     NEW. ARM7 read of 0x4000136 round trip;
                                        reset value 0x007F; ARM9 read returns
                                        0 (open-bus); halfword vs byte access;
                                        bit 6 (pen) and bit 7 (hinge)
                                        independent. ~150 lines.
  lid_irq_test.cpp                      NEW. Rising-edge IF.22 raise on 1→0
                                        transition of EXTKEYIN.7; folding edge
                                        raises nothing; IE.22 gates delivery
                                        not raise; ARM9 IF unaffected. ~140 lines.
  keypad_integration_test.cpp          NEW. End-to-end: ARM7 sets IME=1, IE=
                                        (1<<12 | 1<<22), writes its KEYCNT,
                                        host calls set_keypad_state and
                                        set_lid_closed, REQUIRE both IF bits
                                        latch and ARM7 line goes high. ~150 lines.
```

### 4.2 Class designs

#### `Keypad`

```cpp
namespace ds {
class IrqController;

class Keypad {
public:
    enum class Side : u8 { Arm9, Arm7 };

    struct KeyCnt {
        u16  select_mask = 0;       // bits 0..9 of KEYCNT
        bool enable      = false;   // bit 14
        bool mode_and    = false;   // bit 15 (0 = OR, 1 = AND)
    };

    void reset();

    // Host-injection path. Updates the shared physical KEYINPUT state and
    // recomputes IF.12 for both CPUs against their own KEYCNT cells.
    void set_keyinput(u16 v, IrqController& irq9, IrqController& irq7);

    u16  read_keyinput() const;       // active-low; bits 10..15 forced to 0

    // Per-CPU KEYCNT R/W. The Side argument selects which storage cell is
    // touched. Writes recompute IF.12 for both sides (bit 14 / bit 15 /
    // select-mask change on one side can flip its own condition).
    u16  read_keycnt(Side) const;
    void write_keycnt(Side, u16 v,
                      IrqController& irq9, IrqController& irq7);

    // Test introspection only.
    bool prev_condition(Side) const;

private:
    u16    keyinput_ = 0x03FFu;       // bits 0..9 active-low; 10..15 zero
    KeyCnt cnt_[2]{};                 // index = Side
    bool   prev_condition_[2]{};      // rising-edge latch per side

    // Compute (selected, enabled, matching) for one side against current
    // keyinput_. Returns true iff the rising-edge gate should be true now.
    bool compute_condition(Side) const;

    // Recompute both sides' conditions and raise IF.12 on each rising
    // edge. Called from every callsite that can change the inputs to the
    // condition (set_keyinput, write_keycnt on either side).
    void recompute_irqs(IrqController& irq9, IrqController& irq7);

    static constexpr u16 kButtonMask  = 0x03FFu; // bits 0..9
    static constexpr u16 kEnableBit   = 1u << 14;
    static constexpr u16 kModeAndBit  = 1u << 15;
};
} // namespace ds
```

#### `LidSwitch`

```cpp
namespace ds {
class IrqController;

class LidSwitch {
public:
    void reset();

    // Host-injection path. closed=true sets EXTKEYIN.7=1 (folded);
    // closed=false sets it to 0 (open / unfolded). On the rising edge of
    // "unfolded" (i.e. 1→0 of bit 7), raises IF.22 on irq7.
    void set_closed(bool closed, IrqController& irq7);

    u16  read_extkeyin() const;

    // Test introspection only.
    bool prev_unfolded() const;

private:
    // Reset = 0x007F per §5.3. Layout:
    //   bit 0 X      = 1 (released)
    //   bit 1 Y      = 1 (released)
    //   bit 2 ?      = 1 (read as 1 per GBATEK "unknown / set")
    //   bit 3 DEBUG  = 1 (released)
    //   bit 4 ?      = 1
    //   bit 5 ?      = 1
    //   bit 6 pen    = 1 (released / disabled)
    //   bit 7 hinge  = 0 (open / unfolded)
    //   bits 8..15   = 0
    u16  extkeyin_       = 0x007Fu;
    bool prev_unfolded_  = true;        // bit 7 starts at 0 → "unfolded" = true
};
} // namespace ds
```

The lid IRQ semantics ("raise on rising edge of unfolded") mean
`prev_unfolded_` must be initialized to `true` after `reset()` —
otherwise the very first `set_closed(false)` (which sets unfolded =
true) would look like a 0→1 rising edge from the bool's
default-zero-initialization. The lid being open at reset is the normal
power-on state; the first IF.22 raise should only happen after a
genuine close-then-open cycle.

### 4.3 State ownership and dependency graph

```
NDS owns:
  Scheduler         scheduler_;
  IrqController     irq9_, irq7_;       // existing
  IpcSync           ipc_sync_;          // existing
  IpcFifo           ipc_fifo_;          // existing
  Rtc               rtc_;                // existing
  Keypad            keypad_;             // NEW
  LidSwitch         lid_switch_;         // NEW
  Arm7              cpu7_;
  Arm9              cpu9_;
  Arm7Bus / Arm9Bus

NDS adds:
  void set_keypad_state(u16 keyinput);
  void set_lid_closed(bool closed);
  Keypad& keypad();
  LidSwitch& lid_switch();

Call graph (rule 3 — no inter-subsystem pointers):
  Arm9Bus::slow_*          → NDS::arm9_io_{read,write}{8,16,32}(0x4000130)
                           → keypad_.{read_keyinput}            (KEYINPUT read)
                           → keypad_.{read_keycnt, write_keycnt}(Side::Arm9, …)
  Arm7Bus::slow_*          → NDS::arm7_io_{read,write}{8,16,32}(0x4000130/2/6)
                           → keypad_.{read_keyinput}            (KEYINPUT read)
                           → keypad_.{read_keycnt, write_keycnt}(Side::Arm7, …)
                           → lid_switch_.read_extkeyin()        (EXTKEYIN read)
  Frontend                 → NDS::set_keypad_state(v)
                           → keypad_.set_keyinput(v, irq9_, irq7_)
                           → update_arm9_irq_signals()
                           → update_arm7_irq_signals()
  Frontend                 → NDS::set_lid_closed(b)
                           → lid_switch_.set_closed(b, irq7_)
                           → update_arm7_irq_signals()

Keypad and LidSwitch hold no pointers to anything. IrqController is
passed by reference into every path that can raise.
```

`update_arm{9,7}_irq_signals()` bodies are unchanged from slice 3i / 3j.

`on_scheduler_event` is unchanged from slice 3j — keypad / lid raise is
*not* scheduler-driven. The body retains its existing two arms
(`FrameEnd`, `Rtc1HzTick`).

Header-include rule check (rule 8):
- `input/keypad.hpp` includes `ds/common.hpp` only; forward-declares
  `IrqController`. Compliant.
- `input/lid_switch.hpp` includes `ds/common.hpp` only; forward-
  declares `IrqController`. Compliant.
- `input/keypad.cpp` and `input/lid_switch.cpp` include
  `interrupt/irq_controller.hpp`. Same leaf header that `rtc/*.cpp` and
  `ipc/*.cpp` already include. Compliant.
- `nds.hpp` adds `#include "input/keypad.hpp"` and `#include
  "input/lid_switch.hpp"`. NDS is the integration point; allowed to
  include from any subsystem.
- `nds.cpp` adds no new includes — `interrupt/irq_controller.hpp` is
  already pulled in transitively via `nds.hpp`.

### 4.4 IO routing (the new bus paths)

**ARM9 bus** (`NDS::arm9_io_read*/write*`, existing pattern at
`src/nds.cpp:89..295`):

| Address      | Width  | Read                                                           | Write                                                |
|--------------|--------|----------------------------------------------------------------|------------------------------------------------------|
| `0x4000130`  | 8-bit  | `(keypad_.read_keyinput() >> shift) & 0xFF`                    | drop (KEYINPUT is R/O; DEBUG-warn)                   |
| `0x4000130`  | 16-bit | `keypad_.read_keyinput()`                                      | drop                                                 |
| `0x4000130`  | 32-bit | `(read_keycnt(Arm9) << 16) | read_keyinput()`                  | low halfword: drop; high halfword: write_keycnt(Arm9)|
| `0x4000132`  | 8-bit  | byte-shift of `read_keycnt(Arm9)`                              | RMW into `read_keycnt(Arm9)`; call `write_keycnt`    |
| `0x4000132`  | 16-bit | `keypad_.read_keycnt(Arm9)`                                    | `keypad_.write_keycnt(Arm9, value, irq9_, irq7_)`    |
| `0x4000132`  | 32-bit | high half is "next reg" = 0 (no `0x4000134` on ARM9 view); low = `read_keycnt(Arm9)` | low halfword: write_keycnt; high halfword: drop |
| `0x4000136`  | any    | 0 (default fall-through — open-bus on ARM9 view)               | drop (default fall-through)                          |

**ARM7 bus** (`NDS::arm7_io_read*/write*`, existing pattern at
`src/nds.cpp:300..586`):

| Address      | Width  | Read                                                                    | Write                                                |
|--------------|--------|-------------------------------------------------------------------------|------------------------------------------------------|
| `0x4000130`  | 8-bit  | `(keypad_.read_keyinput() >> shift) & 0xFF`                             | drop                                                 |
| `0x4000130`  | 16-bit | `keypad_.read_keyinput()`                                               | drop                                                 |
| `0x4000130`  | 32-bit | `(read_keycnt(Arm7) << 16) | read_keyinput()`                           | low halfword: drop; high halfword: write_keycnt(Arm7)|
| `0x4000132`  | 8-bit  | byte-shift of `read_keycnt(Arm7)`                                       | RMW into `read_keycnt(Arm7)`; call `write_keycnt`    |
| `0x4000132`  | 16-bit | `keypad_.read_keycnt(Arm7)`                                             | `keypad_.write_keycnt(Arm7, value, irq9_, irq7_)`    |
| `0x4000132`  | 32-bit | low = `read_keycnt(Arm7)`; high half is the EXTKEYIN low halfword window at `+4` per 32-bit aliasing of two 16-bit regs at 0x132 and 0x136 — see note | low halfword: write_keycnt; high halfword: drop (EXTKEYIN is R/O) |
| `0x4000136`  | 8-bit  | `(lid_switch_.read_extkeyin() >> shift) & 0xFF`                         | drop                                                 |
| `0x4000136`  | 16-bit | `lid_switch_.read_extkeyin()`                                           | drop                                                 |
| `0x4000136`  | 32-bit | low = `read_extkeyin()`, high = 0 (next reg `0x4000138` is RTC NDS7 1-byte — see note) | drop                                                 |

**Note on 32-bit reads at `0x4000132` on ARM7.** The next 16-bit
register up is EXTKEYIN at `0x4000136`, not at `0x4000134` (there is a
2-byte hole). A 32-bit read at `0x4000132` therefore returns
`(0 << 16) | read_keycnt(Arm7)` — the upper halfword reads as zero
because `0x4000134` is unmapped (open-bus). The 32-bit read at
`0x4000130` does include the KEYCNT halfword because `0x4000132` is
2 bytes higher (within the same 32-bit word). We do *not* extend the
32-bit read at `0x4000130` to include EXTKEYIN because EXTKEYIN sits
in the next 32-bit word, not the same one.

**Note on 32-bit reads at `0x4000136` on ARM7.** The next 32-bit-
aligned byte is `0x4000138` (RTC, NDS7 8-bit). Per existing slice-3j
behavior the upper bytes of an RTC 32-bit read are zero (`src/nds.
cpp:335..341`). For a 32-bit read at `0x4000136` we return
`read_extkeyin()` in the low halfword and 0 in the high halfword.
This matches the spec-compliant "upper unmapped is zero" pattern used
throughout.

**ARM9 access to `0x4000136`**: no route; default fall-through returns 0.

### 4.5 Keypad condition algorithm

Plain-language note: KEYINPUT is active-low — a button press *clears*
its bit. KEYCNT bits 0..9 *select* buttons against the *bit positions*
of KEYINPUT directly — that is, KEYCNT bit 4 = 1 means "include the
Right button (bit 4 of KEYINPUT) in the IRQ condition". The "match" of
a single selected bit is "that button is pressed", i.e. KEYINPUT bit ==
0. In OR mode the condition is true iff at least one selected button
is pressed; in AND mode the condition is true iff every selected button
is pressed.

```text
Keypad::compute_condition(side):
    const KeyCnt& c = cnt_[side]
    if !c.enable:                              // bit 14
        return false                            // source disabled entirely

    selected = c.select_mask & kButtonMask     // bits 0..9
    if selected == 0:
        return false                            // no buttons selected — never fires
                                                // (mode-OR: 0 != 0 → false;
                                                //  mode-AND: 0 == 0 trivially, but no
                                                //  buttons selected is treated as
                                                //  "no source" per GBATEK intent)

    pressed_mask = ~keyinput_ & kButtonMask    // 1 = pressed (active-low inversion)
    matched      = selected & pressed_mask     // selected bits that are also pressed

    if c.mode_and:                             // bit 15
        return matched == selected             // every selected button pressed
    else:
        return matched != 0                    // at least one selected button pressed


Keypad::recompute_irqs(irq9, irq7):
    cond9 = compute_condition(Arm9)
    cond7 = compute_condition(Arm7)

    if cond9 && !prev_condition_[Arm9]:
        irq9.raise(1u << 12)
    if cond7 && !prev_condition_[Arm7]:
        irq7.raise(1u << 12)

    prev_condition_[Arm9] = cond9
    prev_condition_[Arm7] = cond7
```

**Note on the `selected == 0` early-return.** Strict GBATEK reading
would allow OR-mode with selected=0 to be always-false (no bits to OR)
and AND-mode with selected=0 to be vacuously true ("all zero of the
selected bits are pressed" is true). The vacuous-true reading would
mean enabling AND-mode keypad IRQ without selecting any buttons fires
on every recompute, which is nonsensical. We resolve the ambiguity by
returning false when selected == 0 in both modes. This matches melonDS
(`src/Keypad.cpp`, fetched 2026-05-16 — see Appendix B). Documented in
§8 risk register as the most likely point of harmless deviation.

### 4.6 Lid switch transition algorithm

Plain-language note: the lid is a single bit (EXTKEYIN.7) and one
bool (`prev_unfolded_`). When the lid goes from closed (bit=1) to open
(bit=0), the bool transitions from false to true, which is the rising
edge of "unfolded". That edge raises IF.22.

```text
LidSwitch::set_closed(closed, irq7):
    new_bit7         = closed ? 1 : 0
    new_unfolded     = (new_bit7 == 0)         // rising edge of "unfolded" =
                                                // 1→0 of EXTKEYIN.7

    extkeyin_ = (extkeyin_ & ~(1u << 7)) | (new_bit7 << 7)

    if new_unfolded && !prev_unfolded_:
        irq7.raise(1u << 22)

    prev_unfolded_ = new_unfolded
```

The `prev_unfolded_` latch is initialized to `true` after `reset()`
(see §4.2). The very first `set_closed(false)` after reset finds
`new_unfolded == true && prev_unfolded_ == true`, so the rising-edge
gate stays false. The first raise can only happen after a close-then-
open cycle. This matches real-hardware behavior — power-on is "lid
open" and no spurious IF.22 fires at boot.

---

## 5. Hardware details

These are taken directly from GBATEK and verified against the page
fetched 2026-05-16 (`/gbatek-check` run for slice 3k). Cited
verbatim where possible; layouts are reproduced literally to avoid
introducing transcription bugs.

### 5.1 KEYINPUT (`0x04000130`, R, 16-bit; both CPUs at same address; shared physical state)

```
4000130h - KEYINPUT - Key Status (R)   [both CPUs, mirrored — shared state]
  0     Button A   (0=Pressed, 1=Released)     ← ACTIVE-LOW
  1     Button B
  2     Select
  3     Start
  4     Right
  5     Left
  6     Up
  7     Down
  8     Button R
  9     Button L
  10-15 Not used (read as 0)
  Reset = 0x03FF (no keys pressed).
```

[gbatek-2026-05-16 — https://problemkaputt.de/gbatek.htm#gbakeypadinput]

GBATEK note: "On NDS, the keypad is accessible at the same address on
both ARM7 and ARM9 buses." Treated literally: one `keyinput_` u16,
read by both buses.

### 5.2 KEYCNT (`0x04000132`, R/W, 16-bit; TWO independent cells, one per CPU)

```
4000132h - KEYCNT - Key Interrupt Control (R/W)  [TWO independent cells, one per CPU]
  0-9   Per-button select (1=Select for IRQ, 0=Ignore) — same bit layout as KEYINPUT
  10-13 Not used
  14    Button IRQ Enable    (0=Disable, 1=Enable)
  15    Button IRQ Condition (0=Logical OR, 1=Logical AND)
  Reset = 0x0000.
  OR mode: IRQ if ANY selected button pressed.
  AND mode: IRQ if ALL selected buttons pressed.
```

[gbatek-2026-05-16 — same URL]

GBATEK comment: "The keypad IRQ function is intended to terminate the
very-low-power Stop mode, it is not suitable for processing normal user
input."

Plain-language: enable a keypad IRQ in `IE`, set bit 14 of *that CPU's*
KEYCNT, select the wake-up buttons via bits 0..9, choose OR or AND
via bit 15, then HALT. The next time the condition rises, the IRQ
wakes the CPU. Per-CPU KEYCNT exists because NDS9 firmware code and
NDS7 firmware code wake independently from different button
combinations (e.g. ARM7 wakes on any button to acknowledge a system
event; ARM9 wakes on Start to resume from a game's pause menu).

### 5.3 EXTKEYIN (`0x04000136`, R, 16-bit; NDS7 only — ARM9 access is open-bus)

```
4000136h - NDS7 - EXTKEYIN - Key X/Y Input (R)   [NDS7 only — ARM9 access is open-bus]
  0     Button X     (0=Pressed, 1=Released)
  1     Button Y     (0=Pressed, 1=Released)
  2     Unknown / set
  3     DEBUG button (0=Pressed, 1=Released/None such)
  4,5   Unknown / set
  6     Pen down     (0=Pressed, 1=Released/Disabled)
  7     Hinge/folded (0=Open, 1=Closed)
  8-15  Unknown / zero
  Reset = 0x007F (bits 0-6 = 1, bit 7 = 0 'open', bits 8-15 = 0).
```

[gbatek-2026-05-16 — https://problemkaputt.de/gbatek.htm#dskeypadlidandhingestate]

GBATEK comment: "Interrupts are reportedly not supported for X,Y
buttons." — IRQ generation from EXTKEYIN sources X / Y / DEBUG / pen
is *not* implemented (out of scope per §3). Only EXTKEYIN.7 (hinge)
participates in the IRQ flow, and only on the unfolding edge.

GBATEK comment: "RCNT should be set to `80xx` before accessing
EXTKEYIN." Emulators uniformly ignore; documented in §3 as a non-goal.

### 5.4 IF / IE bits relevant to slice 3k

```
IF (0x4000214) bit map — slice 3k touches:
  Bit 12   Keypad                        (both CPUs; each gated by its own KEYCNT)
  Bit 22   NDS7 only: Screens unfolding  (no source-side enable; mask via IE.22)
```

[gbatek-2026-05-16 — https://problemkaputt.de/gbatek.htm#dsinterrupts]

GBATEK comment: "The hinge generates an interrupt request (there seems
to be no way to disable this, unlike as for all other IRQ sources),
however, the interrupt execution can be disabled in IE register."

Plain-language: IF.22 has no source-side enable. The chip pulls its
hinge line on every 1→0 transition of EXTKEYIN.7. IE.22 controls
delivery (whether the CPU enters the IRQ vector); IF.22 still latches.
This matches RTC's IF.7 semantics in §5.9.3 of the slice 3j spec —
raise is unconditional, delivery is IE-gated.

### 5.5 Per-CPU IF.12 raise

Both NDS9 and NDS7 see IF.12. Each side's raise is independent and
gated by *that side's* KEYCNT. A button press that satisfies ARM7's
KEYCNT but not ARM9's KEYCNT raises IF.12 on the NDS7 controller only.
This is the central reason for two storage cells (§4.0 Q1).

### 5.6 Reset values

Verified against GBATEK fetched 2026-05-16:

| Register     | Reset value | Source                                       |
|--------------|-------------|----------------------------------------------|
| KEYINPUT     | `0x03FF`    | "all keys released" (bits 0..9 = 1)          |
| KEYCNT (×2)  | `0x0000`    | "IRQ disabled, no select bits"               |
| EXTKEYIN     | `0x007F`    | bit 7 = 0 (lid open), bits 0..6 = 1, others 0|

§4.2 reflects these reset values literally.

### 5.7 Behavior walkthroughs

These six walkthroughs are the test specifications. Each enumerates
the bus-and-event sequence, the resulting state delta, and the visible
side effects. Mirrors slice 3j's §5.9.

#### 5.7.1 Reading KEYINPUT from both CPUs returns the same value

Plain-language: KEYINPUT is one physical register that both CPUs see
at the same address. A change made by the host is visible immediately
from both bus views.

Precondition: `reset()` ran. `keyinput_ == 0x03FF`.

Steps:
1. ARM9 bus reads `0x4000130` (16-bit): REQUIRE `0x03FF`.
2. ARM7 bus reads `0x4000130` (16-bit): REQUIRE `0x03FF`.
3. Host calls `set_keypad_state(0x03FE)` (Button A pressed).
4. ARM9 bus reads `0x4000130` (16-bit): REQUIRE `0x03FE`.
5. ARM7 bus reads `0x4000130` (16-bit): REQUIRE `0x03FE`.

Expected: identical values; no per-CPU divergence.

#### 5.7.2 Writing KEYCNT from ARM9 does not affect ARM7's view

Precondition: `reset()` ran. Both KEYCNT cells are `0x0000`.

Steps:
1. ARM9 bus writes `0x4001` to `0x4000132` (bit 0 = A selected, bit 14
   = enable; OR mode).
2. ARM9 bus reads `0x4000132`: REQUIRE `0x4001`.
3. ARM7 bus reads `0x4000132`: REQUIRE `0x0000` (unaffected).
4. ARM7 bus writes `0xC008` to `0x4000132` (bit 3 = Start selected,
   bits 14 + 15 = enable + AND mode).
5. ARM7 bus reads `0x4000132`: REQUIRE `0xC008`.
6. ARM9 bus reads `0x4000132`: REQUIRE `0x4001` (still).

Expected: storage cells are independent.

#### 5.7.3 Keypad IRQ in OR mode fires on rising edge of "any selected pressed"

Precondition: `reset()` ran. ARM7 KEYCNT = `0x4003` (A + B selected,
enable, OR). IE.12 and IME set on ARM7 so the line will be sampled
(test only inspects IF, but uses this to confirm the integration
path).

Steps:
1. Host calls `set_keypad_state(0x03FF)`: all released. `recompute_
   irqs` runs: `condition == false` on both sides. `prev_condition_
   [Arm7] = false`.
2. Host calls `set_keypad_state(0x03FE)`: A pressed. `recompute_irqs`
   runs: OR-mode, selected = `0x0003`, pressed_mask = `0x0001`,
   matched = `0x0001`, matched != 0 → `condition = true`. **Rising
   edge**: `irq7.raise(1u << 12)`. REQUIRE `irq7_.read_if() & (1<<12)
   != 0`. `prev_condition_[Arm7] = true`.
3. Host calls `set_keypad_state(0x03FC)`: A and B pressed. `recompute_
   irqs`: matched = `0x0003`, condition still true. No re-raise
   (rising edge already consumed). REQUIRE no second IF.12 set
   (already set; bit is still set, but the raise was idempotent — the
   test asserts the count of distinct raises via a wrapping
   IrqController mock or by acknowledging IF.12 between calls).
4. ARM7 acknowledges IF: `irq7_.write_if(1<<12)`. REQUIRE IF.12 clear.
5. Host calls `set_keypad_state(0x03FC)` again: no state change. No
   raise. REQUIRE IF.12 stays clear.
6. Host calls `set_keypad_state(0x03FF)`: all released. condition →
   false. `prev_condition_[Arm7] = false`. No raise.
7. Host calls `set_keypad_state(0x03FE)`: A pressed again. Rising edge
   → REQUIRE IF.12 set.

Expected: exactly one raise per rising edge of the OR-gated
condition. Falling edges do not raise. Re-pressing after acknowledge
without going through a fall does not produce a second raise.

#### 5.7.4 Keypad IRQ in AND mode requires every selected button

Precondition: ARM9 KEYCNT = `0xC03F` (D-pad + R + L + A + B + Sel +
Start selected — bits 0..5 — and bit 14 + bit 15 set). Wait, that's
ten bits; let me re-do: select A + B + Start, KEYCNT = `0xC00B` (bits
0, 1, 3 = `0x000B`, bit 14, bit 15 = `0xC000`, total `0xC00B`).

Steps:
1. Host: A pressed, B released, Start released. KEYINPUT = `0x03FE`
   (only bit 0 cleared). `pressed_mask = 0x0001`. `matched = 0x0001`.
   AND mode: `matched == selected (0x000B)`? No → condition false.
2. Host: A and B pressed, Start released. KEYINPUT = `0x03FC`.
   `pressed_mask = 0x0003`. `matched = 0x0003`. AND: 3 == 11b? No.
3. Host: A, B, Start pressed. KEYINPUT = `0x03F4`. `pressed_mask =
   0x000B`. `matched = 0x000B`. AND: 0x0B == 0x0B → condition true.
   **Rising edge** → `irq9.raise(1<<12)`. REQUIRE IF.12 set on irq9_.
4. Host: still all three pressed. No state change → no raise.
5. Host: release Start. matched = `0x0003` != `0x000B` → condition
   false. No raise (falling edge). `prev_condition_[Arm9] = false`.
6. Host: re-press Start. condition true → raise again.

Expected: exactly one raise per rising edge of "all selected
pressed". ARM7 side, which has KEYCNT = `0x0000` (disabled), never
raises.

#### 5.7.5 Lid IRQ on rising edge of "unfolded"

Precondition: `reset()` ran. `extkeyin_ == 0x007F` (bit 7 = 0 →
unfolded). `prev_unfolded_ == true`.

Steps:
1. Host calls `set_lid_closed(false)`. `new_bit7 = 0`, `new_unfolded =
   true`. Edge: `new_unfolded && !prev_unfolded_` = `true && !true` =
   false. No raise. `prev_unfolded_` stays true.
2. Host calls `set_lid_closed(true)`. `new_bit7 = 1`, `new_unfolded =
   false`. Edge: false → no raise. `prev_unfolded_ = false`. EXTKEYIN
   bit 7 now set; ARM7 bus read of `0x4000136` returns `0x00FF`.
3. Host calls `set_lid_closed(false)`. `new_bit7 = 0`, `new_unfolded
   = true`. Edge: `true && !false` = **rising** → `irq7.raise(1 <<
   22)`. REQUIRE `irq7_.read_if() & (1 << 22) != 0`. `prev_unfolded_
   = true`. EXTKEYIN bit 7 now clear; ARM7 bus read of `0x4000136`
   returns `0x007F`.
4. Host calls `set_lid_closed(false)` again. No change → no edge → no
   raise.

Expected: exactly one raise per genuine close-then-open cycle. No
raise on the closing transition.

#### 5.7.6 IF.22 latches even with IE.22 clear

Precondition: lid IRQ scenario from §5.7.5 step 3 produced IF.22 raise.
Now also: ARM7 has `IE = 0x0000` (no sources enabled in IE).

Steps:
1. After the rising-edge raise, `irq7_.read_if() & (1<<22)` is set.
2. `irq7_.line()` evaluates `(IME & 1) && (IE & IF)`. IE has bit 22
   clear, so the line is false. ARM7 stays in whatever state it was
   in.
3. A later ARM7 read of IF still shows bit 22 set (the latch
   persisted).
4. ARM7 writes `1<<22` to IF (write-1-clear). REQUIRE IF.22 clears.

Expected: raise is unconditional; delivery is IE-gated. Matches RTC's
IF.7 §5.9.3 behavior.

### 5.8 GBATEK ambiguity resolutions

#### 5.8.1 KEYCNT with `selected == 0` in AND mode

GBATEK does not specify. We return `false` (no raise) — see §4.5.
Justification: melonDS source (`src/Keypad.cpp`, fetched 2026-05-16)
implements the same; reasoning is that an AND condition with zero
operands has no practical "press anything" meaning, and no Pokemon
game writes KEYCNT with bit 14 = 1 and bits 0..9 = 0.

**Decision:** treat `selected == 0` as "no source", returning `false`
in both OR and AND modes. (Justification: melonDS `src/Keypad.cpp`;
no Pokemon-game pattern exercises the vacuous-true case.)

#### 5.8.2 KEYINPUT bits 10..15 read value

GBATEK says "Not used (read as 0)". We treat literally: the 16-bit
read masks to `0x03FF` regardless of what the host pushed. The host
API only takes bits 0..9 meaningfully; we still store the full u16
internally and mask on read, so a host that pushes garbage bits 10..15
gets `0x03FF`-masked output without corrupting state.

#### 5.8.3 EXTKEYIN "unknown / set" bits (2, 4, 5)

GBATEK calls these "Unknown / set". Real hardware presumably pulls
them high. Reset value `0x007F` has them set (= 1). We do not touch
them in `set_lid_closed` (it only RMWs bit 7). Bits 0, 1, 3, 6 are
out-of-scope per §3 and will be wired in their respective slices
(touch / X+Y / DEBUG). Until then they stay at their reset value of 1.

#### 5.8.4 Rising-edge raise when bit 14 toggles from 0→1 while condition holds

Same case as slice 3i's IF.17/IF.18 when the per-side enable bit was
toggled while the gated condition was already true. We treat it
identically: enabling bit 14 while buttons are already pressed *is* a
rising edge (the gate-output goes from false to true), so a raise
fires. Tested explicitly in §6.3 (`Keypad_KeycntEnableBitToggleRaises`).

### 5.9 Behavior summary

```text
KEYINPUT (R only, both CPUs):
  reset = 0x03FF; bit = 0 means pressed.
  Write: silently dropped (DEBUG-warn).

KEYCNT (R/W, two cells, one per CPU):
  reset per cell = 0x0000.
  bits 0..9: select bits, masked against pressed-active-low.
  bits 10..13: reserved (RW as zero).
  bit 14: enable. 0 disables this CPU's keypad IRQ entirely.
  bit 15: 0=OR, 1=AND.

EXTKEYIN (R only, NDS7 only):
  reset = 0x007F.
  bit 7 = 0 → lid open / unfolded.
  Other bits: §3 non-goals; stay at reset.
  ARM9 access: open-bus 0.

IF.12 (both CPUs):
  Raised on rising edge of condition(side).
  Recomputed at: set_keyinput, write_keycnt(side, …).

IF.22 (NDS7 only):
  Raised on rising edge of "unfolded" (1→0 of EXTKEYIN.7).
  Recomputed at: set_lid_closed.
  No source-side enable; only IE.22 gates delivery.
```

---

## 6. Testing strategy

CTest count goes from 71 to 77 (six new test binaries). Each binary
links `ds_core` only, uses `REQUIRE` from `tests/support/require.hpp`,
and runs in milliseconds.

### 6.1 `keypad_state_test.cpp` (commits 1 + 2)

- `Keypad_Reset_KeyinputIs03FF`: post-`reset()`, REQUIRE
  `keypad_.read_keyinput() == 0x03FF`.
- `Keypad_Arm9Bus_KeyinputReadAfterReset`: ARM9 bus 16-bit read of
  `0x4000130` returns `0x03FF`.
- `Keypad_Arm7Bus_KeyinputReadAfterReset`: ARM7 bus 16-bit read of
  `0x4000130` returns `0x03FF`.
- `Keypad_SharedState_ArmsSeeSameValueAfterHostChange`: host pushes
  `0x03FA` (B + Start pressed), REQUIRE both ARM9 and ARM7 16-bit
  reads return `0x03FA`.
- `Keypad_KeyinputBits10To15_AlwaysReadZero`: host pushes `0xFFFA`,
  REQUIRE both bus reads return `0x03FA` (masked).
- `Keypad_KeyinputByteReads_HighLowSliceCorrectly`: host pushes
  `0x03FA`, REQUIRE byte-read at `0x4000130` returns `0xFA`,
  byte-read at `0x4000131` returns `0x03`.
- `Keypad_KeyinputWordRead_IncludesKeycntHighHalfword`: ARM7 writes
  KEYCNT to `0xC005`, host pushes KEYINPUT `0x03FA`, REQUIRE ARM7
  32-bit read of `0x4000130` returns `0xC005_03FA`.
- `Keypad_KeyinputWrite_SilentlyDropped`: ARM9 bus writes `0x0000` to
  `0x4000130`, REQUIRE `read_keyinput()` still returns `0x03FF`.

### 6.2 `keypad_keycnt_test.cpp` (commit 3)

- `Keypad_KeycntReset_BothCellsZero`: post-`reset()`, REQUIRE
  `read_keycnt(Arm9) == 0` and `read_keycnt(Arm7) == 0`.
- `Keypad_KeycntArm9Write_DoesNotAffectArm7`: ARM9 bus writes
  `0xC005` to KEYCNT, REQUIRE ARM7 bus read returns `0`.
- `Keypad_KeycntArm7Write_DoesNotAffectArm9`: ARM7 bus writes
  `0x4003` to KEYCNT, REQUIRE ARM9 bus read returns previous value.
- `Keypad_KeycntRoundTrip_AllBitsExceptReserved`: write `0xFFFF`,
  REQUIRE read returns `0xC3FF` (bits 10..13 forced to zero).
- `Keypad_KeycntByteWrites_RmwIntoOwnCell`: write byte `0xC0` to
  `0x4000133` from ARM9, REQUIRE ARM9 KEYCNT reads as `0xC000`; ARM7
  KEYCNT unaffected.
- `Keypad_KeycntWordWrite_BothHalvesApplied_OwnCellOnly`: ARM7 writes
  word `0xDEAD_8005` to `0x4000130`, REQUIRE: low halfword dropped
  (KEYINPUT R/O); high halfword applied to ARM7 KEYCNT → `0x8005 &
  0xC3FF = 0x8005`; ARM9 KEYCNT unaffected.
- `Keypad_KeycntDisabledBit14_AlwaysFalseCondition`: ARM9 writes
  `0x0003` (selects A+B, bit 14 = 0), REQUIRE `prev_condition(Arm9)`
  stays false after `set_keypad_state(0x03FC)`.

### 6.3 `keypad_irq_test.cpp` (commit 4)

Rising-edge IF.12 raise; OR vs AND mode; per-CPU independence:

- `Keypad_OrMode_RisingEdge_RaisesIf12`: ARM7 KEYCNT = `0x4003`,
  KEYINPUT was `0x03FF`, host pushes `0x03FE`. REQUIRE
  `irq7_.read_if() & (1<<12) != 0`.
- `Keypad_OrMode_NoChange_NoRaise`: same setup, host pushes `0x03FE`
  twice in a row. REQUIRE only the first triggers a raise. (Ack via
  IF write-1-clear between calls; assert IF.12 reappears only on
  rising edge, not on idempotent calls.)
- `Keypad_OrMode_FallingEdge_NoRaise`: condition currently true, host
  releases all → `0x03FF`. REQUIRE no raise.
- `Keypad_OrMode_AfterFallAndRise_RaisesAgain`: after falling, press
  again → REQUIRE rising-edge raise.
- `Keypad_AndMode_AllRequired`: ARM9 KEYCNT = `0xC00B` (A+B+Start, AND,
  enabled). Press A only → no raise. Press A+B → no raise. Press
  A+B+Start → REQUIRE `irq9_.read_if() & (1<<12) != 0`.
- `Keypad_AndMode_ReleaseOneFallsThenAllAgainRaises`: from all-three-
  pressed, release Start (condition falls), then re-press → fresh
  rising-edge raise.
- `Keypad_PerCpuIndependence_Arm9RaisesNotArm7`: ARM9 KEYCNT enabled
  for A, ARM7 KEYCNT disabled. Press A → REQUIRE `irq9_.read_if() &
  (1<<12) != 0` AND `irq7_.read_if() & (1<<12) == 0`.
- `Keypad_PerCpuIndependence_Arm7RaisesNotArm9`: symmetric.
- `Keypad_KeycntEnableBitToggleRaises`: keypad already pressed
  matching ARM7 KEYCNT bits 0..9 with bit 14 = 0 → no raise. ARM7
  writes KEYCNT to set bit 14 (no other change) → REQUIRE rising-
  edge raise (condition goes from false to true because gate opened).
- `Keypad_KeycntModeChange_OrToAndDropsCondition`: in OR mode,
  selected = `0x0003`, only A pressed → condition true. Switch to AND
  mode (same KEYCNT bits) → condition becomes false (not all
  selected pressed). REQUIRE no new raise (falling). Then press B:
  condition becomes true → rising-edge raise.
- `Keypad_SelectedZero_InAndMode_NoRaise`: ARM7 KEYCNT = `0xC000`
  (bits 14 + 15 set, no buttons selected, AND mode). REQUIRE no
  raise regardless of input changes.
- `Keypad_RaiseDoesNotReachArm9_WhenOnlyArm7Configured`: as above,
  ARM7 raise fires but `irq9_.read_if() & (1<<12) == 0`.

### 6.4 `extkeyin_test.cpp` (commit 5)

- `Extkeyin_Reset_Is007F`: post-`reset()`, REQUIRE
  `lid_switch_.read_extkeyin() == 0x007F`.
- `Extkeyin_Arm7Bus_HalfwordReadReturnsRegister`: REQUIRE ARM7 16-bit
  read of `0x4000136` returns `0x007F`.
- `Extkeyin_Arm9Bus_OpenBusReturnsZero`: REQUIRE ARM9 reads of
  `0x4000136` (all widths) return `0x0000`.
- `Extkeyin_Arm7ByteReads_HighLowSplit`: REQUIRE byte-read at
  `0x4000136` returns `0x7F`, byte-read at `0x4000137` returns `0x00`.
- `Extkeyin_Bit7AfterClose_ReadsAs0x00FF`: host calls
  `set_lid_closed(true)`, REQUIRE ARM7 16-bit read returns `0x00FF`.
- `Extkeyin_Bit7AfterReopen_ReadsAs0x007F`: host close then open,
  REQUIRE ARM7 16-bit read returns `0x007F`.
- `Extkeyin_Bit6_StaysReleased_NoTouchSupportYet`: host close/open
  cycle, REQUIRE bit 6 (pen) stays 1. (§3 non-goal — locked.)
- `Extkeyin_Bits01345_StayReleased_NoXYDebugYet`: REQUIRE bits 0, 1,
  3 stay 1 across the lid cycle; bits 2, 4, 5 also stay 1.
- `Extkeyin_Arm7BusWrite_SilentlyDropped`: ARM7 bus writes `0x0000`
  to `0x4000136`, REQUIRE `read_extkeyin()` returns `0x007F`.

### 6.5 `lid_irq_test.cpp` (commit 6)

- `Lid_Reset_PrevUnfoldedTrue`: post-`reset()`, REQUIRE
  `lid_switch_.prev_unfolded() == true` and `irq7_.read_if() & (1 <<
  22) == 0`.
- `Lid_OpenAtReset_FirstOpenCallNoRaise`: from reset, host calls
  `set_lid_closed(false)`. REQUIRE no IF.22 raise (lid already open,
  not a rising edge).
- `Lid_Close_NoRaise`: host calls `set_lid_closed(true)`. REQUIRE
  no IF.22 raise and EXTKEYIN.7 = 1.
- `Lid_CloseThenOpen_RaisesIf22OnUnfold`: host close → open. REQUIRE
  `irq7_.read_if() & (1<<22) != 0` after the open. EXTKEYIN.7 = 0.
- `Lid_OpenAgain_NoSecondRaise`: from open state, host calls
  `set_lid_closed(false)` again. REQUIRE no second raise.
- `Lid_IeBit22Clear_StillLatches`: IE = 0; close → open. REQUIRE IF.22
  set; REQUIRE `irq7_.line() == false`.
- `Lid_Arm9_IfBit22_NeverSet`: close → open. REQUIRE `irq9_.read_if()
  & (1<<22) == 0` (NDS7-only source).
- `Lid_RaiseDoesNotAffectIf12`: ARM7 KEYCNT disabled. Close → open.
  REQUIRE `irq7_.read_if() & (1<<12) == 0` (only IF.22 set).
- `Lid_RaiseDoesNotClearOnFold`: after a successful unfold raise,
  closing the lid again does NOT clear IF.22 (only write-1-clear
  does).
- `Lid_AckThenReopen_LatchesAgain`: raise → ack via `write_if(1<<22)`
  → close → open → REQUIRE IF.22 set again.

### 6.6 `keypad_integration_test.cpp` (commit 7)

End-to-end test driving NDS through the full IRQ flow:

- `Keypad_FullBoot_OrModeRaisesAndLine`:
  1. NDS construction. Implicit `reset()` sets KEYINPUT = `0x03FF`,
     both KEYCNTs = `0x0000`, EXTKEYIN = `0x007F`,
     `prev_unfolded_ = true`, `prev_condition_[*] = false`.
  2. ARM7 writes IME=1, IE=`(1<<12) | (1<<22)`. REQUIRE
     `irq7_.line() == false` (no sources pending).
  3. ARM7 writes its KEYCNT to `0x4001` (A selected, enable, OR mode).
  4. Host calls `nds.set_keypad_state(0x03FE)` (A pressed). REQUIRE
     `irq7_.read_if() & (1<<12) != 0`. REQUIRE `cpu7_.irq_line()` ==
     true (or the cached halt-wake bit, depending on CPU state).
  5. ARM7 acks: writes `1<<12` to IF. REQUIRE IF.12 clear.
  6. Host calls `nds.set_lid_closed(true)`. No raise.
  7. Host calls `nds.set_lid_closed(false)`. REQUIRE IF.22 set on
     irq7_; ARM7 line goes high.
  8. ARM7 acks IF.22. REQUIRE IF clear.
  9. REQUIRE `irq9_.read_if() == 0` throughout (only ARM7 KEYCNT was
     configured; IF.22 is ARM7-only).

- `Keypad_FullBoot_Arm9KeycntConfigured_RaisesOnArm9Only`:
  1. NDS construction.
  2. ARM9 writes its KEYCNT to `0xC003` (A+B selected, enable, AND).
  3. Host pushes `0x03FF` → `0x03FC` (A and B pressed). REQUIRE
     `irq9_.read_if() & (1<<12) != 0` and `irq7_.read_if() & (1<<12)
     == 0`.

### 6.7 What stays green from prior slices

All 71 existing CTest binaries must remain green at every commit.
The highest-churn changes are commits 1 and 7 (introduce two new
members on NDS and the new public API). Verify after commit 1 that
`nds_integration_test`, the IPC tests, the RTC tests, and the ARM7
IRQ + halt tests still pass.

---

## 7. Cross-references

- **Project design spec:** `docs/specs/2026-04-12-nds-emulator-design.md`
  §3.5 (interrupt model — confirms IF.12 is both CPUs and IF.22 is
  NDS7-only), §13 Phase 1 deliverables (keypad/lid listed under Phase
  1, slice 3k position).
- **Prior slice specs:**
  - `docs/specs/2026-05-05-rtc-phase1-slice3j-design.md` §1 (handoff
    line proposed slice 3k as keypad + lid), §4.6 (rising-edge raise
    template), §8.1 point 1 (latch-placement risk applies
    identically), Appendix A (commit-sequence template).
  - `docs/specs/2026-04-24-ipc-phase1-slice3i-design.md` §4.5
    (edge-trigger algorithm — same discipline applies here, with
    per-side latches indexed by Side enum).
- **CLAUDE.md:** rule 1 (scheduler is the clock — not applicable to
  input-driven raises; documented as non-scheduler path), rule 3 (no
  inter-subsystem pointers), rule 4 (per-CPU IO tables — KEYCNT is
  per-CPU storage), rule 5 (`reset()` mandatory; save_state deferred),
  rule 6 (no SDL — input injection API is platform-free), rule 7
  (file size caps), rule 8 (no cross-subsystem includes).
- **Existing IRQ scaffolding:**
  - `src/interrupt/irq_controller.hpp` lines 12-44 (the class Keypad
    and LidSwitch will pass by reference into raise paths).
  - `src/nds.cpp:304..313` (`update_arm7_irq_signals` and
    `update_arm9_irq_signals` — both invoked from
    `set_keypad_state`; only the ARM7 variant from `set_lid_closed`).
  - `src/nds.cpp:335..339` (RTC ARM7 word-read pattern — KEYINPUT
    word-read on ARM7 follows the same "low halfword = data, upper
    halfword = next reg" idiom but with `read_keycnt` filling the
    upper halfword on KEYINPUT reads).
- **Existing bus pattern:**
  - `src/nds.cpp:335, 370, 405, 443, 503, 582` (RTC ARM7 routes
    across 8 / 16 / 32 read+write) — keypad ARM7 routes append to the
    same six methods.
  - `src/nds.cpp:89..162, 163..295` (ARM9 IO read/write all widths) —
    keypad ARM9 routes append KEYINPUT (R) and KEYCNT (R/W) here.
- **Existing per-side state pattern:**
  - `src/ipc/ipc_sync.{hpp,cpp}` (Side enum + per-side state).
  - `src/ipc/ipc_fifo.{hpp,cpp}` (per-side rising-edge latch + raise
    by reference).
- **GBATEK references** (fetched 2026-05-16):
  - `https://problemkaputt.de/gbatek.htm#gbakeypadinput` (KEYINPUT
    bit layout; shared between GBA and DS).
  - `https://problemkaputt.de/gbatek.htm#dskeypadlidandhingestate`
    (DS-specific EXTKEYIN + hinge; KEYCNT per-CPU storage note).
  - `https://problemkaputt.de/gbatek.htm#dsinterrupts` (IF.12 both
    CPUs; IF.22 NDS7-only with "no source-side enable" comment).
- **External cross-check:** melonDS source `src/Keypad.cpp` (fetched
  2026-05-16) for `selected == 0` resolution in AND mode. Used as a
  reference, not copied.

---

## 8. Risk and rollback

### 8.1 Highest-risk pitfalls

1. **Per-side rising-edge latch desync.** `prev_condition_[Arm9]` and
   `prev_condition_[Arm7]` are independent. If `recompute_irqs`
   updates one cell but not the other (e.g. an `if (side == Arm9)`
   branch that forgets the other side after a `write_keycnt`), the
   other side's latch becomes stale and either raises spuriously or
   misses a rising edge forever. Mitigation: `recompute_irqs` always
   computes both sides unconditionally, regardless of which side
   triggered the call. Tested by `Keypad_PerCpuIndependence_*` pair.
2. **Forgetting to call `recompute_irqs` on one of the four
   callsites.** A correctly-implemented `recompute_irqs` is useless
   if `set_keyinput` or `write_keycnt(Side, …)` forgets to call it.
   Mitigation: the four callsites are all inside `Keypad`, and the
   API forces every external caller to provide both `IrqController&`
   references — making it visually obvious in the call signature that
   a raise can happen. Tested by `Keypad_KeycntEnableBitToggleRaises`
   (KEYCNT change without KEYINPUT change must still raise on rising
   edge).
3. **Active-low confusion.** KEYINPUT bit = 0 means *pressed*. The
   `pressed_mask = ~keyinput_ & kButtonMask` line is the only place
   the inversion happens. If anyone reads the code months from now
   and adds a "fix" turning `~` into a positive mask, every keypad
   test fails immediately. Mitigation: §4.5 pseudocode is the
   reference; `Keypad_OrMode_RisingEdge_RaisesIf12` test asserts
   pressed → raise unambiguously.
4. **EXTKEYIN.7 polarity confusion.** The bit is 0 when open, 1 when
   closed. The IRQ fires on "unfolded" = 1→0, not 0→1. The
   `new_unfolded = (new_bit7 == 0)` line is the inversion. Same
   mitigation as #3 — pseudocode + named `Lid_CloseThenOpen_Raises*`
   test.
5. **`prev_unfolded_` initialization.** Must be `true` after reset
   (lid is open at boot, "unfolded" is true). Otherwise the first
   `set_lid_closed(false)` looks like a spurious rising edge.
   Mitigation: §4.2 documents the init explicitly; `Lid_Reset_
   PrevUnfoldedTrue` test asserts it.
6. **KEYCNT reserved bits leaking through.** Bits 10..13 are
   reserved. Per GBATEK they "read as zero". If the cell stores
   them, a round-trip read of `0xFFFF` returns `0xFFFF`, not
   `0xC3FF`. Mitigation: `write_keycnt` masks input to `0xC3FF`
   before storing; `Keypad_KeycntRoundTrip_AllBitsExceptReserved`
   test asserts the mask.
7. **ARM9 EXTKEYIN access wasn't actually open-bus.** If a future
   refactor adds an ARM9 fall-through case that catches `0x4000136`,
   the open-bus property breaks. Mitigation: §4.0 Q5 documents the
   approach (no ARM9 route; rely on default fall-through); test
   `Extkeyin_Arm9Bus_OpenBusReturnsZero` is the canary.
8. **Word-read aliasing across the KEYCNT/EXTKEYIN gap.** Address
   `0x4000134` is unmapped; `0x4000136` is EXTKEYIN. A naive 32-bit
   read at `0x4000132` should NOT bleed EXTKEYIN into its upper half
   (different 32-bit word). Mitigation: §4.4 documents the gap;
   tests covering 32-bit reads at both addresses assert exact byte
   layouts.
9. **Save-state debt.** Both rising-edge latches must be in the
   serialized state when the save-state pass lands. If only the
   storage cells are serialized and the latches aren't, a load-
   state could replay a spurious or missed rising edge. Tracked as
   debt; flagged here so the save-state slice picks it up.

### 8.2 Rollback strategy

Each commit is independently revertable:

- Commit 7 (integration test) reverts cleanly without touching
  production code.
- Commit 6 (lid IRQ raise) reverts cleanly without touching the
  keypad path. Reverts to "lid state stored but no IF.22 raise".
- Commit 5 (EXTKEYIN scaffold) reverts cleanly without touching
  KEYINPUT / KEYCNT. Reverts to "no lid_switch_ member; lid IRQ not
  possible".
- Commit 4 (keypad IRQ raise) reverts cleanly without touching the
  KEYCNT storage path. Reverts to "KEYINPUT and KEYCNT work but no
  IF.12 raise".
- Commit 3 (per-CPU KEYCNT) reverts cleanly without touching
  KEYINPUT. Reverts to "KEYINPUT read-only, KEYCNT not yet routed".
- Commit 2 (KEYINPUT read path) reverts cleanly without touching
  commit 1. Reverts to "scaffold only".
- Commit 1 (scaffold + bus routes that return zero) is the lowest-
  risk single-file group. Reverting it requires reverting commits
  2-7 first.

### 8.3 What this slice does NOT break

Confirmed unchanged behaviors after each commit:
- ARM7 / ARM9 instruction execution. Keypad / lid raises are
  additive into existing `irq{9,7}_.raise()` paths.
- RTC, IPC, IRQ controller registers — untouched.
- Scheduler events — untouched. No new EventKind enumerator.
- All 71 existing CTest binaries must remain green at every commit.

---

## 9. Slice completion criteria

- [x] `class Keypad` exists in `src/input/keypad.{hpp,cpp}` with the
      public surface declared in §4.2 and `reset()` implemented.
      (`keypad.hpp` 52 lines, `keypad.cpp` 82 lines. Commit `44662f2`
      scaffold; condition + raise across `91df15a..6c75fc9`.)
- [x] `class LidSwitch` exists in `src/input/lid_switch.{hpp,cpp}`
      with the public surface declared in §4.2 and `reset()`
      implemented. (`lid_switch.hpp` 34 lines, `lid_switch.cpp` 34
      lines. Commit `44662f2` scaffold; raise in `ba083ca`.)
- [x] `IO_KEYINPUT`, `IO_KEYCNT`, `IO_EXTKEYIN` constants in
      `src/bus/io_regs.hpp`. Exactly three new definitions; no other
      constant changed. Verified at `src/bus/io_regs.hpp:36-38`.
      (Commit `44662f2`.)
- [x] `Keypad keypad_` member on `NDS`. Accessor `keypad()`. Include
      `input/keypad.hpp` in `nds.hpp`. Verified at `src/nds.hpp:140`
      (member), `:63` (accessor), `:9` (include). (Commit `44662f2`.)
- [x] `LidSwitch lid_switch_` member on `NDS`. Accessor `lid_switch()`.
      Include `input/lid_switch.hpp` in `nds.hpp`. Verified at
      `src/nds.hpp:141` (member), `:66` (accessor), `:10` (include).
      (Commit `44662f2`.)
- [x] ARM9 IO routes for `0x4000130` (R / W) and `0x4000132` (R / W)
      wired in `arm9_io_read{8,16,32}` and `arm9_io_write{8,16,32}`
      per §4.4. Verified at `src/nds.cpp:107-112, 140-145, 174-181`
      (read 32/16/8) and `:214-225, 273-275, 339-345` (write 32/16/8).
      (Commits `44662f2`, `91df15a`.)
- [x] ARM7 IO routes for `0x4000130` (R), `0x4000132` (R / W), and
      `0x4000136` (R) wired in `arm7_io_read{8,16,32}` and
      `arm7_io_write{8,16,32}` per §4.4. KEYINPUT and EXTKEYIN writes
      are silently dropped. Verified at `src/nds.cpp:404-411, 447-455,
      489-501` (read 32/16/8) and `:542-552, 617-619, 700-706` (write
      32/16/8). (Commits `44662f2`, `91df15a`, `e01fa44`.)
- [x] ARM9 access to `0x4000136` returns zero via the default fall-
      through (no route added). Documented and tested in
      `extkeyin_test.cpp`. (Commit `e01fa44`.)
- [x] `NDS::reset()` calls `keypad_.reset()` and `lid_switch_.reset()`.
      Verified at `src/nds.cpp:45-46`. (Commit `44662f2`.)
- [x] `NDS::set_keypad_state(u16)` body forwards to `keypad_.
      set_keyinput(...)` and calls both `update_arm9_irq_signals()`
      and `update_arm7_irq_signals()`. Verified at `src/nds.cpp:371-375`.
      (Commit `44662f2`.)
- [x] `NDS::set_lid_closed(bool)` body forwards to `lid_switch_.
      set_closed(...)` and calls `update_arm7_irq_signals()` only.
      Verified at `src/nds.cpp:377-380`. (Commit `44662f2`.)
- [x] `Keypad::compute_condition` and `recompute_irqs` per §4.5;
      rising-edge latch indexed by `Side`. (Commit `6c75fc9`.)
- [x] `LidSwitch::set_closed` rising-edge latch per §4.6;
      `prev_unfolded_` initialized to `true` on reset. (Commit `ba083ca`.)
- [x] Six new test binaries: `keypad_state_test`,
      `keypad_keycnt_test`, `keypad_irq_test`, `extkeyin_test`,
      `lid_irq_test`, `keypad_integration_test`. All registered via
      `add_ds_unit_test()` in `tests/CMakeLists.txt` (CTest indices
      72..77).
- [x] `ctest --output-on-failure` reports 77/77 passing in Debug.
- [x] `clang-format` clean on all new files (enforced by the
      `PostToolUse` hook).
- [x] `ds-architecture-rule-checker` and `gbatek-reviewer` ran clean
      on every commit's uncommitted diff.
- [x] `quality-reviewer` ran clean on every commit.
- [x] No file exceeds the 500-line soft cap. (Max new production file
      `keypad.cpp` 82 lines; max new test `keypad_irq_test.cpp` 226
      lines.)
- [x] No new SDL include path. No new platform headers in
      `libds_core`.
- [x] No new pointers held across subsystems. No `cpu/` or `bus/`
      includes from `input/`.
- [x] No save-state code added (deferred per CLAUDE.md rule-5
      carve-out). `reset()` implemented on both new classes; rising-
      edge latches tracked as save-state debt in §8.1 #9.

---

## Appendix A. Commit sequence

Seven commits, shippable individually. Each ends with `ctest` green.

### Commit 1 — `input: scaffold Keypad + LidSwitch + bus routes returning reset values`

- Create `src/input/keypad.{hpp,cpp}` with the public surface from
  §4.2. `reset()` zeroes both KEYCNT cells, sets `keyinput_ = 0x03FF`,
  and clears both `prev_condition_` latches. `set_keyinput` and
  `write_keycnt` accept the `IrqController&` args but the raise body
  is empty (rising-edge logic lands in commit 4).
- Create `src/input/lid_switch.{hpp,cpp}` with the public surface
  from §4.2. `reset()` sets `extkeyin_ = 0x007F` and `prev_unfolded_
  = true`. `set_closed` body updates `extkeyin_` and the latch but
  does not raise (lands in commit 6).
- `src/CMakeLists.txt`: add `input/keypad.cpp` and `input/
  lid_switch.cpp` to the `ds_core` target.
- `src/bus/io_regs.hpp`: add `IO_KEYINPUT`, `IO_KEYCNT`,
  `IO_EXTKEYIN`.
- `src/nds.hpp`: include both new headers, add `Keypad keypad_` and
  `LidSwitch lid_switch_` members, accessor methods,
  `set_keypad_state` and `set_lid_closed` declarations.
- `src/nds.cpp`: `reset()` calls both new `.reset()`s.
  `set_keypad_state` / `set_lid_closed` bodies forward to the
  subsystems. ARM9 + ARM7 bus routes for the three addresses per
  §4.4 (read paths only this commit — they return the reset values
  since nothing has been written yet; write paths land in commit 3
  for KEYCNT).
- No new tests this commit — the read paths are trivially exercised
  in commit 2's tests. Rationale: keep the scaffold commit small.

**Diff size estimate:** ~220 lines added across 7 files. CTest count
71 → 71 (no new tests).

### Commit 2 — `input: KEYINPUT shared-state read path + tests`

- Implement `Keypad::set_keyinput` body (updates `keyinput_`; raise
  body still empty pending commit 4).
- Add tests in `keypad_state_test.cpp` per §6.1 covering: reset
  value, ARM9 + ARM7 bus reads return same value, bits 10..15 mask
  to zero, byte reads slice high/low, word read combines KEYCNT
  high with KEYINPUT low, writes silently dropped.
- `tests/CMakeLists.txt`: `add_ds_unit_test(keypad_state_test)`.

**Diff size estimate:** ~30 lines in `keypad.cpp`, ~150 lines new
test. CTest count 71 → 72.

### Commit 3 — `input: KEYCNT per-CPU storage + R/W path + tests`

- Implement `Keypad::read_keycnt(Side)` and `Keypad::
  write_keycnt(Side, value, irq9, irq7)`. The write masks input to
  `0xC3FF` (clear reserved bits 10..13). `recompute_irqs` is called
  but the raise body inside it is still empty (lands in commit 4).
- ARM9 + ARM7 bus write routes for `0x4000132` per §4.4 (all
  widths).
- New test binary `keypad_keycnt_test.cpp` per §6.2.
- `tests/CMakeLists.txt`: `add_ds_unit_test(keypad_keycnt_test)`.

**Diff size estimate:** ~70 lines in `keypad.cpp`, ~50 lines in
`nds.cpp`, ~190 lines new test. CTest count 72 → 73.

### Commit 4 — `input: keypad condition match + IF.12 raise on rising edge`

- Implement `Keypad::compute_condition(Side)` and the rising-edge
  raise inside `recompute_irqs` per §4.5.
- No bus-routing changes (the routes already call `recompute_irqs`
  through `set_keyinput` / `write_keycnt`).
- New test binary `keypad_irq_test.cpp` per §6.3.
- `tests/CMakeLists.txt`: `add_ds_unit_test(keypad_irq_test)`.

**Diff size estimate:** ~50 lines in `keypad.cpp`, ~230 lines new
test. CTest count 73 → 74.

### Commit 5 — `input: EXTKEYIN read path + ARM7-only routing + tests`

- ARM7 bus read routes for `0x4000136` per §4.4 (all widths).
- ARM9 access to `0x4000136` validated to fall through to zero
  (default behavior already; documented).
- `LidSwitch::read_extkeyin` body returns `extkeyin_`.
- New test binary `extkeyin_test.cpp` per §6.4.
- `tests/CMakeLists.txt`: `add_ds_unit_test(extkeyin_test)`.

**Diff size estimate:** ~20 lines in `lid_switch.cpp`, ~30 lines in
`nds.cpp`, ~150 lines new test. CTest count 74 → 75.

### Commit 6 — `input: lid IRQ on rising-edge of "unfolded" + tests`

- Implement the rising-edge raise inside `LidSwitch::set_closed`
  per §4.6.
- New test binary `lid_irq_test.cpp` per §6.5.
- `tests/CMakeLists.txt`: `add_ds_unit_test(lid_irq_test)`.

**Diff size estimate:** ~20 lines in `lid_switch.cpp`, ~140 lines
new test. CTest count 75 → 76.

### Commit 7 — `input: end-to-end keypad + lid integration test`

- No production code changes (or only any small fixes that surface
  during writing the integration test, in which case fold them back
  into commits 2-6 and reorder).
- New test binary `keypad_integration_test.cpp` per §6.6.
- `tests/CMakeLists.txt`: `add_ds_unit_test(keypad_integration_test)`.
- After this commit, `ctest` reports 77/77 passing.

**Diff size estimate:** ~150 lines new test, no production change.
CTest count 76 → 77.

---

## Appendix B. Provenance audit

- KEYINPUT bit layout in §5.1: verbatim from GBATEK fetched 2026-05-16
  from `https://problemkaputt.de/gbatek.htm#gbakeypadinput`. No
  interpretation, no rephrasing beyond inlining the "active-low"
  emphasis already present in GBATEK's prose.
- KEYCNT bit layout in §5.2 + per-CPU storage note: GBATEK same URL,
  fetched 2026-05-16. The "TWO independent cells, one per CPU"
  encoding is GBATEK's own footnote that KEYCNT appears in both NDS9
  and NDS7 I/O maps at the same address.
- EXTKEYIN bit layout in §5.3 + NDS7-only / open-bus note: GBATEK
  fetched 2026-05-16 from
  `https://problemkaputt.de/gbatek.htm#dskeypadlidandhingestate`.
- IF.12 / IF.22 bit map in §5.4: GBATEK fetched 2026-05-16 from
  `https://problemkaputt.de/gbatek.htm#dsinterrupts`. Filtered to
  bits 12 and 22 only; full table in upstream.
- IF.22 "no source-side enable" comment in §5.4: GBATEK quoted
  verbatim ("The hinge generates an interrupt request (there seems
  to be no way to disable this, unlike as for all other IRQ sources),
  however, the interrupt execution can be disabled in IE register.").
- `selected == 0` resolution in §5.8.1: GBATEK is silent; resolution
  chosen from melonDS source (`src/Keypad.cpp`, fetched 2026-05-16).
  Used as a reference, not copied. Decision and alternative
  documented in §5.8.1.
- Rising-edge raise on bit-14 toggle in §5.8.4: extension of slice
  3i's IF.17/IF.18 edge-trigger discipline. Documented in §8.1 #2 as
  the highest-risk callsite for missed raises.
- No BIOS dump, no firmware dump, no proprietary data, no
  copyrighted code referenced by this design.
