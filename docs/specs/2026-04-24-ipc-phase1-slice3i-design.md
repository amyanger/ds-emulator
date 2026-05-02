# IPC + ARM9 IRQ Plumbing — Phase 1, Slice 3i Design

**Date:** 2026-04-24
**Slice:** IPC SYNC + IPC FIFO subsystem, plus ARM9-side IRQ controller storage
and IO routing (no ARM9 line sampling — deferred to the ARM9 decoder slice).
**Status:** draft, awaiting approval
**Prior slice:** 3h (ARM7 BIOS sound table-lookup SWIs) — landed through
`5ed0ca4`; 60 CTest binaries green.
**Next slice (proposed):** 3j (ARM7 RTC + scheduler-driven IRQ sources, e.g.
keypad / RTC / SIO) — keeps the IRQ infrastructure exercised by real raises
before ARM9 work begins. Still to be scoped.

---

## 1. Summary

The ARM7 IRQ controller has existed since slice 3d: `class Arm7IrqController`
in `src/interrupt/irq_controller.hpp` (IME/IE/IF + line/halt-wake), wired
through `NDS::update_arm7_irq_signals()` from twelve callsites in `nds.cpp`.
The ARM7 already samples `irq_line` at instruction boundaries and enters the
IRQ vector via `arm7_enter_irq`. **Nothing currently calls `raise()`** — the
ARM7 BIOS HLE handlers (`IntrWait`, `Halt`, `Sleep`) sit in front of an IRQ
system that has no sources yet.

Slice 3i builds the **first IRQ source**: the IPC subsystem. Both CPUs see
four shared registers — IPCSYNC (`0x4000180`), IPCFIFOCNT (`0x4000184`),
IPCFIFOSEND (`0x4000188`, write-only), IPCFIFORECV (`0x4100000`, read-only)
— and writes from one CPU must surface as IRQs on the other CPU's IF
(`bit 16` for SYNC, `bit 17`/`bit 18` for FIFO send-empty / recv-not-empty,
both edge-triggered).

To raise IRQs on **either** side, this slice also introduces an ARM9-side
`IrqController` (renamed from `Arm7IrqController`, instantiated twice) and
routes IME/IE/IF on the ARM9 bus. ARM9 *line sampling* — the `set_irq_line`
hookup that lets the CPU enter the vector — is deferred. The ARM9 stub
cannot execute instructions, so an IRQ line going into it would have nowhere
to land. Storage and bus routing exist today so that ARM9-side firmware
writes during direct-boot (which happen on the very first frame) land in
real state instead of being silently dropped, and so that the day the ARM9
decoder lights up, the only missing wire is the call into the CPU.

### Plain-language summary

Two CPUs that share memory need a way to wake each other up — "I just
wrote a thing for you, look at it." The DS gives them three ways to do
this through one shared address range:

1. **IPCSYNC** — a 16-bit register that's literally a 4-bit pipe in each
   direction (each CPU writes 4 bits into the other CPU's view) plus a
   one-shot "ping the other CPU" bit. The remote CPU has to opt in by
   setting an enable bit; without that, the ping is silent.

2. **IPC FIFO** — two 16-deep / 64-byte ring buffers, one per direction,
   with hardware that signals on edges: "the FIFO you're sending into just
   went empty" (so you can refill) and "the FIFO you're receiving from
   just got its first word" (so you can read). Both edges go through a
   per-side enable bit.

3. **The IRQ controller** — IME (master enable), IE (per-source enable),
   IF (per-source pending). Writing 1 to a bit in IF *clears* it
   (acknowledge); the controller asserts a level-sensitive line that the
   CPU samples at instruction boundaries. We already have it on ARM7. We're
   adding the same thing on ARM9 even though ARM9 isn't running
   instructions yet — the registers need to be writable before firmware
   tries to write them.

The architectural decisions in this slice are deliberately small: the
existing `Arm7IrqController` is already CPU-agnostic in implementation
(only its name says ARM7), so the rename to `IrqController` is mechanical
and lets ARM9 instantiate the same class. The new IPC code lives in
`src/ipc/`, gets to NDS through method calls (rule 3), and never reaches
into CPU or interrupt code beyond passing `IrqController&` arguments.

The hard parts — the parts where bugs hide — are not the data structure.
They are:

- **Edge-triggered IRQ semantics.** GBATEK is explicit: IF.17 fires only
  when `(IPCFIFOCNT.2 AND IPCFIFOCNT.0)` transitions 0→1, and IF.18 only
  when `(IPCFIFOCNT.10 AND NOT IPCFIFOCNT.8)` transitions 0→1. Latching the
  previous gated value and comparing has to happen *after* every state
  change with the right ordering, or rising edges will be lost.
- **Disabled-master-enable quirks.** With `CNT.15 = 0`: writes to SEND are
  silently dropped (no error bit set, unlike the full case), and reads from
  RECV peek `last_word` without removing data from the FIFO. Easy to
  implement as the same path as full / empty and quietly get it wrong.
- **The `last_word` semantics.** Empty-RECV returns the most recent word
  popped, *or* zero if the FIFO was cleared via CNT.3 *or* if no word was
  ever received. Three distinct sources, easy to conflate.

**What this slice builds:**

- `class IrqController` (renamed from `Arm7IrqController` — body unchanged).
- `class IpcSync` in `src/ipc/ipc_sync.{hpp,cpp}`.
- `class IpcFifo` in `src/ipc/ipc_fifo.{hpp,cpp}`.
- `IrqController irq9_` on `NDS`, in addition to the existing ARM7 one.
- `NDS::update_arm9_irq_signals()` stub that recomputes line-state but does
  not push into the CPU stub.
- ARM9 bus IO routes for IME/IE/IF and the four IPC addresses. ARM7 bus
  routes for the four IPC addresses (existing IME/IE/IF routes unchanged).
- Six new constants in `src/bus/io_regs.hpp`.
- Four new test binaries: `arm9_irq_io_test`, `ipc_sync_test`,
  `ipc_fifo_test`, `ipc_fifo_irq_test`, plus an integration test
  `ipc_integration_test`.

**What this slice deliberately does NOT build:**

- ARM9 IRQ *line sampling*. ARM9 is still a `run_until` stub with no
  decoder; pushing a line into it would push into nothing. The ARM9 decoder
  slice connects `update_arm9_irq_signals()` to `cpu9_.set_irq_line(...)`.
- Any IRQ source other than IPC. No VBlank, HBlank, VCount, timers, DMA,
  keypad, RTC, NDS-cart transfers, SIO, GXFIFO, etc. Those land slice by
  slice as their owning subsystems do.
- Scheduler events for IPC. All IPC writes are synchronous; the
  instruction-boundary IRQ sampling already in place on ARM7 picks up the
  raise at the next instruction. GBATEK explicitly notes that IPCSYNC is
  no-waitstate, and FIFO drain timing is not modeled at this level of
  fidelity (revisit only if a Pokemon-game divergence demands it).
- Sub-32-bit access to IPCFIFOSEND / IPCFIFORECV. GBATEK is silent on
  hardware behavior and no correctly-written DS code does this. Routed as
  drop-with-DEBUG-warn for SEND, return-zero-with-DEBUG-warn for RECV.
- Save state serialization for the new classes. Per the CLAUDE.md rule-5
  carve-out, `reset()` only.

### Scope boundary

**In scope:** the rename, the two new IPC classes, the ARM9 IRQ controller
storage, IO routes for IME/IE/IF on ARM9 and the four IPC addresses on
both buses, edge-triggered raises into the per-side `IrqController`, four
new test binaries plus an end-to-end integration test, six new I/O
register constants.

**Out of scope:** see §3.

---

## 2. Goals

1. **IPCSYNC R/W behaves per GBATEK on both CPU views.** ARM9 reads see
   ARM7's bits 8-11 in its own bits 0-3 and vice versa; bit 13 is W-only
   one-shot that raises IF.16 on the remote IF the remote's bit 14 is set;
   bit 14 is R/W persisted; bits 12, 15-31 are zero / not used.
2. **IPCFIFO R/W behaves per GBATEK on both CPU views.** SEND/RECV are 32-bit
   crossed (ARM9's send is ARM7's recv), max 16 words / 64 bytes. CNT
   reflects per-side empty/full/error/master-enable/IRQ-enable correctly.
   Master-enable-off semantics for SEND (silently dropped, no error) and
   RECV (peek `last_word` without consuming) match GBATEK exactly. Empty
   RECV returns `last_word` and sets error; CNT.3 clear zeroes data and
   `last_word`; CNT.14 ack clears error.
3. **IF.16 / IF.17 / IF.18 raise correctly with edge semantics.** IF.16 is a
   one-shot from IPCSYNC.13. IF.17 raises only on the rising edge of
   `(CNT.2 AND CNT.0)` per side. IF.18 raises only on the rising edge of
   `(CNT.10 AND NOT CNT.8)` per side. Falling edges raise nothing. Bits
   stay raised until acknowledged via IF write-1-clear. Toggling the IRQ
   enable bit on while the gated condition is already true *does* raise
   (rising edge of the gated value).
4. **ARM9-side IO writes for IME/IE/IF land in real state.** Direct-boot's
   ARM9 firmware will write these immediately; today they would silently
   drop. After this slice they round-trip correctly and `update_arm9_irq_signals()`
   recomputes a stored line bool for X-Ray observability. CPU sampling is
   deferred to the ARM9 decoder slice.
5. **Six commits, each shippable individually with its test.** Same
   discipline as 3g / 3h. CTest count 60 → 64 (four new test binaries +
   one integration test, with one of them appended to an existing file —
   see Appendix A).
6. **No new architectural debt.** `IrqController` is one CPU-agnostic class
   instantiated twice; no duplication. No subsystem holds a pointer to
   another (rule 3); no `cpu/` or `bus/` includes from `ipc/` (rule 8).
   File layout matches the planned `src/ipc/` structure in CLAUDE.md.
7. **Tests link `ds_core` only, use `REQUIRE`, run in milliseconds.** Same
   discipline as every prior slice.
8. **All new files stay well under the 500-line soft cap.** Estimated
   sizes: `ipc_sync.{hpp,cpp}` ~50 / ~80 lines, `ipc_fifo.{hpp,cpp}` ~120
   / ~250 lines.

### 2.1 IRQ source coverage matrix

| Bit | Source                       | This slice? | Notes                                |
|-----|------------------------------|-------------|--------------------------------------|
| 16  | IPC Sync                     | **YES**     | Raised by remote IPCSYNC.13 W.       |
| 17  | IPC Send FIFO Empty          | **YES**     | Edge-triggered; rising of gated cond.|
| 18  | IPC Recv FIFO Not Empty      | **YES**     | Edge-triggered; rising of gated cond.|
| 0-15, 19-31 | (everything else)    | NO          | Future slices.                       |

**3 IRQ sources go from "no implementation" to "fully wired on both sides"
+ 4 new test binaries + 1 integration test.** Total commits = 6 (see
Appendix A). CTest count 60 → 64.

---

## 3. Non-goals

- **ARM9 IRQ line sampling.** No `Arm9::set_irq_line()`, no IRQ vector
  entry, no `arm9_enter_irq` helper. The ARM9 stub doesn't execute
  instructions; pushing a line into it is meaningless until the decoder
  lands. `update_arm9_irq_signals()` recomputes a stored bool for X-Ray
  but does not call into the CPU. Documented as deliberate stub.
- **Any other IRQ source.** No VBlank, HBlank, VCount (PPU slice), timers
  0-3 (timer slice), DMA 0-3 (DMA slice), keypad (input slice), GBA cart
  (out of scope entirely), NDS-Slot game card transfers (cart slice),
  IPC SYNC bit-13 raised by IF.16 *to ARM7* is in scope; raised *to ARM9*
  is in scope as state, just unwired to a CPU. SIO/RCNT/RTC IRQ (bit 7,
  ARM7 only), screens-unfolded (bit 22), SPI (bit 23), wifi (bit 24),
  GXFIFO (bit 21, ARM9-only) — all future slices.
- **Scheduler events for IPC.** All IPC writes propagate synchronously
  inside the bus write call. No `EventKind::IpcDrain` or similar. The
  instruction-boundary IRQ sampling already in place on ARM7 picks up
  raises at the next instruction. Real-hardware FIFO timing is documented
  by GBATEK as no-waitstate for SYNC and unspecified-but-fast for FIFO;
  no Pokemon-game pattern is sensitive to single-digit-cycle FIFO latency
  in our level of fidelity.
- **Sub-32-bit access to IPCFIFOSEND / IPCFIFORECV.** Routed as drop-with-
  DEBUG-warn for SEND, return-zero-with-DEBUG-warn for RECV. GBATEK is
  silent and no correctly-written DS code does this; if a real-game
  divergence ever surfaces, revisit then.
- **Save state serialization.** Per CLAUDE.md rule-5 carve-out, `reset()`
  only this slice.
- **8-bit access to IPCSYNC / IPCFIFOCNT bits in the upper-reserved
  region.** Treated as no-op (matches the existing IME/IE/IF 8-bit handler
  pattern in `nds.cpp:213..253`).
- **DSi quirks.** No DSi7 IE2/IF2, no DSi-mode SYNC behaviors. Out of
  emulator scope entirely (per the project's NDS-only mandate).
- **Refactoring `update_arm7_irq_signals()` into a single
  `update_irq_signals(side)` template.** Mechanical and small, but
  requires one of the two sides to actually push into a CPU before the
  symmetry is real. Deferred to the ARM9 decoder slice when the symmetry
  becomes meaningful.

---

## 4. Architecture

### 4.1 File layout

```
src/interrupt/
  irq_controller.hpp                MODIFIED. Rename Arm7IrqController to
                                    IrqController. Body unchanged. Optional
                                    `using Arm7IrqController = IrqController`
                                    alias for one commit, then removed.

src/ipc/                            NEW directory.
  ipc_sync.hpp                      NEW. ~50 lines. class IpcSync declaration:
                                      enum class Side { Arm9, Arm7 };
                                      void reset();
                                      u16 read(Side) const;
                                      bool write(Side, u16, IrqController&
                                                 remote_irq);
                                    Internal: SideState { u8 out_lo;
                                                          bool irq_enable; }
                                              arm9_, arm7_;
  ipc_sync.cpp                      NEW. ~80 lines. Implements read/write/reset.
                                    Includes `interrupt/irq_controller.hpp`
                                    only — no cpu/ or bus/ includes.
  ipc_fifo.hpp                      NEW. ~120 lines. class IpcFifo declaration:
                                      enum class Side { Arm9, Arm7 };
                                      void reset();
                                      u16 read_cnt(Side) const;
                                      void write_cnt(Side, u16,
                                                     IrqController& local_irq,
                                                     IrqController& remote_irq);
                                      void write_send(Side, u32, …);
                                      u32 read_recv(Side, …);
                                      bool send_empty/full/recv_empty/full(Side);
                                      u8 fill_level(Side send_side) const;
                                    Private: Direction { array<u32,16> data;
                                                         u8 head, tail, count;
                                                         u32 last_word;
                                                         bool ever_received; }
                                                a2b_, b2a_;
                                            SideCnt { bool send_empty_irq;
                                                      bool recv_not_empty_irq;
                                                      bool error;
                                                      bool master_enable;
                                                      bool prev_send_empty_gated;
                                                      bool prev_recv_not_empty_gated; }
                                                arm9_cnt_, arm7_cnt_;
                                            send_dir/recv_dir/cnt accessors.
                                            recompute_irqs(IrqController&,
                                                           IrqController&).
  ipc_fifo.cpp                      NEW. ~250 lines. Implements push/pop,
                                    edge-trigger recompute, all CNT decode
                                    rules, master-enable quirks. Same include
                                    rule as ipc_sync.cpp.

src/bus/
  io_regs.hpp                       MODIFIED. Add:
                                      constexpr u32 IO_IPCSYNC      = 0x04000180;
                                      constexpr u32 IO_IPCFIFOCNT   = 0x04000184;
                                      constexpr u32 IO_IPCFIFOSEND  = 0x04000188;
                                      constexpr u32 IO_IPCFIFORECV  = 0x04100000;

src/
  nds.hpp                           MODIFIED. Add:
                                      IrqController irq9_{};   // new
                                      IpcSync ipc_sync_{};
                                      IpcFifo ipc_fifo_{};
                                      void update_arm9_irq_signals();
                                      Accessors: irq9(), ipc_sync(), ipc_fifo()
                                    Rename existing `irq7_ctrl_` member to
                                    `irq7_` (matches the rename of the class).
  nds.cpp                           MODIFIED. ~+120 lines:
                                    - Reset adds new members.
                                    - update_arm9_irq_signals() body (see §4.3).
                                    - arm9_io_read/write{8,16,32}: route IO_IME,
                                      IO_IE, IO_IF, IO_IPCSYNC, IO_IPCFIFOCNT,
                                      IO_IPCFIFOSEND, IO_IPCFIFORECV.
                                    - arm7_io_read/write{8,16,32}: append IPC
                                      register routes (existing IME/IE/IF
                                      routes unchanged).

tests/
  arm9_irq_io_test.cpp              NEW. ARM9 IME/IE/IF round-trip.
  ipc_sync_test.cpp                 NEW. IPCSYNC R/W + bit-13 raise gating.
  ipc_fifo_test.cpp                 NEW. FIFO push/pop, full/empty/error,
                                    master-enable quirks, CNT.3 clear,
                                    CNT.14 ack.
  ipc_fifo_irq_test.cpp             NEW. Edge-trigger semantics for IF.17/18.
  ipc_integration_test.cpp          NEW. End-to-end ARM9↔ARM7 round-trip via
                                    bus IO.
```

### 4.2 Class designs

#### `IrqController` (renamed from `Arm7IrqController`, body unchanged)

```cpp
namespace ds {
class IrqController {
public:
    void reset();
    void write_ime(u32 value);                  // bit 0 only
    void write_ie(u32 value);
    void write_if(u32 value);                   // write-1-clear
    void raise(u32 source_bits);                // OR into IF
    u32 read_ime() const;
    u32 read_ie() const;
    u32 read_if() const;
    bool line() const;                          // (IME&1) && (IE & IF)
    bool halt_wake_pending() const;             // (IE & IF)
private:
    u32 ime_ = 0, ie_ = 0, if_ = 0;
};
} // namespace ds
```

The 12 callsites in `nds.cpp` and the existing tests (`arm7_halt_test`,
`arm7_bios_intrwait_test`, `arm7_exception_sequence_test`) update via a
mechanical rename. Optional one-commit `using Arm7IrqController =
IrqController` alias to keep tests building during the transition.

#### `IpcSync`

```cpp
namespace ds {
class IrqController;                            // forward decl

class IpcSync {
public:
    enum class Side { Arm9, Arm7 };
    void reset();
    // 16-bit read: bits 0-3 = remote.out_lo; bits 8-11 = local.out_lo;
    // bit 14 = local.irq_enable; all other bits = 0.
    u16 read(Side side) const;
    // 16-bit write: stores bits 8-11 + bit 14 of `value` into local state.
    // If value bit 13 is set AND remote.irq_enable, calls
    // remote_irq.raise(1 << 16). Returns true iff raise was issued.
    bool write(Side side, u16 value, IrqController& remote_irq);
private:
    struct SideState {
        u8   out_lo     = 0;
        bool irq_enable = false;
    };
    SideState arm9_{}, arm7_{};
};
} // namespace ds
```

#### `IpcFifo`

```cpp
namespace ds {
class IrqController;

class IpcFifo {
public:
    enum class Side { Arm9, Arm7 };
    void reset();
    u16 read_cnt(Side side) const;
    void write_cnt(Side side, u16 value,
                   IrqController& local_irq, IrqController& remote_irq);
    void write_send(Side side, u32 value,
                    IrqController& local_irq, IrqController& remote_irq);
    u32 read_recv(Side side,
                  IrqController& local_irq, IrqController& remote_irq);
    // Test introspection — not for cross-subsystem callers.
    bool send_empty(Side side) const;
    bool send_full(Side side) const;
    bool recv_empty(Side side) const;
    bool recv_full(Side side) const;
    u8   fill_level(Side send_side) const;
private:
    struct Direction {
        std::array<u32, 16> data{};
        u8   head = 0, tail = 0, count = 0;
        u32  last_word = 0;
        bool ever_received = false;
    };
    struct SideCnt {
        bool send_empty_irq      = false;       // CNT.2
        bool recv_not_empty_irq  = false;       // CNT.10
        bool error               = false;       // CNT.14
        bool master_enable       = false;       // CNT.15
        bool prev_send_empty_gated     = false; // edge latch
        bool prev_recv_not_empty_gated = false; // edge latch
    };
    Direction a2b_{};   // ARM9 → ARM7
    Direction b2a_{};   // ARM7 → ARM9
    SideCnt arm9_cnt_{}, arm7_cnt_{};

    // After every state change, recompute both sides' edge-trigger gated
    // values, raise on rising edges, then update the latches.
    void recompute_irqs(IrqController& arm9_irq, IrqController& arm7_irq);

    // Direction selectors. ARM9's send is a2b_; ARM9's recv is b2a_;
    // ARM7's send is b2a_; ARM7's recv is a2b_.
    Direction&       send_dir(Side side);
    Direction&       recv_dir(Side side);
    const Direction& send_dir(Side side) const;
    const Direction& recv_dir(Side side) const;
    SideCnt&         cnt(Side side);
    const SideCnt&   cnt(Side side) const;
};
} // namespace ds
```

### 4.3 State ownership and dependency graph

```
NDS owns:
  IrqController irq9_;            // new
  IrqController irq7_;            // renamed from irq7_ctrl_
  IpcSync       ipc_sync_;        // new
  IpcFifo       ipc_fifo_;        // new
  Arm7          cpu7_;            // existing (samples irq7_.line())
  Arm9          cpu9_;            // existing (stub; no sampling yet)
  Arm7Bus / Arm9Bus               // existing

NDS adds:
  void update_arm9_irq_signals();
  void update_arm7_irq_signals(); // existing — no behavioral change

Call graph (rule 3 — no inter-subsystem pointers):
  Arm{9,7}Bus::slow_*  → NDS::arm{9,7}_io_{read,write}{8,16,32}
  NDS dispatcher       → ipc_sync_.write(side, val, irq_other)
                       → ipc_sync_.read(side)
                       → ipc_fifo_.{write_cnt,write_send,read_recv,read_cnt}
                                  (side, val, irq9_, irq7_)
                       → update_arm{9,7}_irq_signals()

IpcSync / IpcFifo hold no pointers — IRQ controllers are passed by ref
into each method that can raise.
```

`update_arm9_irq_signals()` body:

```cpp
void NDS::update_arm9_irq_signals() {
    // Storage-only this slice. Recompute the line bool so X-Ray pages can
    // observe it when they land. Cannot push into cpu9_ — the ARM9 stub
    // has no set_irq_line() yet. The ARM9 decoder slice replaces this
    // body with: cpu9_.set_irq_line(irq9_.line());
    arm9_irq_line_cached_ = irq9_.line();
}
```

(`arm9_irq_line_cached_` is a private bool on NDS, exposed via accessor for
later X-Ray use.)

Header-include rule check (rule 8):
- `ipc/*.hpp` includes `ds/common.hpp` only; forward-declares `IrqController`.
- `ipc/*.cpp` includes `interrupt/irq_controller.hpp`. `interrupt/` is its
  own subsystem (not `cpu/` or `bus/`); this is a leaf header with no
  upstream deps. Compliant.
- `nds.cpp` includes `ipc/*.hpp` and `interrupt/irq_controller.hpp`. NDS is
  the integration point and is allowed to include from any subsystem.

### 4.4 IO routing (the new bus paths)

**ARM7 bus** (`NDS::arm7_io_read*/write*`, existing pattern at `nds.cpp:96..250`):

| Address      | Width  | Read                                   | Write                                                 |
|--------------|--------|----------------------------------------|-------------------------------------------------------|
| `0x4000180`  | 16-bit | `ipc_sync_.read(Side::Arm7)`           | `ipc_sync_.write(Side::Arm7, value, irq9_)`           |
| `0x4000184`  | 16-bit | `ipc_fifo_.read_cnt(Side::Arm7)`       | `ipc_fifo_.write_cnt(Side::Arm7, value, irq7_, irq9_)`|
| `0x4000188`  | 32-bit | `0` (W-only register)                  | `ipc_fifo_.write_send(Side::Arm7, value, irq7_, irq9_)`|
| `0x4100000`  | 32-bit | `ipc_fifo_.read_recv(Side::Arm7, irq7_, irq9_)` | (R-only — write ignored with DEBUG warn)     |

**ARM9 bus** (`NDS::arm9_io_read*/write*`, currently all stubs):

| Address      | Width  | Read                                   | Write                                                 |
|--------------|--------|----------------------------------------|-------------------------------------------------------|
| `0x4000208`  | 32-bit | `irq9_.read_ime()`                     | `irq9_.write_ime(value); update_arm9_irq_signals()`   |
| `0x4000210`  | 32-bit | `irq9_.read_ie()`                      | `irq9_.write_ie(value); update_arm9_irq_signals()`    |
| `0x4000214`  | 32-bit | `irq9_.read_if()`                      | `irq9_.write_if(value); update_arm9_irq_signals()`    |
| `0x4000180`  | 16-bit | `ipc_sync_.read(Side::Arm9)`           | `ipc_sync_.write(Side::Arm9, value, irq7_); update_arm7_irq_signals()` |
| `0x4000184`  | 16-bit | `ipc_fifo_.read_cnt(Side::Arm9)`       | `ipc_fifo_.write_cnt(Side::Arm9, value, irq9_, irq7_); update_arm{9,7}_irq_signals()` |
| `0x4000188`  | 32-bit | `0` (W-only register)                  | `ipc_fifo_.write_send(Side::Arm9, value, irq9_, irq7_); update_arm{9,7}_irq_signals()` |
| `0x4100000`  | 32-bit | `ipc_fifo_.read_recv(Side::Arm9, irq9_, irq7_); → returned word; update_arm{9,7}_irq_signals()` | (R-only) |

**Bus access width handling:**
- IPCSYNC and IPCFIFOCNT (16-bit registers): native at 16-bit. 32-bit
  read/write zero-extends / accesses the low 16 bits. 8-bit access slides
  a byte window (matches `nds.cpp:213..253` IME/IE/IF 8-bit pattern).
- IPCFIFOSEND (32-bit W-only): native at 32-bit. 16-bit and 8-bit writes
  silently dropped with DEBUG warn-log; reads return 0.
- IPCFIFORECV (32-bit R-only): native at 32-bit. 16-bit and 8-bit reads
  return 0 with DEBUG warn-log (no FIFO state change). Writes ignored
  with DEBUG warn-log.

### 4.5 Edge-trigger recompute algorithm

Inside `IpcFifo::recompute_irqs(IrqController& arm9_irq, IrqController& arm7_irq)`:

```text
for each side ∈ {Arm9, Arm7}:
    // ARM9's send-empty looks at a2b_; ARM7's send-empty looks at b2a_.
    new_send_empty_gated     = cnt(side).send_empty_irq
                               && (send_dir(side).count == 0)
    new_recv_not_empty_gated = cnt(side).recv_not_empty_irq
                               && (recv_dir(side).count > 0)

    if new_send_empty_gated && !cnt(side).prev_send_empty_gated:
        irq_for(side).raise(1 << 17)
    if new_recv_not_empty_gated && !cnt(side).prev_recv_not_empty_gated:
        irq_for(side).raise(1 << 18)

    cnt(side).prev_send_empty_gated     = new_send_empty_gated
    cnt(side).prev_recv_not_empty_gated = new_recv_not_empty_gated
```

`recompute_irqs` is called at the end of `write_cnt`, `write_send`, and
`read_recv`. Reset zeroes both `prev_*_gated` latches. The ordering
discipline: state changes first, then latch comparison, then raise, then
latch update. Doing the latch update before the comparison loses the
rising edge.

---

## 5. Hardware details

These are taken directly from GBATEK and verified against the page
fetched 2026-04-24 (lines 13003-13057 of the upstream HTML).

### 5.1 IPCSYNC (`0x4000180`, NDS9/NDS7, R/W, 16-bit)

| Bit   | Dir | Meaning                                                |
|-------|-----|--------------------------------------------------------|
| 0-3   | R   | Data input from remote IPCSYNC bits 8-11 (00h..0Fh).   |
| 4-7   | -   | Not used (reads as 0).                                 |
| 8-11  | R/W | Data output to remote IPCSYNC bits 0-3 (00h..0Fh).     |
| 12    | -   | Not used.                                              |
| 13    | W   | Send IRQ to remote (0=none, 1=send IRQ to remote IF.16).|
| 14    | R/W | Enable IRQ from remote (0=disabled, 1=enabled).        |
| 15-31 | -   | Not used.                                              |

The register can be accessed simultaneously by both CPUs without violating
access permissions or generating waitstates.

### 5.2 IPCFIFOCNT (`0x4000184`, NDS9/NDS7, R/W, 16-bit)

| Bit   | Dir | Meaning                                                |
|-------|-----|--------------------------------------------------------|
| 0     | R   | Send FIFO empty (0=not empty, 1=empty).                |
| 1     | R   | Send FIFO full (0=not full, 1=full).                   |
| 2     | R/W | Send FIFO empty IRQ enable.                            |
| 3     | W   | Send FIFO clear (0=nothing, 1=flush send FIFO).        |
| 4-7   | -   | Not used.                                              |
| 8     | R   | Receive FIFO empty.                                    |
| 9     | R   | Receive FIFO full.                                     |
| 10    | R/W | Receive FIFO not-empty IRQ enable.                     |
| 11-13 | -   | Not used.                                              |
| 14    | R/W | Error (0=no error, 1=error/acknowledge — write-1-clear).|
| 15    | R/W | Enable send/receive FIFO (master enable).              |
| 16-31 | -   | Not used.                                              |

### 5.3 IPCFIFOSEND (`0x4000188`, NDS9/NDS7, W, 32-bit)

Bits 0-31 are FIFO data. Max 16 words = 64 bytes.

### 5.4 IPCFIFORECV (`0x4100000`, NDS9/NDS7, R, 32-bit)

Bits 0-31 are FIFO data. Max 16 words = 64 bytes.

### 5.5 IPCFIFO behavior quirks (verbatim from GBATEK)

- **CNT.15 master-enable disabled:** writes to IPCFIFOSEND are ignored (no
  data stored, **error bit does not get set**); reads from IPCFIFORECV
  return the oldest FIFO word **without removing it from the FIFO**.
- **Receive FIFO empty:** reading IPCFIFORECV returns the most recently
  received word (if any), or ZERO (if there was no data ever, or if the
  FIFO was cleared via IPCFIFOCNT.3), and in **either case the error bit
  gets set** (error bit is on the *reader's* IPCFIFOCNT).
- **FIFO IRQs are edge-triggered:**
  - IF.17 (send-empty) gets set when `(IPCFIFOCNT.2 AND IPCFIFOCNT.0)`
    transitions 0→1.
  - IF.18 (recv-not-empty) gets set when
    `(IPCFIFOCNT.10 AND NOT IPCFIFOCNT.8)` transitions 0→1.
- The IRQ flags **can be acknowledged even while those conditions are
  still true** (because they're edge-triggered).

### 5.6 IRQ controller registers (already implemented; for reference)

- IME (`0x4000208`): bit 0 = master enable, bits 1-31 unused.
- IE (`0x4000210`): 32-bit per-source enable. Bits 16/17/18 are IPC.
- IF (`0x4000214`): 32-bit per-source pending. Reads = 0/1, writes acknowledge
  the bits set in `value` (write-1-clear).

### 5.7 Per-CPU IRQ source map (relevant bits only)

| Bit | Source                      | ARM9 | ARM7 | This slice raises? |
|-----|-----------------------------|------|------|--------------------|
| 16  | IPC Sync                    | yes  | yes  | YES                |
| 17  | IPC Send FIFO Empty         | yes  | yes  | YES                |
| 18  | IPC Recv FIFO Not Empty     | yes  | yes  | YES                |
| (rest) | (other sources)          | various | various | NO              |

### 5.8 Behavior walkthroughs

These six walkthroughs are the test specifications. Each enumerates the
bus call, the resulting state delta, and the visible side effects.

#### 5.8.1 ARM9 writes IPCSYNC.13 (send IRQ to ARM7)

Precondition: `ipc_sync_.arm7_.irq_enable == true` (ARM7 previously set
its CNT.14).

Bus call: `arm9_io_write16(0x4000180, 0x2000)` — bit 13 set.

Steps:
1. `Arm9Bus` slow path forwards to `NDS::arm9_io_write16(0x4000180, 0x2000)`.
2. NDS dispatches: `ipc_sync_.write(Side::Arm9, 0x2000, irq7_)`.
3. Inside write: store `arm9_.out_lo = (value >> 8) & 0xF` (zero),
   `arm9_.irq_enable = (value >> 14) & 1` (false). Bit 13: since
   `arm7_.irq_enable == true`, call `irq7_.raise(1u << 16)`.
4. NDS calls `update_arm7_irq_signals()`. `irq7_.line()` becomes true if
   ARM7's IME and IE.16 are both set.
5. ARM7 picks the line up at the next instruction boundary and enters the
   vector.

Expected post-state: `irq7_.read_if() & (1u << 16)` is set. `irq9_`
untouched. SYNC bit 13 *does not persist* in any stored state.

Negative case: same write with `arm7_.irq_enable == false` → no `raise()`
call, IF.16 unchanged.

#### 5.8.2 ARM9 writes IPCFIFOSEND with FIFO empty

Precondition: `ipc_fifo_.arm9_cnt_.master_enable == true`,
`a2b_.count == 0`. ARM7 has CNT.10 = 1 to exercise the IRQ.

Bus call: `arm9_io_write32(0x4000188, 0xDEAD'BEEF)`.

Steps:
1. NDS dispatches `ipc_fifo_.write_send(Side::Arm9, 0xDEAD'BEEF, irq9_, irq7_)`.
2. Master-enable check: ARM9's CNT.15 == 1, accept. (If 0, return
   immediately, no error.)
3. Full check: `a2b_.count == 16` would set ARM9's CNT.14 error and
   return. Here count == 0, so push: `a2b_.data[a2b_.tail] = value;
   tail = (tail+1) & 15; ++count; a2b_.last_word = value;
   a2b_.ever_received = true`.
4. `recompute_irqs(irq9_, irq7_)`:
   - ARM9 send-empty gated: `arm9_cnt_.send_empty_irq && a2b_.count == 0`.
     Was true (assume CNT.2 was 1 and FIFO was empty). Now false. Falling
     edge — no raise.
   - ARM7 recv-not-empty gated:
     `arm7_cnt_.recv_not_empty_irq && a2b_.count > 0`. Was false. Now true.
     Rising edge → `irq7_.raise(1u << 18)`.
   - Update `prev_*_gated` latches.
5. NDS calls `update_arm9_irq_signals()` and `update_arm7_irq_signals()`.

Expected: `a2b_.count == 1`, ARM7 IF.18 set.

#### 5.8.3 ARM9 writes IPCFIFOSEND with FIFO full

Precondition: `arm9_cnt_.master_enable == true`, `a2b_.count == 16`.

Bus call: `arm9_io_write32(0x4000188, 0x1234)`.

Steps:
1. Dispatch into `write_send(Side::Arm9, …)`.
2. Master enable OK.
3. Full check: `count == 16` → `arm9_cnt_.error = true`. Return without
   modifying FIFO data.
4. `recompute_irqs`: no condition changed. No edges, no raise.
5. NDS still calls `update_*` (cheap, idempotent).

Expected: FIFO unchanged. ARM9 reading CNT shows bit 14 = 1. ARM7 IF
unchanged.

#### 5.8.4 ARM7 reads IPCFIFORECV when empty

ARM7's recv direction = `a2b_` (ARM9→ARM7).

Precondition A (never received): `a2b_.ever_received == false`,
`arm7_cnt_.master_enable == true`.

Bus call: `arm7_io_read32(0x4100000)`.

Steps:
1. Dispatch `ipc_fifo_.read_recv(Side::Arm7, irq7_, irq9_)`.
2. `recv_dir(Arm7) == a2b_`, `count == 0`.
3. Master enable on, continue with the empty path.
4. Empty: set `arm7_cnt_.error = true`. Return value is `a2b_.last_word`
   == 0 because `ever_received == false`.
5. `recompute_irqs`: no condition changed.

Expected: returned word `0`, ARM7 CNT.14 set.

Precondition B (had data, then drained, then read again):
`last_word = 0xC0FFEE`, `count == 0`. Same call returns `0xC0FFEE`, sets
error.

#### 5.8.5 ARM7 reads IPCFIFORECV when full → empty

Precondition: `arm7_cnt_.master_enable == true`,
`arm7_cnt_.recv_not_empty_irq == true`, `a2b_.count == 16`,
`arm9_cnt_.send_empty_irq == true`, `arm9_cnt_.master_enable == true`.

Sequence: ARM7 reads RECV 16 times.

Reads 1-15: pop, `last_word = popped`, count decreases. No edges
(send-empty stays false because count > 0 after each pop except the last;
recv-not-empty stays true because count > 0 after each pop except the last).

Read 16 (drains last word):
1. After pop, `count == 0`.
2. ARM9 send-empty gated: was false (count == 1), now true. Rising edge
   → `irq9_.raise(1u << 17)`.
3. ARM7 recv-not-empty gated: was true (count == 1), now false. Falling
   edge — no raise.
4. NDS calls `update_*`.

Expected: ARM9 IF.17 set after the 16th read. `a2b_.count == 0`.
`arm7_cnt_.error` unchanged.

Read 17 (one more on now-empty FIFO):
1. Empty path: `arm7_cnt_.error = true`. Returns `last_word` (the 16th
   word). No edges.

#### 5.8.6 ARM7 writes IPCFIFOCNT.3 to clear send FIFO

ARM7's send direction = `b2a_` (ARM7→ARM9).

Precondition: `b2a_.count == 5`, `arm9_cnt_.recv_not_empty_irq == true`,
`arm7_cnt_.send_empty_irq == true`.

Bus call: `arm7_io_write16(0x4000184, 0x8008)` — bit 15 (master enable
on) + bit 3 (clear).

Steps:
1. Dispatch `write_cnt(Side::Arm7, 0x8008, irq7_, irq9_)`.
2. Decode: `master_enable = 1`, `send_empty_irq = 0`, `recv_not_empty_irq = 0`.
   Bit 14 in `value` is 0 → no acknowledge of existing error.
3. Bit 3 (write-only one-shot): `b2a_.data.fill(0)`, head/tail/count = 0,
   `last_word = 0`, `ever_received = false` (per GBATEK: "ZERO if cleared
   via IPCFIFOCNT.3").
4. Apply CNT bit changes.
5. `recompute_irqs`:
   - ARM9 recv-not-empty gated (b2a_): was true. After: false. Falling
     edge — no raise.
   - ARM7 send-empty gated (b2a_): was false. After: false (because
     `send_empty_irq` is now 0 after the write). No edge.
6. NDS calls `update_*`.

Expected: `b2a_.count == 0`, `last_word == 0`. ARM9 CNT shows recv-empty
(bit 8 = 1). No spurious IRQs raised.

Aside: if ARM7 had instead written `0xC008` (bit 14 = 1, ack error), step
2 would also clear `arm7_cnt_.error`.

Note on a counterintuitive case: if ARM7's `send_empty_irq` had been kept
on across the same write (e.g. `0x8408` instead of `0x8008`), step 5
would compute the new send-empty gated value as `true && (count == 0)` =
true, which is a rising edge from false → raises IF.17. **Per GBATEK
this is the correct behavior** — clearing via .3 is a count transition
like any other, and the gated condition recompute fires naturally.
Counterintuitive but per-spec.

---

## 6. Testing strategy

CTest count goes from 60 to 64 (four new test binaries plus one
integration test, with one of them — `ipc_sync_test` — receiving an
appendage from commit 4 rather than counting twice).

Wait — recount: commit 2 adds `arm9_irq_io_test`, commit 3 adds
`ipc_sync_test`, commit 4 appends to `ipc_sync_test` (no new binary),
commit 5 adds `ipc_fifo_test`, commit 6 adds `ipc_fifo_irq_test`,
commit 7 adds `ipc_integration_test`. That's **5 new binaries**. CTest
count 60 → **65**.

### 6.1 `arm9_irq_io_test.cpp` (commit 2)

- `Arm9IoIme_WritesAndReadsBack`: REQUIRE arm9 IME=1 after write; REQUIRE
  arm7 IME unchanged.
- `Arm9IoIe_HalfwordSplit`: REQUIRE writing low half then high half
  assembles a 32-bit IE correctly.
- `Arm9IoIf_WriteOneClear`: REQUIRE bits set via direct `irq9_.raise()`
  are cleared by writing 1s to IF; REQUIRE other bits untouched.

### 6.2 `ipc_sync_test.cpp` (commits 3-4)

- `IpcSync_Reset_AllZero`: REQUIRE both sides read 0 after reset.
- `IpcSync_Arm9OutToArm7In`: ARM9 writes `0x0500` → ARM7 reads bits 0-3
  == 5; high half of ARM7's read still reflects ARM7's own state.
- `IpcSync_Arm7OutToArm9In`: symmetric.
- `IpcSync_Bit13_RaisesArm7Irq_WhenArm7CnEnabled` (commit 4): ARM7 first
  writes `0x4000` (bit 14 = 1); ARM9 writes `0x2000` (bit 13). REQUIRE
  `irq7_.read_if() & (1<<16)`.
- `IpcSync_Bit13_NoRaiseWhenArm7CnDisabled` (commit 4): same minus ARM7's
  bit 14 enable. REQUIRE IF.16 still zero.
- `IpcSync_Bit13_DoesNotRaiseLocalIrq` (commit 4): ARM9 writes bit 13 →
  REQUIRE `irq9_.read_if() & (1<<16) == 0`.
- `IpcSync_Bit13_NotPersisted` (commit 4): after bit-13 write, REQUIRE
  neither side reads back bit 13 set.

### 6.3 `ipc_fifo_test.cpp` (commit 5)

- `IpcFifo_Reset_AllEmpty_NoError_BothEnabled0`: REQUIRE CNT bit 0 = 1
  (send empty), bit 8 = 1 (recv empty), bit 1 = 0, bit 9 = 0, bit 14 = 0,
  bit 15 = 0.
- `IpcFifo_SendWhileDisabled_Dropped`: ARM9 writes SEND with master
  enable off → REQUIRE count still 0, error still 0.
- `IpcFifo_RecvWhileDisabled_PeeksLastWord`: enable, send 0xCAFE, drain.
  Disable. Read RECV → REQUIRE returns 0xCAFE, count unchanged at 0.
- `IpcFifo_FillAndDrain_16`: enable both sides, push 16 unique words,
  REQUIRE full bit set, REQUIRE 17th push sets error and does not modify
  FIFO. Drain 16, REQUIRE values come back in order, REQUIRE empty bit set.
- `IpcFifo_EmptyRecv_ReturnsLastWord_SetsError`: send 0xDEAD, drain. Read
  empty → REQUIRE 0xDEAD returned, REQUIRE error bit set.
- `IpcFifo_EmptyRecvNeverReceived_ReturnsZero`: just-reset, read RECV →
  REQUIRE returns 0, REQUIRE error set.
- `IpcFifo_Cnt3_ClearSendFifo_ZeroesData_AndLastWord`: send 5 words,
  write CNT bit 3 set → REQUIRE count 0, REQUIRE empty-recv on other side
  returns 0 (last_word + ever_received cleared).
- `IpcFifo_Cnt14_AckError`: trigger error, write CNT with bit 14 set →
  REQUIRE error cleared.
- `IpcFifo_Cnt3_DoesNotPersist`: write bit 3, read CNT back → REQUIRE
  bit 3 reads 0.

### 6.4 `ipc_fifo_irq_test.cpp` (commit 6)

- `Fifo_RecvNotEmpty_RisingEdge_RaisesIf18`: enable, set ARM7 CNT.10,
  ARM9 sends one word → REQUIRE IF.18 set on ARM7.
- `Fifo_RecvNotEmpty_NoMask_NoRaise`: enable, ARM7 CNT.10 = 0, ARM9 sends
  → REQUIRE IF.18 still zero.
- `Fifo_SendEmpty_RisingEdge_RaisesIf17`: enable, ARM9 CNT.2 = 1, send 1
  word, ARM7 reads it (count returns to 0) → REQUIRE IF.17 set on ARM9.
- `Fifo_SendEmpty_OnlyRisesAtTrueZero`: enable, ARM9 CNT.2 = 1, send 2
  words, ARM7 reads first → REQUIRE IF.17 still zero (count 2→1, not →0).
  Read second → REQUIRE IF.17 now set.
- `Fifo_FallingEdgeNeverRaises`: enable, fill, drain to non-empty, then
  fill again — confirm no spurious bits set.
- `Fifo_TogglingCntMaskOn_WhileConditionTrue_Raises`: send a word with
  receiver's CNT.10 = 0 → no raise. Now write CNT with .10 = 1 → REQUIRE
  IF.18 set (rising edge of the gated value).
- `Fifo_AckIfThenReassert`: trigger IF.18, write 1 to IF.18 to clear.
  Send another word into already-non-empty FIFO → REQUIRE IF.18 NOT
  re-raised (no rising edge — count went from N→N+1, never crossed 0→1
  in the gated sense). Send only re-raises after a full drain + re-fill.
- `Fifo_Cnt3_Clear_RaisesIf17_OnSenderSide_WhenMaskOn`: sender has
  CNT.2 = 1, count > 0. Clear via CNT.3 → count goes >0→0. REQUIRE IF.17
  *does* raise (per-GBATEK behavior: gated condition transitioned 0→1).
- `Fifo_BothSidesIndependent`: a2b and b2a tracked independently — sending
  on one direction does not affect IRQs on the other.

### 6.5 `ipc_integration_test.cpp` (commit 7)

- `Ipc_Boot_Handshake`: full handshake sequence:
  1. Both sides write IME=1, IE = `(1<<16) | (1<<17) | (1<<18)`.
  2. Both sides write CNT = `0x8404` (master enable + send-empty IRQ +
     recv-not-empty IRQ).
  3. ARM9 writes IPCSYNC = `0x4000` (enable receive of remote SYNC IRQ).
  4. ARM7 writes IPCSYNC = `0x4500` (enable + out_lo = 5 + bit 13 trigger).
  5. REQUIRE ARM9 IF.16 set; ARM9 reads SYNC and sees bits 0-3 = 5.
  6. ARM9 acks IF.16, writes 4 distinct words (e.g. `0x11111111`,
     `0x22222222`, `0x33333333`, `0x44444444`) to SEND.
  7. REQUIRE ARM7 IF.18 set; ARM7 reads 4 words back, REQUIRE values match
     in FIFO order.
  8. REQUIRE ARM9 IF.17 set after ARM7's last read; REQUIRE the `line()`
     helpers report the expected booleans on both controllers.

### 6.6 What stays green from prior slices

All 60 existing CTest binaries must remain green throughout the slice.
The `Arm7IrqController` rename is the highest-churn step — verify after
commit 1 that `arm7_halt_test`, `arm7_bios_intrwait_test`, and
`arm7_exception_sequence_test` (which directly name the type) still
compile and pass.

---

## 7. Cross-references

- **Project design spec:** `docs/specs/2026-04-12-nds-emulator-design.md`
  §3.5 (interrupt model), §9 (scheduler internals — confirms no scheduler
  involvement for IPC), §13 Phase 1 deliverables.
- **Prior slice spec:** `docs/specs/2026-04-19-arm7-core-phase1-slice3h-design.md`
  §1 ("Next slice" handoff line proposed this work).
- **CLAUDE.md:** rule 3 (no inter-subsystem pointers), rule 4 (per-CPU IO
  tables), rule 5 (`reset()` mandatory; save_state/load_state deferred),
  rule 7 (file size caps), rule 8 (no cross-subsystem includes).
- **Existing IRQ scaffolding:**
  - `src/interrupt/irq_controller.hpp` (the class to be renamed).
  - `src/cpu/arm7/arm7.hpp` lines 19-34 (line/halt-wake interface).
  - `src/cpu/arm7/arm7.cpp` lines 14-65 (IRQ sampling at instruction
    boundaries).
  - `src/nds.cpp` lines 91-253 (existing `update_arm7_irq_signals()` and
    the IO routing pattern that ARM9 mirrors).
- **Existing bus pattern:** `src/bus/arm7_bus.cpp` lines 120-168 (slow-path
  IO dispatch), `src/bus/io_regs.hpp` (where new constants go).
- **GBATEK references** (fetched 2026-04-24):
  - https://problemkaputt.de/gbatek.htm#dsinterprocesscommunicationipc
    (lines 13003-13057 of upstream HTML — IPC SYNC + FIFO behavior).
  - https://problemkaputt.de/gbatek.htm#dsinterrupts (lines 12804-12900 —
    IRQ source map).

---

## 8. Risk and rollback

### 8.1 Highest-risk pitfalls

1. **Edge-trigger latch placement.** The `prev_*_gated` latches must be
   updated *after* the comparison-and-raise, not before. Inline updates
   lose the rising edge. Mitigation: §4.5 algorithm + the
   `Fifo_TogglingCntMaskOn_WhileConditionTrue_Raises` test, which exercises
   the exact ordering bug.
2. **Direction-naming confusion (a2b vs b2a vs send vs recv).** Easy to
   swap `send_dir(side)` and `recv_dir(side)` and produce a one-character
   bug that mirrors all data. Mitigation: helper accessors are the *only*
   place the mapping lives. The integration test uses asymmetric payloads
   (no `0xAAAA / 0x5555` — every word distinct) so a swap is loud.
3. **`last_word` rules across CNT.3 clear and master-enable disable.**
   GBATEK distinguishes "FIFO cleared (last_word becomes 0)" from "FIFO
   drained naturally (last_word retains last popped value)". Mitigation:
   the `IpcFifo_Cnt3_ClearSendFifo_ZeroesData_AndLastWord` test exists
   specifically to catch conflation.
4. **CNT.3 raises send-empty IRQ when sender's CNT.2 is on.** Per GBATEK
   the gated condition transitioned 0→1, so the raise *is* correct, but
   counterintuitive. Mitigation: explicit test
   `Fifo_Cnt3_Clear_RaisesIf17_OnSenderSide_WhenMaskOn` and a comment in
   the implementation pointing back to GBATEK.
5. **ARM9 IRQ line going nowhere.** Commit 2 adds `irq9_` and routes IO,
   but `update_arm9_irq_signals()` cannot push the line into the CPU
   stub. Mitigation: explicit comment in the function body explaining
   it's a deliberate stub awaiting the ARM9 decoder slice.
6. **8-bit IPCFIFOSEND writes.** GBATEK is silent. We're choosing to drop
   silently with DEBUG warn. If a Pokemon game's firmware does this
   (unlikely), the IPC channel goes quiet. Mitigation: warn-log makes it
   discoverable in trace. Revisit only on real divergence.

### 8.2 Rollback strategy

Each commit is independently revertable:

- Commits 5-7 (IPC) revert cleanly without touching the rename or the
  ARM9 IRQ controller. Reverting them restores the bus to "IPC writes
  silently dropped," which matches behavior before this slice.
- Commit 2 (ARM9 IRQ controller + IO) reverts cleanly without touching
  IPC. Reverting restores ARM9 IO IME/IE/IF as stubs.
- Commit 1 (rename) is the lowest-risk single-file change. Reverting it
  requires reverting commits 2-7 first, since they reference
  `IrqController` by name. If commit 1 needs to be redone, it can be
  reapplied in isolation.

### 8.3 What this slice does NOT break

Confirmed unchanged behaviors after each commit:
- ARM7 instruction execution and IRQ vector entry.
- ARM7 BIOS HLE handlers (math, memcpy, intrwait, decompressors, sound
  tables) — they exercise `irq7_` (renamed) but the functionality is
  identical.
- ARM7 HALTCNT halt/wake logic.
- ARM7 IO routes for IME/IE/IF/HALTCNT/SOUNDBIAS.
- All 60 existing CTest binaries must remain green at every commit.

---

## 9. Slice completion criteria

- [ ] `class IrqController` exists in `src/interrupt/irq_controller.hpp`
      (renamed from `Arm7IrqController`). All 12 callsites in `nds.cpp`
      and the affected tests updated. `arm7_halt_test`,
      `arm7_bios_intrwait_test`, `arm7_exception_sequence_test` pass.
- [ ] `IrqController irq9_` member on `NDS`. Accessor `irq9()`.
      `update_arm9_irq_signals()` defined with the documented stub body.
      ARM9 IO routes for IME/IE/IF wired (`arm9_io_read/write{8,16,32}`).
- [ ] `src/ipc/ipc_sync.{hpp,cpp}` exist with `class IpcSync` declared
      and implemented per §4.2. ARM7 and ARM9 IO routes for `0x4000180`
      wired. Bit 13 raise gated on remote bit 14.
- [ ] `src/ipc/ipc_fifo.{hpp,cpp}` exist with `class IpcFifo` declared
      and implemented per §4.2. ARM7 and ARM9 IO routes for `0x4000184`,
      `0x4000188`, `0x4100000` wired. Master-enable quirks, `last_word`
      semantics, CNT.3 clear, CNT.14 ack all per GBATEK §5.
- [ ] Edge-triggered IF.17 / IF.18 implemented per §4.5. IF.16 raised by
      IPCSYNC.13 W per §5.8.1.
- [ ] Six new I/O constants in `src/bus/io_regs.hpp`.
- [ ] Five new test binaries: `arm9_irq_io_test`, `ipc_sync_test`,
      `ipc_fifo_test`, `ipc_fifo_irq_test`, `ipc_integration_test`. All
      registered via `add_ds_unit_test()` in `tests/CMakeLists.txt`.
- [ ] `ctest --output-on-failure` reports **65/65 passing in Debug**.
- [ ] `clang-format` clean on all new files (enforced by the
      `PostToolUse` hook on every Edit/Write).
- [ ] `ds-architecture-rule-checker` and `gbatek-reviewer` run clean on
      the uncommitted diff for every commit.
- [ ] `quality-reviewer` runs clean on every commit.
- [ ] No file exceeds the 500-line soft cap.
- [ ] No new SDL include path. No new pointers held across subsystems.
      No `cpu/` or `bus/` includes from `ipc/`.
- [ ] No save-state code added (deferred per CLAUDE.md rule-5 carve-out).
      `reset()` implemented on every new class.

---

## Appendix A. Commit sequence

Six commits, shippable individually. Each ends with `ctest` green.

### Commit 1 — `interrupt: rename Arm7IrqController to IrqController`

- Pure rename in `src/interrupt/irq_controller.hpp`.
- Optional one-commit `using Arm7IrqController = IrqController;` alias if
  needed to keep tests building during the same commit; remove the alias
  in the same commit if a sweep of all 12 NDS callsites + the test
  references is feasible (it is — sed-and-build).
- Update `nds.hpp`/`nds.cpp` (rename `irq7_ctrl_` member to `irq7_` to
  match new convention).
- Update tests that name the type directly: `arm7_halt_test`,
  `arm7_bios_intrwait_test`, `arm7_exception_sequence_test`.
- No new tests this commit — the rename is exercised by every existing
  IRQ test.

**Diff size estimate:** ~30 lines changed across ~8 files, all mechanical.

### Commit 2 — `nds: instantiate Arm9 IrqController + route ARM9 IME/IE/IF`

- Add `IrqController irq9_{};` to `NDS`.
- Add `void update_arm9_irq_signals();` and a `bool arm9_irq_line_cached_`
  member; implement the body per §4.3.
- Mirror the ARM7 IO routing pattern in `arm9_io_read/write{8,16,32}` for
  `IO_IME` (`0x4000208`), `IO_IE` (`0x4000210`), `IO_IF` (`0x4000214`).
- New test binary `arm9_irq_io_test.cpp` per §6.1.
- `tests/CMakeLists.txt`: `add_ds_unit_test(arm9_irq_io_test)`.

### Commit 3 — `ipc: scaffold IpcSync class + 0x4000180 routing`

- Create `src/ipc/ipc_sync.{hpp,cpp}` with `class IpcSync` per §4.2,
  WITHOUT bit-13 raise (write handler stores bits 8-11 + 14, ignores
  bit 13). `read()` returns the documented bit assembly.
- `src/ipc/CMakeLists.txt` (or update `src/CMakeLists.txt`) to compile
  the new files into `ds_core`.
- Add `IpcSync ipc_sync_{};` to NDS.
- Route `IO_IPCSYNC` (`0x4000180`) on both ARM7 and ARM9 buses (16-bit
  native, 8/32-bit access widths handled per §4.4).
- Add `constexpr u32 IO_IPCSYNC = 0x04000180;` to `src/bus/io_regs.hpp`.
- New test binary `ipc_sync_test.cpp` covering the first three
  `IpcSync_*` REQUIRE blocks from §6.2 (no bit-13 yet).
- `tests/CMakeLists.txt`: `add_ds_unit_test(ipc_sync_test)`.

### Commit 4 — `ipc/sync: implement bit 13 cross-CPU IRQ raise`

- Implement bit-13 path in `IpcSync::write` per §5.8.1.
- NDS dispatch passes the *remote* `IrqController&` as the third arg.
- After every `ipc_sync_.write(...)` in NDS, call the appropriate
  `update_arm{9,7}_irq_signals()`.
- Append to `ipc_sync_test.cpp` the four bit-13 REQUIRE blocks from §6.2.
- No new test binary (existing one grows).

### Commit 5 — `ipc: scaffold IpcFifo + CNT/SEND/RECV routing without IRQs`

- Create `src/ipc/ipc_fifo.{hpp,cpp}` with `class IpcFifo` per §4.2,
  WITHOUT `recompute_irqs` (stub it out — body empty).
- Add `IpcFifo ipc_fifo_{};` to NDS.
- Route `IO_IPCFIFOCNT` (`0x4000184`), `IO_IPCFIFOSEND` (`0x4000188`),
  `IO_IPCFIFORECV` (`0x4100000`) on both buses per §4.4.
- Add three new constants to `src/bus/io_regs.hpp`.
- Implement push/pop, full/empty, master-enable quirks, error bit, CNT.3
  clear, CNT.14 ack — all per §5.
- New test binary `ipc_fifo_test.cpp` per §6.3.
- `tests/CMakeLists.txt`: `add_ds_unit_test(ipc_fifo_test)`.

### Commit 6 — `ipc/fifo: implement edge-triggered IF.17 / IF.18 raise`

- Implement `recompute_irqs` per §4.5.
- Wire it into the end of `write_cnt`, `write_send`, `read_recv`.
- After every IPC FIFO IO write/read in NDS, call both
  `update_arm{9,7}_irq_signals()`.
- New test binary `ipc_fifo_irq_test.cpp` per §6.4.
- `tests/CMakeLists.txt`: `add_ds_unit_test(ipc_fifo_irq_test)`.

### Commit 7 — `ipc: end-to-end ARM9↔ARM7 round-trip integration test`

- No production code changes (or only any small fixes that surface
  during writing the integration test, in which case fold them back into
  commit 5 or 6 and reorder).
- New test binary `ipc_integration_test.cpp` per §6.5.
- `tests/CMakeLists.txt`: `add_ds_unit_test(ipc_integration_test)`.
- After this commit, `ctest` reports 65/65 passing.

---

## Appendix B. Provenance audit

- IPC register bit layouts in §5.1-5.5: verbatim from GBATEK fetched
  2026-04-24 from `https://problemkaputt.de/gbatek.htm`, lines
  13003-13057 of the upstream HTML. No interpretation, no rephrasing.
- IRQ source bit map in §5.7: from GBATEK fetched 2026-04-24, lines
  12804-12900 of the upstream HTML. Filtered to bits 16-18 only; full
  table is in the upstream doc.
- IRQ controller register layouts in §5.6: existing implementation in
  `src/interrupt/irq_controller.hpp`, originally documented in slice 3d.
- No BIOS dump, no proprietary data, no copyrighted code referenced by
  this design.
