# ARM7 Core — Phase 1, Slice 3e Design

**Date:** 2026-04-16
**Slice:** ARM7 BIOS HLE — the SWI jump table (math, halt/wait, memcpy, tables)
**Status:** proposed
**Prior slice:** 3d (exceptions) — landed as `ec81cfe`, `0a4ad8e`, `9c99abc`; 43 CTest binaries green
**Next slice:** 3f (decompressor family — LZ77 / Huff / RLE / BitUnPack)

---

## 1. Summary

Slice 3d built the machinery that carries the CPU *into* the supervisor handler when a game executes `SWI #n`. The handler itself — the table of ~32 BIOS routines indexed by the SWI number — was deliberately stubbed. Every ARM7 SWI currently warn-logs and returns without doing anything. This slice fills in that table.

The ARM7 BIOS is the small collection of routines a DS game calls to do things the CPU can't do cheaply itself: wait for an interrupt, halt until an event, integer divide and square root, byte/word memory fill and copy, CRC-16 for cart-header validation, and a handful of constant sine/pitch/volume lookup tables used by the sound driver. Every Gen-4/5 Pokemon title uses most of these during boot. Without them, the game either spins in a `SWI 0x04` IntrWait forever, or reaches uninitialized state because a `CpuSet` memset silently did nothing.

After this slice, the ARM7 side can run a real game through boot up to the point where it's waiting on peripherals (timers, DMA, IPC, cart). Those peripherals are separate later slices; their absence — not BIOS coverage — becomes the next blocker.

### Plain-language summary

Think of the BIOS as a small library of helper functions baked into the console at the factory. A game doesn't jump to these functions with a normal `BL` — it uses the special `SWI` instruction, which traps into supervisor mode. Our slice 3d made the trap work. Slice 3e teaches the trap handler what to actually do once it gets there.

There are about thirty documented ARM7 SWIs. We implement the ones Pokemon games actually use, plus a few "free" trivial ones (`IsDebugger` returning 0, `Sqrt` being a one-liner). The SWIs fall into five natural families:

1. **Halt and wait family** (`Halt`, `IntrWait`, `VBlankIntrWait`, `Sleep`, `CustomHalt`) — the CPU stops executing until something external happens. This is the most complicated group because it introduces a new CPU state: "halted, waiting for an IRQ to wake me." We add a `halted_` flag on `Arm7State` and teach `run_until` to burn cycles while halted until the IRQ line comes up.
2. **Math family** (`Div`, `Sqrt`, `CpuFastSet`, `CpuSet`) — pure arithmetic or memory copies. Done entirely in C++ against the register file and the bus.
3. **Memory family** (`CpuSet`, `CpuFastSet`, `GetCRC16`) — copy/fill memory or compute a CRC over bus-visible memory. Must go through the bus (the destination may be VRAM or TCM).
4. **Tables family** (`GetSineTable`, `GetPitchTable`, `GetVolumeTable`) — return a constant from a precomputed lookup table. The tables are static data.
5. **Misc family** (`IsDebugger`, `GetCRC16`, `SoundBias`, `GetBootProcs`, `WaitByLoop`) — one-offs. Most are one-liners.

Two SWIs (`IntrWait` / `VBlankIntrWait`) need a shadow register at `[0x0380FFF8]` that the BIOS HLE reads and the game's IRQ handler is expected to write into. This is flagged in slice 3d's §5.7 as a slice 3e concern; we document the contract here.

**What this slice builds:**
- The full ARM7 SWI dispatch table in `bios7_hle.cpp`, split across `bios7_halt.cpp`, `bios7_math.cpp`, `bios7_memcpy.cpp`, `bios7_tables.cpp` by family when the central file approaches the 500-line soft cap.
- Three prerequisite hardware pieces: the `halted_` CPU flag and `run_until` fast-path; the HALTCNT I/O register at `0x04000301`; the SOUNDBIAS I/O register at `0x04000504` (warn-stub).
- Per-SWI unit tests — one per non-trivial SWI, shared binaries for trivial ones.

**What this slice deliberately does NOT build:**
- The decompressor family (`LZ77UnCompWrite8`, `LZ77UnCompReadByCallback16`, `HuffUnCompReadByCallback`, `RLUnCompWrite8`, `RLUnCompReadByCallback16`, `BitUnPack`). They warn-stub in this slice; slice 3f implements them as their own focused slice with per-algorithm commits.
- `SoftReset`. Direct-boot never calls it. Warn-stub.
- ARM9 BIOS HLE. Separate slice once ARM9 scaffolding exists.
- Real `Sleep` semantics. Writing `HALTCNT = 0xC0` parks like `Halt` for now; true sleep wake (cart-insert / key IRQ wakeup with different mask rules) lands when those IRQ sources exist.
- Cycle-accurate SWI costs. Each SWI records 1 cycle, matching how slice 3d treats exception entry. Phase 6 does cycle accuracy across the board.

### Scope boundary

**In scope:** the 21 SWIs listed in §2.1 (all non-decompressor SWIs Pokemon HG/SS uses, plus the trivial ones we get for free); the `halted_` flag on `Arm7State` and corresponding fast-path branch in `run_until`; the HALTCNT register at `0x04000301` (8-bit); the SOUNDBIAS register at `0x04000504` (16-bit, warn-stub acceptable); an explicit contract docstring for the `[0x0380FFF8]` BIOS IRQ-check-bits shadow documenting what `IntrWait` reads and what the game-side IRQ handler is expected to write; the per-family file split inside `src/cpu/arm7/bios/`; unit tests per family.

**Out of scope (§3 for detail):**
- Decompressors (deferred to slice 3f).
- `SoftReset` SWI 0x00 (warn-stub with no real implementation).
- ARM9 BIOS HLE.
- Full sleep-mode semantics.
- Cycle-accurate SWI costs.
- Wiring the `[0x0380FFF8]` shadow from a real IRQ handler (no IRQ sources exist yet — game test programs fake it in unit tests).

---

## 2. Goals

1. **Every SWI Pokemon HG/SS touches during boot returns a correct result**, verified against GBATEK and cross-checked against melonDS `src/ARMInterpreter.cpp` for edge cases. "Correct" means the right register file after return, the right bus effects for copies, and the right halt/wake behavior for the wait family.
2. **One SWI per commit**, each with its test. Trivial ones (IsDebugger, Sqrt, GetSineTable) can share a test binary but still get their own implementation commit.
3. **Clean file split from day one.** `bios7_hle.cpp` becomes a thin dispatcher — the switch statement mapping `swi_number` → per-family function — and per-family implementation files live alongside. Nothing approaches the 500-line soft cap.
4. **`halted_` is cheap and correct.** When set, `run_until` burns cycles to the target without stepping the pipeline. The IRQ-line check still runs every boundary, and a rising line clears `halted_` *before* the IRQ entry path fires (so the CPU is in "running, about to enter IRQ" state, not "halted while entering IRQ"). No scheduler events introduced.
5. **Bus discipline on memcpy-like SWIs.** `CpuSet` and `CpuFastSet` go through `bus_.read*/write*` even when the source/dest happens to live in main RAM. This keeps VRAM / TCM / mirrors working transparently.
6. **`Fixed<I, F>` not required here.** All math SWIs work on integer types by hardware definition (`Div` is signed 32-bit integer division, `Sqrt` is `u32 → u16`, CRC-16 is bitwise). 3D code is where `Fixed` is mandatory; BIOS math is not.
7. **Invalid SWI policy documented and enforced.** Invalid SWI numbers (0x01, 0x02, 0x0A, 0x16–0x19, 0x1E) warn-log and NOP-return. Real hardware "jumps to 0" on unimplemented — we deliberately diverge because jumping to zero in direct-boot hits whatever garbage is at 0x00000000 and hard-crashes the game. NOP-return lets ROM triage continue.
8. **Per-SWI tests link `ds_core` only, use `REQUIRE`, run in ms.** Same discipline as every prior slice.
9. **No new subsystem coupling.** BIOS HLE takes `Arm7State&` and `Arm7Bus&` references; the per-family dispatch functions have the same signature. No subsystem pointer held anywhere.

### 2.1 SWI coverage matrix

| # | Name | Family | This slice? | Notes |
|---|---|---|---|---|
| 0x00 | SoftReset | halt | warn-stub | Direct boot never calls. |
| 0x01 | (invalid) | — | warn-stub | |
| 0x02 | (invalid) | — | warn-stub | |
| 0x03 | WaitByLoop | halt | **YES** | R0-cycle busy loop; HLE burns R0 cycles on state. |
| 0x04 | IntrWait | halt | **YES** | See §5.2. |
| 0x05 | VBlankIntrWait | halt | **YES** | Tail-call of IntrWait(1, 1). |
| 0x06 | Halt | halt | **YES** | HALTCNT = 0x80. |
| 0x07 | Sleep | halt | **YES** | HALTCNT = 0xC0; functionally equal to Halt for now. |
| 0x08 | SoundBias | misc | **YES** | One-shot write to SOUNDBIAS. Ramp deferred. |
| 0x09 | Div | math | **YES** | Signed 32-bit; R0=quot, R1=rem, R3=\|quot\|. |
| 0x0A | (invalid) | — | warn-stub | |
| 0x0B | CpuSet | memory | **YES** | Copy / fill 16- or 32-bit. Via bus. |
| 0x0C | CpuFastSet | memory | **YES** | 32-bit only, word-count. Via bus. |
| 0x0D | Sqrt | math | **YES** | `R0 = u16(isqrt(u32))`. |
| 0x0E | GetCRC16 | memory | **YES** | Polynomial lookup; bus reads. |
| 0x0F | IsDebugger | misc | **YES** | One-liner: R0 = 0. |
| 0x10 | BitUnPack | decomp | warn-stub | Deferred to slice 3f. |
| 0x11 | LZ77UnCompWrite8 | decomp | warn-stub | Deferred to slice 3f. |
| 0x12 | LZ77UnCompReadByCallback16 | decomp | warn-stub | Deferred to slice 3f. |
| 0x13 | HuffUnCompReadByCallback | decomp | warn-stub | Deferred to slice 3f. |
| 0x14 | RLUnCompWrite8 | decomp | warn-stub | Deferred to slice 3f. |
| 0x15 | RLUnCompReadByCallback16 | decomp | warn-stub | Deferred to slice 3f. |
| 0x16–0x19 | (invalid) | — | warn-stub | |
| 0x1A | GetSineTable | tables | **YES** | 64-entry constant, bounds-checked. |
| 0x1B | GetPitchTable | tables | **YES** | 768-entry constant. |
| 0x1C | GetVolumeTable | tables | **YES** | 724-entry constant. |
| 0x1D | GetBootProcs | misc | **YES** | Stub: write zeros to out-pointers. |
| 0x1E | (invalid) | — | warn-stub | |
| 0x1F | CustomHalt | halt | **YES** | HALTCNT = R2. |

**21 real implementations + 13 explicit warn-stubs (6 decomp, 6 invalid, 1 SoftReset).** Real total = 21 commits' worth of implementation, bundled into ~10 commits by trivial grouping (see Appendix A).

---

## 3. Non-goals

Each deferred item names the slice that will pick it up.

- **Decompressor family** (`BitUnPack`, `LZ77UnCompWrite8`, `LZ77UnCompReadByCallback16`, `HuffUnCompReadByCallback`, `RLUnCompWrite8`, `RLUnCompReadByCallback16`). Six SWIs, each 50–100 lines, each with distinctive state machines and callback conventions. Slice 3f.
- **`SoftReset` (SWI 0x00).** Direct-boot never calls it; games that intentionally soft-reset to return to the firmware menu are not a direct-boot concern. Warn-stub documented in this slice.
- **ARM9 BIOS HLE.** ARM9 has its own SWI set (subset overlaps ARM7 but numbering and some semantics differ — e.g. ARM9 has `WaitForIRQ` instead of `IntrWait`). Separate slice once ARM9 scaffolding exists.
- **Ramp timing in `SoundBias` SWI 0x08.** Real hardware ramps the SOUNDBIAS register toward the target with per-step delay `R1 * N` cycles. Audibly indistinguishable from a one-shot write — we write once, warn, and move on. Revisit in Phase 6 polish if a game's audio depends on it.
- **True sleep-mode wake semantics.** Real hardware: writing `HALTCNT = 0xC0` puts the CPU in sleep mode, woken only by specific sources (keypad with IRQ, cart insert) and with different IRQ-mask rules than Halt. We treat sleep as halt — correct functional behavior for every game we target, incorrect for a DS power-manager demo.
- **Cycle-accurate SWI costs.** Each SWI records 1 cycle today. Real costs range from ~10 cycles (`IsDebugger`) to thousands (`CpuSet` with a large length). Phase 6.
- **Automatic population of `[0x0380FFF8]` from IRQ entry.** The BIOS shadow of IF is written by the game's IRQ handler, not by BIOS. `IntrWait` only *reads* the shadow. Slice 3e documents the contract; no slice needs to write on the BIOS side.
- **ARM7 BIOS ROM backing at `0x00000000`.** We remain in direct-boot — no BIOS ROM at 0x00000000–0x00003FFF. `SWI` still lands in our HLE handler via slice 3d's mechanism.
- **Privilege-gated SWI access.** Real hardware: user-mode code can SWI (that's the whole point); we don't check modes. Pokemon games run in System mode anyway.

---

## 4. Architecture

One central dispatcher file, four family-specific implementation files, two new I/O registers, one new CPU state flag, one new `run_until` branch. No cross-subsystem coupling added — BIOS HLE remains a callee of ARM7 decode, nothing else.

### 4.1 File layout

```
src/cpu/arm7/bios/
  bios7_hle.hpp                 MODIFIED. Adds per-family dispatch function
                                declarations. arm7_bios_hle_dispatch_swi
                                signature unchanged.
  bios7_hle.cpp                 MODIFIED. Stub becomes a switch(swi_number)
                                that forwards to per-family functions. ~80 lines.
  bios7_halt.hpp                NEW. Halt/wait family declarations.
  bios7_halt.cpp                NEW. Halt, IntrWait, VBlankIntrWait, Sleep,
                                CustomHalt, WaitByLoop. ~160 lines.
  bios7_math.hpp                NEW. Math family declarations.
  bios7_math.cpp                NEW. Div, Sqrt. ~60 lines.
  bios7_memcpy.hpp              NEW. Memory family declarations.
  bios7_memcpy.cpp              NEW. CpuSet, CpuFastSet, GetCRC16. ~180 lines.
  bios7_tables.hpp              NEW. Tables family declarations + constexpr
                                array definitions (embedded lookup data).
  bios7_tables.cpp              NEW. GetSineTable, GetPitchTable, GetVolumeTable,
                                IsDebugger, GetBootProcs, SoundBias. ~220 lines
                                including the ~1500-entry constant data.
                                (Will approach the 500-line soft cap; if the
                                constant tables grow, split into
                                bios7_tables_data.cpp.)

src/cpu/arm7/
  arm7_state.hpp                MODIFIED. Add:
                                   bool halted = false;
                                Single boolean, no bank replication, cleared
                                on reset.
  arm7.hpp                      unchanged.
  arm7.cpp                      MODIFIED. run_until() top-of-loop now:
                                   sample IRQ line;
                                   if halted and !line: burn cycles to target, return;
                                   if halted and line: halted = false, fall through;
                                   continue with existing IRQ entry / step dispatch.

src/bus/
  io_regs.hpp                   MODIFIED. Add:
                                   constexpr u32 IO_HALTCNT   = 0x04000301;
                                   constexpr u32 IO_SOUNDBIAS = 0x04000504;
  arm7_bus.cpp                  unchanged (slow path already routes 0x04 → nds_).

src/nds.hpp                     MODIFIED. Forward-declare nothing new. Add a
                                u16 soundbias_ = 0x0200; // reset value
                                member (see §4.6). No new dependencies.
src/nds.cpp                     MODIFIED. arm7_io_write8/write16/write32 route
                                IO_HALTCNT and IO_SOUNDBIAS. HALTCNT write
                                calls cpu7_.state().halted = true (with mode
                                bit logic per GBATEK); SOUNDBIAS write stores
                                the low 10 bits.

tests/unit/
  arm7_bios_halt_test.cpp          NEW. Halt fast-path + wake via IRQ line.
                                   WaitByLoop cycle-burn. CustomHalt.
  arm7_bios_intrwait_test.cpp      NEW. IntrWait / VBlankIntrWait: IF-shadow
                                   contract, wake, R0 mode bit.
  arm7_bios_math_test.cpp          NEW. Div (all sign combos, div-by-zero),
                                   Sqrt (boundary values).
  arm7_bios_memcpy_test.cpp        NEW. CpuSet (16/32-bit, copy/fill, zero-len),
                                   CpuFastSet (word count, zero-len), GetCRC16.
  arm7_bios_tables_test.cpp        NEW. GetSineTable bounds, GetPitchTable
                                   known values, GetVolumeTable known values,
                                   IsDebugger, SoundBias, GetBootProcs.
  arm7_bios_dispatch_test.cpp      NEW. Every invalid SWI warn-returns; every
                                   decomp SWI warn-returns; correct routing
                                   from ARM and Thumb entry.
  arm7_bios_sequence_test.cpp      NEW. Capstone. Mini game program that
                                   IntrWait-halts, fires IRQ, handler ORs
                                   [0x0380FFF8], IntrWait wakes, returns.
```

Seven new test binaries, taking CTest count from **43** (end of slice 3d) to **50**.

### 4.2 Dispatcher — `bios7_hle.cpp`

The slice-3d stub is:

```cpp
u32 arm7_bios_hle_dispatch_swi(Arm7State&, Arm7Bus&, u32 swi_number) {
    DS_LOG_WARN("arm7/bios: SWI 0x%02X not implemented (slice 3d stub)", ...);
    return 0;
}
```

After slice 3e, it becomes:

```cpp
// Pseudocode — not to be pasted.
u32 arm7_bios_hle_dispatch_swi(Arm7State& s, Arm7Bus& b, u32 n) {
    switch (n & 0xFFu) {
        case 0x00: return bios7_soft_reset(s, b);        // warn-stub, deferred
        case 0x03: return bios7_wait_by_loop(s, b);
        case 0x04: return bios7_intr_wait(s, b);
        case 0x05: return bios7_vblank_intr_wait(s, b);
        case 0x06: return bios7_halt(s, b);
        case 0x07: return bios7_sleep(s, b);
        case 0x08: return bios7_sound_bias(s, b);
        case 0x09: return bios7_div(s, b);
        case 0x0B: return bios7_cpu_set(s, b);
        case 0x0C: return bios7_cpu_fast_set(s, b);
        case 0x0D: return bios7_sqrt(s, b);
        case 0x0E: return bios7_get_crc16(s, b);
        case 0x0F: return bios7_is_debugger(s, b);
        case 0x10: case 0x11: case 0x12: case 0x13:
        case 0x14: case 0x15: return bios7_decomp_stub(s, b, n);
        case 0x1A: return bios7_get_sine_table(s, b);
        case 0x1B: return bios7_get_pitch_table(s, b);
        case 0x1C: return bios7_get_volume_table(s, b);
        case 0x1D: return bios7_get_boot_procs(s, b);
        case 0x1F: return bios7_custom_halt(s, b);
        default:
            DS_LOG_WARN("arm7/bios: invalid SWI 0x%02X (NOP-return)", n & 0xFFu);
            return 1;
    }
}
```

The switch is flat, the per-family functions are in the family files, and the dispatcher stays under 100 lines.

**Invalid-SWI policy.** The `default` branch catches 0x01, 0x02, 0x0A, 0x16–0x19, 0x1E. Per §5.8, we NOP-return with a warn, deliberately diverging from "jump to 0" hardware behavior. Real hardware's jump-to-0 hits BIOS ROM; direct-boot has nothing at 0, so jump-to-0 would hard-crash the emulator on an invalid SWI. NOP-return is the safer default while bringing up new ROMs. If a specific ROM depends on the hardware behavior (none known), we revisit.

### 4.3 Halt state machine — `run_until` fast-path

The question from the task prompt: **scheduler-driven or `run_until` fast-path?**

**Decision: `run_until` fast-path, no scheduler events.**

Reasoning:
- The wake condition is purely the IRQ-line input, which `run_until` already samples at every boundary. Adding a scheduler event would duplicate that sampling on a different cadence.
- The scheduler exists to model future events (timer expiry, cart DMA complete). Halt wake is not a "future event at time T" — it's "the moment an asynchronous input changes." Scheduler isn't the right primitive.
- A `run_until`-local burn loop is three lines and zero new public API.

The loop becomes:

```cpp
// Slice 3e — pseudocode, incremental over slice 3d's run_until:
while (state_.cycles < arm7_target) {
    const bool line_active = irq_line_ && !(state_.cpsr & (1u << 7));

    // Halt fast-path: burn to target (or to wake) without pipeline work.
    if (state_.halted) {
        if (line_active) {
            // An enabled IRQ pending. Wake *before* entering — the CPU is
            // briefly "running but about to vector." Matches hardware where
            // the wake path is a purely level-driven sample.
            state_.halted = false;
            // fall through to IRQ entry below.
        } else {
            // Idle. Burn cycles to the budget in one step and return —
            // no further work this tick. The next tick re-enters run_until
            // and re-samples.
            state_.cycles = arm7_target;
            return;
        }
    }

    // Existing slice 3d logic:
    if (line_active) {
        arm7_enter_irq(state_, *bus_);
        continue;
    }
    if (state_.cpsr & (1u << 5)) step_thumb();
    else                         step_arm();
}
```

**Why the wake path falls through to the same IRQ entry:** it guarantees wake-then-vector happens in a single `run_until` iteration. If we instead `continue`'d after clearing `halted`, the next iteration would re-evaluate `halted` (false now), re-sample the line (still true), and then enter IRQ — same net result, one extra branch. Falling through is one branch cheaper and reads as one atomic wake-and-vector.

**What if `CPSR.I` is set when the line goes active while halted?** `line_active` is already `irq_line_ && !I`. If `I=1`, `line_active` is false, so we stay halted. Correct: a masked IRQ doesn't wake the CPU from plain `Halt`. (*Exception:* real hardware's `Halt` wakes on `IE & IF != 0` regardless of `CPSR.I` — but then doesn't vector if `I=1`. The practical result is the same as our simpler model because games always pair `Halt` with `CPSR.I=0`. Documented as a divergence in §5.2; revisit if a ROM surfaces the difference.)

**Burn-to-target semantics.** Setting `state_.cycles = arm7_target` and returning is the coarsest possible halt — we consume the entire remaining budget as halt time. This is correct in isolation, but combined with the scheduler at the `NDS` level, it means `NDS::run_frame` sees ARM7 finish its budget instantly while the scheduler hasn't advanced. `NDS::run_frame` already slices budgets by the next scheduler event, so as long as `NDS` calls `arm7_.run_until(next_event_time)` in its inner loop, the halt-burn never overshoots an event. Design check: confirm before commit 4 that NDS's outer loop does not assume ARM7 "runs to the full frame target in one call" — it doesn't today (no events, no frame loop), and the future NDS frame loop will be event-sliced from the start.

### 4.4 `halted_` flag on `Arm7State`

Smallest-possible change: one `bool` member.

```cpp
// src/cpu/arm7/arm7_state.hpp — new field
struct Arm7State {
    // ... existing fields ...
    bool halted = false;
};
```

- `reset()` clears to `false` (existing reset logic covers this if we zero-initialize; confirm in commit 2).
- Save/load state: flagged for the phase-1 serialization pass; a single bool is trivial to add.
- No bank replication — halt is a CPU-global state, not per-mode.

### 4.5 HALTCNT register (`0x04000301`, 8-bit)

Writes-only from the game's perspective. The bits we care about:

- `0x00` — clears halt (no-op if not halted; games don't write this).
- `0x40` — GBA mode; not applicable on NDS, warn-ignore.
- `0x80` — Halt (standard). Sets `state_.halted = true`.
- `0xC0` — Sleep (wake sources restricted). For slice 3e, **treat as halt** — set `state_.halted = true` with the same semantics. Slice 3e §3 documents the simplification.

The BIOS SWIs `Halt`, `Sleep`, and `CustomHalt` all terminate by writing HALTCNT. We implement the SWIs by writing HALTCNT directly from the handler — *not* by setting `state_.halted` from inside the handler — so the register is the single source of truth and a game that writes HALTCNT directly (some crusty homebrew does this) gets the same behavior.

**Routing.** `nds.cpp::arm7_io_write8(0x04000301, value)` dispatches on the top bits:

```cpp
// pseudocode
case 0x80: cpu7_.state().halted = true; break;
case 0xC0: cpu7_.state().halted = true; break;       // sleep — treated as halt
case 0x40: DS_LOG_WARN("arm7: HALTCNT=0x40 (GBA mode) ignored"); break;
case 0x00: cpu7_.state().halted = false; break;
default:   DS_LOG_WARN("arm7: HALTCNT=0x%02X unexpected", value); break;
```

16- and 32-bit writes that cover this byte route the relevant byte through the same logic.

**Non-routing detail:** NDS reaches into `cpu7_.state()` to flip the flag. That looks like a rule-3 violation (one subsystem reaching into another's internals), but it is not — `state()` is a public accessor explicitly added in slice 3a for exactly this kind of HAL-level coordination, and `Arm7State::halted` is a flag, not a method-bearing subsystem. NDS remains the owner of cross-subsystem wiring.

### 4.6 SOUNDBIAS register (`0x04000504`, 16-bit)

The SoundBias SWI ramps this register; on real hardware, it also sets the DAC center bias for mixed audio output. We don't have audio output yet, so SOUNDBIAS is a warn-log stub that stores the low 10 bits into a `u16 soundbias_` field on `NDS`.

```cpp
// nds.hpp
u16 soundbias_ = 0x0200;  // post-firmware default per GBATEK
// nds.cpp::arm7_io_write16
case IO_SOUNDBIAS:
    soundbias_ = value & 0x03FFu;
    // No audible effect — SPU not yet wired.
    break;
```

When the SPU slice lands, SOUNDBIAS gets plumbed into the DAC mix. Until then, writes are just accepted. Why include it here rather than in the SPU slice? Because `SoundBias` SWI 0x08's one job is to write this register; without the register accepting writes, the SWI would either be a double-stub (SWI warns, register warns) or would skip its effect entirely. Landing the register here makes the SWI's one-shot-write implementation complete and trivially testable.

### 4.7 The `[0x0380FFF8]` contract

`IntrWait` / `VBlankIntrWait` poll this address while halted. The contract:

- **BIOS HLE side (this slice):** reads `[0x0380FFF8]` via bus, compares against R0-specified mask, clears matching bits on consume, loops on halt-and-re-check.
- **Game side (no slice owns this):** the game's IRQ handler, after acking `0x04000214` (hardware IF) for the serviced source, must OR the corresponding bit into `[0x0380FFF8]` (the BIOS shadow). On real hardware, the BIOS IRQ dispatcher code does this; in direct-boot, the game's handler (installed at `[0x0380FFFC]`) must.
- **Emulator plumbing:** none. It's just bus memory in ARM7 WRAM. `bus_.read32(0x0380FFF8)` and `bus_.write32(0x0380FFF8, value)` are the only touches.

Documented as a docstring in `bios7_halt.cpp` at the top of `bios7_intr_wait`. Risk section covers the deadlock case (game doesn't write shadow → `IntrWait` spins forever).

### 4.8 IntrWait implementation shape

The question from the task prompt:

**Option 1 (halt-and-re-check):** SWI handler checks shadow; if hit, consume+return; else set `halted=true`, rewind PC to re-execute the SWI on wake.

**Option 2 (blocking state):** record `waiting_for_irq_mask_` on `Arm7State`; `run_until` checks this between instructions; when the condition is met, clear state and advance past SWI.

**Decision: Option 1.**

Reasoning:
- **Closer to real hardware.** The actual BIOS at 0x04 is literally a tiny loop: check shadow, `HALTCNT=0x80`, on wake re-check. Our HLE handler mirrors that loop exactly.
- **No new Arm7State fields.** Option 2 needs two new fields (`waiting_mask`, `clear_on_return`) and a new code path in `run_until` that understands "advance past SWI." Option 1 needs zero extra state — the existing `halted` flag plus re-execution do the work.
- **Re-execution is free with PC rewind.** Inside the SWI handler, we know the instruction address (we're still at the SWI's return address). Setting `halted=true` and rewinding PC back to `ret - 4` (ARM) or `ret - 2` (Thumb) means the next `run_until` iteration — after wake — re-enters the SWI handler, which re-checks the shadow. If satisfied, it returns normally. If not, it halts again.
- **Testability.** Option 1's behavior is observable step-by-step (halt, wake, halt, wake); Option 2's is a single opaque transition. Option 1 is easier to unit-test.

Pseudocode:

```cpp
// bios7_halt.cpp::bios7_intr_wait
u32 bios7_intr_wait(Arm7State& s, Arm7Bus& b) {
    // R0 = discard-old flag, R1 = requested mask. Force IME=1 per GBATEK
    // spec (the BIOS explicitly sets IME=1 before waiting).
    const u32 discard = s.r[0];
    const u32 mask    = s.r[1];

    // Write IME=1 via bus so the IRQ line recomputes.
    b.write32(IO_IME, 1);

    if (discard == 1) {
        // Clear matching bits in the shadow before waiting, so a stale
        // pre-SWI bit doesn't cause immediate return.
        const u32 shadow = b.read32(0x0380FFF8u);
        b.write32(0x0380FFF8u, shadow & ~mask);
    }

    const u32 shadow = b.read32(0x0380FFF8u);
    if ((shadow & mask) != 0) {
        // Condition already met — consume and return.
        b.write32(0x0380FFF8u, shadow & ~mask);
        return 1;
    }

    // Condition not met — halt, then rewind PC so we re-execute this SWI
    // on wake. PC currently holds return-addr (the instruction after SWI).
    // ARM-state SWI was 4 bytes; Thumb was 2. SPSR.T tells us which.
    s.halted = true;
    const u32 spsr = *s.spsr_slot();
    const u32 instr_size = (spsr & (1u << 5)) ? 2u : 4u;
    s.r[14] -= instr_size;  // R14_svc now points at the SWI itself

    // MOVS PC, R14 at the caller's end-of-BIOS path will send us back into
    // the SWI instruction, which we'll re-execute on wake. But the BIOS
    // handler's MOVS PC, R14 is what runs after we return from this C++
    // function — so we've already set up the PC-rewind. See §4.9 for the
    // subtle "who issues MOVS PC, R14" question.
    return 1;
}
```

**Wait — where does `MOVS PC, R14` come from in the HLE flow?** See §4.9.

`VBlankIntrWait` (SWI 0x05) is a one-line tail call:

```cpp
u32 bios7_vblank_intr_wait(Arm7State& s, Arm7Bus& b) {
    s.r[0] = 1;          // discard-old
    s.r[1] = 1;          // mask = VBlank (bit 0)
    return bios7_intr_wait(s, b);
}
```

### 4.9 HLE return semantics — who issues `MOVS PC, R14`?

This is a subtle piece of the HLE plumbing that slice 3d set up but didn't test end-to-end. Slice 3e makes it concrete.

The flow when a game executes `SWI #0x04`:

1. `dispatch_swi_coproc` (in `arm7_decode.cpp`) calls `enter_exception(Swi, return_addr)`. CPSR now SVC mode, R14_svc = return_addr, SPSR_svc = old_cpsr, PC = 0x08.
2. Without returning control to `run_until`, `dispatch_swi_coproc` calls `arm7_bios_hle_dispatch_swi(state, bus, 0x04)`. That forwards to `bios7_intr_wait`.
3. `bios7_intr_wait` does its thing (consume or halt), adjusts R14_svc as needed, and returns a cycle count.
4. `dispatch_swi_coproc` returns the cycle count to `step_arm`, which wrote the count back into `state_.cycles`.
5. **The game's PC is still 0x08** (the SWI vector). The very next `run_until` iteration will step an instruction at 0x08.

On real hardware, 0x08 is BIOS code that starts with a dispatcher and ends with `MOVS PC, R14`. In direct-boot, we have nothing at 0x08. So who issues the return?

**Answer: the BIOS HLE dispatch function does, implicitly, by restoring state before returning.**

Concrete mechanism: at the end of `arm7_bios_hle_dispatch_swi`, after the per-family function returns, we *execute the return logic ourselves*:

```cpp
u32 arm7_bios_hle_dispatch_swi(Arm7State& s, Arm7Bus& b, u32 n) {
    const u32 cycles = /* ... switch dispatch ... */;

    // HLE return: replicate "MOVS PC, R14" without executing it.
    // This is what real BIOS code at the end of the handler does.
    const u32* spsr = s.spsr_slot();
    if (spsr != nullptr) {
        const u32 new_cpsr = *spsr;
        s.pc = s.r[14];
        const Mode target = static_cast<Mode>(new_cpsr & 0x1Fu);
        s.switch_mode(target);
        s.cpsr = new_cpsr;
    }

    return cycles;
}
```

This is exactly the DP S=1+Rd=R15 fix from slice 3d, called explicitly instead of via a `MOVS PC, R14` instruction we don't have memory for. The semantics are identical.

**What this means for `IntrWait`'s PC-rewind trick (§4.8):** when we rewind R14_svc by `instr_size` and then the dispatcher's return logic copies R14 to PC, PC ends up at the SWI instruction. Next iteration re-executes the SWI, which re-enters HLE, which re-checks the shadow. Perfect.

**Why didn't slice 3d do this?** Because slice 3d's handler was a warn-stub that never needed to return to caller code (nothing past the warn). Slice 3e makes the dispatcher a proper handler that *must* return, so we add the return logic centrally at the dispatcher level — every per-family function stays simple.

**Open question answered:** this is the place we realize slice 3d's `dispatch_swi_coproc` routing works because of this missing piece. Without the dispatcher's implicit return, every SWI we add in slice 3e would stick at PC = 0x08 forever. The return logic is the critical glue.

### 4.10 Per-family pseudocode

Full per-SWI pseudocode in §5. High level:

- **bios7_halt.cpp:** `halt` writes HALTCNT=0x80 via bus. `sleep` writes HALTCNT=0xC0. `custom_halt` writes HALTCNT=R2. `wait_by_loop` increments `state_.cycles` by R0 (coarse — real hardware loops). `intr_wait` as above. `vblank_intr_wait` as above.
- **bios7_math.cpp:** `div` computes signed division, writes R0/R1/R3. `sqrt` computes integer square root, writes R0.
- **bios7_memcpy.cpp:** `cpu_set` loops over length, using bus reads/writes at 16-bit or 32-bit width per the mode bit, handling copy vs fill via the other mode bit. `cpu_fast_set` loops at word width, rounded up to multiples of 8 words per GBATEK. `get_crc16` loops halfword-by-halfword through bus reads, applying the 8-entry polynomial table.
- **bios7_tables.cpp:** the three table SWIs are "bounds-check R0, load from constexpr array, write R0." `is_debugger` sets R0 = 0. `sound_bias` writes target value to SOUNDBIAS. `get_boot_procs` writes zeros to out-pointers via bus.

### 4.11 What does NOT change

- `enter_exception` and the entire slice 3d exception machinery.
- `Arm7IrqController` — slice 3d's IME/IE/IF implementation serves slice 3e unchanged.
- `Arm7Bus` interface — no new public methods. BIOS HLE uses existing `read32`/`write32`/`read16`/`write16`/`read8`/`write8`.
- `dispatch_swi_coproc` in `arm7_decode.cpp` and Thumb SWI routing in `arm7_thumb_branch.cpp` — unchanged; they already call into the dispatcher.
- Every slice 3a–3d test.

### 4.12 Known technical debt deferred

- **Decompressor family (SWIs 0x10–0x15)** — six warn-stubs here, full implementations in slice 3f.
- **SoftReset (SWI 0x00)** — warn-stub, implementation deferred indefinitely (direct-boot never calls).
- **SoundBias ramp timing** — one-shot write acceptable for audibility; ramp revisit in Phase 6.
- **Sleep-mode distinct wake rules** — treated as halt; distinguish when keypad-IRQ-wake source exists.
- **`[0x0380FFF8]` shadow written by game-side handler** — emulator plumbing is done; games hitting the deadlock case are flagged as game-side bugs.
- **Cycle-accurate SWI costs** — uniformly 1 cycle today; Phase 6.
- **Save/load state for `halted` flag** — single bool, trivial when serialization pass lands.

---

## 5. Hardware details

All GBATEK references are to `https://problemkaputt.de/gbatek.htm` §"BIOS Functions (ARM7)" unless noted. Cross-checked against melonDS `src/ARMInterpreter_Branch.cpp` and `src/NDS.cpp`'s SWI handlers.

### 5.1 `WaitByLoop` (SWI 0x03)

**Inputs:** R0 = loop count.
**Outputs:** none. R0–R3 may be clobbered per convention.
**Behavior:** real hardware runs a busy loop `count` times, ~4 cycles per iteration. HLE: add `4 * R0` to `state_.cycles`, matching the approximate wall-clock without actually looping.

```cpp
u32 bios7_wait_by_loop(Arm7State& s, Arm7Bus&) {
    s.cycles += 4u * s.r[0];  // coarse — real hardware has tighter loop
    return 1;
}
```

### 5.2 `IntrWait` (SWI 0x04) and `VBlankIntrWait` (SWI 0x05)

See §4.8 for shape. Hardware semantics:

- Forces `IME = 1`.
- If R0 == 1, clears bits in `[0x0380FFF8]` that match R1 before waiting.
- Reads `[0x0380FFF8] & R1`. If non-zero, clears those bits and returns (SWI consumed the interrupt).
- Otherwise writes `HALTCNT = 0x80` and halts until wake. On wake, re-runs the check.

**Divergence from real hardware wake semantics.** Real hardware wakes plain `Halt` when `IE & IF != 0` regardless of `CPSR.I`. Our §4.3 model wakes only when `irq_line_ && !CPSR.I`. In practice: before `Halt` / `IntrWait`, BIOS sets `CPSR.I = 0` (un-mask IRQ). Games always issue these SWIs from un-masked context. The divergence is invisible to real code. If a ROM surfaces it (none known), we add a second condition for "halted wake ignores CPSR.I".

**The `[0x0380FFF8]` shadow.** Owned by the game's IRQ handler — the game's vector (installed at `[0x0380FFFC]`) is expected to OR the serviced source's bit into `[0x0380FFF8]` before returning. `IntrWait` only reads and consumes. See §4.7.

**Deadlock case.** If a game enables a source, IntrWaits on it, but its IRQ handler fails to write `[0x0380FFF8]`, the wait spins forever: every wake from the IRQ line samples the shadow, sees zero, halts again. This is a game-side bug, not an emulator bug. Warn-log on first spin-without-progress to surface it during ROM bring-up. Risk §8.3.

### 5.3 `Halt` (SWI 0x06) and `Sleep` (SWI 0x07) and `CustomHalt` (SWI 0x1F)

- `Halt`: writes `HALTCNT = 0x80` via bus. Returns.
- `Sleep`: writes `HALTCNT = 0xC0` via bus. Returns. (Treated as halt per §3.)
- `CustomHalt`: writes `HALTCNT = R2 & 0xFF` via bus. Returns.

```cpp
u32 bios7_halt(Arm7State&, Arm7Bus& b) { b.write8(IO_HALTCNT, 0x80u); return 1; }
u32 bios7_sleep(Arm7State&, Arm7Bus& b) { b.write8(IO_HALTCNT, 0xC0u); return 1; }
u32 bios7_custom_halt(Arm7State& s, Arm7Bus& b) {
    b.write8(IO_HALTCNT, static_cast<u8>(s.r[2] & 0xFFu));
    return 1;
}
```

The NDS I/O handler for HALTCNT (§4.5) sets `state_.halted = true`. The CPU then enters halt on the next `run_until` iteration.

### 5.4 `SoundBias` (SWI 0x08)

**Inputs:** R0 = target level (0 means 0x000, non-zero means 0x200), R1 = delay per step.
**Behavior:** real hardware ramps SOUNDBIAS from current toward target with R1-cycle delay per step. HLE: one-shot write to SOUNDBIAS, ignore R1.

```cpp
u32 bios7_sound_bias(Arm7State& s, Arm7Bus& b) {
    const u16 target = (s.r[0] != 0) ? 0x200u : 0x000u;
    b.write16(IO_SOUNDBIAS, target);
    return 1;
}
```

### 5.5 `Div` (SWI 0x09)

**Inputs:** R0 = signed numerator, R1 = signed denominator.
**Outputs:** R0 = quotient, R1 = remainder, R3 = |quotient|.
**Behavior:** C++ `/` and `%` on `int32_t` match ARM's signed-integer division semantics. Div-by-zero on real hardware loops forever; HLE returns garbage with a warn and sets R0/R1/R3 to match melonDS for compatibility.

```cpp
u32 bios7_div(Arm7State& s, Arm7Bus&) {
    const i32 num = static_cast<i32>(s.r[0]);
    const i32 den = static_cast<i32>(s.r[1]);
    if (den == 0) {
        DS_LOG_WARN("arm7/bios: Div by zero at PC=0x%08X (returning 0/0)", s.pc);
        s.r[0] = 0;
        s.r[1] = num;                                 // remainder = num
        s.r[3] = 0;
        return 1;
    }
    const i32 q = num / den;
    const i32 r = num % den;
    s.r[0] = static_cast<u32>(q);
    s.r[1] = static_cast<u32>(r);
    s.r[3] = static_cast<u32>(q < 0 ? -q : q);
    return 1;
}
```

C++ integer division is implementation-defined for `INT32_MIN / -1`; on every target platform we build for, this wraps to `INT32_MIN`. Document in a comment.

### 5.6 `CpuSet` (SWI 0x0B)

**Inputs:** R0 = source pointer, R1 = destination pointer, R2 = control word:
- bits[20:0] = length in *unit count* (not bytes).
- bit 24 = mode (0 = copy, 1 = fill).
- bit 26 = width (0 = 16-bit, 1 = 32-bit).
- bits[23:21], bit 25, bits[31:27] = reserved.

**Unit count semantics:**
- 16-bit width: length is halfword count; copy `2 * length` bytes.
- 32-bit width: length is word count; copy `4 * length` bytes.
- Length == 0: no-op.

**Behavior:**
- If fill mode (bit 24 = 1): read one unit from source, write it `length` times to destination (advancing destination, not source).
- If copy mode (bit 24 = 0): read `length` units from source, write them to destination (both advance).

**Bus width matters.** 16-bit mode uses `bus.read16`/`write16`; 32-bit mode uses `bus.read32`/`write32`. Hitting the wrong width misses VRAM mirror behavior.

**BIOS region check.** ARM7 real hardware silently rejects copies whose source is in the BIOS region (0x00000000–0x00003FFF). We have no BIOS ROM — the address range is zero-backed in direct-boot. **Decision: no check.** Rationale: (a) there's literally no ROM there to "protect"; (b) adding a check adds surface area for divergence from melonDS (which also skips this check in HLE); (c) a game that reads from 0x00000000 in direct-boot is broken regardless of whether the BIOS SWI lets it through. Note in code comment for the future BIOS-ROM slice.

Pseudocode:

```cpp
u32 bios7_cpu_set(Arm7State& s, Arm7Bus& b) {
    u32 src  = s.r[0];
    u32 dst  = s.r[1];
    const u32 ctrl = s.r[2];
    const u32 len  = ctrl & 0x001FFFFFu;
    const bool fill = (ctrl & (1u << 24)) != 0;
    const bool wide = (ctrl & (1u << 26)) != 0;

    if (len == 0) return 1;

    if (wide) {
        src &= ~3u; dst &= ~3u;
        const u32 seed = fill ? b.read32(src) : 0;
        for (u32 i = 0; i < len; ++i) {
            const u32 v = fill ? seed : b.read32(src + i * 4u);
            b.write32(dst + i * 4u, v);
        }
    } else {
        src &= ~1u; dst &= ~1u;
        const u16 seed = fill ? b.read16(src) : 0;
        for (u32 i = 0; i < len; ++i) {
            const u16 v = fill ? seed : b.read16(src + i * 2u);
            b.write16(dst + i * 2u, v);
        }
    }
    return 1;
}
```

### 5.7 `CpuFastSet` (SWI 0x0C)

**Inputs:** R0 = source, R1 = dest, R2 = control: bits[20:0] = word count, bit 24 = fill.
**Semantics:** 32-bit only, word-count. Length rounded up to multiple of 8 words per GBATEK (real hardware processes 8-word blocks). Length == 0: no-op.

```cpp
u32 bios7_cpu_fast_set(Arm7State& s, Arm7Bus& b) {
    u32 src  = s.r[0] & ~3u;
    u32 dst  = s.r[1] & ~3u;
    const u32 ctrl = s.r[2];
    u32 len  = ctrl & 0x001FFFFFu;
    const bool fill = (ctrl & (1u << 24)) != 0;
    if (len == 0) return 1;
    len = (len + 7u) & ~7u;        // round up to multiple of 8

    const u32 seed = fill ? b.read32(src) : 0;
    for (u32 i = 0; i < len; ++i) {
        const u32 v = fill ? seed : b.read32(src + i * 4u);
        b.write32(dst + i * 4u, v);
    }
    return 1;
}
```

### 5.8 `Sqrt` (SWI 0x0D)

**Inputs:** R0 = u32 value.
**Outputs:** R0 = u16 integer square root (truncated).
**Behavior:** straight integer isqrt. C++ `static_cast<u32>(std::sqrt(static_cast<double>(val)))` is correct for the full u32 range but introduces a float dependency in the CPU core. Use a pure-integer Newton-Raphson instead (matches melonDS):

```cpp
u32 bios7_sqrt(Arm7State& s, Arm7Bus&) {
    const u32 v = s.r[0];
    if (v == 0) { s.r[0] = 0; return 1; }
    u32 x = v, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + v / x) / 2; }
    s.r[0] = x & 0xFFFFu;       // truncate to 16 bits
    return 1;
}
```

### 5.9 `GetCRC16` (SWI 0x0E)

**Inputs:** R0 = initial CRC, R1 = halfword-aligned pointer, R2 = byte length (even).
**Outputs:** R0 = final CRC, R3 = last halfword read at R1 + R2 - 2.
**Algorithm:** the polynomial table `{0xC0C1, 0xC181, 0xC301, 0xC601, 0xCC01, 0xD801, 0xF001, 0xA001}` and the bit-by-bit CRC loop documented in GBATEK §"BIOS GetCRC16 Function".

```cpp
static constexpr std::array<u16, 8> CRC16_TABLE = {
    0xC0C1, 0xC181, 0xC301, 0xC601, 0xCC01, 0xD801, 0xF001, 0xA001,
};

u32 bios7_get_crc16(Arm7State& s, Arm7Bus& b) {
    u16 crc = static_cast<u16>(s.r[0]);
    const u32 ptr = s.r[1] & ~1u;
    const u32 len = s.r[2] & ~1u;         // even-byte length
    u16 last = 0;

    for (u32 off = 0; off < len; off += 2) {
        const u16 word = b.read16(ptr + off);
        last = word;
        for (u32 bit = 0; bit < 8; ++bit) {
            const bool xor_now = (crc ^ (word >> bit)) & 1u;
            crc >>= 1;
            if (xor_now) crc ^= (CRC16_TABLE[bit] << 0);
        }
        for (u32 bit = 0; bit < 8; ++bit) {
            const bool xor_now = (crc ^ (word >> (bit + 8))) & 1u;
            crc >>= 1;
            if (xor_now) crc ^= (CRC16_TABLE[bit] << 0);
        }
    }
    s.r[0] = crc;
    s.r[3] = last;
    return 1;
}
```

(The exact bit-processing loop per GBATEK is slightly denser than the above; match GBATEK's pseudocode exactly in the implementation and cross-check against melonDS. The test verifies against a known CRC of the HG/SS cart header.)

### 5.10 `IsDebugger` (SWI 0x0F)

```cpp
u32 bios7_is_debugger(Arm7State& s, Arm7Bus&) { s.r[0] = 0; return 1; }
```

Retail DS always returns 0. Debug-unit DS returns 1.

### 5.11 `GetSineTable` / `GetPitchTable` / `GetVolumeTable` (SWIs 0x1A / 0x1B / 0x1C)

**Inputs:** R0 = index.
**Outputs:** R0 = table[index].

Table sizes from GBATEK:
- SineTable: 64 entries (signed 16-bit).
- PitchTable: 768 entries (unsigned 16-bit).
- VolumeTable: 724 entries (unsigned 8-bit, returned as u32).

**Decision on table origin:** compute sine and pitch from documented formulas; copy volume from melonDS with attribution.

- **SineTable** is 64 entries of `sin(2π * i / 64) * 0x7FFF`, rounded to int16. Trivial to compute in a `constexpr` initializer.
- **PitchTable** is 768 entries derived from a semitone frequency model per GBATEK: `pitch[i] = round(2^(i/12) * 65536 / 2^(60/12))` for the relevant range. Computable at `constexpr` but tedious — use a single-file generator (or a `consteval` lambda). If `consteval` feels heavy, generate once at static-init time and cache (`static const std::array<...>`).
- **VolumeTable** is 724 entries of a logarithmic-ish volume curve that DS BIOS uses; there is no widely-agreed closed form in GBATEK (it references the BIOS ROM's literal table). **Copy from melonDS** (`src/ARMInterpreter.cpp` near the `A_SWI_GetVolumeTable` handler), with a comment citing melonDS's MIT license allowing this and noting we link against a DS-hardware-derived value table.

Bounds check each: if `R0 >= table_size`, warn and return 0 (per §1 of the task prompt's guidance).

```cpp
u32 bios7_get_sine_table(Arm7State& s, Arm7Bus&) {
    const u32 i = s.r[0];
    if (i >= SINE_TABLE.size()) { s.r[0] = 0; return 1; }
    s.r[0] = static_cast<u32>(static_cast<u16>(SINE_TABLE[i]));
    return 1;
}
// Pitch and Volume follow the same shape.
```

Tables live in `bios7_tables.cpp` as `static constexpr std::array` (sine) / `static const std::array` (pitch, volume) depending on whether the initialization is `constexpr`-feasible. If the file grows past ~450 lines, split the tables into `bios7_tables_data.cpp` with externs in the header.

### 5.12 `GetBootProcs` (SWI 0x1D)

**Inputs:** R0, R1, R2 = out-pointers for three function-pointer values BIOS would normally write.
**Behavior:** real BIOS writes boot-function addresses; post-direct-boot, HG/SS doesn't consume them. Stub: write zeros.

```cpp
u32 bios7_get_boot_procs(Arm7State& s, Arm7Bus& b) {
    b.write32(s.r[0], 0);
    b.write32(s.r[1], 0);
    b.write32(s.r[2], 0);
    return 1;
}
```

### 5.13 Decompressor SWIs (0x10–0x15) — stub for slice 3f

```cpp
u32 bios7_decomp_stub(Arm7State& s, Arm7Bus&, u32 n) {
    DS_LOG_WARN("arm7/bios: decompressor SWI 0x%02X deferred to slice 3f", n & 0xFFu);
    return 1;
}
```

One function covers all six decomp SWIs. Slice 3f replaces it with six real implementations and their own dispatch entries.

### 5.14 Invalid SWIs (0x01, 0x02, 0x0A, 0x16–0x19, 0x1E) and 0x00

Caught by the `default` branch of the dispatcher (§4.2). Each warn-logs and returns 1. SoftReset (0x00) is explicit (not in `default`) so it warn-logs with the name instead of "invalid."

---

## 6. Testing strategy

Seven new test binaries. Trivial SWIs share a binary; non-trivial SWIs get their own test.

### 6.1 `arm7_bios_halt_test.cpp`

- **Halt fast-path:** set `state_.halted = true`, call `run_until(10000)`; assert `state_.cycles == 5000` (ARM7 half-rate) and `halted` still true. No instruction executed.
- **Halt wake via IRQ line:** halted state plus `set_irq_line(true)` with `CPSR.I=0`. Call `run_until`. Assert: `halted` cleared, IRQ entry taken, PC = `[0x0380FFFC]`.
- **Halt does not wake with masked IRQ:** `CPSR.I=1`. `run_until` burns cycles, leaves halted.
- **SWI 0x06 Halt:** execute SWI from ARM; after HLE returns, HALTCNT has been written, `halted = true`. PC restored via dispatcher's implicit MOVS.
- **SWI 0x07 Sleep:** same as Halt but HALTCNT = 0xC0.
- **SWI 0x1F CustomHalt:** R2 = 0x80, R2 = 0xC0, R2 = 0x40 (GBA mode warn), R2 = 0x00.
- **SWI 0x03 WaitByLoop:** R0 = 100; assert `state_.cycles` advanced by 400.

### 6.2 `arm7_bios_intrwait_test.cpp`

- **IntrWait condition already met:** write 1 to `[0x0380FFF8]` bit 0; call SWI 0x04 with R0=1, R1=1. Assert: SWI returns, bit 0 cleared in shadow, `halted = false`, IME = 1.
- **IntrWait halts when condition not met:** shadow = 0; SWI 0x04 with R0=1, R1=1. Assert: `halted = true`, R14_svc rewound by 4 (ARM) to the SWI address.
- **IntrWait wakes when shadow bit is ORed in:** continue previous test; externally set shadow bit 0 and raise IRQ line. `run_until`. Assert: halted cleared, IRQ entry, then handler runs, then IntrWait re-executes, sees bit, consumes, returns normally.
- **IntrWait from Thumb:** same as above with Thumb SWI (instr_size = 2).
- **VBlankIntrWait:** SWI 0x05. Assert R0 and R1 set before the tail-call behavior matches bit 0.
- **Discard-old flag R0=0:** shadow has stale bit set, `IntrWait(R0=0, R1=mask)`. Assert: does NOT clear stale bits before waiting — if any stale bit matches, it returns immediately.

### 6.3 `arm7_bios_math_test.cpp`

- **Div positive / positive:** 100 / 7 → R0=14, R1=2, R3=14.
- **Div negative / positive:** -100 / 7 → R0=-14, R1=-2, R3=14.
- **Div positive / negative:** 100 / -7 → R0=-14, R1=2, R3=14.
- **Div negative / negative:** -100 / -7 → R0=14, R1=-2, R3=14.
- **Div by zero:** R0=0, R1=100, R3=0 with warn.
- **Div INT32_MIN / -1:** documents the platform-dependent wrap.
- **Sqrt 0:** 0.
- **Sqrt 1:** 1.
- **Sqrt 4, 9, 16, 25:** 2, 3, 4, 5.
- **Sqrt 10:** 3 (truncated).
- **Sqrt 0xFFFFFFFF:** 0xFFFF.
- **Sqrt 0x40000000:** 0x8000.

### 6.4 `arm7_bios_memcpy_test.cpp`

- **CpuSet 16-bit copy, 4 halfwords:** write pattern to src, call SWI, assert dst matches.
- **CpuSet 16-bit fill, 8 halfwords:** fill with 0xBEEF, assert all 8 halfwords.
- **CpuSet 32-bit copy, 16 words.**
- **CpuSet 32-bit fill, 16 words of 0xDEADBEEF.**
- **CpuSet length 0:** no-op.
- **CpuSet unaligned pointers:** src=0x0200_0001, dst=0x0200_0003 with 16-bit mode → pointers masked to even.
- **CpuFastSet 8 words copy.**
- **CpuFastSet 3 words → rounded to 8.**
- **CpuFastSet fill.**
- **GetCRC16 over a known byte sequence:** assert result matches pre-computed CRC.
- **GetCRC16 with CRC init != 0:** verify init folds in correctly.
- **GetCRC16 over HG/SS cart-header range (if fixture available):** matches expected 0xCF56 per cart. *(Optional — include if cart fixture is trivial; otherwise skip.)*

### 6.5 `arm7_bios_tables_test.cpp`

- **GetSineTable index 0, 16, 32, 48:** assert specific values (sin 0, sin π/2, sin π, sin 3π/2).
- **GetSineTable out-of-range (index 64, 100):** returns 0 with warn.
- **GetPitchTable index 0, 300, 767:** matches melonDS-known values (pre-compute with the formula in a test helper; do not hardcode).
- **GetPitchTable out-of-range:** returns 0.
- **GetVolumeTable index 0, 360, 723:** matches melonDS-copied table values.
- **GetVolumeTable out-of-range:** returns 0.
- **IsDebugger:** R0 = 0.
- **SoundBias R0=0:** SOUNDBIAS stored as 0x000.
- **SoundBias R0=1:** SOUNDBIAS stored as 0x200.
- **GetBootProcs:** three output pointers each receive 0.

### 6.6 `arm7_bios_dispatch_test.cpp`

- **Invalid SWIs warn-return:** 0x01, 0x02, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1E — each SWI returns without change to registers (other than PC/CPSR via the dispatcher's implicit return).
- **Decomp SWIs warn-return:** 0x10–0x15.
- **SoftReset (0x00) warn-returns.**
- **Thumb-dispatched SWI:** THUMB.17 SWI #0x09 (Div) executes correctly — exercises the Thumb SWI path through the dispatcher.
- **ARM-dispatched SWI:** ARM-state `SWI #0x09` executes correctly.
- **PC restoration:** after every SWI in the dispatch test, CPSR and PC are restored to the caller's context.

### 6.7 `arm7_bios_sequence_test.cpp` — capstone

Mini ARM program simulating a game's boot-wait-for-VBlank pattern:

1. Program:
   ```
   main:    MOV R0, #1
            STR R0, [IME_addr]            ; enable IME
            MOV R0, #1
            STR R0, [IE_addr]             ; enable VBlank (bit 0)
            MOV R0, #1                    ; R0 = discard-old flag
            MOV R1, #1                    ; R1 = mask = VBlank
            SWI #0x04                     ; IntrWait(1, 1)
            ; ... post-wait code
            B end

   handler: ; the game's IRQ handler at [0x0380FFFC]
            LDR R0, =(1 << 0)
            STR R0, [IF_addr]             ; ack hardware IF
            STR R0, [0x0380FFF8]           ; OR bit 0 into BIOS shadow
            SUBS PC, R14, #4               ; return from IRQ
   ```
2. Write `&handler` to `[0x0380FFFC]`. Clear `[0x0380FFF8]`.
3. Run `run_until` for N cycles.
4. Assert: IntrWait has halted the CPU (no progress past SWI).
5. Externally call `irq_ctrl_.raise(1 << 0); update_arm7_irq_line();`.
6. Continue run. Assert:
   - Halt wakes.
   - IRQ entry to handler.
   - Handler runs, clears hardware IF, writes shadow, returns.
   - IntrWait re-executes, sees shadow bit, consumes, returns.
   - Program proceeds past the SWI.
7. Final state: `[0x0380FFF8] == 0` (bit consumed), `IF == 0` (ack'd), PC past the SWI, CPSR = original mode.

This test exercises: the full boot-wait pattern, halt fast-path, wake, IRQ entry via slice 3d, game-side shadow-write contract, IntrWait re-check, dispatcher's implicit return, full round-trip back to main.

### 6.8 Total test binary count

End of slice 3d: 43. Slice 3e adds 7 → **50**. Every new test links `ds_core` only, uses `REQUIRE`, runs in milliseconds, no SDL.

---

## 7. Cross-references

- **GBATEK §"BIOS Functions (ARM7)"** — per-SWI input/output and semantic spec. Authoritative.
- **GBATEK §"BIOS GetCRC16 Function"** — CRC polynomial table and bit-processing loop.
- **GBATEK §"DS Memory Control"** — HALTCNT register layout and mode bits.
- **GBATEK §"DS Sound Bias"** — SOUNDBIAS register layout.
- **melonDS `src/ARMInterpreter.cpp`** — reference implementation of every SWI; cross-check CpuSet, GetCRC16, and the volume table verbatim.
- **melonDS `src/NDS.cpp`** — HALTCNT handling on write.
- **Predecessor slice:** `docs/specs/2026-04-16-arm7-core-phase1-slice3d-design.md` — the exception entry / dispatcher wiring we depend on.
- **Parent spec:** `docs/specs/2026-04-12-nds-emulator-design.md` §4.2 (ARM7 interpreter) and §13.3 (Phase 1 milestones).
- **Existing stub:** `src/cpu/arm7/bios/bios7_hle.cpp` (slice 3d warn-only dispatcher) and `src/cpu/arm7/bios/bios7_hle.hpp`.
- **Decompressor SWI deferral:** tracked for slice 3f design doc (not yet written).

---

## 8. Risk and rollback

Low-to-medium risk. All changes are behind SWI boundaries that slice 3d already routes through an HLE dispatcher. The riskiest pieces are the halt fast-path (new `run_until` branch) and the IntrWait PC-rewind.

### Risk 1 — halt fast-path burns past a scheduler event

When the scheduler lands, `run_until` is called with a target that's the next event time. If the halted CPU burns its cycle budget to that target and returns, the scheduler fires the event next, which may raise an IRQ, which wakes the CPU — exactly what we want.

But if a *future* NDS design hands ARM7 the full frame target in one call (instead of slicing by scheduler events), halted ARM7 would burn the entire frame without ever sampling an IRQ that would have fired mid-frame. Slice 3e doesn't hit this because scheduler events don't exist yet; we need to lock in the invariant.

**Mitigation:** add a prominent comment in `run_until` stating "call me with the next-scheduler-event time, not the frame end" — reinforce at the NDS boundary when it lands. The scheduler slice's spec will re-raise this.

**Rollback:** remove the halt fast-path, revert to "step through NOPs until IRQ." Works but wastes cycles; no correctness impact.

### Risk 2 — IntrWait PC rewind assumes ARM vs Thumb correctly

The rewind is by 4 (ARM) or 2 (Thumb), based on `SPSR_svc.T`. Slice 3d's SWI entry saves the pre-SWI CPSR into SPSR_svc, so SPSR.T reflects the caller's state, which is exactly what we need. Test `arm7_bios_intrwait_test.cpp` from Thumb explicitly verifies this.

**Mitigation:** the capstone test (§6.7) covers ARM → Thumb → ARM (if we add a Thumb variant). If ROM bring-up surfaces a rewind miscalculation, inspect the SPSR at the point of rewind.

**Rollback:** switch to Option 2 (blocking-state model) — more invasive.

### Risk 3 — IntrWait deadlock (game-side bug)

If a game's IRQ handler doesn't OR into `[0x0380FFF8]`, IntrWait spins forever: halt → wake → re-check shadow → see 0 → halt → …

**Mitigation:** after N consecutive halt-wake-halt cycles with no shadow change, emit a once-per-run warn with the expected mask and current shadow. N = 16 is arbitrary but high enough to avoid false positives and low enough to surface bugs early.

**Rollback:** remove the warn; the loop still works if the game's handler is correct.

### Risk 4 — Volume table license

We're copying melonDS's hardware-derived volume table into our repo. melonDS is GPLv3, not MIT. **We cannot copy GPLv3 code into our codebase without making the whole project GPL.** Re-check before writing the table.

**Mitigation:** one of:
- (a) Derive the table from GBATEK's text spec (GBATEK attempts to describe the curve but the exact values are per-ROM; this may be viable for slice 3e if the curve is documented sufficiently).
- (b) Consume the table from a published-under-compatible-license source (NanoboyAdvance is MIT; check its SWI handler).
- (c) Generate the table ourselves from the hardware curve once we have a DS hardware harness — blocking and defer.

**Decision requirement before commit 9 (tables commit):** confirm license compatibility of the source we copy from. If none works, defer the volume table SWI until we have a legal source. This is the one open question in the slice.

### Risk 5 — CpuSet bus width correctness

`bus.read16(addr)` and `bus.read32(addr)` go through different paths in the ARM7 bus. A game that CpuSets VRAM at 16-bit width when the source is VRAM relies on both paths behaving identically for the mapped region. Slice 2 / 3a's existing bus tests cover this; the CpuSet-specific test doesn't need to re-verify bus internals, only that we call the correct width.

**Mitigation:** test writes to a sentinel memory range and verifies byte layout after both 16-bit and 32-bit copy; any bus-layer bug shows up as a specific byte-swap.

**Rollback:** none needed — correctness issue would be in the bus, not in CpuSet.

### Risk 6 — SOUNDBIAS future clash with SPU slice

SOUNDBIAS gets a home on `NDS` (as `soundbias_` field) in slice 3e, written from SWI 0x08 and from direct bus writes. The SPU slice will want to *read* SOUNDBIAS from a different subsystem. The field is on `NDS` (correct location — rule 3 says subsystems don't point at each other; NDS-owned state is fine). SPU reads via `NDS&` reference during audio tick. No conflict.

**Mitigation:** none needed; pre-designed compatible.

---

## 9. Slice completion criteria

Slice 3e is complete when:

1. All 50 test binaries build and pass under `ctest --output-on-failure`.
2. Every one of the 21 SWIs in §2.1 marked "YES" is implemented in its family file and wired in the dispatcher switch (§4.2).
3. The 6 decompressor SWIs and SoftReset warn-return via their stubs.
4. The 7 invalid SWI numbers warn-return via the dispatcher's `default` branch.
5. `Arm7State::halted` exists as a boolean, is cleared on `reset()`, and is set by HALTCNT writes of 0x80 / 0xC0 / other non-zero values.
6. `run_until` fast-paths the halted case: burns cycles to target when no IRQ pending, wakes on rising IRQ line with `CPSR.I=0`.
7. `HALTCNT` (0x04000301) is routed through `arm7_io_write8` / `write16` / `write32` and dispatches on the top bits per §4.5.
8. `SOUNDBIAS` (0x04000504) is routed through `arm7_io_write16` / `write32` and stores the low 10 bits into `NDS::soundbias_`.
9. `IntrWait` and `VBlankIntrWait` implement the halt-and-re-check model (§4.8) including the PC-rewind trick for re-execution on wake.
10. The dispatcher's implicit `MOVS PC, R14` (§4.9) restores CPSR and PC from R14_svc / SPSR_svc after every SWI. Thumb and ARM entry both return correctly.
11. `CpuSet` and `CpuFastSet` go through `bus_.read*/write*` at the correct width; `GetCRC16` goes through `bus_.read16`.
12. `bios7_hle.cpp` ≤ 100 lines, `bios7_halt.cpp` ≤ 200 lines, `bios7_math.cpp` ≤ 100 lines, `bios7_memcpy.cpp` ≤ 250 lines, `bios7_tables.cpp` ≤ 500 lines (split `bios7_tables_data.cpp` if exceeded). All within the 500-soft / 800-hard house rule.
13. No new compiler warnings. All new files follow the existing `src/cpu/arm7/` style.
14. Volume table license risk (§8.4) resolved before commit 9: either licensed-compatible source identified, or SWI 0x1C defers to a later slice.
15. The full mandatory pipeline has run for every implementation commit: `/gbatek-check` (done, upfront) → `senior-architect` (done, upfront — this doc) → baseline → implement → `/simplify` → (`ds-architecture-rule-checker` ∥ `gbatek-reviewer`) → `quality-reviewer` → `ctest` → commit.

---

## Appendix A. Commit sequence

**Twelve commits.** Prerequisite hardware lands first (commits 1–3), then the families in dependency order (halts depend on HALTCNT + halted_; math/memcpy/tables are independent), then the capstone.

### Phase A — prerequisite hardware (commits 1–3)

1. `cpu/arm7: halted_ flag on Arm7State + run_until fast-path (burn + wake)`
   - Adds the `bool halted` field; extends `run_until` with the fast-path from §4.3.
   - Test: `arm7_bios_halt_test.cpp` halt/wake-via-line cases (the SWI-triggered cases land in commit 5).
2. `bus/arm7: HALTCNT I/O register (0x04000301) + NDS routing`
   - Adds `IO_HALTCNT`; routes all widths to the dispatch logic in §4.5.
   - Test: add a small block to `arm7_bios_halt_test.cpp` verifying HALTCNT write sets the flag.
3. `bus/arm7: SOUNDBIAS I/O register (0x04000504) + NDS::soundbias_ field`
   - Adds `IO_SOUNDBIAS`; routes 16/32-bit writes to store `low 10 bits`.
   - Test: add a block to `arm7_bios_tables_test.cpp` verifying register storage.

### Phase B — BIOS dispatcher + trivial SWIs (commits 4–5)

4. `cpu/arm7/bios: dispatcher switch + implicit MOVS PC, R14 return + invalid/decomp/SoftReset stubs`
   - Rewrites `bios7_hle.cpp` as the §4.2 switch; adds the dispatcher's implicit return logic from §4.9; routes 0x00 / 0x10–0x15 / invalid to their stubs.
   - Test: `arm7_bios_dispatch_test.cpp` (all stubs, PC restoration, ARM vs Thumb entry).
5. `cpu/arm7/bios: trivial SWIs — IsDebugger, Sqrt, WaitByLoop, GetBootProcs`
   - Implements the four one-liners into their respective family files (tables, math, halt, tables).
   - Test: `arm7_bios_math_test.cpp` (Sqrt), `arm7_bios_tables_test.cpp` (IsDebugger, GetBootProcs), `arm7_bios_halt_test.cpp` (WaitByLoop).

### Phase C — halt family (commits 6–7)

6. `cpu/arm7/bios: Halt + Sleep + CustomHalt SWIs`
   - Implements the three HALTCNT-writing SWIs in `bios7_halt.cpp`.
   - Test: adds to `arm7_bios_halt_test.cpp`.
7. `cpu/arm7/bios: IntrWait + VBlankIntrWait with halt-and-re-check + PC rewind`
   - Implements the two wait SWIs per §4.8.
   - Test: `arm7_bios_intrwait_test.cpp`.

### Phase D — math and memory families (commits 8–9)

8. `cpu/arm7/bios: Div (signed, div-by-zero handling)`
   - Implements Div in `bios7_math.cpp`.
   - Test: adds to `arm7_bios_math_test.cpp`.
9. `cpu/arm7/bios: CpuSet + CpuFastSet + GetCRC16 via bus`
   - Implements the memory family in `bios7_memcpy.cpp`.
   - Test: `arm7_bios_memcpy_test.cpp`.

### Phase E — tables and misc (commits 10–11)

10. `cpu/arm7/bios: GetSineTable + GetPitchTable + GetVolumeTable`
    - Implements the three tables in `bios7_tables.cpp`.
    - **Blocks on §8.4 license resolution.** If VolumeTable can't be sourced compatibly, this commit lands with a placeholder-warn for 0x1C only, and a slice-3f-or-later TODO.
    - Test: `arm7_bios_tables_test.cpp`.
11. `cpu/arm7/bios: SoundBias SWI (one-shot write to SOUNDBIAS)`
    - Implements SoundBias in `bios7_tables.cpp` (grouped with misc SWIs in the tables family file).
    - Test: adds to `arm7_bios_tables_test.cpp`.

### Phase F — capstone (commit 12)

12. `cpu/arm7: slice 3e capstone — IntrWait boot-wait sequence test`
    - No new production code. Adds `arm7_bios_sequence_test.cpp` exercising the full IME/IE/IntrWait/handler/shadow/wake/return flow from §6.7.

**Commit-count check:** 12 commits, all within 10–14 target. Commits 6 and 7 could plausibly merge (both halt-family), kept separate so IntrWait — the hardest SWI in the slice — has a crisp bisect target. Commits 8 and 9 could plausibly split further (Div alone, then CpuSet+CpuFastSet, then GetCRC16) to hit 14 commits; bundled here because they share a family file and testing fixtures. If either grows unexpectedly during implementation, split.

---

## Appendix B. SWI cheat sheet

```
SWI   Name                  Family    R0 in       R1 in       R2 in       Outputs
0x00  SoftReset             halt      —           —           —           warn-stub
0x03  WaitByLoop            halt      count       —           —           (cycles advance)
0x04  IntrWait              halt      discard     mask        —           (halts/returns)
0x05  VBlankIntrWait        halt      —           —           —           (tail-calls 0x04)
0x06  Halt                  halt      —           —           —           (HALTCNT=0x80)
0x07  Sleep                 halt      —           —           —           (HALTCNT=0xC0)
0x08  SoundBias             misc      target      delay       —           (SOUNDBIAS write)
0x09  Div                   math      num         den         —           R0=q, R1=r, R3=|q|
0x0B  CpuSet                memory    src         dst         ctrl        (bus writes)
0x0C  CpuFastSet            memory    src         dst         ctrl        (bus writes)
0x0D  Sqrt                  math      value       —           —           R0=isqrt
0x0E  GetCRC16              memory    crc0        ptr         len         R0=crc, R3=last hw
0x0F  IsDebugger            misc      —           —           —           R0=0
0x10  BitUnPack             decomp    —           —           —           warn-stub (3f)
0x11  LZ77UnCompWrite8      decomp    —           —           —           warn-stub (3f)
0x12  LZ77UnCompRdCb16      decomp    —           —           —           warn-stub (3f)
0x13  HuffUnCompReadByCb    decomp    —           —           —           warn-stub (3f)
0x14  RLUnCompWrite8        decomp    —           —           —           warn-stub (3f)
0x15  RLUnCompReadByCb16    decomp    —           —           —           warn-stub (3f)
0x1A  GetSineTable          tables    index       —           —           R0=sine[i]
0x1B  GetPitchTable         tables    index       —           —           R0=pitch[i]
0x1C  GetVolumeTable        tables    index       —           —           R0=volume[i]
0x1D  GetBootProcs          misc      out0        out1        out2        (zeros written)
0x1F  CustomHalt            halt      —           —           halt_val    (HALTCNT=R2)

Prerequisite hardware:
  HALTCNT   0x04000301  u8   0x80=halt, 0xC0=sleep, sets state.halted
  SOUNDBIAS 0x04000504  u16  low 10 bits stored; no audio effect yet
  halted_   Arm7State bool   run_until fast-paths; cleared on rising IRQ line

HLE flow per SWI:
  1. enter_exception(Swi, ret)       [slice 3d]
  2. arm7_bios_hle_dispatch_swi(...)  [switch to per-family function]
  3. per-family function executes    [this slice]
  4. dispatcher's implicit MOVS PC, R14  [restores caller CPSR, sets PC=R14_svc]

IntrWait re-execution trick:
  rewind R14_svc by 4 (ARM) or 2 (Thumb) based on SPSR_svc.T before halting.
  next wake runs dispatcher implicit return → PC = SWI instruction → re-decode → SWI.
```
