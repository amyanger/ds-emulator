# ARM9 Core (ARMv5TE) — ARMv4T-Subset ARM-State Foundation — Phase 1, Slice 3l Design

**Date:** 2026-05-28
**Slice:** First slice of the ARM9 CPU core (ARM946E-S, ARMv5TE, 66 MHz). This
slice replaces the `Arm9::run_until` stub with a real fetch/decode/execute
ARM-state core covering the **ARMv4T instruction subset** (data processing,
branch, single data transfer, halfword/signed transfer, block transfer,
multiply, PSR transfer, swap) plus SWI/Undefined exception entry, wired into
the scheduler at full ARM9 rate and into the dual-CPU IRQ controller. The
ARMv5TE-only instructions (slice 3m), ARM9 Thumb state (3n), and CP15 + TCM
(3o) are explicitly deferred.
**Status:** draft — not yet implemented. No commits landed.
**Prior slice:** 3k (keypad + lid switch + input-driven IRQs) — landed
through `14d3cac`; 77 CTest binaries green.
**Next slice (proposed):** 3m (ARMv5TE-only ARM instructions: CLZ, BLX,
QADD/QSUB family, signed halfword multiplies, LDRD/STRD, PLD, BKPT). See the
roadmap in §1.

---

## 1. Summary

### Plain-language summary

The Nintendo DS has two processors. We have a complete, tested ARM7 (the
"helper" CPU, ARMv4T, 33 MHz). The ARM9 (the "main" CPU, ARMv5TE, 66 MHz) —
the one that runs almost all of a Pokémon game's actual code — is still a
**stub**: its `run_until` pretends to execute by advancing the cycle counter
and nothing else. Until the ARM9 really executes instructions, no game boots.
This is the single biggest remaining piece of Phase 1.

The ARM9 is *mostly the same machine* as the ARM7 plus extras. It uses the
same 16 registers, the same condition codes, the same barrel shifter, the
same exception model. The differences fall into three buckets:

1. **A handful of new instructions** (`CLZ`, `BLX`, saturating math,
   signed-halfword multiplies, `LDRD`/`STRD`, `PLD`, `BKPT`). These are
   *additive* — they don't change how existing instructions work.
2. **A few subtle behavior changes to instructions that already exist on
   ARM7.** The important ones: loading the program counter from memory now
   switches between ARM and Thumb based on the low bit of the loaded value
   ("interworking"); `LDM`/`STM` handle some malformed register lists
   differently; and `MUL`/`MLA` no longer corrupt the carry flag. These are
   the dangerous ones, because the code *looks* identical to ARM7 but must
   behave differently.
3. **A system-control coprocessor (CP15) and tightly-coupled memory
   (ITCM/DTCM).** This is a whole subsystem of its own and is deferred to
   slice 3o.

Because of bucket 2, we will **not** reuse the ARM7's instruction executors
directly. We will share only the pieces that are provably identical on both
machines — the pure arithmetic/shift/flag helpers — and give the ARM9 its own
executor files with the v5 behavior baked in. This keeps the working ARM7
untouched and makes the v5 differences explicit instead of hidden behind a
shared flag.

This first slice (3l) builds the ARMv4T-compatible subset of the ARM9's
ARM-state instruction set — which is the overwhelming majority of the code a
DS game runs during early boot — and wires it into the emulator so both CPUs
run together under the scheduler. We defer the genuinely new v5 instructions,
Thumb state, and CP15/TCM to later slices so this slice stays bounded and
independently testable with hand-assembled programs (no ROM required).

### The ARM9 core roadmap (context for this slice)

The full ARM9 core is built over four slices, mirroring how the ARM7 core was
built across 3a–3h:

| Slice | Scope | Milestone |
|-------|-------|-----------|
| **3l** *(this slice)* | ALU extraction to `common/`, `Arm9State`, `class Arm9` replacing the stub, NDS full-rate wiring, IRQ line, high-vector reset + exception entry, and the **ARMv4T-subset ARM-state** instruction core | ARM9 executes a hand-assembled ARMv4T program; both cores run together under the scheduler; no ROM |
| **3m** | ARMv5TE-only ARM instructions: CLZ; BLX imm+reg; QADD/QSUB/QDADD/QDSUB + sticky Q flag; SMLAxy/SMULxy/SMLAWy/SMULWy/SMLALxy; LDRD/STRD; PLD (nop); BKPT; MCRR/MRRC + `2`-form coproc (undef/nop) | Each v5 family has its own test; saturation/Q-flag edge cases covered |
| **3n** | ARM9 Thumb state (Thumb.1–.10, mirror of ARM7 3c–3h) plus the v5 Thumb delta: BLX(label) switch-to-ARM | ARM9 runs hand-assembled Thumb; ARM↔Thumb interworking verified |
| **3o** | `class Cp15` + TCM (ITCM/DTCM region regs, enable bits, base/size), Arm9Bus page-table rebuild on CP15 writes, CP15-driven high-vector bit, DTCM IRQ-vector indirection, cache commands as NOPs, Wait-For-Interrupt halt | TCM remap test; CP15 ID reads `0x41059461`; DTCM at default `0x027C0000` |

Slice 3o trips the complexity circuit breaker (it touches the CPU core, the
`Arm9Bus` page tables, and NDS ownership simultaneously) and will get its own
Plan-mode pass before implementation. Slices 3l–3n are CPU-core-local.

### Scope boundary

**In scope (3l):**
- Move `arm7_alu.hpp` → `src/cpu/common/arm_alu.hpp`; re-point ARM7's
  includes. No behavior change.
- `Arm9State` (register file, CPSR, banked registers, mode switching) — a
  copy of `Arm7State`'s banking logic with ARM9 reset defaults.
- `class Arm9 : public CpuCore` replacing the stub: full-rate `run_until`,
  `reset`, IRQ sampling at instruction boundaries, `attach_bus`, `state()`,
  `bus()`.
- NDS wiring: attach the ARM9 bus before reset; push a real IRQ line from
  `update_arm9_irq_signals()`; delete the now-redundant cached-line member.
- ARM-state decode dispatcher (switch on bits[27:25]) and the ARMv4T-subset
  family executors, each in its own ARM9-owned file with **v5 semantics baked
  in** (see §5.2 for the divergences).
- SWI + Undefined-instruction exception entry, with the exception vector base
  hardcoded **high** (`0xFFFF0000`) and IRQ entry routed through the default
  DTCM indirection address as a named `constexpr`.

**Out of scope (deferred):**
- All ARMv5TE-only instructions → 3m.
- ARM9 Thumb state → 3n.
- CP15 coprocessor, ITCM/DTCM, caches, Wait-For-Interrupt halt → 3o.
- Cycle-accurate timing / interlocks (everything is 1 cycle this slice).
- `save_state`/`load_state` (project-wide serialization debt, deferred to the
  Phase 1 serialization pass; `reset()` is implemented this slice).
- Direct boot / cart header parsing (separate, later work — this slice's
  `reset()` establishes the post-reset high-vector state; direct boot will
  later overwrite PC with the cart entry point).

---

## 2. Goals

1. **Make the ARM9 actually execute.** Replace the stub with a real
   fetch/decode/execute loop for the ARMv4T instruction subset in ARM state.
2. **Get the cycle model right.** ARM9 runs at scheduler rate (`now` is in
   ARM9 cycles); unlike ARM7 it does **not** halve its target. This is the
   number-one wiring difference and getting it wrong desynchronizes the two
   cores.
3. **Bake in the v5 behavioral divergences** for the shared instructions so
   we never silently inherit ARM7's v4 semantics: PC-load interworking,
   `LDM`/`STM` invalid-rlist rules, `MUL`/`MLA` carry preservation, the live
   Q flag (CPSR bit 27).
4. **Share only what is provably identical.** Extract the pure ALU helpers to
   `src/cpu/common/` and reuse them; duplicate the executor scaffolding.
5. **Leave the green ARM7 untouched** apart from the one-line-per-file include
   re-point that the ALU move requires.
6. **Wire both cores together under the scheduler** and prove it with an
   integration test.
7. **Establish ARM9 exception entry** (SWI, Undefined, IRQ) with the correct
   high-vector base, structured so slice 3o can swap the hardcoded constants
   for CP15 reads with a clean diff.

### 2.1 Instruction coverage matrix (post-slice)

| Family | ARM7 (v4T) | ARM9 after 3l | ARM9 after 3m |
|--------|:----------:|:-------------:|:-------------:|
| Data processing (DP) | ✅ | ✅ | ✅ |
| Branch (B, BL) | ✅ | ✅ | ✅ |
| BX | ✅ | ✅ | ✅ |
| Single data transfer (LDR/STR) | ✅ (v4) | ✅ (**v5 PC interwork**) | ✅ |
| Halfword/signed transfer | ✅ | ✅ | ✅ |
| Block transfer (LDM/STM) | ✅ (v4) | ✅ (**v5 rules**) | ✅ |
| Multiply (MUL/MLA/long) | ✅ (v4) | ✅ (**C preserved**) | ✅ |
| PSR transfer (MRS/MSR) | ✅ | ✅ (**Q live**) | ✅ |
| Swap (SWP/SWPB) | ✅ | ✅ | ✅ |
| SWI / Undefined entry | ✅ | ✅ (**high vectors**) | ✅ |
| CLZ, BLX, QADD…, SMLAxy…, LDRD/STRD, PLD, BKPT | — | — | ✅ |
| Thumb state | ✅ | — (3n) | — (3n) |

---

## 3. Non-goals

- **No CP15, no TCM, no caches.** The ARM9 in this slice reaches main RAM and
  the high-vector region through `Arm9Bus` exactly as it exists today. TCM
  shadowing is 3o.
- **No cycle accuracy.** Every instruction costs 1 ARM9 cycle. GBATEK's
  S/N/I formulas and the 66/33/8 MHz bus-throttle discussion are recorded for
  later but not modeled. Interlocks (a v5 concept) are not modeled.
- **No halt path.** ARM9 halts via the CP15 Wait-For-Interrupt command, which
  does not exist until 3o. Until then the ARM9 has no `halted_` branch; in an
  idle spin it simply burns its scheduler quantum, which is harmless during
  early boot. Tracked as a known gap (§8.1).
- **No Thumb.** ARM-state only this slice. `step_thumb` may exist as a stub
  that asserts/`unreachable` if the T bit is ever set, since nothing in 3l
  sets it (BX/BLX-to-Thumb is exercised in 3n).
- **No direct boot.** `reset()` establishes architectural post-reset state;
  loading a cart and jumping to its entry point is separate later work.
- **No save states.** `reset()` only, consistent with the rest of the
  codebase at this phase.

---

## 4. Architecture

### 4.1 Reuse strategy — what is shared and what is duplicated

The pure ALU layer is the **only** genuinely architecture-neutral logic, and
it is ~100% of the truly common code. Everything else either differs between
v4 and v5 or is cheap scaffolding that is clearer duplicated than coupled.

**Shared (moved to `src/cpu/common/arm_alu.hpp`):**
- `ShiftType`, `DpOp` enums; `ShifterResult`, `AddResult` structs.
- `barrel_shift_imm`, `barrel_shift_reg`, `rotated_imm`.
- `eval_condition`.
- `set_nz`, `set_c`, `set_v`.
- `adc`, `sbc`.

These are value-in/value-out pure functions with zero dependency on any CPU
state type. The barrel shifter and adder are bit-identical on ARMv4T and
ARMv5TE. Moving the header and re-pointing ARM7's `#include` is covered by the
existing `barrel_shift_test`, `condition_test`, and `flags`/ALU tests — near
zero regression risk, and it makes the shared-ness explicit rather than ARM9
reaching into an `arm7_`-prefixed file (which would violate the spirit of "no
cross-subsystem includes").

**Duplicated (ARM9 gets its own files):**
- The decode dispatcher and every family executor.

**Rejected alternatives** (documented so we don't relitigate):
- *Templating the executors* on `<State, ArchPolicy>` would force the bodies
  into headers (fighting the one-component-per-file rule and 500/800-line
  caps), couple the two cores so any ARM9 change risks destabilizing ARM7, and
  turn the v4/v5 divergences into `if constexpr` branches — two bodies wearing
  a trenchcoat. This is exactly the over-engineering the house rules forbid.
- *Runtime arch-flag branches* (`if (is_v5)`) put a test on a compile-time
  constant in the hottest loop in the emulator and still share a translation
  unit with ARM7. Worse on every axis.

The identical parts of the executors (e.g. the `AND`/`EOR`/`ORR` DP cases)
cost a few dozen duplicated lines per file; coupling them to save that costs
architectural clarity and the ARM7's stability. Duplication is the correct
call here.

**State structs stay independent — no shared base, no CRTP.** Because the
executors are not shared, no code needs to operate polymorphically over "some
ARM state," so a base class buys nothing and would reintroduce the
virtual-inheritance-in-hardware-components smell. `Arm9State` copies
`Arm7State`'s Mode enum, banking logic (`switch_mode`, banked load/store,
`spsr_slot`) verbatim as its starting point, then changes only the reset
defaults and adds the live Q flag.

### 4.2 File layout

New files under `src/cpu/common/`:

```
src/cpu/common/
  arm_alu.hpp          Moved from arm7/arm7_alu.hpp. Pure ALU helpers shared
                       by both cores. (refactor commit, no behavior change)
```

New files under `src/cpu/arm9/` (mirroring `src/cpu/arm7/`):

```
src/cpu/arm9/
  arm9.hpp             class Arm9 : public CpuCore (grown from the stub)
  arm9.cpp             run_until (full rate), reset, step_arm, IRQ sample
  arm9_state.hpp       struct Arm9State (banking copied from Arm7State;
                       ARM9 reset defaults; live Q flag)
  arm9_decode.cpp      ARM-state fetch/decode dispatcher (bits[27:25] switch)
  arm9_decode_internal.hpp   ARM9 executor signatures (take Arm9State&)
  arm9_decode_000.cpp  bits[27:25]==000 sub-dispatch (BX, multiply, halfword,
                       swap, PSR)
  arm9_dp.cpp          Data-processing executor
  arm9_branch.cpp      B, BL
  arm9_loadstore.cpp   LDR/STR (v5 PC-load interworking)
  arm9_halfword.cpp    LDRH/LDRSB/LDRSH/STRH
  arm9_multiply.cpp    MUL/MLA/UMULL/UMLAL/SMULL/SMLAL (v5 C preserved)
  arm9_block.cpp       LDM/STM (v5 interworking + v5 invalid-rlist rules)
  arm9_swap.cpp        SWP/SWPB
  arm9_psr.cpp         MRS/MSR (Q bit live)
  arm9_exception.hpp   ARM9 exception vectors/types (high-vector base)
  arm9_exception.cpp   enter_exception, SWI/UND/IRQ entry
```

Existing `arm9.hpp`/`arm9.cpp` are grown in place (they already exist as the
stub). Every new `.cpp` is expected well under the 500-line soft cap; the
largest ARM7 analogue is `arm7_block.cpp` at 308 lines.

### 4.3 Class design — `class Arm9`

Mirrors `class Arm7` (`src/cpu/arm7/arm7.hpp`) with the full-rate divergence.
Sketch:

```cpp
class Arm9 : public CpuCore {
public:
    void attach_bus(Arm9Bus& bus) { bus_ = &bus; }

    void run_until(Cycle arm9_target) override;   // NO halving — full rate
    void reset() override;

    void set_irq_line(bool level) { irq_line_ = level; }
    bool irq_line() const { return irq_line_; }

    // Exception entry hooks for tests (mirror ARM7)
    void raise_undefined(u32 instr_addr);

    Arm9State& state() { return state_; }
    const Arm9State& state() const { return state_; }
    Arm9Bus& bus() { return *bus_; }

    void step_one_instruction();   // used by tests / future BIOS-HLE

private:
    void step_arm();
    void step_thumb();   // 3l: stub — unreachable unless T set (set in 3n)

    Arm9State state_{};
    Arm9Bus* bus_ = nullptr;
    bool irq_line_ = false;
    // NOTE (3o): halted_/halt_wake_pending_ added with CP15 Wait-For-Interrupt
};
```

Key contrast with ARM7:

```cpp
// ARM7 (existing): halves the target because now is ARM9 cycles
void Arm7::run_until(Cycle arm9_target) {
    const Cycle arm7_target = arm9_target / 2;
    while (state_.cycles < arm7_target) { ... }
}

// ARM9 (this slice): runs at the scheduler rate directly
void Arm9::run_until(Cycle arm9_target) {
    while (state_.cycles < arm9_target) {
        if (irq_line_ && !(state_.cpsr & (1u << 7))) {   // I bit clear
            arm9_enter_irq(*this);
        }
        step_arm();   // bumps state_.cycles by the instruction's cost (1)
    }
}
```

### 4.4 State ownership and dependency graph

`NDS` already owns `Arm9 cpu9_` and `Arm9Bus arm9_bus_`
(`src/nds.hpp:117,132`). No new top-level subsystem is introduced this slice.
The dependency edges are unchanged and remain a tree:

```
NDS
 ├── Arm9 cpu9_         ── holds Arm9Bus* (set via attach_bus, like ARM7)
 │     └── Arm9State    (owned, value member)
 ├── Arm9Bus arm9_bus_  ── holds NDS& + backing-store pointers (existing)
 ├── Arm7 cpu7_         (unchanged)
 └── ... (irq9_, irq7_, ipc, rtc, keypad, ... unchanged)
```

CP15 (slice 3o) will be owned **by `Arm9`** (intra-core composition, the way
`Arm9` owns `Arm9State`), not as an NDS-level peer — so its later
page-table-rebuild calls go `cp15 → Arm9 → Arm9.bus()` without any subsystem
holding a pointer to another subsystem. Flagged here so 3l's structure
anticipates it; nothing about CP15 is built this slice.

### 4.5 NDS wiring and cycle model

Three touch points in `NDS`, all anticipated by existing comments:

1. **Constructor** (`src/nds.cpp:~22`): add `cpu9_.attach_bus(arm9_bus_);`
   alongside the existing `cpu7_.attach_bus(arm7_bus_);`, before `reset()`.
   Same construction-order contract (`bus_` must be non-null at reset).
2. **`run_frame` loop** (`src/nds.cpp:~64`): already calls
   `cpu9_.run_until(next); cpu7_.run_until(next);`. **No change** — the cores
   self-pace. The ARM9 now actually executes instead of jumping its counter.
3. **`update_arm9_irq_signals()`** (`src/nds.cpp:~361`): currently only writes
   `arm9_irq_line_cached_` because the stub had no `set_irq_line`. Change to
   `cpu9_.set_irq_line(irq9_.line());` and delete the cached member +
   `arm9_irq_line_cached()` accessor (`src/nds.hpp:~75`). The ~24 existing
   call sites that invoke `update_arm9_irq_signals()` after IE/IF/IME writes
   continue to work unchanged — they now push a real line.

**Cycle model:** `now` is ARM9 cycles. `Arm9State::cycles` counts ARM9 cycles
1:1 with the scheduler. ARM9 `run_until(arm9_target)` loops
`while (state_.cycles < arm9_target)`. ARM7 keeps its internal `/2`. The
scheduler and `run_frame` are untouched.

### 4.6 Decode dispatch

Mirror ARM7's structure exactly (`arm7_decode.cpp`): `step_arm()` fetches the
32-bit word at PC via `bus()`, advances PC, sets `r[15] = instr_addr + 8`
(pipeline read-ahead; `+12` only for the shift-by-register DP operand form,
identical to ARM7), evaluates the condition field, and switches on
bits[27:25]:

```
000 → dispatch_000_space()   (BX, multiply, halfword, swap, PSR; falls
                              through to DP for the operand form)
001 → dispatch_dp()          (immediate-operand DP)
010/011 → dispatch_single_data_transfer()   (LDR/STR)
100 → dispatch_block()       (LDM/STM)
101 → dispatch_branch()      (B, BL)
110/111 → dispatch_coproc_or_swi()   (SWI; else Undefined — coproc/CP15 is 3o)
```

In 3l the `110/111` space recognizes only `SWI`; any coprocessor encoding
(MCR/MRC etc.) takes the **Undefined-instruction** path (CP15 lands in 3o,
the v5-only coproc forms in 3m). The `1111` (NV) condition slot, which v5
repurposes for BLX-imm and PLD, is also Undefined in 3l and handled in 3m —
this slice treats `cond == 1111` as "condition never true / undefined" per
v4, which is safe because no 3l test emits those encodings.

### 4.7 Exception entry and high vectors

Mirror `arm7_exception.{hpp,cpp}` (mode switch, SPSR save, R14 pipeline
offset, vector jump) with two ARM9 differences:

1. **Vector base is high.** ARM9 exceptions live at `0xFFFF0000 + offset`
   (vs ARM7's `0x00000000`). DS firmware always sets CP15 control bit 13, so
   the base is high for the entire life of every DS program. 3l hardcodes it
   as a named `constexpr kArm9VectorBase = 0xFFFF0000u;`. Slice 3o swaps that
   constant for `cp15_.vector_base()` — a clean one-line diff, not a new code
   path.
2. **IRQ handler indirection.** ARM9's IRQ entry reads the handler pointer
   from `[DTCM_base + 0x3FFC]` (ARM7 uses `[0x0380FFFC]`). DTCM does not exist
   until 3o, so 3l uses the **default DTCM base** as a named constant:
   `constexpr kArm9DtcmBaseDefault = 0x027C0000u;` →
   handler pointer at `kArm9DtcmBaseDefault + 0x3FFC`, IRQ check bits at
   `+0x3FF8`. This keeps the IRQ path structurally complete; 3o makes the base
   register-driven.

Vector offsets and priority are the standard ARM table (Reset +00, Undef +04,
SWI +08, Prefetch Abort +0C, Data Abort +10, IRQ +18, FIQ +1C). 3l implements
SWI, Undefined, and IRQ entry; FIQ/abort entry hooks mirror ARM7's
test-facing `raise_*` methods if cheap, otherwise deferred (no 3l path raises
them).

---

## 5. Hardware details

The ARMv4T-subset semantics (DP operations, barrel shifter edge cases,
addressing modes, halfword sign-extension, block transfer ordering, multiply
operand widths, MRS/MSR field masks, swap atomicity) are **identical** to the
ARM7 and are fully documented in the ARM7 slice specs (3a, 3b1–3b3). This
section documents only the **v5 divergences** baked into the ARM9 executors
and the reset/exception state. Provenance for every fact is in Appendix B.

### 5.1 PC-load interworking (LDR to PC, LDM with PC)

> GBATEK, *Memory: Single Data Transfer* and *Block Data Transfer*.

- **ARMv4 (ARM7):** `LDR PC,<op>` / `LDM` with R15 in the list leaves
  `CPSR.T` unchanged; PC is masked `& ~3`.
- **ARMv5 (ARM9):** the loaded value's **bit 0 sets `CPSR.T`** (1 → switch to
  Thumb), and `PC = value & ~1`.

ARM9's `arm9_loadstore.cpp` (LDR→PC) and `arm9_block.cpp` (LDM→PC) bake this
in. This is the highest-risk divergence (§8.1 #1): the code looks like ARM7's
but must set T from bit 0. Even though Thumb execution itself lands in 3n, the
interworking *write* (setting T and masking PC) is implemented and tested in
3l so the semantics are correct from day one.

### 5.2 LDM/STM invalid-rlist edge cases

> GBATEK, *Block Data Transfer — Strange Effects on Invalid Rlist's*.

- **Empty rlist:** ARMv4 transfers R15; **ARMv5 does not** transfer R15. Both
  adjust the base by ±0x40.
- **STM with base in rlist:** ARMv4 stores OLD base if Rb is the first entry
  else NEW base; **ARMv5 always stores OLD base.**
- **LDM with base in rlist:** ARMv4 = no writeback; **ARMv5 = writeback if Rb
  is the only register, or not the last register, in the list.**

ARM9's `arm9_block.cpp` implements the v5 column. Reusing ARM7's
`execute_block_transfer` (which bakes in the v4 column,
`arm7_decode_internal.hpp:136-155`) would silently import v4 semantics — this
is the concrete reason the executors are not shared.

### 5.3 Multiply C/V flag preservation

> GBATEK, *Multiply and Multiply-Accumulate*, *Multiply Long*.

- **ARMv4 (ARM7):** `MUL`/`MLA`/long multiplies with `S=1` **corrupt** the C
  flag (and possibly V for long — GBATEK marks v4 V as `???`).
- **ARMv5 (ARM9):** C is **not affected** (and V is not affected for long
  multiplies).

ARM9's `arm9_multiply.cpp` sets only N and Z on `S=1`, leaving C and V
untouched.

### 5.4 The Q flag (CPSR bit 27) becomes live

> GBATEK, *PSR Transfer*; *Special ARM9 Instructions*.

On ARM7, CPSR bit 27 is reserved. On ARM9 it is the **sticky saturation (Q)
flag**, set by the saturating instructions (3m) and the accumulating signed
multiplies (3m), and cleared **only** by an explicit `MSR`. In 3l the only
interaction is in `arm9_psr.cpp`: `MRS` must expose bit 27 and `MSR` (with the
flags field selected) must be able to read/write it. No 3l instruction *sets*
Q (the producers are all v5-only, slice 3m), but the PSR path must treat
bit 27 as a live, software-visible bit rather than masking it off.

### 5.5 Reset state

> GBATEK, *ARM CPU Exceptions*; ARM946E-S reset behavior.

`Arm9State::reset()` (mirroring `Arm7State::reset`):

| Field | Value | Note |
|-------|-------|------|
| `cpsr` | `0x000000D3` | Supervisor mode, I=1, F=1, T=0 (same as ARM7) |
| `pc` | `0xFFFF0000` | High reset vector. Direct boot later overwrites with cart ARM9 entry |
| Q flag (bit 27) | 0 | Cleared |
| banked regs | established via `load_banked_registers(Mode::Supervisor)` | invariant setup |
| `cycles` | 0 | |

`reset()` also asserts `bus_ != nullptr` (construction-order contract copied
from `arm7.cpp`).

### 5.6 IRQ entry indirection address

> GBATEK, *NDS9 Memory Map* / *ARM CPU Exceptions*.

ARM9 IRQ entry: handler pointer at `[0x027C0000 + 0x3FFC]` (default DTCM
base), check bits at `+0x3FF8`. Hardcoded as named `constexpr` this slice;
register-driven in 3o.

### 5.7 What stays identical to ARM7 (no divergence)

For the record, these are confirmed **unchanged** between v4T and v5TE and so
the ARM9 implementation matches the documented ARM7 behavior exactly:
- `BX` (including `BX R15` acting as `BX $+8`).
- Pipeline read-ahead: R15 = instr+8 (ARM), +12 for shift-by-register, +4
  (Thumb).
- `MSR` cannot change the T bit (use BX for state switch).
- DP barrel-shifter carry edge cases (LSL #0, LSR/ASR #0→#32, ROR #0→RRX).
- Halfword/signed-byte sign extension and addressing.
- Swap atomicity.

---

## 6. Testing strategy

A new fixture `tests/support/arm9_step.hpp` mirrors `arm7_step.hpp`: write a
hand-assembled instruction to `nds.arm9_bus()`, set `pc` and `r[15]=pc+8`,
step, assert on `nds.cpu9().state()`. Tests construct a bare `NDS`, link
`ds_core` only, use `REQUIRE` (Release-safe), no SDL. One test binary per
family, registered via `add_ds_unit_test`.

Because the ARMv4T-subset semantics are already proven in ARM7's tests, the
ARM9 tests focus on (a) a representative smoke test per family proving the
ARM9 executor works at all, and (b) **targeted tests for every v5
divergence** — these are the ones that catch silent v4-inheritance bugs.

| Test binary | Covers | Key divergence assertions |
|-------------|--------|---------------------------|
| `arm9_state_test.cpp` | reset defaults, mode banking, Q-bit storage | PC=0xFFFF0000 after reset; bit 27 round-trips |
| `arm9_dp_test.cpp` | DP smoke (MOV/ADD/SUB/CMP/flags) | matches ARM7 DP behavior |
| `arm9_branch_test.cpp` | B, BL | LR = instr+4; PC = target |
| `arm9_loadstore_test.cpp` | LDR/STR smoke | `ldr_pc_sets_thumb_from_bit0`; PC masked `& ~1` |
| `arm9_halfword_test.cpp` | LDRH/LDRSB/LDRSH/STRH smoke | sign extension |
| `arm9_block_test.cpp` | LDM/STM smoke | `ldm_pc_interworking`; `empty_rlist_v5_no_r15_transfer`; `stm_base_in_list_stores_old_base`; LDM writeback-with-base v5 rule |
| `arm9_multiply_test.cpp` | MUL/MLA/long smoke | `muls_preserves_c_flag` (and V for long) |
| `arm9_psr_test.cpp` | MRS/MSR | Q (bit 27) readable/writable; T-bit not changeable |
| `arm9_swap_test.cpp` | SWP/SWPB | atomic exchange |
| `arm9_exception_test.cpp` | SWI/UND/IRQ entry | `vectors_are_high` (0xFFFF0000+off); `irq_reads_handler_from_dtcm_indirection`; SPSR/mode/R14 correct |
| `arm9_integration_test.cpp` | both cores under scheduler | a hand-assembled ARMv4T program runs to completion on ARM9 while ARM7 also runs; cycle accounting (ARM9 = 2× ARM7 progress) holds |

The ALU-move commit is covered by the **existing** ARM7 ALU tests (they must
stay green after the include re-point — that is the regression check).

### What stays green from prior slices

All 77 existing CTest binaries must remain green. The ALU move is value-in/
value-out and covered by existing tests; the NDS wiring change replaces a
cached IRQ line with a live push and must not disturb the ARM7 IRQ tests, the
IPC/RTC/keypad IRQ tests, or `nds_integration_test`.

---

## 7. Cross-references

- **ARM7 core specs** (the ARMv4T-subset semantics this slice ports):
  `docs/specs/2026-04-14-arm7-core-phase1-slice3b1.md` and siblings;
  `docs/plans/2026-04-14-arm7-core-phase1-slice3b*.md`.
- **Design spec §13** (`docs/specs/2026-04-12-nds-emulator-design.md:1046`):
  Phase 1 deliverables and milestone.
- **ARM7 implementation** mirrored by this slice: `src/cpu/arm7/arm7.hpp`,
  `arm7.cpp`, `arm7_state.hpp`, `arm7_decode*.cpp`, `arm7_exception.*`,
  `arm7_alu.hpp` (the move source).
- **NDS wiring points:** `src/nds.cpp` (constructor ~22, run loop ~64,
  `update_arm9_irq_signals` ~361); `src/nds.hpp` (cached-line member ~75,
  ownership ~117/132).
- **GBATEK** primary reference: https://problemkaputt.de/gbatek.htm —
  sections cited per fact in Appendix B.

---

## 8. Risk and rollback

### 8.1 Highest-risk pitfalls

1. **v5 PC-load interworking silently copied as v4.** `LDR`/`LDM` to PC must
   set `CPSR.T = value & 1` and mask `PC = value & ~1`. The bug is invisible
   until a game uses a PC-load return that should switch to Thumb, then it
   runs ARM garbage far from the change. *Mitigation:* ARM9 owns its own
   load/store and block executors (the reason executors are not shared);
   dedicated tests `ldr_pc_sets_thumb_from_bit0`, `ldm_pc_interworking`.
2. **MUL/MLA corrupting C.** ARM7's path corrupts C on `S=1`; ARM9 must
   preserve it. *Mitigation:* independent `arm9_multiply.cpp`; test
   `muls_preserves_c_flag`.
3. **LDM/STM invalid-rlist v4-vs-v5.** Empty rlist and base-in-list rules
   differ. *Mitigation:* §5.2 spells out the v5 column; dedicated tests.
4. **Cycle model: forgetting ARM9 runs at full rate.** If ARM9 accidentally
   halves like ARM7, the two cores desync and every timing-dependent boot
   path drifts. *Mitigation:* §4.5 makes the no-`/2` explicit;
   `arm9_integration_test` asserts ARM9 advances 2× ARM7 over the same window.
5. **High-vector reset / DTCM indirection wrong before CP15 exists.** Wrong
   vector base (low instead of high) or skipped DTCM indirection sends every
   IRQ into garbage, surfacing only when interrupts fire. *Mitigation:* 3l
   hardcodes high vectors and the default-DTCM indirection as named
   `constexpr` (so 3o swaps cleanly); tests `vectors_are_high`,
   `irq_reads_handler_from_dtcm_indirection`.
6. **Known gap — no halt path.** ARM9 has no Wait-For-Interrupt halt until 3o;
   it busy-spins in idle loops, burning its quantum. Harmless during early
   boot (no real game idles the ARM9 in 3l-reachable code) but must be closed
   in 3o. Tracked, not a 3l blocker.

### 8.2 Rollback strategy

Each commit is independently revertible. The riskiest commit (the ALU move +
ARM7 include re-point) is first and is purely mechanical; if it breaks an ARM7
test, revert that single commit and the tree is back to 77-green with the
stub intact. The NDS wiring commit (replacing the cached IRQ line) is the only
change to existing ARM-shared code paths; if it regresses an IRQ test, revert
it and the ARM9 falls back to the stub (the family-executor commits before it
are inert until wiring lands — order the wiring commit so executors exist but
are only reachable once `run_until` is real).

### 8.3 What this slice does NOT break

- ARM7 behavior: unchanged. The ALU move is covered by existing tests; no
  ARM7 executor, decode, or exception code is edited.
- Scheduler, buses, IPC, RTC, keypad, IRQ controllers: untouched except the
  one `update_arm9_irq_signals()` body change (cached write → live push),
  which is behavior-preserving for every existing IRQ test.

---

## 9. Slice completion criteria

- [ ] `arm7_alu.hpp` moved to `src/cpu/common/arm_alu.hpp`; ARM7 includes
      re-pointed; all prior ARM7 ALU tests green.
- [ ] `Arm9State` implemented with ARM9 reset defaults (PC=0xFFFF0000,
      CPSR=0xD3) and live Q flag; `arm9_state_test` green.
- [ ] `class Arm9` replaces the stub: full-rate `run_until`, `reset`, IRQ
      sampling, `attach_bus`, `state()`, `bus()`.
- [ ] NDS wires the ARM9 bus before reset; `update_arm9_irq_signals()` pushes
      a live line; cached-line member deleted.
- [ ] ARMv4T-subset ARM-state families implemented in ARM9-owned files (DP,
      branch, LDR/STR, halfword, block, multiply, PSR, swap) with the §5
      divergences baked in.
- [ ] SWI + Undefined + IRQ exception entry with high-vector base and
      default-DTCM IRQ indirection (named `constexpr`).
- [ ] Per-family tests + `arm9_exception_test` + `arm9_integration_test`
      green.
- [ ] All 77 prior CTest binaries still green; new binaries registered.
- [ ] `/simplify` run on the diff; `ds-architecture-rule-checker` and
      `gbatek-reviewer` clean; `quality-reviewer` clean.

---

## Appendix A. Commit sequence

Each commit is one feature with its test, message format
`cpu/arm9: <specific feature>` (the ALU move is `cpu/common:`).

1. **`cpu/common: extract pure ALU helpers to common/arm_alu.hpp`** — move
   header, re-point ARM7 includes. No behavior change. Existing ARM7 ALU
   tests are the regression check.
2. **`cpu/arm9: Arm9State register file + banking + ARM9 reset defaults`** —
   copy banking from `Arm7State`; PC=0xFFFF0000, CPSR=0xD3, live Q.
   `arm9_state_test`.
3. **`cpu/arm9: class Arm9 skeleton — full-rate run_until + reset + IRQ
   sample`** — dispatch to a stubbed `step_arm`. (Inert until wiring.)
4. **`cpu/arm9: wire ARM9 into NDS — attach bus, live IRQ line, drop cache`** —
   the three NDS touch points. ARM9 now executes (stubbed decode).
5. **`cpu/arm9: ARM-state decode dispatcher (bits[27:25] switch)`** — routes
   to family dispatchers; SWI vs Undefined in the 110/111 space.
6. **`cpu/arm9: data-processing executor + tests`** — `arm9_dp_test`.
7. **`cpu/arm9: branch (B, BL) + tests`** — `arm9_branch_test`.
8. **`cpu/arm9: single data transfer (LDR/STR) + v5 PC interwork + tests`** —
   `arm9_loadstore_test` incl. `ldr_pc_sets_thumb_from_bit0`.
9. **`cpu/arm9: halfword/signed data transfer + tests`** —
   `arm9_halfword_test`.
10. **`cpu/arm9: block transfer (LDM/STM) + v5 interwork + invalid-rlist +
    tests`** — `arm9_block_test` incl. the v5 edge-case assertions.
11. **`cpu/arm9: multiply family + v5 C/V preservation + tests`** —
    `arm9_multiply_test` incl. `muls_preserves_c_flag`.
12. **`cpu/arm9: PSR transfer (MRS/MSR) + live Q bit + tests`** —
    `arm9_psr_test`.
13. **`cpu/arm9: swap (SWP/SWPB) + tests`** — `arm9_swap_test`.
14. **`cpu/arm9: exception entry (SWI/UND/IRQ) + high vectors + tests`** —
    `arm9_exception_test`.
15. **`cpu/arm9: end-to-end dual-core integration test`** —
    `arm9_integration_test`; no production change (or fold small fixes back).

*(Commit 4 makes the ARM9 live with a stubbed decoder; commits 5–14 fill in
families one at a time. If commit count proves unwieldy in practice, the
natural split point is after commit 9 — DP/branch/load-store/halfword form a
"can execute simple straight-line code" milestone — but the slice is scoped as
one unit because each commit is a port of proven ARM7 logic plus a targeted
divergence test, not from-scratch design.)*

---

## Appendix B. Provenance audit

All hardware facts cross-referenced against GBATEK
(https://problemkaputt.de/gbatek.htm), fetched 2026-05-28 via the
`gbatek-check` skill. The ARMv4T-subset semantics being ported are additionally
documented in the ARM7 slice specs (3a–3b3).

- **ARM9 = ARMv5TE / ARM946 rev1**, CP15 Main ID `0x41059461`: GBATEK *ARM
  CP15 ID Codes*.
- **PC-load interworking** (LDR/LDM to PC sets T from bit 0 on v5): GBATEK
  *Memory: Single Data Transfer* and *Block Data Transfer*.
- **LDM/STM invalid-rlist v4-vs-v5 rules**: GBATEK *Block Data Transfer —
  Strange Effects on Invalid Rlist's*.
- **MUL/MLA C preserved on v5** (corrupted on v4); long-multiply V not
  affected on v5: GBATEK *Multiply and Multiply-Accumulate*, *Multiply Long*.
  GBATEK marks the v4 V-corruption `???`; v5 "not affected" is stated
  confidently.
- **Q flag = CPSR bit 27, sticky, MSR-clear-only**: GBATEK *PSR Transfer* and
  *Special ARM9 Instructions (CLZ, QADD/QSUB)*.
- **Pipeline read-ahead identical to v4** (PC+8 / +12 shift-by-reg / +4
  Thumb): GBATEK *R15 Register (PC)*, *Data Processing — Using R15*, *ARM CPU
  Exceptions* ("an identical ARM-style offset is generated").
- **High exception vectors (0xFFFF0000) via CP15 control bit 13; ARM7 fixed
  low (0x00000000)**: GBATEK *ARM CP15 Control Register*, *ARM CPU
  Exceptions*.
- **ARM9 IRQ handler indirection at [DTCM+0x3FFC], check bits at +0x3FF8;
  default DTCM base 0x027C0000**: GBATEK *NDS9 Memory Map*, *DS Memory Control
  - Cache and TCM*.
- **ARM9 66 MHz / ARM7 33 MHz (scheduler `now/2` for ARM7 is correct)**:
  GBATEK *The two Processors*.
- **CP15 / TCM facts** (deferred to 3o, recorded for the roadmap): GBATEK *ARM
  CP15 Overview / Control Register / TCM*, *DS Memory Control - Cache and
  TCM*.
- No BIOS dump, no firmware dump, no proprietary data, no copyrighted code
  referenced by this design.
