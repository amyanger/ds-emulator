# ARM7 Core — Phase 1, Slice 3d Design

**Date:** 2026-04-16
**Slice:** ARM7 exceptions (IRQ line sampling, SWI entry, undefined-instruction entry, prefetch/data abort entry, IME/IE/IF registers)
**Status:** proposed
**Prior slice:** 3c (Thumb) — landed as `cb22763`, 31 CTest binaries green
**Next slice:** TBD (candidates: ARM7 BIOS HLE for the SWI jump table; IRQ sources — timers / DMA / IPC; ARM9 scaffolding)

---

## 1. Summary

This slice gives the ARM7 its **exception machinery**: the code that runs when an instruction traps, when a software interrupt is requested, or when an interrupt-generating device raises its line. Until now, every instruction the decoder couldn't understand logged a warn and advanced the program counter. From here on, the CPU actually enters the correct banked-mode, saves the old state, and jumps to the hardware vector.

After this slice, every ARMv4T ARM7-side behavior Pokemon games rely on is in place. The only remaining ARM7 work before real game code can run top-to-bottom is the **BIOS HLE** layer (what slice 3d deliberately stubs — the table of SWI implementations) and the **IRQ sources** (timers, DMA, IPC, keypad — the things that actually set IF bits). Both are their own slices.

### Plain-language summary

An ARM7 CPU has five things that can interrupt normal execution:

1. **Reset** — power-on. Already handled by `Arm7State::reset()`; no change here.
2. **SWI (software interrupt)** — a game deliberately calls a BIOS function by executing a `SWI` instruction. The CPU jumps to a fixed address (0x00000008 on ARM7), the BIOS there reads the SWI number, runs the corresponding routine, and returns.
3. **Undefined instruction** — the decoder got 16 or 32 bits that don't match any valid encoding. Well-written software doesn't hit this, but the CPU has a documented trap for it.
4. **Prefetch abort / data abort** — a bus error fetching an instruction or reading/writing data. The DS bus doesn't actually abort, but the machinery has to exist.
5. **IRQ** — a device (timer, DMA completion, VBlank, etc.) wants the CPU's attention. The ARM7 checks its IRQ input at every instruction boundary. If it's high AND the IRQ-mask bit in CPSR is clear AND the IME master-enable bit is set AND at least one `IE & IF` bit is set, the CPU swaps into IRQ mode and jumps to a vector.

All five share the same recipe: save the return address in a mode-banked `R14`, save the old CPSR in a mode-banked `SPSR`, switch into the new mode (which automatically swaps in different R13/R14 from the bank file), set some CPSR control bits, and jump to a fixed vector address.

**What this slice builds:**
- A single small helper `enter_exception()` that implements that recipe.
- Hooks in the ARM-state and Thumb-state dispatchers that call it from the right places with the right parameters.
- A top-of-loop check in `run_until()` that samples the IRQ input and calls `enter_exception()` with IRQ parameters when appropriate.
- The three memory-mapped registers (`IME`, `IE`, `IF`) at addresses `0x04000208`, `0x04000210`, `0x04000214` that games write to control interrupts.
- A direct-boot workaround for the fact that we have no ARM7 BIOS ROM — the real hardware vector at `0x00000018` indirects through `[0x0380FFFC]` in BIOS code, so we emulate that indirection manually on IRQ entry.

**What this slice deliberately does NOT build:**
- The actual table of ~32 SWI implementations. Slice 3d gets the CPU *into* the supervisor handler; the handler itself — which reads the SWI number out of the instruction and runs the right routine — lands in a later BIOS-HLE slice.
- The IRQ sources. No timer, no DMA, no IPC, no keypad raises an IF bit in this slice. Tests drive IF directly to exercise the machinery.
- Cycle-accurate exception entry timing (real hardware takes 3 cycles for most entries, 2 for IRQ; we record 1).

### Scope boundary

**In scope:** IRQ line sampling; SWI entry for ARM (bits `27:24 = 1111`) and Thumb (THUMB.17); undefined-instruction entry for both states; prefetch-abort and data-abort entry helpers (reachable by unit test only — no real bus fault source yet); a shared `enter_exception()` helper; the FIQ entry helper (code path for ARMv4T correctness, never triggered — DS doesn't wire nFIQ); IME/IE/IF register handlers on the ARM7 bus; the direct-boot IRQ-vector indirection through `[0x0380FFFC]`; the ARMv4 "S-bit + Rd=R15 copies SPSR→CPSR" pre-requisite fix in `execute_dp_op` so `MOVS PC, R14` / `SUBS PC, R14, #N` returns from exceptions correctly.

**Out of scope (§3 for detail):**
- ARM7 BIOS HLE jump table (the per-SWI-number implementations).
- IRQ sources (timers, DMA, IPC FIFO, keypad, etc.).
- The `[0x0380FFF8]` "IRQ Check Bits" shadow used by `IntrWait` / `VBlankIntrWait` — that's a BIOS-HLE concern.
- ARM9 side (CP15, high-vector base, different mode registers).
- Cycle-accurate exception entry timing.

---

## 2. Goals

1. **ARMv4T-correct exception entry for all five vectors**, verified against GBATEK §"ARM CPU Exceptions". Each vector gets the right `R14_<mode>` return-address formula, the right `SPSR_<mode>` save, the right CPSR mode/T/I/F bits, and the right vector PC.
2. **A single `enter_exception()` helper** used by every vector. No duplication of banking logic between SWI, UNDEF, IRQ, FIQ, and the two aborts.
3. **IRQ-line sampling at instruction boundaries only.** Top of `run_until`, between steps. Never inside a single `step_arm` / `step_thumb` body — that would be ARMv5 "precise preempt" semantics, which the ARM7 doesn't have.
4. **Return-from-exception works.** After `SUBS PC, R14, #4` from IRQ mode, the CPU is back in the pre-exception mode with the pre-exception CPSR and the right PC. Same for `MOVS PC, R14` from SVC mode after SWI. This requires fixing the current `execute_dp_op` gap where S=1 + Rd=R15 logs a warn but does nothing.
5. **Direct-boot IRQ-vector workaround** that reads the handler address from `[0x0380FFFC]` at the end of the IRQ entry path. Documented explicitly as a workaround until the BIOS HLE slice lands the real 0x00000000–0x00003FFF backing.
6. **IME / IE / IF registers** at `0x04000208` / `0x04000210` / `0x04000214`, wired into `arm7_io_read32` / `arm7_io_write32` and their 16/8-bit variants. Write-1-clear semantics on IF.
7. **TDD discipline.** Every exception path lands in its own commit with its own unit test. The slice capstone is a sequence test exercising the complete "game runs, IRQ fires, handler acks IF, returns" cycle.
8. **File count within house rules.** Two new `.cpp` files (`arm7_exception.cpp`, `bios7_hle.cpp` stub). No file exceeds the 500-line soft cap.

---

## 3. Non-goals

Each deferred item names the slice that will pick it up.

- **BIOS HLE SWI jump table.** Slice 3d creates `src/cpu/arm7/bios/bios7_hle.cpp` with a single entry point `arm7_bios_hle_dispatch_swi(state, bus, swi_number)` that warn-logs and returns 0. Every SWI currently warn-stubs per-number. The ~32 real implementations land in the BIOS-HLE slice. See §4.7.
- **IRQ sources.** Timer overflow (IF bits 3–6), DMA completion (bits 8–11), VBlank / HBlank / VCounter (bits 0–2), IPC sync (bit 16), IPC FIFO (bits 17–18), card DMA (bit 19), card IREQ (bit 20), screens-unfold (bit 22), SPI (bit 23), keypad (bit 12), RTC/SIO (bit 7), Wifi (bit 24) — each lands with its owning subsystem. Slice 3d exposes a single `Arm7::set_irq_line(bool)` method that the future IRQ controller will drive; unit tests drive it directly.
- **The `[0x0380FFF8]` "IRQ Check Bits" shadow.** Used by `IntrWait` / `VBlankIntrWait`. Purely a BIOS-HLE mechanism — the BIOS routine polls this location while waiting. Slice 3d does not touch it. Flagged explicitly so the BIOS-HLE slice knows to maintain it.
- **ARM9 exception machinery.** ARM9 has CP15 VBAR for a movable vector base, different SWI calling convention in some ROMs, and more exception kinds (unaligned-access trap, debug). Slice 3d is ARM7-only. The ARM9 implementer can reuse `enter_exception()` after generalizing the vector-base parameter.
- **FIQ sources.** DS has no nFIQ input wired. The FIQ entry path is implemented and unit-tested for ARMv4T correctness, but nothing in the emulator ever triggers it.
- **Cycle-accurate exception entry.** Real hardware takes 2 cycles for IRQ/FIQ entry and 3 for SWI/UNDEF/abort. We record 1 for all of them. Cycle accuracy is a Phase 6 cross-cutting slice.
- **Interworking on exception return (ARMv5 T-bit-from-PC-bit-0).** ARMv4 returns always stay in the state they came from. We match ARMv4.
- **ARMv5 MSR privilege model.** Slice 3d's IME/IE/IF register handlers are direct reads/writes with no privilege gating. Real hardware ignores user-mode writes to some of these; modeling that is a correctness delta not needed for any real DS code.

---

## 4. Architecture

Two new translation units, one new shared helper, one IRQ-sampling hook, one DP-executor fix, one IO register block. The DP fix lands first (commit 1) because it's a pre-requisite for observable return-from-exception. Every other commit is additive.

### 4.1 File layout

```
src/cpu/arm7/
  arm7_exception.hpp             NEW. enter_exception() declaration, ExceptionKind enum,
                                 vector/mode/return-offset tables.
  arm7_exception.cpp             NEW. enter_exception() body + the direct-boot IRQ
                                 indirection helper arm7_enter_irq(). ~150 lines.
  bios/
    bios7_hle.hpp                NEW. arm7_bios_hle_dispatch_swi() declaration.
    bios7_hle.cpp                NEW. Warn-stub table. ~80 lines.

  arm7.hpp                       MODIFIED. Add:
                                   - void set_irq_line(bool level);
                                   - bool irq_line_ member (default false).
  arm7.cpp                       MODIFIED. run_until() samples irq_line_ at the top
                                   of the loop, calls arm7_enter_irq() on rising
                                   edge when CPSR.I == 0.
  arm7_dp.cpp                    MODIFIED (commit 1, pre-req). execute_dp_op handles
                                   S=1 + Rd=R15 + writeback by copying SPSR to CPSR
                                   and switching mode.
  arm7_decode.cpp                MODIFIED. The `0b110`/`0b111` warn stub in dispatch_arm
                                   becomes a dispatch_swi_coproc() call; ARM SWI
                                   (bits[27:24]==1111) routes to enter_exception(SWI);
                                   coproc encodings still warn (ARMv5 territory).
                                   An explicit UNDEF fall-through calls enter_exception(UND).
  arm7_thumb_branch.cpp          MODIFIED. THUMB.17 SWI stub becomes a real
                                   enter_exception(SWI, Thumb) call. THUMB.16 cond=0xE
                                   warn stub becomes enter_exception(UND).

src/bus/
  io_regs.hpp                    MODIFIED. Add:
                                   constexpr u32 IO_IME = 0x04000208;
                                   constexpr u32 IO_IE  = 0x04000210;
                                   constexpr u32 IO_IF  = 0x04000214;
  arm7_bus.cpp                   unchanged (slow path already routes 0x04 to nds_->arm7_io_*).

src/interrupt/
  irq_controller.hpp             NEW. class Arm7IrqController — owns IME/IE/IF
                                 storage, recomputes line from (IME.0 && (IE & IF)).
                                 Does NOT take Arm7&; exposes current_line() for NDS
                                 to push into Arm7::set_irq_line on change.
  irq_controller.cpp             NEW. ~60 lines.

src/nds.hpp / src/nds.cpp        MODIFIED. NDS owns an Arm7IrqController instance.
                                 arm7_io_read/write 32/16/8 route IME/IE/IF to it.
                                 Changes to IF (write-1-clear) or IE or IME recompute
                                 the line and call arm7_.set_irq_line(new_level).

tests/unit/
  arm7_exception_swi_test.cpp       NEW. SWI from ARM and Thumb.
  arm7_exception_undef_test.cpp     NEW. Undefined instruction entry from ARM and Thumb.
  arm7_exception_abort_test.cpp     NEW. Prefetch + data abort helpers (synthetic drive).
  arm7_exception_irq_test.cpp       NEW. IRQ line sampling, direct-boot vector indirection.
  arm7_exception_fiq_test.cpp       NEW. FIQ entry (synthetic).
  arm7_exception_return_test.cpp    NEW. SUBS/MOVS PC, R14 return paths.
  arm7_irq_controller_test.cpp      NEW. IME/IE/IF write-1-clear, line computation.
  arm7_exception_sequence_test.cpp  NEW. Capstone: game loop, IRQ, ack IF, return.
```

Eight new test binaries, taking the CTest count from 31 (end of slice 3c) to **39**.

### 4.2 `enter_exception` — the shared helper

One helper handles every exception kind. Signature:

```cpp
enum class ExceptionKind : u8 {
    Reset, Undef, Swi, PrefetchAbort, DataAbort, Irq, Fiq,
};

// Enter `kind`. `return_addr` is the address to stash in the new mode's R14
// (the caller computes it per §5.3). The helper:
//   1. Writes state.r[14] = return_addr in the new mode's bank (by saving
//      old CPSR, switching mode, then writing r[14] directly — switch_mode
//      handles the bank swap).
//   2. Writes *state.spsr_slot() = old_cpsr after the switch.
//   3. Sets CPSR.T = 0, CPSR.I = 1, and optionally CPSR.F.
//   4. Writes state.pc = vector_address_for(kind).
u32 enter_exception(Arm7State& state, ExceptionKind kind, u32 return_addr);
```

The helper's body is almost mechanical:

```cpp
// Pseudocode — not to be pasted.
const u32 old_cpsr = state.cpsr;
const Mode new_mode = mode_for(kind);
state.switch_mode(new_mode);           // swaps in new R13/R14 bank and sets CPSR.M
state.r[14] = return_addr;              // lives in the new bank
*state.spsr_slot() = old_cpsr;          // the new mode always has an SPSR
state.cpsr &= ~(1u << 5);               // CPSR.T = 0 (always ARM after entry)
state.cpsr |= (1u << 7);                // CPSR.I = 1 (always mask IRQ)
if (set_f_for(kind)) state.cpsr |= (1u << 6);  // Reset + FIQ only
state.pc = vector_for(kind);            // 0x00, 0x04, 0x08, 0x0C, 0x10, 0x18, 0x1C
return 3;                               // coarse; cycle-accuracy is Phase 6
```

**Three small tables** live alongside the helper, all `constexpr`:

| Kind            | `mode_for`  | `vector_for` | set F |
|-----------------|-------------|--------------|-------|
| Reset           | Supervisor  | 0x00000000   | yes   |
| Undef           | Undefined   | 0x00000004   | no    |
| Swi             | Supervisor  | 0x00000008   | no    |
| PrefetchAbort   | Abort       | 0x0000000C   | no    |
| DataAbort       | Abort       | 0x00000010   | no    |
| Irq             | Irq         | 0x00000018   | no    |
| Fiq             | Fiq         | 0x0000001C   | yes   |

The "SPSR always exists" claim is true for every `mode_for(kind)` above — User/System are never targets of exception entry, and those are the only modes without an SPSR slot. `spsr_slot()` is guaranteed non-null; the helper asserts in DEBUG.

**Rejected alternative:** inline the banking logic at each entry site. That duplicates the `old_cpsr → switch_mode → r[14] → *spsr_slot → CPSR bits → pc` dance five times and makes correctness review per-site instead of per-helper. One helper is clearly right.

### 4.3 Where exception entry is called from

- **ARM SWI.** `dispatch_arm` case `0b110`/`0b111` checks `bits[27:24] == 1111`. If matched, calls `enter_exception(ExceptionKind::Swi, instr_addr + 4)`. Coprocessor encodings (non-matching) still warn-stub — they're ARMv5 territory.
- **Thumb SWI.** `dispatch_thumb_bcond_swi` in `arm7_thumb_branch.cpp` currently warn-stubs THUMB.17 (cond == 0xF). Becomes `enter_exception(ExceptionKind::Swi, instr_addr + 2)`. Note: `step_thumb` already pre-advanced `state.pc` to `instr_addr + 2`, so `instr_addr + 2` is the address of the instruction *following* the SWI, which is exactly what ARMv4 specifies for the Thumb SWI return address.
- **ARM UNDEF.** Currently the `dispatch_arm` coproc/SWI warn stub catches anything in `0b110`/`0b111` that isn't SWI. After slice 3d, coproc instructions (bit 24 clear plus MRC/MCR/LDC/STC patterns) are legitimate UNDEFs on the ARMv4T ARM7 — route them to `enter_exception(ExceptionKind::Undef, instr_addr + 4)`. Similarly, any other encoding that falls off the bottom of a family handler's recognizer cascade (e.g. multiply with illegal bit patterns) routes to UNDEF rather than a silent warn. The per-family warn stubs are audited and replaced commit-by-commit.
- **Thumb UNDEF.** `dispatch_thumb_bcond_swi` cond == 0xE warn stub becomes `enter_exception(ExceptionKind::Undef, instr_addr + 2)`. Same rationale: `instr_addr + 2` is the post-instruction PC by the time we're here, and ARMv4 stashes that in R14_und.
- **Prefetch abort / data abort.** No real trigger exists yet. A test-only entry point `Arm7::raise_prefetch_abort(u32 instr_addr)` / `raise_data_abort(u32 instr_addr)` is added in slice 3d so the helpers can be unit-tested in isolation. Real bus-fault sources wire into these later.
- **FIQ.** Same treatment as prefetch/data abort — a test-only `Arm7::raise_fiq()` method. DS never wires nFIQ, but the code path must be correct and testable.
- **IRQ.** Described in §4.4.

### 4.4 IRQ line sampling — `run_until` top-of-loop check

The current `run_until`:

```cpp
// current (slice 3c):
while (state_.cycles < arm7_target) {
    if (state_.cpsr & (1u << 5)) step_thumb();
    else                         step_arm();
}
```

becomes:

```cpp
// slice 3d — pseudocode:
while (state_.cycles < arm7_target) {
    if (irq_line_ && !(state_.cpsr & (1u << 7))) {
        arm7_enter_irq(state_, *bus_);        // §4.5
        continue;                              // re-check; IRQ entry consumed cycles
    }
    if (state_.cpsr & (1u << 5)) step_thumb();
    else                         step_arm();
}
```

**Why at the top of the loop and not inside `step_arm`/`step_thumb`:**
- ARMv4T samples nIRQ at instruction boundaries only. "Between steps" is the exact right place.
- Inside `step_*` would require exactness about when the sample happens relative to the pipeline stages we don't model. Sample-at-boundary sidesteps that entirely.
- `step_arm` and `step_thumb` don't need to know IRQ exists — they stay focused on instruction execution. Testability: unit tests can call `step_arm()` directly without tripping IRQ entry.

**Edge ordering:** the sample happens *before* we pick ARM vs Thumb, which matches hardware: when CPSR.T is 1 (Thumb) and an IRQ fires, the CPU enters IRQ mode (in ARM state) — CPSR.T is read as a snapshot, then overwritten to 0 by `enter_exception`. Our top-of-loop ordering naturally gives that.

**`continue` after IRQ entry:** the next loop iteration picks up in ARM state (CPSR.T cleared) starting at the IRQ handler. We don't drop through to `step_arm` in the same iteration because `arm7_enter_irq` consumed the "budget" for this tick.

**Level vs edge:** `irq_line_` is a level. The IRQ controller asserts while `IME.0 && (IE & IF) != 0` and de-asserts otherwise. The CPU doesn't latch edges; it re-samples every boundary. So a pending IRQ that the handler doesn't ack by writing IF will immediately re-fire on exit. This matches real hardware and is the reason handlers always start with "write `IF` with the bit you're handling" (write-1-clear).

### 4.5 The direct-boot IRQ-vector workaround

Real hardware: the ARM7 BIOS ROM at `0x00000000` contains a tiny dispatcher at `0x00000018` that loads a function pointer from `[0x0380FFFC]` and branches there. We direct-boot — there is no BIOS ROM — so on IRQ entry we have to synthesize that behavior.

```cpp
// src/cpu/arm7/arm7_exception.cpp
void arm7_enter_irq(Arm7State& state, Arm7Bus& bus) {
    // Normal ARMv4T IRQ return address (ARM-style format, per GBATEK).
    // This formula is identical from ARM state and Thumb state — see §5.3.
    const u32 return_addr =
        state.pc + (/* Thumb? */ (state.cpsr & (1u << 5)) ? 4u : 4u);
    enter_exception(state, ExceptionKind::Irq, return_addr);
    // enter_exception just set PC = 0x18. Real BIOS at 0x18 is a tiny
    // thunk that reads [0x0380FFFC] and branches. We direct-boot with
    // no BIOS, so synthesize that thunk's effect here.
    state.pc = bus.read32(0x0380FFFC);
}
```

**Why keep this explicit at the IRQ site rather than in `enter_exception`:**
- `enter_exception` is a hardware-correctness helper. Adding a direct-boot quirk into it makes it do something real hardware doesn't.
- Putting the quirk at the IRQ site keeps SWI/UNDEF/abort/FIQ clean — they'd otherwise accidentally pick up the indirection.
- When the BIOS-HLE slice lands a real ARM7 BIOS backing at `0x00000000–0x00003FFF`, `arm7_enter_irq` loses its `state.pc = bus.read32(...)` line and becomes a one-line `enter_exception(state, Irq, return_addr)`. The refactor is local.
- The name `arm7_enter_irq` (as opposed to folding into `enter_exception`) signals the quirk at the call site.

**SWI also enters at a vector (0x08) that real BIOS would handle.** Slice 3d routes to `bios7_hle.cpp`'s stub handler — §4.7 — rather than indirecting through memory, because the SWI number comes from the instruction word and real BIOS code would read it too. Different mechanism, same idea: we substitute HLE for the missing ROM.

### 4.6 The pre-requisite fix — `execute_dp_op` S=1 + Rd=R15

At `src/cpu/arm7/arm7_dp.cpp:108-119` the current code handles S=1 + Rd=R15 by logging a warn and doing nothing. ARMv4 semantics: if `writeback` is true (any DP opcode that writes Rd — i.e. not TST/TEQ/CMP/CMN), the SPSR of the current mode is copied to CPSR, which also means a possible mode switch that re-banks R8–R14.

**Why this is a slice-3d pre-requisite:**
- Exception *entry* works without this fix (we're not going through a DP op).
- Exception *return* doesn't. `SUBS PC, R14, #4` (IRQ/abort return) and `MOVS PC, R14` (SWI/UNDEF return) are both S=1 DP ops with Rd=R15. Without the fix, the CPU writes R14 to PC but leaves CPSR in IRQ mode, which is not a return. Every unit test that exercises full round-trip through exception entry and return would fail.

**The fix:**

```cpp
// In execute_dp_op, replacing the current warn stub at arm7_dp.cpp:108-119
if (s_flag && rd == 15 && writeback) {
    const u32* spsr = state.spsr_slot();
    if (spsr == nullptr) {
        DS_LOG_WARN("arm7: S-flag DP with Rd=R15 from mode without SPSR at 0x%08X", instr_addr);
        // Fall through: flags unchanged, CPSR unchanged, PC already written by write_rd.
    } else {
        const u32 new_cpsr = *spsr;
        // Apply mode change via switch_mode so banked R8..R14 rebank.
        const Mode target = static_cast<Mode>(new_cpsr & 0x1Fu);
        state.switch_mode(target);
        // Overwrite CPSR with the full saved value (switch_mode only set M bits).
        state.cpsr = new_cpsr;
    }
} else if (s_flag) {
    // existing NZCV update path, unchanged.
}
```

**Ordering:** `write_rd` already stomped R15 and state.pc *before* this block runs (it's at line 105 of the current file, inside `if (writeback)`). So by the time we restore CPSR, PC already holds the return address from R14, which is what we want — the CPSR restore doesn't affect PC.

**Restored CPSR.T:** if the saved SPSR had T=1 (e.g. the original SWI was from Thumb), this path naturally flips CPSR.T=1 as part of the full `cpsr = *spsr` write. The next loop iteration of `run_until` picks ARM or Thumb correctly.

**Restored CPSR.I:** same. If the SPSR had I=0 and an IRQ is still pending, the next `run_until` top-of-loop sample fires again. Correct behavior.

**Test that forces this:** `arm7_exception_return_test.cpp` sets up an IRQ entry from both ARM and Thumb state, then executes `SUBS PC, R14, #4`, and asserts mode/CPSR/PC match expected. Without the fix, the asserts on CPSR.M fail.

This is commit 1 of the slice.

### 4.7 `bios7_hle.cpp` — stub file, real handlers deferred

The SWI exception enters at PC = 0x00000008. Real hardware's BIOS at 0x08 reads the SWI number from the instruction word that triggered the SWI and dispatches. We replace that with an HLE call:

```cpp
// arm7_decode.cpp — new dispatch_swi_coproc handler
u32 dispatch_swi_coproc(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    if ((instr & 0x0F000000u) == 0x0F000000u) {
        // ARM-state SWI. bits[23:0] hold the SWI comment field.
        const u32 swi_number = instr & 0x00FFFFFFu;
        enter_exception(state, ExceptionKind::Swi, instr_addr + 4);
        arm7_bios_hle_dispatch_swi(state, bus, swi_number);   // stub: warn + return
        return 3;
    }
    // Coproc encoding on ARMv4T ARM7: UNDEF.
    enter_exception(state, ExceptionKind::Undef, instr_addr + 4);
    return 3;
}
```

`arm7_bios_hle_dispatch_swi` in `bios7_hle.cpp` is a warn-per-number stub:

```cpp
u32 arm7_bios_hle_dispatch_swi(Arm7State& /*state*/, Arm7Bus& /*bus*/, u32 swi_number) {
    DS_LOG_WARN("arm7/bios: SWI 0x%02X not implemented (slice 3d stub)", swi_number & 0xFFu);
    return 1;
}
```

The BIOS-HLE slice replaces the stub with a dispatch table. Slice 3d does not implement any SWI bodies — the mechanism is the deliverable, the BIOS is not.

**Thumb SWI is identical in shape:** `dispatch_thumb_bcond_swi`'s SWI branch extracts `instr & 0xFFu` as the number, calls `enter_exception(Swi, instr_addr + 2)`, calls `arm7_bios_hle_dispatch_swi(state, bus, number)`, returns.

**Key question — does the HLE dispatcher run INSIDE or AFTER exception entry?**
Inside. The flow is:
1. Instruction executes, calls `enter_exception(Swi, ret)`. CPSR is now SVC mode, PC is 0x08, R14_svc is ret, SPSR_svc is old CPSR.
2. Without returning from `execute_*`, we call `arm7_bios_hle_dispatch_swi` immediately. It runs in SVC mode with IRQs masked (CPSR.I=1) — same as if real BIOS had run from 0x08.
3. The stub warn-logs and returns. The game's next instruction executes as SVC-mode code, which will immediately issue a `MOVS PC, R14` to return. That return works because of the §4.6 pre-req fix.

When the stub gets replaced by a real dispatch table in a later slice, the pattern is the same — the table body sets up the SWI's return value in `state.r[0]` or does a sleep-scheduler trick or whatever, then returns. The game's `MOVS PC, R14` still returns correctly.

### 4.8 `Arm7IrqController` — IME/IE/IF storage and line computation

Small class with three fields and a handful of methods:

```cpp
class Arm7IrqController {
public:
    void reset();
    void write_ime(u32 value);          // bit 0 only; upper bits ignored on DS
    void write_ie (u32 value);
    void write_if (u32 value);          // write-1-clear
    u32  read_ime() const { return ime_ & 1u; }
    u32  read_ie()  const { return ie_; }
    u32  read_if()  const { return if_; }
    void raise(u32 source_bits);        // sets bits in IF (no auto-clear)
    bool line() const { return (ime_ & 1u) != 0 && (ie_ & if_) != 0; }
private:
    u32 ime_ = 0;
    u32 ie_  = 0;
    u32 if_  = 0;
};
```

`write_if`: `if_ &= ~value;`. Reads return the pending state.
`raise`: `if_ |= source_bits;`. Future IRQ-source slices call this from their subsystem code when a source fires.

**Who pushes the line into `Arm7::set_irq_line`:** NDS owns the controller. After every operation that can change the line (`write_ime`, `write_ie`, `write_if`, `raise`), NDS reads `ctrl.line()` and calls `arm7_.set_irq_line(line)`. Keeping that glue in NDS — not in the controller — preserves the non-negotiable rule that no subsystem holds a pointer to another.

**Reset values:** IME=0, IE=0, IF=0. Direct-boot may want to seed non-zero values to match post-firmware state; that's a direct-boot-initialization concern and is handled outside this slice.

### 4.9 IME/IE/IF bus routing

`io_regs.hpp` gains three address constants. `nds.cpp`'s `arm7_io_read32` / `arm7_io_write32` / the 16/8 variants gain cases:

```cpp
// arm7_io_read32
case IO_IME: return irq_ctrl_.read_ime();
case IO_IE:  return irq_ctrl_.read_ie();
case IO_IF:  return irq_ctrl_.read_if();

// arm7_io_write32
case IO_IME: irq_ctrl_.write_ime(value); update_arm7_irq_line(); break;
case IO_IE:  irq_ctrl_.write_ie (value); update_arm7_irq_line(); break;
case IO_IF:  irq_ctrl_.write_if (value); update_arm7_irq_line(); break;
```

`update_arm7_irq_line()` is a three-line helper: `arm7_.set_irq_line(irq_ctrl_.line());`.

**16-bit and 8-bit accesses** to these registers are rare in real code but legal. IME is only bit 0 — 8-bit and 16-bit writes to 0x04000208 act on the bit; higher bytes are ignored. IE and IF are 32-bit but GBATEK allows halfword access; we implement both widths cleanly. 8-bit writes to IE / IF are warn-logged (real hardware behavior is ignored / promoted per-region — we warn and do the natural byte update).

### 4.10 `Arm7::set_irq_line` and field

```cpp
// arm7.hpp — new members
public:
    void set_irq_line(bool level) { irq_line_ = level; }
private:
    bool irq_line_ = false;
```

No events, no scheduler coupling. The IRQ controller is synchronous — the write that makes IF & IE non-zero immediately sets the line, which immediately makes the next `run_until` iteration enter IRQ mode. This is coarser than real hardware's 2-cycle latency but correct for functional behavior; cycle accuracy is Phase 6.

`set_irq_line` is callable from the IRQ controller (via NDS), from tests, and from the future per-source subsystems. No one caches the pointer.

### 4.11 What does NOT change

- `Arm7State` — slice 3c's banking + SPSR machinery is reused as-is.
- `Arm7Bus` interface — no new public methods; I/O routing is already in place.
- `Scheduler` — slice 3d does not add events.
- Any subsystem other than ARM7, NDS glue, the new IRQ controller, and the new exception helper.
- Existing ARM-state and Thumb-state instruction behavior — every slice 3a–3c test continues to pass. The `execute_dp_op` fix changes behavior in exactly one previously-warn path (S=1 + Rd=R15 with writeback), and no prior test exercised that path.

### 4.12 Known technical debt deferred

- **BIOS HLE SWI jump table** — §4.7 stub, real implementations in a later slice.
- **IRQ sources** — timers/DMA/IPC/keypad/etc. land with their owning subsystems.
- **Exception entry cycle accuracy** — return 1; correct is 2 (IRQ/FIQ) or 3 (others).
- **ARM9 generalization of `enter_exception`** — will need a `vector_base` parameter for CP15 high-vector-base support.
- **ARMv5 interworking on exception return** — out of scope; ARMv4 stays in prior state.
- **Imprecise-abort handling** — real ARMv4 has two abort variants (precise vs imprecise data abort); we treat all data aborts as precise. DS doesn't raise imprecise aborts from any source we model.

---

## 5. Hardware details

All GBATEK references are to `https://problemkaputt.de/gbatek.htm` §"ARM CPU Exceptions" and §"DS System Control (IRQ Control)".

### 5.1 Vector table (ARM7, BASE = 0x00000000)

| Vector      | Address      | Target mode    | I | F        |
|-------------|--------------|----------------|---|----------|
| Reset       | 0x00000000   | Supervisor     | 1 | 1        |
| Undef       | 0x00000004   | Undefined      | 1 | unchanged |
| SWI         | 0x00000008   | Supervisor     | 1 | unchanged |
| Prefetch    | 0x0000000C   | Abort          | 1 | unchanged |
| Data Abort  | 0x00000010   | Abort          | 1 | unchanged |
| (reserved)  | 0x00000014   | —              | — | —        |
| IRQ         | 0x00000018   | Irq            | 1 | unchanged |
| FIQ         | 0x0000001C   | Fiq            | 1 | 1        |

### 5.2 Entry actions (all exceptions)

Always executed in ARM state on entry — CPSR.T=0 regardless of prior state.

1. `R14_<new_mode> = return_addr` (formula below, §5.3).
2. `SPSR_<new_mode> = old_CPSR`.
3. `CPSR.M4-0 = new_mode; CPSR.T = 0; CPSR.I = 1; CPSR.F = old or 1 per table`.
4. `PC = vector_address`.

### 5.3 Return-address formulas (GBATEK)

`instr_addr` is the address of the triggering instruction.

| Exception       | `return_addr` from ARM state | `return_addr` from Thumb state | Return instruction | Result |
|-----------------|------------------------------|--------------------------------|--------------------|--------|
| SWI             | `instr_addr + 4`             | `instr_addr + 2`               | `MOVS PC, R14`     | Resume at next instruction |
| Undef           | `instr_addr + 4`             | `instr_addr + 2`               | `MOVS PC, R14`     | Resume at next instruction |
| IRQ             | `instr_addr + 4` (ARM-style) | `instr_addr + 4` (ARM-style)   | `SUBS PC, R14, #4` | Resume at next instruction |
| Prefetch abort  | `instr_addr + 4`             | n/a                            | `SUBS PC, R14, #4` | Re-execute faulting instr |
| Data abort      | `instr_addr + 8`             | `instr_addr + 8`               | `SUBS PC, R14, #8` | Re-execute faulting instr |
| Reset           | undefined                    | undefined                      | none               | — |
| FIQ             | `instr_addr + 4` (ARM-style) | `instr_addr + 4` (ARM-style)   | `SUBS PC, R14, #4` | Resume at next instruction |

GBATEK note on IRQ/FIQ: "the return address is always saved in ARM-style format". The Thumb-state formulas look the same as ARM on paper — both yield "address of the next instruction plus 4" — but the path to get there differs:

- From ARM: instruction at `instr_addr`; next instruction at `instr_addr + 4`; ARM-style return is `next + 4` would back up by 4, so stored is `instr_addr + 4`.
- From Thumb: instruction at `instr_addr`; next instruction at `instr_addr + 2`; ARM-style return needs back-up by 4, so stored is `instr_addr + 2 + 4 = instr_addr + 6`. But GBATEK specifies `instr_addr + 4` (not +6). The hardware models the Thumb case as "the following instruction is at `instr_addr + 2`, backed up by `4 - 2 = 2` halfwords in some sense, giving the same stored value as ARM" — in practice the value ARMv4T actually stores is well-defined by the TRM and melonDS uses `instr_addr + 4` uniformly for both ARM and Thumb IRQ entry. We match melonDS.

`SUBS PC, R14, #4` at the end of the IRQ handler yields `instr_addr + 4 - 4 = instr_addr`... which is the faulting instruction, not the next one. That is wrong if taken literally and works in practice because IRQ fires *between* instructions — by the time the sample happens, `instr_addr` in our model is the address of the *next* instruction that hasn't run yet. The formula stashes the "current pc" as return address; the handler subtracts 4 to get there.

Concretely, in our code: when `run_until` samples the IRQ line, `state.pc` already holds the address of the next instruction to execute (because the previous `step_*` advanced it). The return address to stash is therefore `state.pc + 4` in both ARM and Thumb state — matching "ARM-style format, one word past the next instruction".

### 5.4 CPSR layout

```
31  30  29  28  27  [26-8 reserved]  7    6    5    [4:0]
 N   Z   C   V   Q                    I    F    T    M
```

- `I = 1` masks IRQ. `F = 1` masks FIQ.
- `T = 1` selects Thumb state.
- `M` is the 5-bit mode field. ARMv4T modes: User `0x10`, FIQ `0x11`, IRQ `0x12`, SVC `0x13`, Abort `0x17`, Undef `0x1B`, System `0x1F`.
- Bits `[27:8]` other than `Q` are reserved — we leave whatever the game writes, same as slice 3c.

### 5.5 DS interrupt registers (ARM7 side)

| Register | Address      | Size | Semantics |
|----------|--------------|------|-----------|
| IME      | 0x04000208   | 32-bit | Bit 0 is master enable. Bits [31:1] ignored. |
| IE       | 0x04000210   | 32-bit | One bit per source. Written by game to enable a source. |
| IF       | 0x04000214   | 32-bit | One bit per source. **Writing 1 clears.** Reading returns pending state. |

**IRQ line:** `asserted iff (IME.0 == 1) && ((IE & IF) != 0)`.

**ARM7 source bits (for reference; slice 3d does not raise them):**

| Bit | Source |
|-----|--------|
| 0   | VBlank |
| 1   | HBlank |
| 2   | VCounter match |
| 3   | Timer 0 overflow |
| 4   | Timer 1 overflow |
| 5   | Timer 2 overflow |
| 6   | Timer 3 overflow |
| 7   | SIO / RTC |
| 8   | DMA 0 |
| 9   | DMA 1 |
| 10  | DMA 2 |
| 11  | DMA 3 |
| 12  | Keypad |
| 16  | IPC Sync |
| 17  | IPC Send-FIFO Empty |
| 18  | IPC Recv-FIFO Not Empty |
| 19  | Game Card |
| 20  | Card IREQ (Slot-2 on DS) |
| 22  | Screens unfold |
| 23  | SPI bus |
| 24  | Wifi |

### 5.6 Direct-boot IRQ vector indirection

Real ARM7 BIOS at `0x00000018` (16 bytes):
```
LDR PC, [PC, #-4]       ; load from 0x00000020
.word 0x03FFFFFC        ; actually mirrored to 0x0380FFFC
```

Effectively the BIOS code at 0x18 is `PC = *(u32*)0x0380FFFC`. Direct-boot has no BIOS ROM — slice 3d synthesizes this on IRQ entry by calling `bus.read32(0x0380FFFC)` after the normal `enter_exception(Irq, ...)` and overwriting `state.pc`. Game startup code is responsible for writing its handler address to `[0x0380FFFC]` before enabling IRQs; every real DS title does this in its early-boot initializer.

### 5.7 `[0x0380FFF8]` — IRQ Check Bits (BIOS-HLE concern, NOT this slice)

The ARM7 BIOS `IntrWait` / `VBlankIntrWait` SWIs poll a shadow IF at `[0x0380FFF8]` while halted. The BIOS's IRQ handler code writes a 1 to the appropriate bit of `[0x0380FFF8]` so the waiting SWI can see which interrupt fired.

**This is not slice 3d's concern.** Slice 3d's IF is the hardware IF at `0x04000214`. When the BIOS-HLE slice lands `IntrWait`, that slice's handler maintains `[0x0380FFF8]` in parallel. Flagged so the BIOS-HLE slice knows.

### 5.8 Exception return — the S-bit + Rd=R15 DP rule

ARMv4 behavior, applies to every DP opcode that writes Rd (i.e. not TST/TEQ/CMP/CMN):

- If `S == 1` AND `Rd == 15` AND the current mode has an SPSR:
  - `CPSR = SPSR_<current_mode>` (full 32-bit copy; also re-banks R8–R14).
  - The normal NZCV update is **skipped** (flags come from the SPSR copy).
- If `S == 1` AND `Rd == 15` AND the current mode has no SPSR (User/System):
  - UNPREDICTABLE. We warn, leave CPSR unchanged, and let the normal Rd-write-to-R15 happen (plain branch). No real code emits this.

Both `MOVS PC, R14` (DP-MOV with S=1, Rd=15) and `SUBS PC, R14, #N` (DP-SUB with S=1, Rd=15) exercise this rule. This is what makes exception return work.

### 5.9 UNPREDICTABLE / edge cases to handle deterministically

- **Exception from User/System mode via a path that normally targets User/System** — impossible; exception targets are never User or System.
- **IRQ sampled during a BL two-halfword sequence** — slice 3c's stateless BL encoding means the halves are individually atomic. An IRQ between them is safe: LR holds the partial-target value, the IRQ handler runs, returns, and the second halfword reads LR and branches. Matches real hardware per GBATEK's "implementation-defined" clause.
- **`MOVS PC, R14` from User/System** — warn, do a normal branch, leave CPSR untouched.
- **Entering a mode when its SPSR slot is somehow queried while null** — asserted in DEBUG; release builds warn-log and skip the SPSR write (which would be a latent bug we want loud).
- **Write to IF that sets bits already zero** — no-op. The `if_ &= ~value` formula handles it naturally.
- **Write to IE or IME with IRQ line already asserted** — the line recomputes; it may stay asserted, in which case IRQ entry fires on the next `run_until` iteration as if nothing changed.
- **`bus.read32(0x0380FFFC)` before the game has written anything there** — returns the ARM7 WRAM contents (whatever happens to be there after `reset()`, which zero-fills). If the handler jumps to 0, the game crashes on `LDR PC, [...]` from 0x00000000 (which also reads zero in direct-boot with no BIOS). This is a game-side bug; we don't paper over it.

---

## 6. Testing strategy

Eight new test binaries. Each follows the slice-3c pattern of "one binary per exception kind, plus a capstone sequence test."

### 6.1 `arm7_exception_swi_test.cpp`

SWI from ARM state and Thumb state.

- SWI from ARM: after entry, assert `current_mode() == Supervisor`, `SPSR_svc == old_cpsr`, `R14_svc == instr_addr + 4`, `CPSR.T == 0`, `CPSR.I == 1`, `CPSR.F == old_f`, `PC == 0x08`. The HLE stub is called with `swi_number == comment_field & 0xFFu`.
- SWI from Thumb: entry is in ARM state (CPSR.T=0 after entry); `R14_svc == instr_addr + 2`; `SPSR_svc.T == 1` (saved before T is cleared).
- SWI while CPSR.I == 1: still entered — SWI is not IRQ-masked.
- SWI comment-field extraction from both ARM (`instr & 0xFFFFFF`) and Thumb (`instr & 0xFF`) — the correct number reaches `arm7_bios_hle_dispatch_swi`.

### 6.2 `arm7_exception_undef_test.cpp`

- ARM UNDEF: decode an illegal coproc-shaped encoding; assert mode=Undefined, SPSR_und=old_cpsr, R14_und=instr_addr+4, CPSR.T=0, CPSR.I=1, PC=0x04.
- Thumb UNDEF: decode a THUMB.16 cond=0xE; assert mode=Undefined, R14_und=instr_addr+2, SPSR_und.T=1 (pre-entry was Thumb).
- Multiply with illegal bit pattern: confirms the per-family warn stub is migrated to UNDEF entry, not silent.

### 6.3 `arm7_exception_abort_test.cpp`

Synthetic drive via `Arm7::raise_prefetch_abort(u32 instr_addr)` and `raise_data_abort(u32 instr_addr)` test-only methods.

- Prefetch abort: mode=Abort, R14_abt=instr_addr+4, PC=0x0C.
- Data abort: mode=Abort, R14_abt=instr_addr+8, PC=0x10. Note the `+8` differs from `+4`.
- Abort during FIQ mode: FIQ not re-masked by abort entry — CPSR.F stays 1 because it was already 1, not because abort sets it. Verify by entering FIQ first, then raising abort, and checking CPSR.F came from SPSR-save path, not the vector table.

### 6.4 `arm7_exception_irq_test.cpp`

- `set_irq_line(true)` while `CPSR.I == 0`: next `step_arm`-or-`step_thumb` boundary triggers entry. Assert mode=Irq, CPSR.T=0, CPSR.I=1, CPSR.F=old_f, PC=handler_address (via `[0x0380FFFC]` indirection).
- `set_irq_line(true)` while `CPSR.I == 1`: no entry. Run 10 steps, assert mode unchanged.
- `set_irq_line(true)` while CPSR.T=1 (Thumb): entry happens; CPSR.T cleared; SPSR_irq.T=1.
- Direct-boot vector workaround: write 0xAABBCCDD to `[0x0380FFFC]`, raise IRQ, assert PC = 0xAABBCCDD (not 0x18).
- `set_irq_line(false)` with no pending entry: no effect.
- IRQ line stays asserted after entry (handler hasn't ack'd IF): return from handler immediately re-enters. Asserted by sequence test in §6.8.
- `R14_irq` value: assert `== state.pc_before_entry + 4` for both ARM and Thumb entry.

### 6.5 `arm7_exception_fiq_test.cpp`

Synthetic drive via `Arm7::raise_fiq()`.

- Entry: mode=Fiq, R14_fiq=pc+4, SPSR_fiq=old_cpsr, CPSR.T=0, CPSR.I=1, CPSR.F=1, PC=0x1C.
- Banking: enter from User with distinctive R8–R12 values; assert FIQ bank is loaded (zero or whatever FIQ had); return via `SUBS PC, R14, #4` and assert User-bank R8–R12 restored.

### 6.6 `arm7_exception_return_test.cpp`

The pre-requisite `execute_dp_op` S=1+Rd=R15 fix is the star here.

- `MOVS PC, R14` from SVC mode after SWI entry: CPSR restored from SPSR_svc; mode back to pre-entry; PC = R14.
- `SUBS PC, R14, #4` from IRQ mode: CPSR restored from SPSR_irq; mode back to pre-entry; PC = R14 - 4.
- `SUBS PC, R14, #8` from Abort mode after data abort: CPSR restored, PC = faulting instr.
- Return with SPSR.T=1: after the op, CPSR.T=1; next loop iteration dispatches Thumb.
- Return from User/System: UNPREDICTABLE — warn, PC written, CPSR untouched.
- Return with SPSR.M = User: mode switches to User; banked R13/R14 swap correctly.

### 6.7 `arm7_irq_controller_test.cpp`

- IME write/read: 32-bit, 16-bit, 8-bit. Only bit 0 is live; upper bits ignored.
- IE write/read: 32-bit round-trip.
- IF write-1-clear: write all-ones, `if_` becomes 0; write specific bits, only those clear.
- IF raised via `raise(bits)` — bits OR'd in.
- Line computation: truth table over IME × (IE & IF).
- NDS glue: writing IF pushes updated line to `arm7_.set_irq_line`. Verified by `arm7_.state().irq_line()` getter (new, test-only).

### 6.8 `arm7_exception_sequence_test.cpp` — capstone

End-to-end: a hand-written ARM program that models a game's main loop plus an IRQ handler.

1. Load program into main RAM. Program:
   - `main:    MOV R0, #1`
   - `         STR R0, [IME_addr]`         ; enable IME
   - `         MOV R0, #2`
   - `         STR R0, [IE_addr]`          ; enable timer1 (bit 3... let's pick source bit 0 = VBlank for the test)
   - `loop:    NOP`
   - `         B loop`
   - `handler: LDR R0, =(1 << 0)`
   - `         STR R0, [IF_addr]`          ; ack VBlank
   - `         SUBS PC, R14, #4`            ; return
2. Write `&handler` to `[0x0380FFFC]`.
3. Run a few cycles to let setup complete.
4. Externally call `irq_ctrl_.raise(1 << 0); update_arm7_irq_line();` — simulates VBlank firing.
5. Step. Expect entry into the handler (PC jumps to `&handler`).
6. Step the handler instructions. Expect IF cleared to 0 after the store.
7. Expect return: mode back to the caller's mode, CPSR restored, PC at the instruction following the NOP where the IRQ was taken.
8. Assert line is now de-asserted (IF=0).
9. Final register state matches expectations.

This test exercises: IRQ sampling, direct-boot vector indirection, SPSR save, banked R14_irq save, handler execution under SVC-style constraints, IF write-1-clear, line recomputation, `SUBS PC, R14, #4` return via the fixed `execute_dp_op`, SPSR → CPSR restore, mode unbanking.

### 6.9 Total test binary count

End of slice 3c: 31. Slice 3d adds 8 → **39**. Every new test links `ds_core` only, uses `REQUIRE`, runs in milliseconds, no SDL.

---

## 7. Cross-references

- **GBATEK §"ARM CPU Exceptions"** — vector table, entry actions, return-address formulas. Authoritative.
- **GBATEK §"DS System Control (IRQ Control)"** — IME/IE/IF semantics and the ARM7 source-bit table.
- **GBATEK §"ARM Opcodes: Data Processing"**, specifically the subsection on S-bit with Rd=R15 — the pre-req fix.
- **ARM Architecture Reference Manual (ARMv4T)** — §A2.6 "Exceptions" and §A4.1.35 "MRS", §A4.1.38 "MSR", §A4.1.29 "Data-processing instructions" for the SPSR→CPSR copy rule.
- **melonDS `src/ARM.cpp` `TriggerIRQ`, `ARMv4`-style branches** — reference for IRQ sampling site and return-address formula (`instr_addr + 4` uniformly for both ARM and Thumb entry).
- **Predecessor slice:** `docs/specs/2026-04-16-arm7-core-phase1-slice3c-design.md` — Thumb state, with the deliberately-stubbed SWI and UNDEF warn paths that slice 3d replaces.
- **Parent spec:** `docs/specs/2026-04-12-nds-emulator-design.md` §4.2 (ARM7 interpreter) and §13.3 (Phase 1 milestone list).
- **State header (unchanged):** `src/cpu/arm7/arm7_state.hpp` — `Mode`, `switch_mode`, `spsr_slot`.
- **Pre-req fix target:** `src/cpu/arm7/arm7_dp.cpp:108-119`.
- **SWI warn stub sites being replaced:** `src/cpu/arm7/arm7_decode.cpp:43` (ARM coproc/SWI fall-through), `src/cpu/arm7/arm7_thumb_branch.cpp:39` (Thumb SWI).

---

## 8. Risk and rollback

Medium risk. The DP pre-req fix changes semantics in a previously-warn code path; the IRQ sampling hook is a new loop-structure change; the rest is additive.

### Risk 1 — DP pre-req fix breaks a passing test

The S=1 + Rd=R15 warn path is exercised by *no* existing test (by construction — it's been a warn the whole time). But if slice 3a/3b test fixtures happened to trip it incidentally (e.g. setting CPSR bits and running a MOV with S=1), behavior changes.

**Mitigation:** the `/run-arm-tests` baseline check confirms all 31 tests green before commit 1. Commit 1 lands with only the fix; commit 2 onward depend on it. If commit 1 breaks anything, fix or defer; do not stack slice 3d on a regressed baseline.

**Rollback:** single-commit revert. Every subsequent slice-3d commit assumes the fix; rolling back commit 1 also requires rolling back the subsequent `enter_exception` return-path tests, which is the entire slice. The fix is load-bearing.

### Risk 2 — IRQ sampling fires mid-Thumb-BL

Slice 3c's THUMB.19 BL is stateless across halves on purpose (§4.4 of 3c spec). An IRQ sampled between the two halves must leave LR correctly set so the handler can return and the second halfword can complete.

**Mitigation:** the slice-3c BL implementation already writes LR at the end of the first halfword — not transient state. Entry with LR holding the partial target, followed by return, followed by the second halfword reading LR, computes the correct target. The sequence test in §6.8 does not exercise this specifically; add a targeted test if a real game surfaces it. The architectural argument in slice 3c §4.4(c) already covered this case.

**Rollback:** no revert possible — this is architectural. If BL breaks under IRQ interleaving, the fix is forward.

### Risk 3 — direct-boot vector indirection masking real bugs

`arm7_enter_irq` reads `[0x0380FFFC]` unconditionally. If the game hasn't written there, we read zero (fresh ARM7 WRAM after reset). PC = 0 jumps to the reset vector — also zero — and the CPU runs into whatever happens to live at 0x00000000 in direct-boot (garbage). That's a loud failure mode, which is actually good; the failure points straight at the bug.

**Mitigation:** warn-log once per run when the vector is zero. Documented in §5.9 as a game-side bug we don't paper over.

**Rollback:** the quirk is isolated to one function; removing the indirection line puts us in "IRQ entry but never the handler" territory, which is equally broken. Forward fix only.

### Risk 4 — IME / IE / IF width handling

Real DS code uses 32-bit accesses almost exclusively for these registers. GBATEK allows halfword and byte. An overzealous "byte writes are ignored" rule would break legitimate code; an overzealous "byte writes work" might mask subtle bus behavior.

**Mitigation:** the controller's public API is 32-bit. 16-bit and 8-bit accesses in `arm7_io_*` mask/shift appropriately and call the 32-bit setter. Warn-log 8-bit writes so any game that actually does one shows up in the logs.

**Rollback:** narrow the 8/16-bit paths if a real game regresses.

### Risk 5 — IRQ controller line not re-pushed after non-IO IF change

Future IRQ-source subsystems (timers, DMA) call `irq_ctrl_.raise(...)`. If they forget to call `update_arm7_irq_line()` afterward, the line stays stale. Slice 3d doesn't hit this because no source fires, but the contract must be documented.

**Mitigation:** `Arm7IrqController::raise` returns the new `line()` value, and the controller is only ever called through NDS, which wraps both the raise and the push. Alternatively, make `raise` itself take an `Arm7&` parameter — but that violates the "no subsystem holds a pointer to another" rule. Keeping the push-from-NDS contract is cleaner.

**Rollback:** refactor the contract if a source slice hits the issue.

---

## 9. Slice completion criteria

Slice 3d is complete when:

1. All 39 test binaries build and pass under `ctest --output-on-failure`.
2. The pre-requisite `execute_dp_op` S=1 + Rd=R15 fix (commit 1) preserves every slice 3a–3c test (all 31 stay green).
3. `enter_exception` correctly implements the entry actions from §5.2 for all seven `ExceptionKind` values. Each vector's test (§6.1–§6.5) passes.
4. `run_until` samples `irq_line_` at each instruction boundary, between ARM/Thumb step selection. `CPSR.I == 1` suppresses entry; `CPSR.I == 0` with `irq_line_ == true` enters IRQ mode via `arm7_enter_irq`.
5. `arm7_enter_irq` reads `[0x0380FFFC]` after `enter_exception` and writes the result to `state.pc`. The §6.4 direct-boot test passes.
6. ARM-state and Thumb-state SWI paths route to `enter_exception(Swi, ...)` and then to `arm7_bios_hle_dispatch_swi` with the correct SWI number.
7. `arm7_bios_hle_dispatch_swi` is a warn-stub — it logs per-number and returns. No SWI has a real implementation in this slice.
8. UNDEF entry is used for any ARM or Thumb encoding that currently warns without any decoder-family handling it (coproc on ARM, THUMB.16 cond=0xE, malformed multiplies, BLX Thumb halfword, etc.). The per-family warn-stub audit is done.
9. Prefetch abort and data abort entry helpers are reachable via `Arm7::raise_prefetch_abort` and `Arm7::raise_data_abort` (test-only methods). No real bus-fault source is wired.
10. FIQ entry helper is reachable via `Arm7::raise_fiq`. No real nFIQ source is wired.
11. `Arm7IrqController` implements IME/IE/IF with 32/16/8-bit reads and writes, write-1-clear on IF, and a `line()` getter.
12. NDS glue routes `arm7_io_read32`/`arm7_io_write32`/16/8 for `IO_IME`/`IO_IE`/`IO_IF` to the controller and calls `arm7_.set_irq_line(ctrl.line())` after every mutating operation.
13. `MOVS PC, R14`, `SUBS PC, R14, #4`, and `SUBS PC, R14, #8` correctly return from their respective exception modes, restoring CPSR from SPSR and re-banking R8–R14.
14. `src/cpu/arm7/arm7_exception.cpp` ≤ 200 lines, `src/interrupt/irq_controller.cpp` ≤ 100 lines, `src/cpu/arm7/bios/bios7_hle.cpp` ≤ 100 lines. All within the 500-soft / 800-hard house rule.
15. No new compiler warnings. All new files follow the existing src/cpu/arm7 style.
16. The full mandatory pipeline has run for every implementation commit: `/gbatek-check` (done, upfront) → `senior-architect` (done, upfront — this doc) → baseline → implement → `/simplify` → (`ds-architecture-rule-checker` ∥ `gbatek-reviewer`) → `quality-reviewer` → `ctest` → commit.

---

## Appendix A. Commit sequence

Twelve commits. Each builds and passes `ctest` independently. Order matters — the pre-req fix must land first, the helper and IO registers before any vector entry can be observably tested, and the capstone last.

### Phase A — pre-requisite + scaffolding (commits 1–3)

1. `cpu/arm7: execute_dp_op S=1 + Rd=R15 copies SPSR to CPSR with rebank`
2. `cpu/arm7: enter_exception helper + ExceptionKind tables (no call sites yet)`
3. `interrupt: Arm7IrqController (IME/IE/IF) + NDS glue + arm7_io routing`

### Phase B — synthetic-trigger exception paths (commits 4–6)

4. `cpu/arm7: undefined-instruction entry from ARM (replaces coproc warn fall-through)`
5. `cpu/arm7: thumb.16 cond=0xE and malformed encodings route to UNDEF entry`
6. `cpu/arm7: prefetch/data abort helpers + Arm7::raise_* test entry points`

### Phase C — SWI + FIQ + IRQ (commits 7–10)

7. `cpu/arm7: arm7_bios_hle_dispatch_swi stub file + ARM SWI entry via dispatch_swi_coproc`
8. `cpu/arm7: thumb.17 SWI entry replaces slice-3c warn stub`
9. `cpu/arm7: FIQ entry helper + Arm7::raise_fiq test entry point`
10. `cpu/arm7: IRQ line sampling in run_until + arm7_enter_irq with [0x0380FFFC] indirection`

### Phase D — return paths + capstone (commits 11–12)

11. `cpu/arm7: exception-return tests (MOVS/SUBS PC, R14) exercising the DP fix round-trip`
12. `cpu/arm7: slice 3d capstone — end-to-end IRQ ack/return sequence test`

Commits 4 and 5 could plausibly merge (both are UNDEF entry); kept separate so each exception kind has a crisp git-bisect target. Commit 11 is a test-only commit — no source changes, but its existence as a commit forces the implementation to be round-trip-clean.

---

## Appendix B. Exception-entry cheat sheet

```
Common entry actions (all kinds except Reset):
  old_cpsr = state.cpsr
  state.switch_mode(mode_for[kind])          # swap R13/R14 bank
  state.r[14] = return_addr                   # in the new bank
  *state.spsr_slot() = old_cpsr               # new mode always has SPSR
  state.cpsr &= ~(1u << 5)                    # T = 0
  state.cpsr |=  (1u << 7)                    # I = 1
  if set_f_for[kind]: state.cpsr |= (1u << 6) # F = 1 for Reset + FIQ
  state.pc = vector_for[kind]

Vector and mode table:
  Kind          Vector  Mode         set F
  Reset         0x00    Supervisor   yes
  Undef         0x04    Undefined    no
  Swi           0x08    Supervisor   no
  PrefetchAbort 0x0C    Abort        no
  DataAbort     0x10    Abort        no
  Irq           0x18    Irq          no
  Fiq           0x1C    Fiq          yes

Return addresses (instr_addr = faulting instruction's address):
  Kind          From ARM              From Thumb
  Swi           instr_addr + 4        instr_addr + 2
  Undef         instr_addr + 4        instr_addr + 2
  IRQ           state.pc + 4          state.pc + 4    (ARM-style, post-pc-advance)
  Prefetch      instr_addr + 4        n/a
  DataAbort     instr_addr + 8        instr_addr + 8
  FIQ           state.pc + 4          state.pc + 4

IRQ sample site (run_until top-of-loop):
  if irq_line_ && !(state.cpsr & (1 << 7)):
      arm7_enter_irq(state, bus)
      continue

arm7_enter_irq:
  return_addr = state.pc + 4
  enter_exception(state, Irq, return_addr)
  state.pc = bus.read32(0x0380FFFC)    # direct-boot workaround

Return instructions:
  SWI / UNDEF:       MOVS PC, R14            (resume at next instr)
  IRQ / FIQ:         SUBS PC, R14, #4        (resume at next instr)
  Prefetch:          SUBS PC, R14, #4        (re-execute faulting)
  DataAbort:         SUBS PC, R14, #8        (re-execute faulting)

  Each of these is a DP op with S=1, Rd=R15. execute_dp_op handles:
    if s_flag && rd == 15 && writeback:
        new_cpsr = *spsr_slot()
        switch_mode(mode from new_cpsr)
        state.cpsr = new_cpsr

IME/IE/IF (0x04000208/210/214, ARM7 side):
  IME: bit 0 = master enable
  IE:  32 source-enable bits
  IF:  32 source-pending bits, write-1-clear
  IRQ line = IME.0 && (IE & IF) != 0
```
