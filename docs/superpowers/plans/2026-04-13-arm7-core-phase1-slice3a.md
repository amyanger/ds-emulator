# ARM7 core — skeleton + ARM data processing (Phase 1, slice 3a) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Phase-0 `Arm7::run_until` no-op with a real ARMv4T interpreter that fetches instructions from `Arm7Bus`, decodes them, and executes the full set of ARM data-processing opcodes. At the end of this slice, a hand-assembled sequence of `MOV`/`ADD`/`SUB`/`AND`/`ORR` etc. placed in ARM7 WRAM runs correctly on the real CPU core via `NDS::run_frame`.

**Architecture:** The CPU is a classic interpreter. `class Arm7` owns a flat state struct (`Arm7State`) holding the current-mode register file (`r[16]`), every banked register set, CPSR, the SPSR bank, and a cycle counter. `run_until` is a tight loop that fetches a 32-bit instruction from `Arm7Bus` at `pc_`, advances `pc_ += 4` (keeping ARMv4T's "R15 reads return current-instruction + 8" semantics), checks the condition code, and dispatches on the top decode bits to an instruction handler. Slice 3a only implements data-processing (opcode field `0b00x`), the barrel shifter, and flag updates. Loads/stores, branches, Thumb, and exceptions are explicitly deferred to slices 3b–3d.

**Tech Stack:** C++20, `std::array` for the register file, `std::memcpy` already used by `Arm7Bus` for aligned reads, no new third-party deps. Unit tests link `ds_core` only via the existing `add_ds_unit_test()` CMake helper.

**Parent spec:** `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` §5.2, §5.4, §13 (Phase 1)

---

## Scope: what this slice covers vs what is deferred

Slice 3 (the "ARM7 core") is broken into four sub-slices because a full ARMv4T + Thumb interpreter is too much code to land in one plan without the reviewer losing context:

| Sub-slice | Scope | Milestone |
|---|---|---|
| **3a (this plan)** | CPU state + banking + pipeline + condition codes + barrel shifter + ARM data processing (`AND`/`EOR`/`SUB`/`RSB`/`ADD`/`ADC`/`SBC`/`RSC`/`TST`/`TEQ`/`CMP`/`CMN`/`ORR`/`MOV`/`BIC`/`MVN`) with all three operand2 modes (immediate, register, immediate-shifted register) | Hand-assembled arithmetic sequence executes end-to-end through `NDS::run_frame` |
| 3b | ARM branches (`B`/`BL`/`BX`), loads/stores (`LDR`/`STR` word + byte, `LDRH`/`STRH`/`LDRSB`/`LDRSH`, `LDM`/`STM`), `MUL`/`MLA`, `MRS`/`MSR` | Run test code that writes to memory, branches, and calls subroutines |
| 3c | Thumb ISA (19 format groups) | Same tests as 3b but executing Thumb bytecode |
| 3d | Exception entry (reset/undef/SWI/prefetch abort/data abort/IRQ/FIQ), pipeline flush modeling | `SWI` into HLE handler returns cleanly; software IRQ raises via `Arm7State::irq_line` |

Register-shifted-register operand2 (e.g. `add r0, r1, r2, lsl r3`) is deferred to the start of slice 3b because it has different cycle semantics (+1I cycle) and interacts with the load/store cycle counting work in 3b. Slice 3a covers immediate-shifted-register operand2 only.

Register-writing to R15 (i.e. `mov r15, r0` as a disguised branch) is handled in 3a for data processing — it is not a real branch instruction but it does set `pc_` and is ubiquitous in compiled DS code.

**IRQ line checking is NOT done in slice 3a.** The CPU state has a placeholder `irq_line` field reserved for 3d; the fetch loop does not consult it yet.

---

## Context for the engineer

- **Read CLAUDE.md first.** The "Architecture Rules (non-negotiable)" and "Code Style" sections are binding. Critical rules for this slice:
  - Rule 1: the scheduler is the clock. `Arm7::run_until(arm9_target)` converts to ARM7 cycles and must not advance past its target. Do not introduce any wall-clock, ad-hoc loop, or sleep.
  - Rule 2: two bus domains. ARM7 talks to `Arm7Bus`, never `Arm9Bus`. Not even for "just debugging."
  - Rule 3: no cross-subsystem pointers to peers. `Arm7` holds a pointer/reference to `Arm7Bus`, not to `Arm9` or to `NDS`. (Attaching via `NDS` in the constructor is fine — the bus reference is to the peer subsystem it directly needs to drive, same pattern buses use with `NDS&`.)
  - Rule 5: `reset()` and state structs from day one. `Arm7State` needs a clean reset path even though save-state serialization is deferred.
  - Rule 6: no SDL in `ds_core`.
  - 4-space indent, pointer alignment left (`u8* p`), fixed-width types from `ds/common.hpp` (`u8`/`u16`/`u32`/`u64`).
  - Compiler warnings treated as errors (`-Wall -Wextra -Wpedantic -Werror`). In particular: unused-parameter warnings on stub handlers will fail the build — use `[[maybe_unused]]` or `(void)name;`.
- **Read the parent spec §5.2 and §5.4.** §5.2 describes the CpuCore interface, §5.4 covers the ARMv4T specifics relevant here. CP15, TCM, and ARMv5TE extensions are ARM9-only and do not apply to ARM7.
- **Primary hardware reference:** GBATEK "ARM CPU Overview" and "ARM Opcodes" sections (https://problemkaputt.de/gbatek.htm#armcpureference). Verify every opcode encoding against GBATEK — do not write the decoder from memory.
- **Build from `build/`**, not the repo root:
  ```bash
  cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
  ```
- **Do not commit** `CLAUDE.md` — it is gitignored as local-only context. Check `git status` before every commit.
- **Do not add Claude (or any AI) as a co-author** on any commit. No `Co-Authored-By` lines, no `--author="Claude..."`.
- **Do not amend previous commits.** When a step fails, fix and create a *new* commit.
- **Do not wire this CPU into `NDS::run_frame` yet in an observable way.** The `on_scheduler_event` path already calls `cpu7_.run_until(target)`; that call currently advances a fake counter. After slice 3a it will fetch real instructions from whatever is in ARM7 WRAM, which at reset is zeros (→ `AND R0, R0, R0` which is a valid nop). The only tests that exercise this end-to-end are the ones this plan adds explicitly in Task 9.

---

## ARMv4T primer — just the bits slice 3a needs

Skim this before Task 1 if you have not written an ARM interpreter before. Everything here is documented in GBATEK; this is the pocket card.

**Register file.** 16 visible registers R0–R15. R13 = SP, R14 = LR, R15 = PC. Of the 16, some are banked by processor mode:

| Mode | Hex | Banked registers | SPSR |
|---|---|---|---|
| User    | 0x10 | — (R0–R14, R15) | — |
| FIQ     | 0x11 | R8_fiq–R14_fiq  | SPSR_fiq |
| IRQ     | 0x12 | R13_irq, R14_irq | SPSR_irq |
| Supervisor (SVC) | 0x13 | R13_svc, R14_svc | SPSR_svc |
| Abort   | 0x17 | R13_abt, R14_abt | SPSR_abt |
| Undefined (UND) | 0x1B | R13_und, R14_und | SPSR_und |
| System  | 0x1F | — (shares User) | — |

"Banked" means: when CPSR mode = IRQ, reads of R13 see `r13_irq`; when CPSR mode = User, reads of R13 see `r13_user`. Switching modes does not move data — it switches which physical register the logical name points to. Slice 3a implements the banking infrastructure but only *uses* SVC mode (the reset default); actual mode switches happen in slice 3d when exceptions land.

**CPSR layout** (32 bits, big-to-small):

```
31 30 29 28 27 ... 8 7 6 5 4 3 2 1 0
 N  Z  C  V  Q   reserved  I F T M4 M3 M2 M1 M0
```

- N = sign (bit 31 of last ALU result)
- Z = zero (1 iff last ALU result == 0)
- C = carry (from adder, from barrel shifter on logical ops)
- V = overflow (signed overflow from adder)
- Q = sticky saturation (ARMv5TE only; ARM7 ignores, always reads 0)
- I = IRQ disable (1 = disabled)
- F = FIQ disable (1 = disabled)
- T = Thumb state (1 = Thumb). ARM7 boots in ARM state (T=0).
- M[4:0] = mode bits from the table above.

**Pipeline semantics (R15 read).** ARMv4T has a 3-stage pipeline. When an instruction executes, the PC visible as R15 is the instruction's *own address + 8* in ARM state (+ 4 in Thumb). In practice our interpreter models this by: fetching at `pc_`, setting `r_[15] = pc_ + 8`, then incrementing `pc_ += 4`. When execution reads R15 as a source operand, it gets `instruction_addr + 8`. When execution writes R15 (as the data-processing destination), we update `pc_` directly so the next fetch jumps — and we still need to handle the case where `pc_` was modified mid-execution, so `pc_ += 4` is done *before* execute, not after.

**Condition codes (top 4 bits of every ARM instruction).**

| Hex | Mnemonic | Condition |
|---|---|---|
| 0 | EQ | Z == 1 |
| 1 | NE | Z == 0 |
| 2 | CS/HS | C == 1 |
| 3 | CC/LO | C == 0 |
| 4 | MI | N == 1 |
| 5 | PL | N == 0 |
| 6 | VS | V == 1 |
| 7 | VC | V == 0 |
| 8 | HI | C == 1 AND Z == 0 |
| 9 | LS | C == 0 OR  Z == 1 |
| A | GE | N == V |
| B | LT | N != V |
| C | GT | Z == 0 AND N == V |
| D | LE | Z == 1 OR  N != V |
| E | AL | always |
| F | NV | never (reserved on ARMv4T; treat as never-execute) |

**Data-processing instruction encoding.**

```
 31..28 | 27..26 | 25 | 24..21 | 20 | 19..16 | 15..12 | 11..0
  cond  |   00   | I  | opcode |  S |   Rn   |   Rd   | operand2
```

- `cond` = condition from the table above
- bits 27..26 = `00` (data-processing)
- `I` = 1 if operand2 is an immediate, 0 if it is a shifted register
- `opcode` = 4-bit ALU op (table below)
- `S` = 1 to update CPSR flags based on the result
- `Rn` = first operand register
- `Rd` = destination register
- `operand2` = 12 bits, interpretation depends on `I`

If `I == 1`: bits [11:8] are a rotation amount (doubled, so 0..30 step 2) applied to the 8-bit immediate in bits [7:0]. The rotation is a right-rotate. If the rotation is non-zero and `S == 1`, the shifter's carry-out is bit 31 of the rotated value; otherwise the shifter carry is the CPSR C flag unchanged.

If `I == 0`: bits [11:0] describe a shifted register operand.
- If bit 4 == 0 (immediate-shift, which is what slice 3a handles):
  - [11:7] = shift amount (0..31)
  - [6:5] = shift type (0 LSL, 1 LSR, 2 ASR, 3 ROR)
  - [3:0] = Rm
- If bit 4 == 1 (register-shift, deferred to slice 3b): skip; treat as "unknown data-processing form" and NOP for now with a `DS_LOG_WARN`.

**Opcode table.**

| Hex | Mnemonic | Operation | Writes Rd? | Flag behavior (if S) |
|---|---|---|---|---|
| 0 | AND | Rd = Rn AND op2 | yes | logical |
| 1 | EOR | Rd = Rn XOR op2 | yes | logical |
| 2 | SUB | Rd = Rn - op2   | yes | arithmetic (sub) |
| 3 | RSB | Rd = op2 - Rn   | yes | arithmetic (sub, reversed) |
| 4 | ADD | Rd = Rn + op2   | yes | arithmetic (add) |
| 5 | ADC | Rd = Rn + op2 + C | yes | arithmetic (add) |
| 6 | SBC | Rd = Rn - op2 - !C | yes | arithmetic (sub) |
| 7 | RSC | Rd = op2 - Rn - !C | yes | arithmetic (sub, reversed) |
| 8 | TST | Rn AND op2 (discarded) | **no** | logical (S must be 1) |
| 9 | TEQ | Rn XOR op2 (discarded) | **no** | logical (S must be 1) |
| A | CMP | Rn - op2 (discarded) | **no** | arithmetic (sub, S must be 1) |
| B | CMN | Rn + op2 (discarded) | **no** | arithmetic (add, S must be 1) |
| C | ORR | Rd = Rn OR op2 | yes | logical |
| D | MOV | Rd = op2       | yes (Rn ignored) | logical |
| E | BIC | Rd = Rn AND NOT op2 | yes | logical |
| F | MVN | Rd = NOT op2   | yes (Rn ignored) | logical |

**Flag semantics.**

"Logical" (AND/EOR/TST/TEQ/ORR/MOV/BIC/MVN):
- N = bit 31 of result
- Z = (result == 0)
- C = shifter carry-out (from barrel shifter)
- V = unchanged

"Arithmetic add" (ADD/ADC/CMN):
- N = bit 31 of result
- Z = (result == 0)
- C = carry-out from the 33-bit addition
- V = signed overflow: `((a ^ ~b) & (a ^ result)) >> 31` for `a + b`

"Arithmetic sub" (SUB/RSB/SBC/RSC/CMP):
- N = bit 31 of result
- Z = (result == 0)
- C = NOT borrow (i.e. 1 when there is no borrow) — equivalent to `a >= b` for unsigned
- V = signed overflow: `((a ^ b) & (a ^ result)) >> 31` for `a - b`

For ADC/SBC/RSC, the carry-in is the current C flag (for SBC/RSC it is `C` interpreted as "not borrow"; the math works out to `a - b - (1 - C)` i.e. `a + ~b + C`).

**PC as destination.** If `Rd == 15`, writing to R15 updates `pc_`. If also `S == 1`, the CPU copies SPSR of the current mode back into CPSR (this is the classic "return from exception" pattern). Slice 3a does not model `S && Rd == 15` — it is only used by exception-return sequences which we have no exceptions for yet. Log a warning and treat as plain R15 write if encountered.

**PC as source.** If `Rm == 15` or `Rn == 15`, the read returns `current_instruction_addr + 8`. Because our execute step has already set `pc_ += 4` before calling the handler, `pc_ + 4` is the right value. (We keep the invariant "`r_[15]` is written to `pc_ + 4` at the top of each execute iteration" so handlers can just `state.r[15]`.)

**That is everything slice 3a needs.** Multiplies, loads/stores, branches, Thumb, and exception entry are all out of scope.

---

## File structure

### Files created

| Path | Responsibility |
|---|---|
| `src/cpu/arm7/arm7_state.hpp` | `enum class Mode`, `struct Arm7State` (all banked register sets, CPSR, SPSR bank, pipeline `pc_`, cycle counter). Pure data + trivial banking helpers `switch_mode(...)` / `current_gpr(...)`. |
| `src/cpu/arm7/arm7_alu.hpp` | Header-only inline functions: `barrel_shift_imm(...)`, `barrel_shift_rot_imm(...)`, `eval_condition(...)`, logical/arithmetic flag updaters. No side effects on CPU state — all return values or take out-params so they are trivially unit-testable. |
| `src/cpu/arm7/arm7_decode.cpp` | `Arm7::decode_and_execute_arm(u32 instr)` — top-level dispatch that consults the condition bits, checks bits [27:26] for data-processing, then calls `exec_data_processing(...)`. Data-processing execution is implemented here. |
| `tests/unit/arm7_register_file_test.cpp` | Banked-register R/W across mode switches. |
| `tests/unit/arm7_barrel_shifter_test.cpp` | LSL/LSR/ASR/ROR + immediate-rotated-immediate, including carry out on boundary cases (shift amount 0, 32, >32). |
| `tests/unit/arm7_condition_test.cpp` | All 16 condition codes against canned CPSR states. |
| `tests/unit/arm7_data_processing_test.cpp` | Hand-assembled MOV/ADD/SUB/AND/ORR/EOR + S-flag variants. Exercises `Arm7` directly without involving `NDS` for quick unit-test feedback. |
| `tests/unit/arm7_run_sequence_test.cpp` | Integration: load a multi-instruction blob into ARM7 WRAM via `Arm7Bus`, run `NDS::run_frame` (or `Arm7::run_until` directly), verify post-state. This is the slice-3a capstone test. |

### Files modified

| Path | Change |
|---|---|
| `src/cpu/arm7/arm7.hpp` | Replace Phase-0 stub with real class: owns `Arm7State`, holds `Arm7Bus*`, exposes `attach_bus()`, `state()` (for tests), the existing `run_until()` and `reset()`. |
| `src/cpu/arm7/arm7.cpp` | Real `reset()` (boots to SVC mode, CPSR = 0xD3, PC = 0). Real `run_until()` fetch/decode/execute loop. Implementation of `decode_and_execute_arm()` lives in `arm7_decode.cpp`; the fetch loop lives here. |
| `src/nds.hpp` | No change to interface. |
| `src/nds.cpp` | In the `NDS` constructor body, call `cpu7_.attach_bus(arm7_bus_);` after both are constructed. In `reset()`, the existing `cpu7_.reset()` call now does real work — no code change needed. |
| `src/CMakeLists.txt` | Add `cpu/arm7/arm7_decode.cpp` to the `ds_core` sources list. |
| `tests/CMakeLists.txt` | Register the five new unit-test binaries via `add_ds_unit_test()`. |

---

## Task 1: CPU state types + register file banking

**Files:**
- Create: `src/cpu/arm7/arm7_state.hpp`
- Create: `tests/unit/arm7_register_file_test.cpp`
- Modify: `tests/CMakeLists.txt`

Goal: land `Arm7State` with all banked register sets and a `switch_mode()` helper that transparently swaps in the right physical bank when CPSR mode changes. No execution yet.

- [ ] **Step 1: Write the failing test**

Create `tests/unit/arm7_register_file_test.cpp`:

```cpp
// Banked register behavior for Arm7State. Covers mode switching
// across User/SVC/IRQ/FIQ, preserving non-banked registers, and
// the reset default (SVC, CPSR=0xD3).

#include "cpu/arm7/arm7_state.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void reset_defaults_to_svc_with_irqs_disabled() {
    Arm7State s;
    s.reset();
    REQUIRE(s.cpsr == 0xD3u);
    REQUIRE(s.current_mode() == Mode::Supervisor);
    REQUIRE(s.r[15] == 0u);
}

static void non_banked_registers_are_visible_across_modes() {
    Arm7State s;
    s.reset();
    s.r[0] = 0xAABBCCDDu;
    s.switch_mode(Mode::User);
    REQUIRE(s.r[0] == 0xAABBCCDDu);
    s.switch_mode(Mode::Irq);
    REQUIRE(s.r[0] == 0xAABBCCDDu);
}

static void r13_r14_are_banked_between_svc_and_irq() {
    Arm7State s;
    s.reset();  // starts in SVC
    s.r[13] = 0x0300'7F00u;  // SVC stack
    s.r[14] = 0x0800'1234u;  // SVC lr

    s.switch_mode(Mode::Irq);
    REQUIRE(s.r[13] == 0u);  // IRQ bank initialized to zero
    REQUIRE(s.r[14] == 0u);
    s.r[13] = 0x0380'FF00u;
    s.r[14] = 0x0800'ABCDu;

    s.switch_mode(Mode::Supervisor);
    REQUIRE(s.r[13] == 0x0300'7F00u);  // SVC bank restored
    REQUIRE(s.r[14] == 0x0800'1234u);
}

static void fiq_banks_r8_through_r14() {
    Arm7State s;
    s.reset();
    // Fill R8..R14 with recognizable values while in SVC mode.
    for (u32 i = 8; i <= 14; ++i) s.r[i] = 0x1000u + i;

    s.switch_mode(Mode::Fiq);
    // FIQ has its own R8..R14 bank, which is fresh (zero).
    for (u32 i = 8; i <= 14; ++i) REQUIRE(s.r[i] == 0u);

    // Stomp them in FIQ mode with different values.
    for (u32 i = 8; i <= 14; ++i) s.r[i] = 0x2000u + i;

    s.switch_mode(Mode::Supervisor);
    // Back in SVC. R8..R12 are shared with the User bank — the SVC writes
    // of 0x1008..0x100C should come back because switching SVC→FIQ saved
    // them to the User bank (SVC uses the User bank for R8..R12).
    for (u32 i = 8; i <= 12; ++i) REQUIRE(s.r[i] == 0x1000u + i);
    // R13/R14 use the SVC-specific bank, restored from 0x100D/0x100E.
    REQUIRE(s.r[13] == 0x100Du);
    REQUIRE(s.r[14] == 0x100Eu);
}

int main() {
    reset_defaults_to_svc_with_irqs_disabled();
    non_banked_registers_are_visible_across_modes();
    r13_r14_are_banked_between_svc_and_irq();
    fiq_banks_r8_through_r14();
    std::puts("arm7_register_file_test OK");
    return 0;
}
```

Modify `tests/CMakeLists.txt`:

```cmake
add_ds_unit_test(fixed_test)
add_ds_unit_test(ring_buffer_test)
add_ds_unit_test(scheduler_test)
add_ds_unit_test(page_table_test)
add_ds_unit_test(arm9_bus_test)
add_ds_unit_test(arm7_bus_test)
add_ds_unit_test(wramcnt_test)
add_ds_unit_test(nds_integration_test)
add_ds_unit_test(arm7_register_file_test)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd build && cmake .. && make arm7_register_file_test 2>&1 | tail -20`
Expected: compilation failure — `cpu/arm7/arm7_state.hpp: No such file`.

- [ ] **Step 3: Create `src/cpu/arm7/arm7_state.hpp` with the banked file**

```cpp
#pragma once

// Arm7State — all CPU-visible state for the ARMv4T core.
// This is pure data plus tiny helpers. `class Arm7` (in arm7.hpp) owns
// an Arm7State and adds the fetch/decode/execute behavior on top.

#include "ds/common.hpp"

#include <array>

namespace ds {

enum class Mode : u8 {
    User       = 0x10,
    Fiq        = 0x11,
    Irq        = 0x12,
    Supervisor = 0x13,
    Abort      = 0x17,
    Undefined  = 0x1B,
    System     = 0x1F,
};

// Banks holding the physical storage for mode-banked registers.
// User and System share the same bank, so only six SPSRs exist (one each
// for FIQ/IRQ/SVC/ABT/UND) and the User/System bank has no SPSR.
struct Arm7Banks {
    // User/System R8..R14 live here when CPSR mode = User or System.
    std::array<u32, 7> user_r8_r14{};    // [0]=R8 .. [6]=R14
    // FIQ has its own R8..R14.
    std::array<u32, 7> fiq_r8_r14{};     // [0]=R8 .. [6]=R14
    // IRQ/SVC/ABT/UND only bank R13 and R14.
    std::array<u32, 2> irq_r13_r14{};    // [0]=R13, [1]=R14
    std::array<u32, 2> svc_r13_r14{};
    std::array<u32, 2> abt_r13_r14{};
    std::array<u32, 2> und_r13_r14{};

    u32 spsr_fiq = 0;
    u32 spsr_irq = 0;
    u32 spsr_svc = 0;
    u32 spsr_abt = 0;
    u32 spsr_und = 0;
};

struct Arm7State {
    // Current-mode visible register file. `r[13]`, `r[14]`, and the FIQ
    // registers are reloaded from `banks` on every `switch_mode()` call.
    // `r[15]` is kept in sync with `pc_` at the top of every execute step;
    // tests should read `state.r[15]` after an instruction executes.
    std::array<u32, 16> r{};

    // Program counter — address of the next instruction to fetch. In ARM
    // state this is always 4-aligned. `r[15]` at the moment an instruction
    // executes is `pc_ + 4` (i.e. "current instruction address + 8").
    u32 pc = 0;

    // CPSR. Reset default is 0xD3: SVC mode (M=0x13), IRQ disabled (I=1),
    // FIQ disabled (F=1), Thumb off (T=0), flags clear.
    u32 cpsr = 0xD3;

    // All banked register sets.
    Arm7Banks banks{};

    // Cycle counter — ARM7 cycles since reset. ARM9 cycles / 2.
    u64 cycles = 0;

    // Reset to DS post-boot defaults. Direct boot later overrides pc and
    // some registers based on the cart header; here we just give sane
    // power-on values.
    void reset() {
        r = {};
        pc = 0;
        cpsr = 0xD3;
        banks = {};
        cycles = 0;
        r[15] = 0;
    }

    Mode current_mode() const {
        return static_cast<Mode>(cpsr & 0x1Fu);
    }

    // Save the currently visible registers into the bank for `from_mode`,
    // then load the registers for `to_mode` from its bank and update
    // CPSR's mode bits.
    void switch_mode(Mode to_mode) {
        const Mode from_mode = current_mode();
        store_banked_registers(from_mode);
        load_banked_registers(to_mode);
        cpsr = (cpsr & ~0x1Fu) | static_cast<u32>(to_mode);
    }

private:
    void store_banked_registers(Mode m) {
        // All non-FIQ modes share the User bank for R8..R12, so every
        // "save" path except FIQ's must write R8..R12 back into the
        // User bank before we load a new mode's R13/R14.
        switch (m) {
            case Mode::User:
            case Mode::System:
                for (int i = 0; i < 7; ++i) banks.user_r8_r14[i] = r[8 + i];
                break;
            case Mode::Fiq:
                for (int i = 0; i < 7; ++i) banks.fiq_r8_r14[i] = r[8 + i];
                break;
            case Mode::Irq:
                for (int i = 0; i < 5; ++i) banks.user_r8_r14[i] = r[8 + i];
                banks.irq_r13_r14[0] = r[13]; banks.irq_r13_r14[1] = r[14];
                break;
            case Mode::Supervisor:
                for (int i = 0; i < 5; ++i) banks.user_r8_r14[i] = r[8 + i];
                banks.svc_r13_r14[0] = r[13]; banks.svc_r13_r14[1] = r[14];
                break;
            case Mode::Abort:
                for (int i = 0; i < 5; ++i) banks.user_r8_r14[i] = r[8 + i];
                banks.abt_r13_r14[0] = r[13]; banks.abt_r13_r14[1] = r[14];
                break;
            case Mode::Undefined:
                for (int i = 0; i < 5; ++i) banks.user_r8_r14[i] = r[8 + i];
                banks.und_r13_r14[0] = r[13]; banks.und_r13_r14[1] = r[14];
                break;
        }
    }

    void load_banked_registers(Mode m) {
        // Non-banked registers (R0..R7, R15) are never touched.
        // R8..R12 are only banked in FIQ mode; all other modes share the
        // User/System copies.
        switch (m) {
            case Mode::User:
            case Mode::System:
                for (int i = 0; i < 7; ++i) r[8 + i] = banks.user_r8_r14[i];
                break;
            case Mode::Fiq:
                for (int i = 0; i < 7; ++i) r[8 + i] = banks.fiq_r8_r14[i];
                break;
            case Mode::Irq:
                for (int i = 0; i < 5; ++i) r[8 + i] = banks.user_r8_r14[i];  // R8..R12
                r[13] = banks.irq_r13_r14[0]; r[14] = banks.irq_r13_r14[1];
                break;
            case Mode::Supervisor:
                for (int i = 0; i < 5; ++i) r[8 + i] = banks.user_r8_r14[i];
                r[13] = banks.svc_r13_r14[0]; r[14] = banks.svc_r13_r14[1];
                break;
            case Mode::Abort:
                for (int i = 0; i < 5; ++i) r[8 + i] = banks.user_r8_r14[i];
                r[13] = banks.abt_r13_r14[0]; r[14] = banks.abt_r13_r14[1];
                break;
            case Mode::Undefined:
                for (int i = 0; i < 5; ++i) r[8 + i] = banks.user_r8_r14[i];
                r[13] = banks.und_r13_r14[0]; r[14] = banks.und_r13_r14[1];
                break;
        }
    }
};

}  // namespace ds
```

Note: the test `r13_r14_are_banked_between_svc_and_irq` checks that switching from SVC (R13=0x0300'7F00) to IRQ shows IRQ's R13 as 0 (fresh), then switching back to SVC restores R13 = 0x0300'7F00. The store/load logic above supports exactly that.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd build && cmake .. && make arm7_register_file_test && ctest -R arm7_register_file_test --output-on-failure`
Expected: `arm7_register_file_test` passes, full test suite still 9/9.

- [ ] **Step 5: Commit**

```bash
git add src/cpu/arm7/arm7_state.hpp tests/unit/arm7_register_file_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: Arm7State with banked register file + unit test

Adds Mode enum, Arm7Banks storage, and Arm7State with a switch_mode()
helper that preserves non-banked registers across mode changes. No
execution yet — this slice only lands the state structure so later
tasks can wire fetch/decode on top."
```

---

## Task 2: Barrel shifter + condition evaluator (pure helpers)

**Files:**
- Create: `src/cpu/arm7/arm7_alu.hpp`
- Create: `tests/unit/arm7_barrel_shifter_test.cpp`
- Create: `tests/unit/arm7_condition_test.cpp`
- Modify: `tests/CMakeLists.txt`

Goal: land the barrel shifter (LSL/LSR/ASR/ROR + immediate-rotated-immediate) and the 16-case condition evaluator as header-only inline helpers. These are pure functions with no access to `Arm7State`, so they are trivially testable and can be called both from the data-processing handler and from load/store addressing-mode code in slice 3b.

- [ ] **Step 1: Write the failing barrel-shifter test**

Create `tests/unit/arm7_barrel_shifter_test.cpp`:

```cpp
// Barrel shifter edge cases. ARMv4T shifts have some non-obvious
// corner cases documented in GBATEK — specifically LSR #0, ASR #0,
// ROR #0 (which is RRX), and LSL #32 (carry-out is bit 0 of operand).

#include "cpu/arm7/arm7_alu.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void lsl_zero_is_identity_with_unchanged_carry() {
    ShifterResult r = barrel_shift_imm(0xF000'0000u, ShiftType::Lsl, 0, /*c_in=*/true);
    REQUIRE(r.value == 0xF000'0000u);
    REQUIRE(r.carry == true);
    r = barrel_shift_imm(0xF000'0000u, ShiftType::Lsl, 0, /*c_in=*/false);
    REQUIRE(r.carry == false);
}

static void lsl_by_1_carries_out_bit_31() {
    ShifterResult r = barrel_shift_imm(0x8000'0001u, ShiftType::Lsl, 1, /*c_in=*/false);
    REQUIRE(r.value == 0x0000'0002u);
    REQUIRE(r.carry == true);
}

static void lsl_by_32_carries_out_bit_0_and_zeroes_value() {
    // Encoded as immediate-shift amount 0 with LSL is LSL #0 (identity),
    // but register-shift amount 32 with LSL produces zero result and
    // carry-out = bit 0 of operand. Our immediate helper does not see
    // amount 32 (imm5 max = 31), so we test this via the "amount" param
    // the register-shift path will use in slice 3b.
    ShifterResult r = barrel_shift_reg(0x0000'0001u, ShiftType::Lsl, 32);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);
}

static void lsr_imm_zero_is_lsr_32() {
    // ARMv4T: LSR #0 in immediate-shift form is actually LSR #32.
    // Result is zero, carry is bit 31 of operand.
    ShifterResult r = barrel_shift_imm(0x8000'0000u, ShiftType::Lsr, 0, /*c_in=*/false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == true);
}

static void asr_imm_zero_is_asr_32_sign_extends() {
    ShifterResult r = barrel_shift_imm(0x8000'0000u, ShiftType::Asr, 0, /*c_in=*/false);
    REQUIRE(r.value == 0xFFFF'FFFFu);
    REQUIRE(r.carry == true);
    r = barrel_shift_imm(0x7FFF'FFFFu, ShiftType::Asr, 0, /*c_in=*/false);
    REQUIRE(r.value == 0u);
    REQUIRE(r.carry == false);
}

static void ror_imm_zero_is_rrx() {
    // ARMv4T: ROR #0 in immediate-shift form is RRX (rotate-right-extend):
    // result = (C << 31) | (op >> 1), new C = bit 0 of operand.
    ShifterResult r = barrel_shift_imm(0x0000'0001u, ShiftType::Ror, 0, /*c_in=*/true);
    REQUIRE(r.value == 0x8000'0000u);
    REQUIRE(r.carry == true);
    r = barrel_shift_imm(0x0000'0002u, ShiftType::Ror, 0, /*c_in=*/false);
    REQUIRE(r.value == 0x0000'0001u);
    REQUIRE(r.carry == false);
}

static void rotated_immediate_no_rotation_passes_carry_through() {
    // Data-processing immediate with rotate==0: result is the imm8
    // zero-extended, carry-out is c_in unchanged.
    ShifterResult r = rotated_imm(0xFFu, 0, /*c_in=*/true);
    REQUIRE(r.value == 0xFFu);
    REQUIRE(r.carry == true);
}

static void rotated_immediate_with_rotation_updates_carry() {
    // imm8=0xFF, rotate=4 (bits [11:8]=2, doubled), result=ROR(0xFF, 4).
    ShifterResult r = rotated_imm(0xFFu, 4, /*c_in=*/false);
    REQUIRE(r.value == 0xF000'000Fu);
    REQUIRE(r.carry == true);  // bit 31 of rotated result
}

int main() {
    lsl_zero_is_identity_with_unchanged_carry();
    lsl_by_1_carries_out_bit_31();
    lsl_by_32_carries_out_bit_0_and_zeroes_value();
    lsr_imm_zero_is_lsr_32();
    asr_imm_zero_is_asr_32_sign_extends();
    ror_imm_zero_is_rrx();
    rotated_immediate_no_rotation_passes_carry_through();
    rotated_immediate_with_rotation_updates_carry();
    std::puts("arm7_barrel_shifter_test OK");
    return 0;
}
```

- [ ] **Step 2: Write the failing condition test**

Create `tests/unit/arm7_condition_test.cpp`:

```cpp
// 16-case condition code evaluator. Every case exercised against a
// CPSR with the relevant flag bit toggled.

#include "cpu/arm7/arm7_alu.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// CPSR flag bit positions.
static constexpr u32 N_BIT = 31;
static constexpr u32 Z_BIT = 30;
static constexpr u32 C_BIT = 29;
static constexpr u32 V_BIT = 28;

static constexpr u32 flag(u32 pos) { return 1u << pos; }

static void al_always_true_nv_always_false() {
    REQUIRE(eval_condition(0xE, 0) == true);
    REQUIRE(eval_condition(0xE, 0xFFFF'FFFFu) == true);
    REQUIRE(eval_condition(0xF, 0) == false);
    REQUIRE(eval_condition(0xF, 0xFFFF'FFFFu) == false);
}

static void eq_ne_track_z() {
    REQUIRE(eval_condition(0x0, flag(Z_BIT)) == true);
    REQUIRE(eval_condition(0x0, 0) == false);
    REQUIRE(eval_condition(0x1, flag(Z_BIT)) == false);
    REQUIRE(eval_condition(0x1, 0) == true);
}

static void cs_cc_track_c() {
    REQUIRE(eval_condition(0x2, flag(C_BIT)) == true);
    REQUIRE(eval_condition(0x2, 0) == false);
    REQUIRE(eval_condition(0x3, flag(C_BIT)) == false);
    REQUIRE(eval_condition(0x3, 0) == true);
}

static void mi_pl_track_n() {
    REQUIRE(eval_condition(0x4, flag(N_BIT)) == true);
    REQUIRE(eval_condition(0x4, 0) == false);
    REQUIRE(eval_condition(0x5, flag(N_BIT)) == false);
    REQUIRE(eval_condition(0x5, 0) == true);
}

static void vs_vc_track_v() {
    REQUIRE(eval_condition(0x6, flag(V_BIT)) == true);
    REQUIRE(eval_condition(0x6, 0) == false);
    REQUIRE(eval_condition(0x7, flag(V_BIT)) == false);
    REQUIRE(eval_condition(0x7, 0) == true);
}

static void hi_ls_compound() {
    // HI = C && !Z
    REQUIRE(eval_condition(0x8, flag(C_BIT))                == true);
    REQUIRE(eval_condition(0x8, flag(C_BIT) | flag(Z_BIT))  == false);
    REQUIRE(eval_condition(0x8, 0)                          == false);
    // LS = !C || Z
    REQUIRE(eval_condition(0x9, flag(C_BIT))                == false);
    REQUIRE(eval_condition(0x9, flag(C_BIT) | flag(Z_BIT))  == true);
    REQUIRE(eval_condition(0x9, 0)                          == true);
}

static void ge_lt_gt_le_track_nv() {
    // N=V → GE true, LT false.
    REQUIRE(eval_condition(0xA, 0) == true);
    REQUIRE(eval_condition(0xA, flag(N_BIT) | flag(V_BIT)) == true);
    REQUIRE(eval_condition(0xA, flag(N_BIT)) == false);
    REQUIRE(eval_condition(0xB, flag(N_BIT)) == true);

    // GT = !Z && N==V
    REQUIRE(eval_condition(0xC, 0) == true);
    REQUIRE(eval_condition(0xC, flag(Z_BIT)) == false);
    REQUIRE(eval_condition(0xC, flag(N_BIT)) == false);
    REQUIRE(eval_condition(0xC, flag(N_BIT) | flag(V_BIT)) == true);
    // LE = Z || N!=V
    REQUIRE(eval_condition(0xD, flag(Z_BIT)) == true);
    REQUIRE(eval_condition(0xD, flag(N_BIT)) == true);
    REQUIRE(eval_condition(0xD, flag(N_BIT) | flag(V_BIT)) == false);
}

int main() {
    al_always_true_nv_always_false();
    eq_ne_track_z();
    cs_cc_track_c();
    mi_pl_track_n();
    vs_vc_track_v();
    hi_ls_compound();
    ge_lt_gt_le_track_nv();
    std::puts("arm7_condition_test OK");
    return 0;
}
```

Modify `tests/CMakeLists.txt` to add both new binaries:

```cmake
add_ds_unit_test(arm7_register_file_test)
add_ds_unit_test(arm7_barrel_shifter_test)
add_ds_unit_test(arm7_condition_test)
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd build && cmake .. && make arm7_barrel_shifter_test arm7_condition_test 2>&1 | tail -10`
Expected: `cpu/arm7/arm7_alu.hpp: No such file`.

- [ ] **Step 4: Create `src/cpu/arm7/arm7_alu.hpp`**

```cpp
#pragma once

// Pure ARMv4T ALU helpers: barrel shifter, rotated-immediate operand,
// condition code evaluation, and flag updaters. No access to Arm7State —
// everything takes and returns plain values so it is trivially testable.

#include "ds/common.hpp"

namespace ds {

enum class ShiftType : u8 {
    Lsl = 0,
    Lsr = 1,
    Asr = 2,
    Ror = 3,
};

struct ShifterResult {
    u32  value;
    bool carry;
};

// Immediate-shift form (bits [11:7] in the instruction). `amount` is 0..31
// as encoded. The ARMv4T quirks for amount == 0 are handled here:
//   LSL #0 → value unchanged, carry = c_in
//   LSR #0 → treat as LSR #32: value = 0, carry = bit 31 of operand
//   ASR #0 → treat as ASR #32: value = signbit ? ~0 : 0, carry = bit 31
//   ROR #0 → RRX: value = (c_in << 31) | (op >> 1), carry = bit 0 of operand
inline ShifterResult barrel_shift_imm(u32 operand, ShiftType type, u32 amount, bool c_in) {
    if (amount == 0) {
        switch (type) {
            case ShiftType::Lsl:
                return { operand, c_in };
            case ShiftType::Lsr:
                return { 0u, (operand >> 31) != 0 };
            case ShiftType::Asr: {
                const bool sign = (operand >> 31) != 0;
                return { sign ? 0xFFFF'FFFFu : 0u, sign };
            }
            case ShiftType::Ror: {
                const u32 new_val = (static_cast<u32>(c_in) << 31) | (operand >> 1);
                return { new_val, (operand & 1u) != 0 };
            }
        }
    }
    // amount in 1..31
    switch (type) {
        case ShiftType::Lsl: {
            const bool carry = ((operand >> (32 - amount)) & 1u) != 0;
            return { operand << amount, carry };
        }
        case ShiftType::Lsr: {
            const bool carry = ((operand >> (amount - 1)) & 1u) != 0;
            return { operand >> amount, carry };
        }
        case ShiftType::Asr: {
            const bool carry = ((operand >> (amount - 1)) & 1u) != 0;
            const i32 signed_op = static_cast<i32>(operand);
            return { static_cast<u32>(signed_op >> amount), carry };
        }
        case ShiftType::Ror: {
            const u32 a = amount & 31u;
            const bool carry = ((operand >> (a - 1)) & 1u) != 0;
            return { (operand >> a) | (operand << (32 - a)), carry };
        }
    }
    return { operand, c_in };  // unreachable
}

// Register-shift form (used later for "Rs" shift amounts). Amounts larger
// than 31 have specific behaviour: LSL/LSR by >=32 zero the result;
// ASR by >=32 sign-extends; ROR by 32 returns operand with carry = bit 31,
// ROR by N>32 folds to ROR(N mod 32).
inline ShifterResult barrel_shift_reg(u32 operand, ShiftType type, u32 amount) {
    if (amount == 0) {
        // LSL/LSR/ASR/ROR by zero from a register leaves value and carry
        // unchanged. Callers that need the "c_in" value must pass it
        // through themselves because this helper does not know it.
        return { operand, false };
    }
    if (amount >= 32) {
        switch (type) {
            case ShiftType::Lsl: {
                const bool carry = (amount == 32) ? ((operand & 1u) != 0) : false;
                return { 0u, carry };
            }
            case ShiftType::Lsr: {
                const bool carry = (amount == 32) ? ((operand >> 31) != 0) : false;
                return { 0u, carry };
            }
            case ShiftType::Asr: {
                const bool sign = (operand >> 31) != 0;
                return { sign ? 0xFFFF'FFFFu : 0u, sign };
            }
            case ShiftType::Ror: {
                const u32 a = amount & 31u;
                if (a == 0) return { operand, (operand >> 31) != 0 };
                const bool carry = ((operand >> (a - 1)) & 1u) != 0;
                return { (operand >> a) | (operand << (32 - a)), carry };
            }
        }
    }
    // 1..31 — same as immediate-shift for amounts in range.
    return barrel_shift_imm(operand, type, amount, false);
}

// Data-processing rotated-immediate operand2. `rotate` is the raw 4-bit
// field from bits [11:8]; the real rotate amount is `rotate * 2`.
// When rotate == 0, carry-out is c_in unchanged; otherwise carry-out is
// bit 31 of the rotated result.
inline ShifterResult rotated_imm(u32 imm8, u32 rotate, bool c_in) {
    if (rotate == 0) {
        return { imm8, c_in };
    }
    const u32 amount = rotate * 2u;
    const u32 value = (imm8 >> amount) | (imm8 << (32 - amount));
    return { value, (value >> 31) != 0 };
}

// 4-bit ARM condition code evaluator. `cpsr_flags` is the full CPSR —
// we only look at bits 28..31.
inline bool eval_condition(u32 cond, u32 cpsr_flags) {
    const bool n = (cpsr_flags & (1u << 31)) != 0;
    const bool z = (cpsr_flags & (1u << 30)) != 0;
    const bool c = (cpsr_flags & (1u << 29)) != 0;
    const bool v = (cpsr_flags & (1u << 28)) != 0;
    switch (cond & 0xFu) {
        case 0x0: return z;
        case 0x1: return !z;
        case 0x2: return c;
        case 0x3: return !c;
        case 0x4: return n;
        case 0x5: return !n;
        case 0x6: return v;
        case 0x7: return !v;
        case 0x8: return c && !z;
        case 0x9: return !c || z;
        case 0xA: return n == v;
        case 0xB: return n != v;
        case 0xC: return !z && (n == v);
        case 0xD: return z || (n != v);
        case 0xE: return true;
        case 0xF: return false;
    }
    return false;
}

// Flag-update helpers. Each takes the *current* CPSR, returns the new CPSR
// with the NZCV bits updated. Callers pass the result and any derived
// carry/overflow values; the helper handles masking.
inline u32 set_nz(u32 cpsr, u32 result) {
    cpsr &= ~((1u << 31) | (1u << 30));
    if ((result & (1u << 31)) != 0) cpsr |= (1u << 31);
    if (result == 0)                cpsr |= (1u << 30);
    return cpsr;
}

inline u32 set_c(u32 cpsr, bool c) {
    cpsr &= ~(1u << 29);
    if (c) cpsr |= (1u << 29);
    return cpsr;
}

inline u32 set_v(u32 cpsr, bool v) {
    cpsr &= ~(1u << 28);
    if (v) cpsr |= (1u << 28);
    return cpsr;
}

// Adder producing result + carry + overflow.
struct AddResult {
    u32  value;
    bool carry;
    bool overflow;
};

inline AddResult adc(u32 a, u32 b, bool carry_in) {
    const u64 sum = static_cast<u64>(a) + static_cast<u64>(b) + (carry_in ? 1u : 0u);
    const u32 r = static_cast<u32>(sum);
    const bool c = (sum >> 32) != 0;
    const bool v = (((a ^ r) & (b ^ r)) >> 31) != 0;
    return { r, c, v };
}

// Subtractor: a - b - (1 - carry_in). For plain SUB/CMP, pass carry_in = true.
inline AddResult sbc(u32 a, u32 b, bool carry_in) {
    const u64 diff = static_cast<u64>(a) - static_cast<u64>(b) - (carry_in ? 0u : 1u);
    const u32 r = static_cast<u32>(diff);
    // Carry flag on subtraction = NOT borrow. For plain SUB: c = (a >= b).
    const bool c = (diff >> 32) == 0;
    const bool v = (((a ^ b) & (a ^ r)) >> 31) != 0;
    return { r, c, v };
}

}  // namespace ds
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cd build && make arm7_barrel_shifter_test arm7_condition_test && ctest -R "arm7_barrel_shifter_test|arm7_condition_test" --output-on-failure`
Expected: both pass.

Run: `ctest --output-on-failure`
Expected: all 11 tests pass (8 previous + 3 new).

- [ ] **Step 6: Commit**

```bash
git add src/cpu/arm7/arm7_alu.hpp tests/unit/arm7_barrel_shifter_test.cpp \
        tests/unit/arm7_condition_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: barrel shifter + condition evaluator helpers

Header-only ALU helpers callable from both data-processing and future
load/store addressing-mode decoding. Unit-tested against ARMv4T edge
cases — LSR/ASR/ROR immediate-zero forms, LSL #32 via register-shift,
RRX, and every compound condition code."
```

---

## Task 3: `Arm7` owns real state + attaches to bus

**Files:**
- Modify: `src/cpu/arm7/arm7.hpp`
- Modify: `src/cpu/arm7/arm7.cpp`
- Modify: `src/nds.cpp`

Goal: `class Arm7` now owns an `Arm7State` and holds an `Arm7Bus*` pointer attached at NDS construction time. `reset()` calls `state.reset()`. `run_until()` still no-ops past reset values (we haven't written the fetch loop yet — that's Task 4), but it compiles cleanly against the new types.

There is no new unit test in this task — Task 4 tests the fetch loop. This task is purely plumbing.

- [ ] **Step 1: Replace `src/cpu/arm7/arm7.hpp`**

```cpp
#pragma once

#include "cpu/arm7/arm7_state.hpp"
#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7Bus;

class Arm7 : public CpuCore {
public:
    void attach_bus(Arm7Bus& bus) { bus_ = &bus; }

    // Takes an ARM9-cycle target; internally converts to ARM7 cycles (half rate).
    void run_until(Cycle arm9_target) override;
    void reset() override;

    // Test access.
    Arm7State&       state()       { return state_; }
    const Arm7State& state() const { return state_; }

private:
    // Fetch one ARM instruction at pc_, advance pc_, set R15 to pc_+4
    // (= instruction_addr + 8), and execute. Defined in arm7_decode.cpp.
    void step_arm();

    Arm7State state_{};
    Arm7Bus*  bus_ = nullptr;
};

}  // namespace ds
```

- [ ] **Step 2: Replace `src/cpu/arm7/arm7.cpp`**

```cpp
#include "cpu/arm7/arm7.hpp"

#include "bus/arm7_bus.hpp"

namespace ds {

void Arm7::reset() {
    state_.reset();
}

void Arm7::run_until(Cycle arm9_target) {
    const Cycle arm7_target = arm9_target / 2;
    // Task 4 wires up a real fetch/decode loop that calls step_arm().
    // For now, just advance the cycle counter to the target so scheduler
    // consumers see a well-behaved CPU that does nothing.
    if (arm7_target > state_.cycles) {
        state_.cycles = arm7_target;
    }
    // Reference bus_ so the compiler does not warn about an unused member
    // until Task 4 actually uses it. This line is removed in Task 4.
    (void)bus_;
}

// Defined in arm7_decode.cpp — Task 4.
// void Arm7::step_arm() { ... }

}  // namespace ds
```

- [ ] **Step 3: Wire `attach_bus` from the NDS constructor**

Open `src/nds.cpp` and find the `NDS::NDS()` constructor body. Add this line after the existing member initializer list completes (i.e. inside the constructor body, after the current code):

```cpp
NDS::NDS()
    : arm9_bus_(*this, main_ram_.data(), shared_wram_.data(), wram_ctl_),
      arm7_bus_(*this, main_ram_.data(), shared_wram_.data(), arm7_wram_.data(), wram_ctl_)
{
    cpu7_.attach_bus(arm7_bus_);
    reset();
}
```

Adjust to match the current constructor shape — if `reset()` is already called at the end, insert `attach_bus` before it so `reset()` sees a bus (even though current `reset()` does not read from it, that will become true in Task 4).

- [ ] **Step 4: Build and run all tests**

Run: `cd build && make && ctest --output-on-failure`
Expected: all 11 tests pass. No new tests added.

- [ ] **Step 5: Commit**

```bash
git add src/cpu/arm7/arm7.hpp src/cpu/arm7/arm7.cpp src/nds.cpp
git commit -m "cpu/arm7: class owns Arm7State, attaches to Arm7Bus

Replaces the Phase-0 cycle-counter stub with an Arm7State member and
a non-owning Arm7Bus pointer attached by NDS at construction time.
run_until still advances cycles only; Task 4 lands the fetch loop."
```

---

## Task 4: ARM fetch/decode dispatch skeleton

**Files:**
- Create: `src/cpu/arm7/arm7_decode.cpp`
- Modify: `src/cpu/arm7/arm7.cpp`
- Modify: `src/CMakeLists.txt`
- Create: `tests/unit/arm7_data_processing_test.cpp`
- Modify: `tests/CMakeLists.txt`

Goal: `Arm7::run_until` now loops until the cycle target, fetching each instruction from `Arm7Bus`, advancing `pc_`, checking the condition code, and dispatching on the top decode bits. For slice 3a the only handled form is data-processing; anything else logs a warning and is treated as a nop. This task also lands the first data-processing handler (`MOV` immediate) so we have end-to-end proof that fetch → decode → execute works before Task 5 expands the opcode set.

Cycle accounting: every executed instruction costs exactly 1 ARM7 cycle for now. Real ARMv4T cycle counts depend on the instruction, bus timing, and the Karnaugh tables in GBATEK §ARM.CPU. Slice 3a does not model that — it just increments by 1. Slice 3b refines this when it lands memory access and branches.

- [ ] **Step 1: Write the failing data-processing test (MOV immediate only)**

Create `tests/unit/arm7_data_processing_test.cpp`:

```cpp
// Hand-assembled ARM data-processing tests. Each case loads a single
// instruction word into ARM7 WRAM at 0x0380'0000, sets pc_, calls
// Arm7::run_until() for one instruction's worth of cycles, and checks
// the post-execution register file.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// Preload a single instruction word at pc and run one instruction.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;  // R15 = pc + 8 at execute time
    const u64 cycles_before = nds.cpu7().state().cycles;
    // Ask scheduler to run the ARM7 for 2 ARM9 cycles (= 1 ARM7 cycle).
    // We call Arm7::run_until() directly to bypass scheduler scaffolding.
    nds.cpu7().run_until((cycles_before + 1) * 2);
    REQUIRE(nds.cpu7().state().cycles == cycles_before + 1);
}

static void mov_imm_zero_writes_register_and_advances_pc() {
    NDS nds;
    // Encoding: cond=AL, 00 I=1 opcode=MOV S=0 Rn=0 Rd=0 rotate=0 imm=0
    //          1110 00 1 1101 0 0000 0000 0000 0000 0000
    //          0xE3A0'0000 = MOV R0, #0
    run_one(nds, 0x0380'0000u, 0xE3A0'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);
}

static void mov_imm_0x42_lands_in_r1() {
    NDS nds;
    // MOV R1, #0x42 → 0xE3A0'1042
    run_one(nds, 0x0380'0000u, 0xE3A0'1042u);
    REQUIRE(nds.cpu7().state().r[1] == 0x42u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);
}

static void condition_ne_skips_when_z_set() {
    NDS nds;
    // MOVNE R2, #0x77 → 0x13A0'2077
    // Pre-set CPSR Z flag so the instruction is skipped.
    nds.cpu7().state().cpsr |= (1u << 30);
    run_one(nds, 0x0380'0000u, 0x13A0'2077u);
    REQUIRE(nds.cpu7().state().r[2] == 0u);
    REQUIRE(nds.cpu7().state().pc == 0x0380'0004u);  // still advances
}

int main() {
    mov_imm_zero_writes_register_and_advances_pc();
    mov_imm_0x42_lands_in_r1();
    condition_ne_skips_when_z_set();
    std::puts("arm7_data_processing_test OK");
    return 0;
}
```

Add to `tests/CMakeLists.txt`:

```cmake
add_ds_unit_test(arm7_data_processing_test)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd build && cmake .. && make arm7_data_processing_test 2>&1 | tail -20`
Expected: compilation failure — `Arm7Bus` not forward-declared for `run_until`, or linker error on `Arm7::step_arm`.

- [ ] **Step 3: Create `src/cpu/arm7/arm7_decode.cpp`**

```cpp
// arm7_decode.cpp implements Arm7::step_arm() and the ARM instruction
// decoder dispatch. In slice 3a only data-processing (bits 27..26 == 00)
// is supported; everything else logs a warning and no-ops.
//
// Cycle cost model: every executed (non-skipped) instruction is 1 ARM7
// cycle. Skipped (condition-failed) instructions also cost 1 cycle.
// This is deliberately simplified — slice 3b revisits cycle counts when
// we model memory access and branch pipeline flush.
//
// NOTE: in ARMv4T, the "bit 4 == 1 with I == 0" encoding space inside
// data-processing actually overlaps with MUL/MLA, MRS/MSR, and halfword
// loads/stores. Slice 3a lumps all of those into the same "unimplemented"
// bucket because they are all deferred to slice 3b. When 3b lands, this
// decoder's dispatch tree has to be refined to break them apart before
// implementing register-shifted data-processing for real.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_alu.hpp"

namespace ds {

namespace {

// Data-processing opcode numbers (bits 24..21 of the instruction).
enum class DpOp : u8 {
    AND = 0x0, EOR = 0x1, SUB = 0x2, RSB = 0x3,
    ADD = 0x4, ADC = 0x5, SBC = 0x6, RSC = 0x7,
    TST = 0x8, TEQ = 0x9, CMP = 0xA, CMN = 0xB,
    ORR = 0xC, MOV = 0xD, BIC = 0xE, MVN = 0xF,
};

// Slice 3a: only MOV-immediate is fully implemented here. Task 5 adds
// ADD/SUB, Task 6 adds the remaining opcodes, Task 7 adds the S flag,
// Task 8 adds shifted-register operand2.

void exec_dp_mov(Arm7State& s, u32 rd, u32 operand2) {
    s.r[rd] = operand2;
    if (rd == 15) {
        s.pc = operand2 & ~0x3u;
    }
}

}  // namespace

void Arm7::step_arm() {
    const u32 instr_addr = state_.pc;
    const u32 instr      = bus_->read32(instr_addr);

    // R15 during execute reads as instruction_addr + 8.
    state_.r[15] = instr_addr + 8;
    state_.pc    = instr_addr + 4;

    const u32 cond = instr >> 28;
    if (!eval_condition(cond, state_.cpsr)) {
        state_.cycles += 1;
        return;
    }

    const u32 bits_27_26 = (instr >> 26) & 0x3u;
    if (bits_27_26 != 0) {
        // Non-data-processing form — branches, loads/stores, coproc, SWI.
        // Deferred to slices 3b/3d. Log once per warning and no-op.
        DS_LOG_WARN("arm7: unimplemented ARM form 0x%08X at 0x%08X", instr, instr_addr);
        state_.cycles += 1;
        return;
    }

    // Bit 4 of instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;
    if (reg_shift) {
        // Register-shifted-register operand2 — deferred to slice 3b.
        DS_LOG_WARN("arm7: register-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        state_.cycles += 1;
        return;
    }

    // Decode fields common to all data-processing forms.
    const u32 opcode   = (instr >> 21) & 0xFu;
    const u32 s_flag   = (instr >> 20) & 1u;
    const u32 rn       = (instr >> 16) & 0xFu;
    const u32 rd       = (instr >> 12) & 0xFu;
    (void)rn;
    (void)s_flag;  // Task 7 uses this.

    // Compute operand2.
    ShifterResult op2;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state_.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
    } else {
        // Immediate-shift form. Task 8 wires this in. Slice 3a only
        // supports immediate operand2 right now.
        DS_LOG_WARN("arm7: immediate-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        state_.cycles += 1;
        return;
    }

    switch (static_cast<DpOp>(opcode)) {
        case DpOp::MOV:
            exec_dp_mov(state_, rd, op2.value);
            break;
        default:
            DS_LOG_WARN("arm7: unimplemented dp opcode 0x%X at 0x%08X",
                        opcode, instr_addr);
            break;
    }

    state_.cycles += 1;
}

}  // namespace ds
```

- [ ] **Step 4: Rewrite `src/cpu/arm7/arm7.cpp` to call `step_arm` in a loop**

```cpp
#include "cpu/arm7/arm7.hpp"

#include "bus/arm7_bus.hpp"

namespace ds {

void Arm7::reset() {
    state_.reset();
}

void Arm7::run_until(Cycle arm9_target) {
    const u64 arm7_target = arm9_target / 2;
    while (state_.cycles < arm7_target) {
        step_arm();
    }
}

// step_arm() is implemented in arm7_decode.cpp.

}  // namespace ds
```

- [ ] **Step 5: Add `arm7_decode.cpp` to the build**

Modify `src/CMakeLists.txt`:

```cmake
add_library(ds_core STATIC
    nds.cpp
    bus/arm9_bus.cpp
    bus/arm7_bus.cpp
    scheduler/scheduler.cpp
    cpu/arm9/arm9.cpp
    cpu/arm7/arm7.cpp
    cpu/arm7/arm7_decode.cpp
)
```

- [ ] **Step 6: Run the data-processing test**

Run: `cd build && cmake .. && make arm7_data_processing_test && ctest -R arm7_data_processing_test --output-on-failure`
Expected: all three cases pass. Full suite: `ctest --output-on-failure` → 12/12 green.

- [ ] **Step 7: Commit**

```bash
git add src/cpu/arm7/arm7_decode.cpp src/cpu/arm7/arm7.cpp src/CMakeLists.txt \
        tests/unit/arm7_data_processing_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: fetch/decode/execute loop with MOV immediate

step_arm() fetches from Arm7Bus, checks the condition code, dispatches
on bits [27:26], and for data-processing it currently handles MOV
with a rotated immediate operand2. Remaining opcodes, S-flag updates,
and register-operand2 forms land in the next four tasks."
```

---

## Task 5: Remaining data-processing opcodes (no S-flag yet)

**Files:**
- Modify: `src/cpu/arm7/arm7_decode.cpp`
- Modify: `tests/unit/arm7_data_processing_test.cpp`

Goal: add executors for AND, EOR, SUB, RSB, ADD, ADC, SBC, RSC, ORR, BIC, MVN. TST, TEQ, CMP, CMN are handled in Task 6 where the S-flag logic lands (they are all S=1-only no-result-write instructions, so it's cleaner to add them with the flag work). Each opcode is a two-to-four line case in the switch; add focused tests for each.

- [ ] **Step 1: Add failing tests for each new opcode**

Append to `tests/unit/arm7_data_processing_test.cpp` (before `main()`):

```cpp
static void add_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    // ADD R0, R1, #5 → 0xE281'0005
    run_one(nds, 0x0380'0000u, 0xE281'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 15u);
}

static void sub_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    // SUB R0, R1, #3 → 0xE241'0003
    run_one(nds, 0x0380'0000u, 0xE241'0003u);
    REQUIRE(nds.cpu7().state().r[0] == 7u);
}

static void rsb_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 3;
    // RSB R0, R1, #10 → 0xE261'000A
    run_one(nds, 0x0380'0000u, 0xE261'000Au);
    REQUIRE(nds.cpu7().state().r[0] == 7u);
}

static void and_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xF0F0'F0F0u;
    // AND R0, R1, #0xFF → 0xE201'00FF
    run_one(nds, 0x0380'0000u, 0xE201'00FFu);
    REQUIRE(nds.cpu7().state().r[0] == 0xF0u);
}

static void eor_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFF'FFFFu;
    // EOR R0, R1, #0xFF → 0xE221'00FF
    run_one(nds, 0x0380'0000u, 0xE221'00FFu);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FF00u);
}

static void orr_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x0000'000Fu;
    // ORR R0, R1, #0xF0 → 0xE381'00F0
    run_one(nds, 0x0380'0000u, 0xE381'00F0u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFu);
}

static void bic_imm() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFu;
    // BIC R0, R1, #0x0F → 0xE3C1'000F
    run_one(nds, 0x0380'0000u, 0xE3C1'000Fu);
    REQUIRE(nds.cpu7().state().r[0] == 0xF0u);
}

static void mvn_imm() {
    NDS nds;
    // MVN R0, #0 → 0xE3E0'0000 (result 0xFFFFFFFF)
    run_one(nds, 0x0380'0000u, 0xE3E0'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
}

static void adc_imm_with_carry_set() {
    NDS nds;
    nds.cpu7().state().cpsr |= (1u << 29);   // C = 1
    nds.cpu7().state().r[1] = 10;
    // ADC R0, R1, #5 → 0xE2A1'0005
    run_one(nds, 0x0380'0000u, 0xE2A1'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 16u);
}

static void sbc_imm_with_carry_clear() {
    NDS nds;
    // C = 0 means SBC subtracts an extra 1.
    nds.cpu7().state().r[1] = 10;
    // SBC R0, R1, #3 → 0xE2C1'0003
    run_one(nds, 0x0380'0000u, 0xE2C1'0003u);
    REQUIRE(nds.cpu7().state().r[0] == 6u);
}

static void rsc_imm_with_carry_set() {
    NDS nds;
    nds.cpu7().state().cpsr |= (1u << 29);  // C = 1 → no extra decrement
    nds.cpu7().state().r[1] = 3;
    // RSC R0, R1, #10 → 0xE2E1'000A
    run_one(nds, 0x0380'0000u, 0xE2E1'000Au);
    REQUIRE(nds.cpu7().state().r[0] == 7u);
}
```

Add calls in `main()`:

```cpp
int main() {
    mov_imm_zero_writes_register_and_advances_pc();
    mov_imm_0x42_lands_in_r1();
    condition_ne_skips_when_z_set();
    add_imm(); sub_imm(); rsb_imm();
    and_imm(); eor_imm(); orr_imm(); bic_imm(); mvn_imm();
    adc_imm_with_carry_set();
    sbc_imm_with_carry_clear();
    rsc_imm_with_carry_set();
    std::puts("arm7_data_processing_test OK");
    return 0;
}
```

- [ ] **Step 2: Verify the tests fail**

Run: `cd build && make arm7_data_processing_test && ctest -R arm7_data_processing_test --output-on-failure`
Expected: failure — ADD/SUB/etc. log "unimplemented dp opcode" and R0 stays 0.

- [ ] **Step 3: Extend the switch in `src/cpu/arm7/arm7_decode.cpp`**

Replace the `switch (static_cast<DpOp>(opcode))` block and the `exec_dp_mov` helper with:

```cpp
namespace {

void write_rd(Arm7State& s, u32 rd, u32 value) {
    s.r[rd] = value;
    if (rd == 15) {
        s.pc = value & ~0x3u;
    }
}

}  // namespace
```

Then the full switch becomes:

```cpp
    const u32 rn_val = state_.r[rn];
    switch (static_cast<DpOp>(opcode)) {
        case DpOp::AND: write_rd(state_, rd, rn_val & op2.value); break;
        case DpOp::EOR: write_rd(state_, rd, rn_val ^ op2.value); break;
        case DpOp::SUB: write_rd(state_, rd, rn_val - op2.value); break;
        case DpOp::RSB: write_rd(state_, rd, op2.value - rn_val); break;
        case DpOp::ADD: write_rd(state_, rd, rn_val + op2.value); break;
        case DpOp::ADC: {
            const bool c = (state_.cpsr & (1u << 29)) != 0;
            write_rd(state_, rd, rn_val + op2.value + (c ? 1u : 0u));
            break;
        }
        case DpOp::SBC: {
            const bool c = (state_.cpsr & (1u << 29)) != 0;
            write_rd(state_, rd, rn_val - op2.value - (c ? 0u : 1u));
            break;
        }
        case DpOp::RSC: {
            const bool c = (state_.cpsr & (1u << 29)) != 0;
            write_rd(state_, rd, op2.value - rn_val - (c ? 0u : 1u));
            break;
        }
        case DpOp::ORR: write_rd(state_, rd, rn_val | op2.value); break;
        case DpOp::MOV: write_rd(state_, rd, op2.value); break;
        case DpOp::BIC: write_rd(state_, rd, rn_val & ~op2.value); break;
        case DpOp::MVN: write_rd(state_, rd, ~op2.value); break;
        // TST/TEQ/CMP/CMN land in Task 6 with flag handling.
        case DpOp::TST:
        case DpOp::TEQ:
        case DpOp::CMP:
        case DpOp::CMN:
            DS_LOG_WARN("arm7: TST/TEQ/CMP/CMN at 0x%08X — Task 6", instr_addr);
            break;
    }
```

Remove the stray `(void)rn;` line from Task 4 (rn_val is now used).

- [ ] **Step 4: Run the tests**

Run: `cd build && make arm7_data_processing_test && ctest -R arm7_data_processing_test --output-on-failure`
Expected: all 14 cases pass.

Run: `ctest --output-on-failure`
Expected: 12 test binaries green.

- [ ] **Step 5: Commit**

```bash
git add src/cpu/arm7/arm7_decode.cpp tests/unit/arm7_data_processing_test.cpp
git commit -m "cpu/arm7: AND/EOR/SUB/RSB/ADD/ADC/SBC/RSC/ORR/BIC/MVN

Completes the arithmetic and logical data-processing executors with
immediate operand2, no flag updates. Each opcode covered by a focused
hand-assembled test. TST/TEQ/CMP/CMN still deferred to Task 6 with the
S-flag work since they only exist in their flag-setting form."
```

---

## Task 6: S-flag updates + TST/TEQ/CMP/CMN

**Files:**
- Modify: `src/cpu/arm7/arm7_decode.cpp`
- Create: `tests/unit/arm7_flags_test.cpp`
- Modify: `tests/CMakeLists.txt`

Goal: when `S == 1` on a data-processing instruction, update CPSR NZCV correctly. Logical ops set N/Z from the result and C from the shifter carry-out (leave V alone). Arithmetic ops set N/Z from the result and C/V from the adder. This task also lands TST/TEQ/CMP/CMN because they only exist in their flag-setting form.

- [ ] **Step 1: Write the failing flag tests**

Create `tests/unit/arm7_flags_test.cpp`:

```cpp
// S-flag behavior for data-processing instructions. Covers logical
// (N/Z from result, C from shifter) and arithmetic (N/Z from result,
// C/V from adder/subtractor) variants.

#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static constexpr u32 N_BIT = 1u << 31;
static constexpr u32 Z_BIT = 1u << 30;
static constexpr u32 C_BIT = 1u << 29;
static constexpr u32 V_BIT = 1u << 28;

static void run_one(NDS& nds, u32 instr) {
    const u32 pc = 0x0380'0000u;
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 c = nds.cpu7().state().cycles;
    nds.cpu7().run_until((c + 1) * 2);
}

static void movs_imm_zero_sets_z_clears_n() {
    NDS nds;
    nds.cpu7().state().cpsr |= N_BIT;  // start with N set
    // MOVS R0, #0 → 0xE3B0'0000
    run_one(nds, 0xE3B0'0000u);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) == 0);
}

static void movs_imm_negative_sets_n_clears_z() {
    NDS nds;
    // MOVS R0, #0x8000'0000 is not directly encodable as imm8<<rotate
    // — use MVNS R0, #0x7FFFFFFF? Simpler: MVNS R0, #0 → R0 = ~0 = 0xFFFF'FFFF.
    // MVNS R0, #0 → 0xE3F0'0000
    run_one(nds, 0xE3F0'0000u);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) == 0);
}

static void adds_overflow_sets_v() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x7FFF'FFFFu;
    // ADDS R0, R1, #1 → 0xE291'0001
    run_one(nds, 0xE291'0001u);
    REQUIRE(nds.cpu7().state().r[0] == 0x8000'0000u);
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) == 0);
}

static void adds_wrap_sets_c_no_v() {
    NDS nds;
    nds.cpu7().state().r[1] = 0xFFFF'FFFFu;
    // ADDS R0, R1, #1 → 0xE291'0001
    run_one(nds, 0xE291'0001u);
    REQUIRE(nds.cpu7().state().r[0] == 0u);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) == 0);
}

static void subs_sets_c_when_no_borrow() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    // SUBS R0, R1, #5 → 0xE251'0005
    run_one(nds, 0xE251'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 5u);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) != 0);  // no borrow
    REQUIRE((nds.cpu7().state().cpsr & V_BIT) == 0);
}

static void subs_clears_c_when_borrow() {
    NDS nds;
    nds.cpu7().state().r[1] = 3;
    // SUBS R0, R1, #5 → 0xE251'0005
    run_one(nds, 0xE251'0005u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFEu);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) == 0);  // borrow
    REQUIRE((nds.cpu7().state().cpsr & N_BIT) != 0);
}

static void cmp_sets_flags_but_no_register_write() {
    NDS nds;
    nds.cpu7().state().r[1] = 7;
    const u32 r0_before = nds.cpu7().state().r[0] = 0x1234;
    // CMP R1, #7 → 0xE351'0007
    run_one(nds, 0xE351'0007u);
    REQUIRE(nds.cpu7().state().r[0] == r0_before);  // CMP does not write
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
    REQUIRE((nds.cpu7().state().cpsr & C_BIT) != 0);
}

static void tst_sets_nz_from_and_but_no_register_write() {
    NDS nds;
    nds.cpu7().state().r[1] = 0x8000'0000u;
    nds.cpu7().state().r[0] = 0xAAAA'AAAAu;
    // TST R1, #0x8000'0000 is not directly encodable; test with lower bits.
    // TST R1, #0 → 0xE311'0000 → result 0 → Z=1
    run_one(nds, 0xE311'0000u);
    REQUIRE(nds.cpu7().state().r[0] == 0xAAAA'AAAAu);
    REQUIRE((nds.cpu7().state().cpsr & Z_BIT) != 0);
}

int main() {
    movs_imm_zero_sets_z_clears_n();
    movs_imm_negative_sets_n_clears_z();
    adds_overflow_sets_v();
    adds_wrap_sets_c_no_v();
    subs_sets_c_when_no_borrow();
    subs_clears_c_when_borrow();
    cmp_sets_flags_but_no_register_write();
    tst_sets_nz_from_and_but_no_register_write();
    std::puts("arm7_flags_test OK");
    return 0;
}
```

Add to `tests/CMakeLists.txt`:

```cmake
add_ds_unit_test(arm7_flags_test)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd build && cmake .. && make arm7_flags_test && ctest -R arm7_flags_test --output-on-failure`
Expected: failures on every case — flags are not being updated, CMP/TST are still stubs.

- [ ] **Step 3: Add flag-update logic and the four compare opcodes**

Edit `src/cpu/arm7/arm7_decode.cpp`. The goal is to compute the result, then — if `s_flag == 1` — update NZCV. Split logical vs arithmetic. Replace the switch body with:

```cpp
    const u32 rn_val = state_.r[rn];
    const bool c_in  = (state_.cpsr & (1u << 29)) != 0;
    u32  result       = 0;
    bool writeback    = true;
    bool logical_form = false;
    AddResult ar{};

    switch (static_cast<DpOp>(opcode)) {
        case DpOp::AND: result = rn_val &  op2.value; logical_form = true; break;
        case DpOp::EOR: result = rn_val ^  op2.value; logical_form = true; break;
        case DpOp::ORR: result = rn_val |  op2.value; logical_form = true; break;
        case DpOp::BIC: result = rn_val & ~op2.value; logical_form = true; break;
        case DpOp::MOV: result =          op2.value;  logical_form = true; break;
        case DpOp::MVN: result =         ~op2.value;  logical_form = true; break;
        case DpOp::TST: result = rn_val &  op2.value; logical_form = true; writeback = false; break;
        case DpOp::TEQ: result = rn_val ^  op2.value; logical_form = true; writeback = false; break;
        case DpOp::ADD: ar = adc(rn_val, op2.value, false);     result = ar.value; break;
        case DpOp::ADC: ar = adc(rn_val, op2.value, c_in);      result = ar.value; break;
        case DpOp::CMN: ar = adc(rn_val, op2.value, false);     result = ar.value; writeback = false; break;
        case DpOp::SUB: ar = sbc(rn_val, op2.value, true);      result = ar.value; break;
        case DpOp::RSB: ar = sbc(op2.value, rn_val, true);      result = ar.value; break;
        case DpOp::SBC: ar = sbc(rn_val, op2.value, c_in);      result = ar.value; break;
        case DpOp::RSC: ar = sbc(op2.value, rn_val, c_in);      result = ar.value; break;
        case DpOp::CMP: ar = sbc(rn_val, op2.value, true);      result = ar.value; writeback = false; break;
    }

    if (writeback) {
        write_rd(state_, rd, result);
    }

    if (s_flag) {
        if (rd == 15 && writeback) {
            // "MOVS R15" and friends copy SPSR → CPSR. Not used in slice 3a;
            // log and leave flags alone for now (slice 3d revisits).
            DS_LOG_WARN("arm7: S-flag set with Rd=R15 at 0x%08X", instr_addr);
        } else {
            state_.cpsr = set_nz(state_.cpsr, result);
            if (logical_form) {
                state_.cpsr = set_c(state_.cpsr, op2.carry);
                // V is unchanged for logical ops.
            } else {
                state_.cpsr = set_c(state_.cpsr, ar.carry);
                state_.cpsr = set_v(state_.cpsr, ar.overflow);
            }
        }
    }
```

Delete the old switch and the old `(void)s_flag;` line. The TST/TEQ/CMP/CMN `DS_LOG_WARN` stubs from Task 5 are gone — they are now real.

- [ ] **Step 4: Run all tests**

Run: `cd build && make && ctest --output-on-failure`
Expected: 13 test binaries green, including the new `arm7_flags_test`.

- [ ] **Step 5: Commit**

```bash
git add src/cpu/arm7/arm7_decode.cpp tests/unit/arm7_flags_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: S-flag updates + TST/TEQ/CMP/CMN

Logical ops set N/Z from result, C from shifter, V unchanged.
Arithmetic ops set N/Z from result, C/V from the adder/subtractor
helpers in arm7_alu.hpp. TST/TEQ/CMP/CMN are now real compare-only
instructions that update flags without writing Rd."
```

---

## Task 7: Immediate-shifted-register operand2

**Files:**
- Modify: `src/cpu/arm7/arm7_decode.cpp`
- Modify: `tests/unit/arm7_data_processing_test.cpp`

Goal: when the I-bit is 0 and bit 4 is 0, operand2 is `Rm shifted by an immediate amount`. This is the form used by things like `add r0, r1, r2, lsl #3`. Wire the existing `barrel_shift_imm` helper into the operand2 decode path.

- [ ] **Step 1: Add failing tests**

Append to `tests/unit/arm7_data_processing_test.cpp` (before `main()`):

```cpp
static void mov_reg_no_shift() {
    NDS nds;
    nds.cpu7().state().r[2] = 0x1234'5678u;
    // MOV R0, R2 → 0xE1A0'0002
    run_one(nds, 0x0380'0000u, 0xE1A0'0002u);
    REQUIRE(nds.cpu7().state().r[0] == 0x1234'5678u);
}

static void add_reg_lsl_3() {
    NDS nds;
    nds.cpu7().state().r[1] = 10;
    nds.cpu7().state().r[2] = 1;  // shifted to 8
    // ADD R0, R1, R2, LSL #3 → 0xE081'0182
    run_one(nds, 0x0380'0000u, 0xE081'0182u);
    REQUIRE(nds.cpu7().state().r[0] == 18u);
}

static void sub_reg_lsr_1() {
    NDS nds;
    nds.cpu7().state().r[1] = 20;
    nds.cpu7().state().r[2] = 6;  // shifted to 3
    // SUB R0, R1, R2, LSR #1 → 0xE041'00A2
    run_one(nds, 0x0380'0000u, 0xE041'00A2u);
    REQUIRE(nds.cpu7().state().r[0] == 17u);
}

static void mov_reg_asr_31_sign_extends() {
    NDS nds;
    nds.cpu7().state().r[2] = 0x8000'0000u;
    // MOV R0, R2, ASR #31 → 0xE1A0'0FC2
    run_one(nds, 0x0380'0000u, 0xE1A0'0FC2u);
    REQUIRE(nds.cpu7().state().r[0] == 0xFFFF'FFFFu);
}
```

Register them in `main()`:

```cpp
    mov_reg_no_shift();
    add_reg_lsl_3();
    sub_reg_lsr_1();
    mov_reg_asr_31_sign_extends();
```

- [ ] **Step 2: Verify the tests fail**

Run: `cd build && make arm7_data_processing_test && ctest -R arm7_data_processing_test --output-on-failure`
Expected: failures — the immediate-shift path currently no-ops with a warning.

- [ ] **Step 3: Implement immediate-shift operand2**

In `src/cpu/arm7/arm7_decode.cpp`, replace the `if (i_bit)` branch and the `immediate-shift dp form` warning with:

```cpp
    ShifterResult op2;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state_.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
    } else {
        // Immediate-shifted register operand2.
        const u32 rm         = instr & 0xFu;
        const u32 shift_type = (instr >> 5) & 0x3u;
        const u32 shift_amt  = (instr >> 7) & 0x1Fu;
        const bool c_in      = (state_.cpsr & (1u << 29)) != 0;
        op2 = barrel_shift_imm(
            state_.r[rm],
            static_cast<ShiftType>(shift_type),
            shift_amt,
            c_in);
    }
```

- [ ] **Step 4: Run the tests**

Run: `cd build && make arm7_data_processing_test && ctest -R arm7_data_processing_test --output-on-failure`
Expected: all 18 cases pass.

Run: `ctest --output-on-failure`
Expected: 13/13 green.

- [ ] **Step 5: Commit**

```bash
git add src/cpu/arm7/arm7_decode.cpp tests/unit/arm7_data_processing_test.cpp
git commit -m "cpu/arm7: immediate-shifted register operand2

Wires barrel_shift_imm into the data-processing operand2 decode so
\"ADD R0, R1, R2, LSL #3\" and friends work. Register-shifted-register
form (bit 4 = 1) is still deferred to slice 3b where the +1I cycle
penalty and the \"Rs = PC\" corner case land together."
```

---

## Task 8: Multi-instruction integration test via `NDS::run_frame`

**Files:**
- Create: `tests/unit/arm7_run_sequence_test.cpp`
- Modify: `tests/CMakeLists.txt`

Goal: the capstone test for slice 3a. Load a 5-instruction hand-assembled program into ARM7 WRAM, ask the CPU to execute it via `run_until`, and check the final register file. This proves the fetch/decode/execute loop, the bus, the cycle counting, and the register file all agree with each other across multiple steps.

- [ ] **Step 1: Write the failing test**

Create `tests/unit/arm7_run_sequence_test.cpp`:

```cpp
// Slice 3a capstone: run a multi-instruction sequence end-to-end via
// the real Arm7::run_until against Arm7Bus-backed memory.

#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void arithmetic_sequence_computes_expected_result() {
    NDS nds;

    const u32 base = 0x0380'0000u;

    // Program:
    //   MOV R0, #5          ; R0 = 5
    //   MOV R1, #3          ; R1 = 3
    //   ADD R2, R0, R1      ; R2 = 8
    //   SUB R3, R2, #1      ; R3 = 7
    //   MOV R4, R3, LSL #2  ; R4 = 28
    const u32 program[] = {
        0xE3A0'0005u,  // MOV R0, #5
        0xE3A0'1003u,  // MOV R1, #3
        0xE080'2001u,  // ADD R2, R0, R1
        0xE242'3001u,  // SUB R3, R2, #1
        0xE1A0'4103u,  // MOV R4, R3, LSL #2
    };
    for (u32 i = 0; i < 5; ++i) {
        nds.arm7_bus().write32(base + i * 4, program[i]);
    }

    nds.cpu7().state().pc = base;
    nds.cpu7().state().r[15] = base + 8;

    // Run exactly 5 ARM7 cycles (= 10 ARM9 cycles).
    nds.cpu7().run_until(10);

    REQUIRE(nds.cpu7().state().r[0] == 5u);
    REQUIRE(nds.cpu7().state().r[1] == 3u);
    REQUIRE(nds.cpu7().state().r[2] == 8u);
    REQUIRE(nds.cpu7().state().r[3] == 7u);
    REQUIRE(nds.cpu7().state().r[4] == 28u);
    REQUIRE(nds.cpu7().state().pc   == base + 5 * 4);
    REQUIRE(nds.cpu7().state().cycles == 5u);
}

static void condition_codes_gate_execution_in_sequence() {
    NDS nds;
    const u32 base = 0x0380'0000u;

    // Program:
    //   MOV  R0, #0          ; R0 = 0, clears flags eventually
    //   MOVS R0, #0          ; R0 = 0 with Z=1
    //   MOVEQ R1, #42        ; R1 = 42 (Z set, so taken)
    //   MOVNE R2, #99        ; skipped
    //   MOVS R0, #1          ; R0 = 1, Z=0
    //   MOVEQ R3, #77        ; skipped
    const u32 program[] = {
        0xE3A0'0000u,  // MOV  R0, #0
        0xE3B0'0000u,  // MOVS R0, #0
        0x03A0'102Au,  // MOVEQ R1, #42
        0x13A0'2063u,  // MOVNE R2, #99
        0xE3B0'0001u,  // MOVS R0, #1
        0x03A0'304Du,  // MOVEQ R3, #77
    };
    for (u32 i = 0; i < 6; ++i) {
        nds.arm7_bus().write32(base + i * 4, program[i]);
    }

    nds.cpu7().state().pc = base;
    nds.cpu7().state().r[15] = base + 8;
    nds.cpu7().run_until(12);  // 6 ARM7 cycles

    REQUIRE(nds.cpu7().state().r[0] == 1u);
    REQUIRE(nds.cpu7().state().r[1] == 42u);
    REQUIRE(nds.cpu7().state().r[2] == 0u);  // MOVNE skipped
    REQUIRE(nds.cpu7().state().r[3] == 0u);  // MOVEQ skipped after Z=0
    REQUIRE(nds.cpu7().state().cycles == 6u);
}

int main() {
    arithmetic_sequence_computes_expected_result();
    condition_codes_gate_execution_in_sequence();
    std::puts("arm7_run_sequence_test OK");
    return 0;
}
```

Add to `tests/CMakeLists.txt`:

```cmake
add_ds_unit_test(arm7_run_sequence_test)
```

- [ ] **Step 2: Run the test**

Run: `cd build && cmake .. && make arm7_run_sequence_test && ctest -R arm7_run_sequence_test --output-on-failure`
Expected: both cases pass. If any instruction fails, hand-verify its encoding against GBATEK — most likely the `i` / `s` / `rn` / `rd` bit layout in a hand-assembled literal is wrong.

Run: `ctest --output-on-failure`
Expected: **14/14 test binaries green.**

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_run_sequence_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: end-to-end multi-instruction sequence test

Capstone test for slice 3a: runs two hand-assembled programs through
Arm7::run_until against real Arm7Bus-backed ARM7 WRAM, verifying that
fetch, decode, execution, flag updates, and condition code gating all
agree across multiple consecutive instructions."
```

---

## Verification checklist (run at the end)

- [ ] `cd build && cmake .. && make` is clean — no warnings, no errors.
- [ ] `ctest --output-on-failure` — all 14 test binaries pass.
- [ ] `git status` shows no tracked `CLAUDE.md` change. If it does, `git rm --cached CLAUDE.md` and commit the untracking — but do not remove the file from disk.
- [ ] No `Co-Authored-By` line in any commit introduced by this slice (`git log main..HEAD --grep "Co-Authored"` should be empty).
- [ ] `grep -R "SDL" src/cpu/arm7/ src/cpu/arm9/ src/bus/ src/scheduler/ src/nds.*` is empty — no SDL in core.
- [ ] `src/cpu/arm7/arm7.hpp` does not include `bus/arm7_bus.hpp` directly (it forward-declares `class Arm7Bus;`). The only file that includes the bus header in the ARM7 core is `src/cpu/arm7/arm7.cpp` and `arm7_decode.cpp`.
- [ ] Fresh `NDS nds; nds.run_frame();` does not crash. (It will execute whatever garbage is in main RAM at reset, which is `AND R0,R0,R0` because the RAM is zero-initialized and the DS ARM7 reset vector is 0 in our setup. Our fetch loop reads from ARM7's bus at PC=0 which is not mapped until slice 3d's direct boot, so this may currently log warnings — that is acceptable for the slice as long as it doesn't crash. Verified by `nds_integration_test`.)

### Deferred to slice 3b (do NOT add in this slice)

- Branches: `B`, `BL`, `BX`.
- Loads/stores: `LDR`, `STR`, `LDRH`, `STRH`, `LDRSB`, `LDRSH`, `LDM`, `STM`.
- Multiplies: `MUL`, `MLA`.
- Status register moves: `MRS`, `MSR`.
- Register-shifted-register operand2 (bit 4 = 1).
- Correct per-instruction cycle counts (real ARMv4T has 1S, 2S+1N, etc.).
- Pipeline flush penalty on branches.

### Deferred to slice 3c

- Thumb ISA in full.

### Deferred to slice 3d

- Exception entry (reset/undef/SWI/prefetch abort/data abort/IRQ/FIQ).
- IRQ line sampling in the fetch loop.
- `MOVS pc, lr` / `LDM ..^{pc}` exception-return semantics.
- CPSR mode switches driven by hardware events rather than by tests.
