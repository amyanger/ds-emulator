# ARM7 Core — Phase 1, Slice 3b1 Design

**Status:** proposed
**Date:** 2026-04-14
**Predecessor:** `2026-04-13-arm7-core-phase1-slice3a.md` (plan, landed)
**Parent spec:** `2026-04-12-nds-emulator-design.md` §4 (CPU cores) and §13
(phase roadmap)

---

## 1. Summary

Extend the ARM7 interpreter with the minimum viable set of control-flow and
memory instructions: branches (`B`, `BL`, `BX`) and single-register word /
byte loads and stores (`LDR`, `STR`, `LDRB`, `STRB`). After this slice, the
ARM7 can execute subroutines that compute values in registers, read and
write main RAM, and return to their caller — the smallest program shape
that does anything useful beyond register arithmetic.

This slice is the first of a planned three-way split of the original
slice 3b scope:

- **3b1 (this slice):** branches + word/byte single data transfer.
- **3b2 (future):** halfword/signed loads/stores, `LDM`/`STM`, `MUL`/`MLA`,
  `MRS`/`MSR`, register-shifted-register operand2 for data-processing.
- **Deferred further:** correct per-instruction cycle counts and branch
  pipeline-flush cycle penalty. These are cross-cutting timing work; they
  do not belong inside an opcode-family slice.

The split exists because slice 3a alone was 2044 lines of plan for
data-processing. Attempting to land every 3b deferred item in one slice
produces an unreviewably large patch and has poor failure-mode isolation.

## 2. Goals

- Execute `B`, `BL`, `BX` with correct target computation, link-register
  behavior, and implicit pipeline flush on PC writes.
- Execute `LDR`, `STR`, `LDRB`, `STRB` in all standard addressing modes:
  immediate offset, immediate-shifted register offset, pre/post-index,
  up/down, and writeback.
- Preserve all slice 3a behavior. All 15 existing tests continue to pass
  unchanged.
- Refactor the decode dispatcher to split by instruction family, keeping
  the top-level dispatch tree readable and giving future slices obvious
  homes (halfword/LDM → same file as LDR/STR; MUL/MLA/MRS/MSR → same file
  as DP).

## 3. Non-goals

The following are explicitly out of scope for 3b1 and remain deferred:

- `LDRH`, `STRH`, `LDRSB`, `LDRSH` (halfword and signed byte/halfword
  loads) — slice 3b2.
- `LDM`, `STM` (load/store multiple) — slice 3b2.
- `MUL`, `MLA` — slice 3b2.
- `MRS`, `MSR` — slice 3b2.
- Register-shifted-register operand2 for data-processing instructions —
  slice 3b2.
- Correct per-instruction cycle counts. Every instruction — branches,
  loads, and stores included — still returns 1 ARM7 cycle from
  `dispatch_*`. Real cycle counting is a separate cross-cutting slice.
- Branch pipeline-flush cycle penalty (2S + 1N). Same reason.
- Thumb ISA dispatch — slice 3c. A `BX` into Thumb is in scope only to
  the extent of setting the T flag; the next fetch will trip a deferred-
  Thumb assert, which is the intended failure mode until 3c lands.
- Exception entry, IRQ sampling, coprocessor, `SWI` — slice 3d.

## 4. Context — what slice 3a already gives us for free

The slice 3a fetch / decode / execute loop in `src/cpu/arm7/arm7_decode.cpp`
is structured so that branches and LDR/STR do not need any new plumbing
around the pipeline. Specifically:

1. `step_arm()` writes `state.pc = instr_addr + 4` and
   `state.r[15] = instr_addr + 8` **before** calling `dispatch_arm`. That
   models the ARMv4T 3-stage pipeline: during execute of an instruction,
   R15 reads as the address of that instruction plus 8.
2. Any opcode handler that writes PC — whether via `write_rd` with
   `rd == 15`, or by a direct `state.pc = ...` — causes the next call to
   `step_arm()` to fetch at the new PC. No separate "pipeline flush" step,
   no prefetch buffer to invalidate.
3. Reads of `state.r[15]` inside a handler naturally return
   `instr_addr + 8`, which is exactly the value ARMv4T defines for R15
   used as a source during branch target computation and LDR/STR offset
   computation.

The cycle-cost side of "pipeline flush" (2S + 1N instead of 1S on a
branch) is separate from the correctness side and stays deferred.

## 5. Architecture

### 5.1 File layout after this slice

```
src/cpu/arm7/
  arm7.hpp                      (unchanged)
  arm7.cpp                      (unchanged)
  arm7_state.hpp                (unchanged)
  arm7_alu.hpp                  (unchanged)

  arm7_decode.cpp               SHRINKS. Owns step_arm() + dispatch_arm().
                                dispatch_arm() becomes a small switch on
                                bits 27..25 that delegates to family
                                handlers below.

  arm7_decode_internal.hpp      NEW. Private header exposing the family
                                dispatch helpers and shared write_rd()
                                to the other arm7_*.cpp files. Not part
                                of any public include path.

  arm7_dp.cpp                   NEW. dispatch_dp(). Contains the slice 3a
                                data-processing logic moved verbatim, plus
                                a special-case `BX` recognizer at the top
                                because BX lives in DP encoding space.

  arm7_branch.cpp               NEW. dispatch_branch(). Handles B and BL.

  arm7_loadstore.cpp            NEW. dispatch_single_data_transfer().
                                Handles LDR/STR/LDRB/STRB in all standard
                                addressing modes.
```

The DP code move (slice 3a behavior → `arm7_dp.cpp`) is a standalone
no-behavior-change commit, so bisect blames a regression on the correct
commit.

### 5.2 Top-level dispatch tree

```
dispatch_arm(instr):
  cond = instr[31:28]
  if !eval_condition(cond, cpsr):
      return 1

  switch bits[27:25]:
      000, 001 → dispatch_dp(instr)                     // includes BX
      010, 011 → dispatch_single_data_transfer(instr)   // LDR/STR(B)
      100      → warn("LDM/STM deferred"); return 1
      101      → dispatch_branch(instr)                 // B / BL
      110, 111 → warn("coproc/SWI deferred");  return 1
```

The condition check stays at the top so family handlers never see a
condition-failed instruction. Condition-failed still returns 1 cycle,
same as slice 3a.

### 5.3 Shared helpers

`write_rd(state, rd, value)` — currently a file-local lambda-like helper in
`arm7_decode.cpp` — moves to `arm7_decode_internal.hpp` as an
`inline` function. Both `arm7_dp.cpp` and `arm7_loadstore.cpp` need it
(data-processing writes, and LDR with Rd == R15). A single implementation
prevents "DP uses `write_rd`, LDR uses its own open-coded version" drift.

## 6. Instruction semantics

All values below use the slice 3a convention that `state.r[15]` has
already been set to `instr_addr + 8` by `step_arm()`.

### 6.1 `B` / `BL`

Encoding: `cond 101 L offset24`. L=1 selects `BL`.

```
offset    = sign_extend_24(instr & 0x00FFFFFF) << 2
target    = state.r[15] + offset            // == instr_addr + 8 + offset
if L:
    state.r[14] = state.pc                   // == instr_addr + 4
state.pc  = target & ~0x3u
return 1 cycle
```

Target computation uses 32-bit wrapping arithmetic, so a maximally negative
24-bit offset sign-extends and wraps correctly.

### 6.2 `BX`

Encoding: `cond 0001 0010 1111 1111 1111 0001 Rm`. Sits inside the DP
encoding space (bits 27..26 == 00, bit 4 == 1), so `dispatch_dp` recognises
it via pattern match on `(instr & 0x0FFFFFF0) == 0x012FFF10` **before**
falling into the generic "register-shifted DP, deferred" warn path.

```
rm        = instr & 0xF
rm_val    = state.r[rm]                      // rm==15 reads instr_addr+8
thumb     = (rm_val & 0x1u) != 0
state.pc  = rm_val & (thumb ? ~0x1u : ~0x3u)
if thumb:
    state.cpsr |= (1u << 5)                  // T bit
return 1 cycle
```

**Thumb mode handling.** The Thumb dispatcher is deferred to slice 3c.
`step_arm()` will gain an assert at the top that trips if `cpsr.T` is set,
so a `BX` into a Thumb target causes a loud, single failure on the next
instruction fetch rather than silent ARM-mode misexecution of a Thumb
instruction stream. Tests in this slice that exercise `BX` use ARM targets
(bit 0 clear) to stay inside scope; a single dedicated test verifies the T
flag is set and stops before the next fetch.

### 6.3 `LDR` / `STR` / `LDRB` / `STRB`

Encoding: `cond 01 I P U B W L Rn Rd offset12`.

- `I` (25): 0 = immediate offset, 1 = register offset (immediate-shifted).
- `P` (24): 1 = pre-index, 0 = post-index.
- `U` (23): 1 = add offset, 0 = subtract.
- `B` (22): 1 = byte, 0 = word.
- `W` (21): writeback (only meaningful with P=1; post-index always writes
  back).
- `L` (20): 1 = load, 0 = store.

**Offset computation:**

```
if I == 0:
    offset = instr & 0xFFF
else:
    rm         = instr & 0xF
    shift_type = (instr >> 5) & 0x3
    shift_amt  = (instr >> 7) & 0x1F
    offset     = barrel_shift_imm(r[rm], shift_type, shift_amt, cpsr.C).value
                 // carry output is discarded for LDR/STR
```

Note that LDR/STR's "register offset" is only ever immediate-shifted;
there is no register-shifted-register form for LDR/STR in ARMv4T. This
means we can reuse the existing `barrel_shift_imm` helper unchanged.

**Address computation:**

```
base        = state.r[rn]                    // rn==15 reads instr_addr+8
signed_off  = U ? +offset : -offset
pre_addr    = base + signed_off               // 32-bit wrap
access_addr = P ? pre_addr : base             // post-index accesses base
wb_addr     = (P && !W) ? base : pre_addr     // writeback target if any
```

**Access:**

```
if L == 1:  // load
    if B == 1:
        loaded = bus.read8(access_addr)       // zero-extended to 32 bits
    else:
        raw    = bus.read32(access_addr & ~0x3u)
        loaded = rotate_right(raw, (access_addr & 0x3u) * 8u)
else:       // store
    stored = (rd == 15) ? (instr_addr + 12) : state.r[rd]
    if B == 1:
        bus.write8(access_addr, stored & 0xFFu)
    else:
        bus.write32(access_addr & ~0x3u, stored)
```

**Writeback:**

```
do_wb = (P == 0) || (W == 1)
// ARMv4T: on LDR, the loaded value wins over writeback if Rn == Rd.
if do_wb && !(L == 1 && rn == rd):
    write_rd(state, rn, wb_addr)              // rn==15 path also stomps state.pc
if L == 1:
    write_rd(state, rd, loaded)               // rd==15 stomps state.pc
return 1 cycle
```

Both writeback paths route through the single shared `write_rd` helper
from §5.3, so the PC-mirror behavior on R15 writes is defined in one
place. When `Rn == Rd` and `L == 1`, the writeback branch is suppressed
and only the load writes `Rd` — matching ARMv4T's "load wins" rule.

**Hardware quirks captured:**

- **Unaligned word load rotate.** ARMv4T `LDR` from an unaligned address
  reads the aligned word and rotates the loaded value right by
  `(addr & 3) * 8` bits. This is not a halfword load — the bus access
  itself is always 4-byte aligned, only the register value rotates. The
  `~0x3u` mask on `bus.read32` captures this.
- **Unaligned word store.** ARMv4T `STR` to an unaligned address silently
  drops the low 2 bits of the address; the stored value is unrotated.
  Captured by the `~0x3u` mask on `bus.write32`.
- **`STR` of R15.** Stored value is `instr_addr + 12`, not `instr_addr + 8`.
  (The +12 reflects the pipeline state at the store-stage of a real
  ARMv4T core.) Captured by the `(rd == 15) ? (instr_addr + 12) : …` line.
- **`LDR` into R15.** Writes PC directly; ARMv4T does not use bit 0 as a
  Thumb switch (that is ARMv5T+). The existing `write_rd` already masks
  PC writes with `~0x3u` which is correct for ARMv4T.
- **`Rn == Rd` with `LDR` writeback.** ARMv4T defines the load as winning.
  The writeback branch skips itself when `L == 1 && rn == rd`.
- **Pre-index writeback with `rn == 15`.** Pathological and unlikely in
  real code, but for correctness it still must update `state.pc` via the
  pc-mirror line in the writeback block. Captured.

## 7. Test plan

New test binaries under `tests/unit/`. All follow the slice 3a pattern:
instantiate `NDS`, write machine code into main RAM, set `state_.r[15]` /
`state_.pc`, run, assert.

### 7.1 `arm7_branch_test.cpp`

- `B` with small positive offset — verify `pc == target`.
- `B` with small negative offset (branch back one instruction) — verify
  `pc == target`.
- `B` with a maximally negative 24-bit offset — verify target computation
  wraps correctly in 32 bits.
- `B` with a condition that fails — verify `pc` advanced by 4, not to the
  target.
- `BL` — verify `r[14] == instr_addr + 4` and `pc == target`.
- `BL` + `MOV pc, lr` return round trip — verify we end up at the
  instruction after the original `BL` and that the DP path handles
  `Rd == R15` correctly in sequence with a branch.
- `BX` with an ARM target — verify `pc == rm_val & ~3` and that the T
  bit in `cpsr` is clear.
- `BX` with a Thumb target (low bit set) — verify `cpsr.T == 1` and that
  `pc == rm_val & ~1`. Test stops before the next fetch (does not exercise
  the deferred-Thumb assert).

### 7.2 `arm7_load_store_test.cpp`

One test per meaningful addressing-mode combination, so a regression
points at exactly one case:

- Word `LDR`, immediate offset, U=1 — seed RAM, load, verify register.
- Word `STR`, immediate offset, U=1 — store, verify RAM.
- Byte `LDRB` — verify zero extension (high 24 bits of destination are 0).
- Byte `STRB` — verify only the target byte changes; neighbors untouched.
- Pre-index with writeback (P=1, W=1) — verify `Rn` updated to `pre_addr`.
- Pre-index without writeback (P=1, W=0) — verify `Rn` unchanged.
- Post-index (P=0) — verify access uses `base`, then `Rn` updated to
  `pre_addr`.
- U=0 (subtract offset) — verify negative-direction addressing.
- Register offset (I=1) with `LSL #2` — common "index × 4" pattern.
- Register offset with `shift_amt == 0` — LSL #0 identity path.
- Unaligned word `LDR`: seed aligned RAM with a known word, load from
  `aligned_addr + 1` — verify rotation matches `ROR by 8`.
- Unaligned word `LDR` with every other rotate amount (+2, +3) — verify
  ROR by 16 and ROR by 24.
- `LDR` with `Rd == R15` — load a target address, verify next fetch lands
  there.
- `STR` with `Rd == R15` — verify stored word equals `instr_addr + 12`.
- `LDR` with `Rn == Rd` writeback — verify the loaded value wins over the
  writeback.

### 7.3 `arm7_branch_load_sequence_test.cpp`

End-to-end program mirroring slice 3a's `arm7_run_sequence_test`:

- LDR a value from main RAM into R0.
- ADD an immediate to R0 via the existing DP path.
- STR R0 back to a different main RAM address.
- B forward to a cleanup block.
- Cleanup block does a BL to a leaf, the leaf does `MOV pc, lr`, then
  the cleanup block does its own `MOV pc, lr` back to the test harness
  exit address.
- Verify final register state AND final memory state.

This test exists to make sure the dispatch tree holds together under
multi-instruction execution, not to exercise any single instruction in
isolation.

### 7.4 Pre-existing tests

All 15 slice-3a and earlier tests must continue to pass unchanged. The
DP-move commit is the proof of non-regression for the refactor itself.

## 8. Risks and mitigations

- **Risk:** the decoder refactor breaks slice 3a behavior in a subtle way.
  **Mitigation:** land the DP move as a standalone "no new opcodes" commit
  and run `ctest` at that checkpoint. If anything regresses, bisect lands
  on the move commit directly.

- **Risk:** a bug in LDR/STR address computation corrupts main RAM during
  unrelated tests.
  **Mitigation:** tests seed RAM at known addresses and assert on both
  target AND neighbor bytes after a byte store. The STRB neighbor-check
  test specifically exists for this.

- **Risk:** `BX` into Thumb silently executes garbage ARM instructions
  until a crash far from the actual bug.
  **Mitigation:** `step_arm()` gains an assert on `cpsr.T` that trips on
  the very next fetch, so the BX site is the failure point.

- **Risk:** sign extension of the 24-bit branch offset wraps wrong.
  **Mitigation:** explicit test with a maximally negative offset.

- **Risk:** LDR's unaligned-rotate behavior gets dropped when someone
  later refactors the access path.
  **Mitigation:** three dedicated tests (rotate by 8, 16, 24) so the
  invariant is pinned down.

## 9. Verification checklist

Before declaring 3b1 done:

- [ ] `cd build && cmake .. && make` is clean — no warnings, no errors.
- [ ] `ctest --output-on-failure` passes — all slice 3a tests still green,
      all new tests green.
- [ ] No SDL include pulled into `src/cpu/arm7/` (`grep -R "SDL"
      src/cpu/arm7/` is empty).
- [ ] `arm7_decode_internal.hpp` is not included from any file outside
      `src/cpu/arm7/`.
- [ ] No `Co-Authored-By` line in any commit introduced by this slice.
- [ ] `CLAUDE.md` is still gitignored and untracked.
- [ ] Fresh `NDS nds; nds.run_frame();` still does not crash.

## 10. Out-of-scope items still tracked for slice 3b2

For continuity with the original slice 3b deferred list:

- `LDRH`, `STRH`, `LDRSB`, `LDRSH`
- `LDM`, `STM`
- `MUL`, `MLA`
- `MRS`, `MSR`
- Register-shifted-register operand2 (bit 4 == 1 in DP space, excluding
  the BX pattern which is handled in 3b1)
- Real per-instruction cycle counts
- Branch pipeline-flush cycle penalty

And for slice 3c:

- Full Thumb ISA
- Thumb dispatcher in `step_thumb` / removal of the cpsr.T assert

And for slice 3d:

- Exception entry (reset / undef / SWI / abort / IRQ / FIQ)
- IRQ line sampling in the fetch loop
- `MOVS pc, lr` / `LDM ..^{pc}` exception-return semantics
