# ARM7 Core — Phase 1, Slice 3c Design

**Date:** 2026-04-16
**Slice:** ARM7 Thumb instruction set (ARMv4T)
**Status:** proposed
**Prior slices:** 3a (scaffolding), 3b1 (data processing), 3b2 (branch + single data transfer), 3b3 (halfword/signed load-store), 3b4 (block transfer + swap)
**Next slice:** 3d (exceptions — IRQ line sampling, SWI dispatch, undefined-instruction entry, abort entry)

---

## 1. Summary

This slice implements the full Thumb (ARMv4T) instruction set on the ARM7. Thumb is the 16-bit opcode state that ARM7TDMI can execute alongside 32-bit ARM state, toggled via the `T` bit in CPSR. Pokemon games — and virtually every commercial DS cart — run the majority of ARM7-side code in Thumb because the cut-down encoding halves cartridge memory pressure.

After this slice, the ARM7 can execute both ARM and Thumb instructions. The only ARMv4T behavior still missing from the ARM7 will be exception entry (IRQ, SWI, undefined, prefetch/data abort) and IRQ line sampling — which together make up slice 3d.

### Plain-language summary of what the slice adds

Think of Thumb as "ARM in a smaller hat." The ARM7 chip has two ways it can read instructions: 32-bit wide (ARM state) and 16-bit wide (Thumb state). It flips between them at runtime. We've already taught the emulator every 32-bit instruction; this slice teaches it every 16-bit instruction. The trick that keeps this slice small is that most 16-bit Thumb instructions are just "fewer-bits shorthand" for instructions we already implement — so the Thumb decoder mostly repackages its operands and delegates into helpers that already exist from slices 3b1–3b4. The notable exceptions are three formats that need new helpers (relative-address computation, SP adjust, and the two-halfword BL encoding) and a handful of format-specific quirks that don't appear in ARM state at all.

### Scope boundary

**In scope:** every Thumb format THUMB.1 through THUMB.19, with the exception of ARMv5-only sub-encodings called out in §3.

**Out of scope:**
- ARMv5-only Thumb forms (Thumb BLX label, Thumb BLX register, BKPT, the ARMv5 POP {PC} interworking behavior, ARMv5 MUL flag behavior). These land when the ARM9 arrives.
- The SWI exception mechanism — slice 3c decodes THUMB.17 to a warn stub only. Actual mode switch, vector jump, and banked-register save land in slice 3d.
- IRQ line sampling and the IRQ entry vector. Slice 3d.

---

## 2. Goals

1. **Correctness against GBATEK** for every Thumb format executed by commercial DS code, including the ARMv4-specific edge cases documented in `docs/specs/.notes/thumb-gbatek-reference.md`.
2. **Maximum reuse.** Most Thumb formats must call into executor helpers factored out of the ARM state decoder rather than duplicating logic. Code duplication between the ARM and Thumb execution paths is a review block.
3. **TDD discipline.** Every Thumb format lands in its own commit with its own unit test. The slice capstone is a sequence test that assembles a hand-written Thumb routine into memory, runs it under the decoder, and verifies the final register state.
4. **Behavior preservation of ARM state.** The five executor-extraction refactors that land in Phase A of the slice must not change any observable ARM-state behavior. Every existing ARM-state unit test must continue to pass unchanged through Phase A.
5. **File count and size within house rules.** Five new `.cpp` files plus one new internal header. No file exceeds the 500-line soft cap or the 800-line hard cap.

---

## 3. Non-goals

- **Thumb BLX label** (second halfword with top bits `11101`) — ARM9 only.
- **Thumb BLX register** (THUMB.5 op=3 with MSBd set) — ARM9 only.
- **BKPT** (THUMB.17 `10111110b` encoding) — ARM9 only.
- **POP {PC} interworking** (ARMv5's rule that bit 0 of the loaded PC value sets CPSR.T) — ARM7 stays in Thumb state regardless, which is the ARMv4 behavior.
- **MUL ARMv5 flag preservation** — slice 3c implements the ARMv4 behavior where THUMB.4 MUL destroys C.
- **SWI dispatch.** THUMB.17 SWI decodes to a warn stub. The mode switch, SPSR_svc save, T-bit clear, and vector jump to 0x00000008 land in slice 3d.
- **IRQ sampling.** `step_thumb` executes exactly one instruction without checking the interrupt line. Slice 3d adds sampling.
- **Thumb versions of 3D engine command buffering / DMA / timers.** Those are separate subsystems entirely.
- **Cycle-accurate Thumb timing.** Slice 3c returns a coarse cycle count per instruction (1 cycle per Thumb instruction with the block/branch cycle adjustments from GBATEK §Execution Time notes). Cycle accuracy against test ROMs is a Phase 6 concern.

---

## 4. Architecture

### 4.1 Decoder dispatch strategy

**Fan-out on bits[15:13].** Eight top-level buckets mapped directly onto the GBATEK binary format table. Each bucket either routes to a single per-family dispatcher, or fans out further on bits[12:10] (or narrower) into a sub-dispatcher. This mirrors the ARM-state decoder's "fan-out on bits[27:25]" style.

**Why this and not alternatives:**

- A 256-entry lookup table on bits[15:8] is faster at dispatch but pays readability and debuggability costs that aren't justified in an interpreter-first build. Slice 3c is not on the critical path — bus traffic and the 3D rasterizer will dominate emulator performance long before Thumb decode does.
- Fan-out on bits[15:12] (16 buckets) splits some format families across buckets for no benefit.
- The chosen bits[15:13] 8-way approach lines up one-for-one with GBATEK's binary format table, so anyone reading the dispatcher against GBATEK can track format by format.

**Top-level dispatcher signature:**

```cpp
u32 dispatch_thumb(Arm7State& state, Arm7Bus& bus, u16 instr, u32 instr_addr);
```

**Bucket routing (bits[15:13]):**

| Bucket | Routes to | Formats |
|--------|-----------|---------|
| `000`  | `dispatch_thumb_shift_or_addsub`  | THUMB.1, THUMB.2 |
| `001`  | `dispatch_thumb_imm_dp`           | THUMB.3 |
| `010`  | `dispatch_thumb_010_space`        | THUMB.4, THUMB.5, THUMB.6, THUMB.7, THUMB.8 |
| `011`  | `dispatch_thumb_ldst_imm_wb`      | THUMB.9 |
| `100`  | `dispatch_thumb_100_space`        | THUMB.10, THUMB.11 |
| `101`  | `dispatch_thumb_101_space`        | THUMB.12, THUMB.13, THUMB.14 |
| `110`  | `dispatch_thumb_110_space`        | THUMB.15, THUMB.16, THUMB.17 (SWI stub) |
| `111`  | `dispatch_thumb_111_space`        | THUMB.18, THUMB.19 halves |

Inner fan-outs within the `_space` handlers use 2-3 additional bits and are single-file `switch` blocks.

### 4.2 File layout

Five new `.cpp` files and one new internal header:

```
src/cpu/arm7/arm7_thumb_decode.cpp    dispatch_thumb + the eight bucket
                                      handlers / sub-fan-outs. ~250 lines.

src/cpu/arm7/arm7_thumb_dp.cpp        THUMB.1, .2, .3, .4, .5 hi-reg
                                      ADD/MOV/CMP, .12, .13. ~300 lines.

src/cpu/arm7/arm7_thumb_ls.cpp        THUMB.6, .7, .8, .9, .10, .11.
                                      Every load/store format. ~300 lines.

src/cpu/arm7/arm7_thumb_branch.cpp    THUMB.5 BX, .16 Bcond, .17 SWI
                                      stub, .18 B, .19 BL (both halves).
                                      ~200 lines.

src/cpu/arm7/arm7_thumb_block.cpp     THUMB.14 PUSH/POP, .15 LDMIA/STMIA.
                                      ~150 lines.

src/cpu/arm7/arm7_thumb_internal.hpp  Dispatcher and helper declarations
                                      shared between the five .cpp files.
```

Updates to existing files:

- `arm7.hpp` — add `u32 step_thumb()` method declaration.
- `arm7.cpp` / `arm7_decode.cpp` — add `step_thumb()` definition alongside `step_arm()`; `run_until` branches between them on CPSR.T.
- `arm7_decode_internal.hpp` — gains five new executor entry points (see §4.3 below).
- `arm7_dp.cpp`, `arm7_decode_000.cpp`, `arm7_loadstore.cpp`, `arm7_halfword.cpp`, `arm7_block.cpp` — refactored to call the new executors. Behavior-preserving.
- `src/cpu/arm7/CMakeLists.txt` — add the five new `.cpp` files to `libds_core`.
- `tests/unit/CMakeLists.txt` — add three new test binaries (see §6).

### 4.3 Delegation strategy — executor extraction

Thumb delegates into five **executor** functions that expose the core execution logic of the ARM-state path with explicit parameters, so the ARM decoder (which parses the full 32-bit instruction word) and the Thumb decoder (which builds a smaller parameter set from the 16-bit encoding) both call the same code.

**Pre-refactor commits 1–5 land before any Thumb format.** Each refactor is strictly behavior-preserving: the extracted function contains the exact body of the original logic, minus the bit-unpacking preamble, and the original dispatcher calls through to it. Every existing ARM-state test continues to pass unchanged.

The five extractions:

1. **`execute_dp_op`** — from `arm7_dp.cpp`:`dispatch_dp`. Takes `(state, op, rn_value, rhs_value, rhs_shifter_carry, s_flag, rd, instr_addr)`. `s_flag` is a parameter, not a bit from the instruction word — this is the seam THUMB.5 hi-reg uses (see §4.4 (a)).

2. **`execute_bx`** — from `arm7_decode_000.cpp`:`dispatch_000_space`. Extracts the inline 7-line BX block into a named function. Takes `(state, rm_value)`. Thumb THUMB.5 op=3 and ARM BX both call it.

3. **`execute_single_data_transfer_core`** — from `arm7_loadstore.cpp`:`dispatch_single_data_transfer`. Takes `(state, bus, addr, rd, is_load, is_byte, instr_addr)`. The ARM-specific address calculation and Rn writeback stay in the wrapper; the bus access, rotate-on-unaligned, and Rd writeback move into the core. Thumb LDR/STR word/byte formats call the core directly with a Thumb-computed address.

4. **`execute_halfword_transfer_core`** — from `arm7_halfword.cpp`:`dispatch_halfword`. Takes `(state, bus, addr, rd, kind, instr_addr)` where `kind ∈ {LDRH, LDSB, LDSH, STRH}`. Same split: ARM keeps address calculation and base writeback in the wrapper. Incidentally, this replaces the anonymous-namespace `load_halfword_unsigned`/`load_halfword_signed`/`load_byte_signed`/`store_halfword` helpers that currently live TU-private inside `arm7_halfword.cpp` — they get absorbed into the core executor and become accessible to Thumb.

5. **`execute_block_transfer`** — from `arm7_block.cpp`:`dispatch_block`. Takes `(state, bus, rn_index, reg_list, p, u, s, w, l, instr_addr)`. The entire LDM/STM body (addressing normalization via `compute_block_addressing`, STM Rn-in-list early-writeback rule, transfer loop, S-bit user-bank path, S-bit exception return for LDM-with-R15, empty-list quirk) moves into the core. The ARM wrapper shrinks to a bit-unpack-and-call. Thumb PUSH/POP and LDMIA/STMIA build their own `(p, u, s, w, l, rn, reg_list)` tuples and call the core.

**Three helpers that need no refactor:** `rotate_read_word` is already a free function in `arm7_decode_internal.hpp`. `eval_condition` is already pure in `arm7_alu.hpp`. The shifter/ALU/flag primitives (`barrel_shift_imm`, `barrel_shift_reg`, `set_nz`, `set_c`, `set_v`, `adc`, `sbc`) are already pure in `arm7_alu.hpp`.

**One more extraction worth flagging, not formally part of the five:** the `write_rd` helper at `arm7_decode_internal.hpp:19` already handles R15 masking correctly. Thumb uses it as-is for every Rd write.

### 4.4 The three awkward cases

#### (a) THUMB.5 hi-reg ADD/MOV not touching CPSR

Unlike THUMB.1–4, where flag updates are implicit in the format, THUMB.5 ADD and MOV explicitly do not touch CPSR — only CMP does. This is easy to get wrong because it's the opposite of every other Thumb DP format.

**Solution:** after the `execute_dp_op` extraction, the `s_flag` parameter is explicit. THUMB.5 passes `s_flag = false` for ADD/MOV and `s_flag = true` for CMP. The NOP encoding (`MOV R8, R8`) lands in the normal execute path as a plain register copy with no flag update — no special case needed.

One additional detail: when Rs or Rd is R15 in THUMB.5, the read value is `(instr_addr + 4) & ~2` (word-align-down). The Thumb hi-reg handler materializes this value at the top of the function and passes it explicitly — see §4.5.

#### (b) THUMB.14 PUSH/POP routing into the block core

PUSH and POP are special forms of STMDB / LDMIA on R13 with fixed P/U/W and a restricted register list (R0–R7 plus an optional LR-or-PC bit). They cannot reuse `dispatch_block` directly because `dispatch_block` parses the 32-bit ARM encoding inline.

**Solution:** commit 5 extracts `execute_block_transfer` as a reusable entry point. After that lands, PUSH and POP become two-line wrappers that build a synthetic `reg_list` (low 8 bits from the Thumb encoding, bit 14 or bit 15 set from the R bit) and call the core:

- PUSH: `execute_block_transfer(state, bus, 13, reg_list_with_lr, p=1, u=0, s=0, w=1, l=0, instr_addr)` — full-descending store (STMDB SP!).
- POP:  `execute_block_transfer(state, bus, 13, reg_list_with_pc, p=0, u=1, s=0, w=1, l=1, instr_addr)` — ascending load with post-increment (LDMIA SP!).

The ARMv4 behavior where POP {PC} stays in Thumb state (no bit-0 interworking) falls out automatically because the block core writes R15 via `write_rd`, which masks the low bits and never touches CPSR.T.

#### (c) THUMB.19 BL two-halfword encoding

THUMB.19 is the only Thumb instruction that occupies 32 bits. It decodes as two sequential 16-bit instructions. GBATEK notes that exceptions may or may not occur between the two halves — "implementation defined."

**Solution: each halfword is a standalone instruction that visibly modifies LR.**

- First halfword (`11110 imm11_hi`): `LR = (instr_addr + 4) + (sext(imm11_hi) << 12)`. PC advances by 2.
- Second halfword (`11111 imm11_lo`): read the *current* `state.r[14]`, compute `target = LR + (imm11_lo << 1)`, save return address as `LR = next_instr_addr | 1` (thumb bit), write `PC = target`.

**Why no transient state on `Arm7`:**

1. Games deliberately emit the second halfword alone as a "BL LR+imm" idiom. Example: Mario Golf Advance Tour uses `0xF800` as "BL LR+0" to call through a pointer pre-staged in LR. A stateful decoder that required the first halfword to have been seen would break this.
2. Transient state complicates save-state serialization (either a new field in every save-state version, or mid-BL state isn't serializable).
3. The model is wrong in the presence of interrupts. Per GBATEK the two halves may be interrupted on real hardware. A stateless "halfword 1 updates LR, halfword 2 reads LR and branches" model yields the correct behavior through any IRQ interleaving for free.

The `11101` second-halfword form (BLX label) is explicitly UNDEF on ARMv4 and routes to a warn stub.

### 4.5 PC offset handling (the `+4` rule)

**ARM state uses `instr_addr + 8`. Thumb state uses `instr_addr + 4`.** Getting this wrong produces off-by-2 bugs that pass aligned-instruction unit tests and only trigger when `instr_addr % 4 == 2`.

**Solution:**

1. `step_thumb()` establishes the invariant `state.r[15] = instr_addr + 4` before calling `dispatch_thumb`. This is the exact parallel of `step_arm`'s `state.r[15] = instr_addr + 8` at `arm7_decode.cpp:62`. Any Thumb handler that reads `state.r[15]` directly gets the Thumb-offset value for free.

2. Literal-pool formats (THUMB.6 LDR PC-rel, THUMB.12 ADD PC, THUMB.5 hi-reg Rs=15) additionally need `(instr_addr + 4) & ~2` — a word-align-down beyond the base +4. `dispatch_thumb` materializes two values at the top:

   ```cpp
   const u32 pc_read     = instr_addr + 4;          // Rs=15 in most formats
   const u32 pc_literal  = (instr_addr + 4) & ~2;   // literal-pool formats
   ```

   and passes whichever one the family handler needs as an explicit parameter. Format handlers for THUMB.6 / .12 / the hi-reg Rs=15 path take `pc_literal`; everything else reads `state.r[15]` directly (== `pc_read`).

**Rejected alternatives:**

- A dedicated `read_gpr_thumb` helper — doesn't handle the literal-pool word-align-down, so every literal-pool format still has to mask bit 1 manually.
- A mode-aware `read_gpr_with_pc_offset` that branches on CPSR.T — adds a branch to every register read, easy to forget for new instructions, classic action-at-a-distance bug source.

### 4.6 What does NOT change

- The ARM-state execution path. Phase A refactors are behavior-preserving and gated on the existing ARM-state unit tests continuing to pass.
- The Arm7Bus interface. Thumb uses the same bus object and the same read/write entry points as ARM.
- The scheduler interface. `step_thumb` returns a cycle count the same way `step_arm` does. The scheduler does not learn about Thumb.
- The register file. Thumb uses R0–R15, CPSR, and (when banked) SPSR the same way ARM does.
- `write_rd` — already masks R15 correctly, used as-is.
- The `arm7_block_internal.hpp`'s `compute_block_addressing` helper — already externally callable, used by both ARM dispatch and the new `execute_block_transfer`.

### 4.7 Known technical debt deferred to later slices

- SWI is a warn stub in this slice. Slice 3d replaces it with the full supervisor-mode entry: save return address in `R14_svc = instr_addr + 2`, save `CPSR` in `SPSR_svc`, `switch_mode(Mode::Supervisor)`, clear T and set I in CPSR, `write_rd(state, 15, 0x00000008)`. The warn stub in slice 3c is deliberate so that slice 3d has exactly one place to change.
- IRQ line sampling. `step_thumb` executes one instruction unconditionally. Slice 3d adds "sample IRQ line, if pending and `I` bit clear then enter IRQ mode" to both `step_arm` and `step_thumb` at the same time.
- Cycle-accurate Thumb timing. The coarse cycle counts from §6 are enough for functional correctness but will need tightening in Phase 6.
- ARM9 delta work (BLX, BKPT, ARMv5 POP {PC} interworking, ARMv5 MUL flag behavior). These go in the ARM9 slice. The ARMv4 choices made in slice 3c are documented so the ARM9 implementer knows which defaults to flip.

---

## 5. Hardware details

All references are to `docs/specs/.notes/thumb-gbatek-reference.md`, which is in turn sourced from `mgba-emu/gbatek` index.md lines 120175–120825.

### 5.1 PC offset in Thumb state

- Thumb PC (`R15` during execute) is `instr_addr + 4`, not `+8`.
- Literal-pool formats (THUMB.6, THUMB.12, THUMB.5 with Rs=15) additionally force-align: `(instr_addr + 4) & ~2`.

### 5.2 Shift-amount zero-encoding quirks (THUMB.1, THUMB.4 reg-shift)

- `LSL #0`: no shift, C flag unchanged.
- `LSR #0` and `ASR #0` in source are encoded as `LSR/ASR #32` in the machine form.
- Register-shift with shift amount == 0 leaves C flag unchanged (`barrel_shift_reg` already handles this, callers must preserve CPSR.C).

### 5.3 Flag-update rules by format

- THUMB.1 shifts: N, Z, C update; V unchanged. C unchanged for LSL #0 specifically.
- THUMB.2 add/sub (incl. MOV pseudo): full N, Z, C, V.
- THUMB.3 imm: MOV updates N, Z only; CMP/ADD/SUB update full NZCV.
- THUMB.4 ALU:
  - ADC/SBC/NEG/CMP/CMN: N, Z, C, V.
  - LSL/LSR/ASR/ROR: N, Z, C (C unchanged if shift amount == 0).
  - MUL on **ARMv4**: N, Z set; **C destroyed** (implementation-defined garbage); V unchanged. We pick a deterministic destroy pattern (see §5.6).
  - AND/EOR/TST/ORR/BIC/MVN: N, Z only.
- THUMB.5 hi-reg: ADD/MOV do NOT touch CPSR; only CMP does.
- All load/store and branch formats: no flag updates.

### 5.4 Literal-pool and PC-relative forms

- THUMB.6 LDR PC-rel: 32-bit word load, always word-aligned (effective address is `pc_literal + (imm8 << 2)`). No rotate-on-misalign — source PC is pre-aligned.
- THUMB.12 ADD PC: `Rd = pc_literal + (imm8 << 2)`. Pre-aligned.
- THUMB.12 ADD SP: `Rd = SP + (imm8 << 2)`. SP is not force-aligned by the instruction — if game code misaligned SP, the arithmetic result simply carries the misalignment.

### 5.5 Halfword alignment rules (THUMB.8, THUMB.10)

Same rules as slice 3b3 ARM-state halfword ops:
- LDRH at address with bit 0 set: read from `addr & ~1`, zero-extend. (We match melonDS's "mask addr[0]" choice from slice 3b3.)
- LDSH at address with bit 0 set: sign-extend the byte at `(addr & ~1) + 1` — slice 3b3's convention.
- STRH: mask `addr[0]` before the write.

Slice 3c reuses `execute_halfword_transfer_core` which encodes these rules.

### 5.6 THUMB.4 MUL on ARMv4

ARMv4 specifies that MUL's effect on the C flag is implementation-defined but destroyed. We pick: **C is set to the inverse of its pre-MUL value**. This makes the destroy observable in a unit test (see risk 2 in §8). The ARM9 slice will flip this to "C unchanged" per ARMv5.

### 5.7 THUMB.14 PUSH/POP addressing

- PUSH = full-descending store: SP pre-decrements, registers stored at the new lower address. Equivalent ARM form: `STMDB SP!, {Rlist, LR?}` → `p=1, u=0, w=1, l=0`.
- POP = ascending load with post-increment: registers loaded starting at SP, SP post-incremented. Equivalent: `LDMIA SP!, {Rlist, PC?}` → `p=0, u=1, w=1, l=1`.
- Lowest register in Rlist goes to the lowest memory address.
- R bit semantics: PUSH R=1 adds LR to the list; POP R=1 adds PC to the list.

### 5.8 THUMB.15 LDMIA/STMIA edge cases

- Base register always writes back (implicit W).
- Empty Rlist (ARMv4): R15 is transferred as if bit 15 were set, and Rb += 0x40. Same as the ARM-state empty-list behavior from slice 3b4.
- STM with Rb in Rlist: if Rb is the lowest-numbered entry, store OLD base value; otherwise store NEW base value. (Matches ARM-state STM rule.)
- LDM with Rb in Rlist: no writeback. (**Different from ARM: the ARM ARMv4 LDM rule is "writeback suppressed" — both match.** Slice 3b4's LDM/STM core already handles this correctly.)

### 5.9 THUMB.16/THUMB.17 overlap

Condition `0xF` in THUMB.16 is reserved for SWI. Encoding `11011111 imm8` is THUMB.17 SWI, which shares top bits `1101` with THUMB.16 but has cond field `1111`. Condition `0xE` is undefined and routes to a warn stub.

### 5.10 THUMB.18/THUMB.19 branch offset encoding

- THUMB.18 B: 11-bit signed offset, pre-shifted left by 1, added to `instr_addr + 4`. Range: PC ± 2 KiB.
- THUMB.19 BL first halfword: 11-bit signed offset, pre-shifted left by 12. Stored in LR.
- THUMB.19 BL second halfword: 11-bit unsigned offset, pre-shifted left by 1, added to LR. Range: PC ± 4 MiB when both halves used.
- Branch targets must be halfword-aligned (bit 0 = 0). The BL second halfword sets LR bit 0 to 1 so `BX LR` returns in Thumb state.

### 5.11 THUMB.5 BX details

- `PC = Rs & ~1`
- `CPSR.T = Rs & 1`
- If `Rs & 1 == 0`: switch to ARM state. Bit 1 of Rs should also be 0 for word alignment on the target.
- `BX PC` from a word-aligned Thumb address is the standard Thumb→ARM switch idiom; the target is `instr_addr + 4` (next halfword effectively skipped because Thumb PC is read as `+4`).

### 5.12 SWI / BKPT encoding disambiguation

- `11011111 imm8` → SWI
- `10111110 imm8` → BKPT (ARMv5; UNDEF on ARMv4 → warn stub)

Slice 3c decodes SWI to a warn stub that leaves CPSR, PC, and registers unchanged and advances the cycle counter. Slice 3d replaces the stub.

---

## 6. Testing strategy

Three new test binaries, following the slice 3b4 pattern of "one test binary per family":

### 6.1 `arm7_thumb_dp_test.cpp`

Covers THUMB.1 (shift imm), THUMB.2 (add/sub reg/imm3), THUMB.3 (MOV/CMP/ADD/SUB imm8), THUMB.4 (all 16 ALU ops), THUMB.5 hi-reg ADD/MOV/CMP, THUMB.12 (ADD PC/SP), THUMB.13 (ADD SP signed).

Per-format cases include:
- Zero-operand and full-range operand values.
- Shift amount zero (LSL #0, LSR/ASR #0 meaning #32) for THUMB.1.
- Register-shift amount zero for THUMB.4 shifts (C must be unchanged).
- **THUMB.3 MOV flag rule: N, Z updated; C, V unchanged.**
- **THUMB.4 MUL on ARMv4: C flag must differ from pre-MUL value.**
- **THUMB.5 hi-reg ADD/MOV: CPSR unchanged.** CMP: N, Z, C, V updated.
- THUMB.5 NOP encoding (`MOV R8, R8`): CPSR unchanged, R8 unchanged.
- THUMB.5 hi-reg Rs=15: read value is `(instr_addr + 4) & ~2`.
- THUMB.5 hi-reg Rd=15 for ADD/MOV: PC updated via `write_rd`.
- THUMB.12 ADD PC with `instr_addr % 4 == 2`: literal-pool align-down active.
- THUMB.13 ADD SP with both S=0 and S=1.

### 6.2 `arm7_thumb_ls_test.cpp`

Covers THUMB.6 (LDR PC-rel), THUMB.7 (LDR/STR/LDRB/STRB reg offset), THUMB.8 (STRH/LDSB/LDRH/LDSH reg offset), THUMB.9 (LDR/STR/LDRB/STRB imm offset), THUMB.10 (LDRH/STRH imm offset), THUMB.11 (LDR/STR SP-rel).

Per-format cases:
- Each opcode with an aligned base address.
- LDR with unaligned base address: rotate-on-misalign active.
- LDRH/LDSH with `addr & 1` set: slice 3b3 alignment rule.
- STRH with `addr & 1` set: low bit masked before write.
- LDSB sign extension across the boundary.
- THUMB.6 literal pool at `instr_addr % 4 == 2`: PC word-align-down active.
- THUMB.11 SP-rel with misaligned SP: rotate-on-misalign for LDR.

### 6.3 `arm7_thumb_block_branch_test.cpp`

Covers THUMB.14 (PUSH/POP ± LR/PC), THUMB.15 (LDMIA/STMIA + edge cases), THUMB.16 (Bcond, including cond 0xE UNDEF), THUMB.17 (SWI warn stub), THUMB.18 (B), THUMB.19 (BL both halves and standalone second halfword), THUMB.5 BX.

Per-format cases:
- PUSH with LR: LR pushed at highest address.
- POP with PC: PC loaded at highest address, stays in Thumb state (ARMv4).
- Full Rlist (R0-R7 + LR/PC): all registers transferred.
- THUMB.15 empty Rlist: R15 transferred, Rb += 0x40.
- THUMB.15 STM with Rb in list lowest: OLD base stored.
- THUMB.15 STM with Rb in list not lowest: NEW base stored.
- THUMB.15 LDM with Rb in list: no writeback.
- THUMB.16 Bcond all 14 condition codes taken and not-taken.
- THUMB.16 cond 0xE UNDEF: warn stub, PC advances by 2, CPSR unchanged.
- THUMB.17 SWI: warn stub, PC advances by 2, CPSR unchanged.
- THUMB.18 B forward and backward, max positive and max negative offsets.
- THUMB.19 BL both halves: full target computation and return address in LR with thumb bit set.
- **THUMB.19 BL second halfword alone (0xF800 style "BL LR+0")**: verifies that a pre-staged LR is consumed correctly without a preceding first halfword.
- THUMB.5 BX to ARM: `Rs & 1 == 0`, CPSR.T clears, PC = `Rs & ~1`.
- THUMB.5 BX to Thumb: `Rs & 1 == 1`, CPSR.T stays set, PC = `Rs & ~1`.
- THUMB.5 BX PC: ARM↔Thumb switch from a word-aligned Thumb address.

### 6.4 Capstone sequence test: `arm7_thumb_sequence_test.cpp`

Loads a hand-written Thumb routine into main RAM (for example, a short factorial loop or a memcpy). Runs it under `step_thumb` until a terminator, verifies the final register state. The sequence must exercise:

- DP shifts and adds (THUMB.1, .2, .3, .4)
- PUSH/POP around a subroutine call (THUMB.14 + THUMB.19 BL + THUMB.5 BX LR)
- A conditional branch loop (THUMB.16 + THUMB.18)
- At least one LDR PC-rel literal pool load (THUMB.6)

### 6.5 Total test binary count

Slice 3b4 ended at 27 CTest binaries. Slice 3c adds 4 (`arm7_thumb_dp_test`, `arm7_thumb_ls_test`, `arm7_thumb_block_branch_test`, `arm7_thumb_sequence_test`), ending at **31**.

---

## 7. Cross-references

- **Authoritative Thumb reference:** `docs/specs/.notes/thumb-gbatek-reference.md`
- **Slice 3b1 (data processing):** `docs/specs/2026-04-14-arm7-core-phase1-slice3b1-design.md`
- **Slice 3b2 (branch + single data transfer):** `docs/specs/2026-04-14-arm7-core-phase1-slice3b2-design.md`
- **Slice 3b3 (halfword):** `docs/specs/2026-04-14-arm7-core-phase1-slice3b3-design.md`
- **Slice 3b4 (block transfer + swap):** `docs/specs/2026-04-15-arm7-core-phase1-slice3b4-design.md`
- **ARM7 state header:** `src/cpu/arm7/arm7_state.hpp`
- **ARM7 decoder internal:** `src/cpu/arm7/arm7_decode_internal.hpp`
- **GBATEK Thumb top-level (raw source):** `mgba-emu/gbatek` `index.md`, section `THUMB Binary Opcode Format`

---

## 8. Risk and rollback

### Risk 1 — PC-offset inconsistency across formats

The `+4` base versus `(+4) & ~2` literal-pool rule is a recurring source of off-by-2 bugs in hand-rolled Thumb decoders. Unit tests on aligned instructions pass; the bug only triggers at `instr_addr % 4 == 2`.

**Mitigation:** the Phase B scaffolding commit (commit 7 in the sequence) establishes the two explicit PC values (`pc_read`, `pc_literal`) at the top of `dispatch_thumb` and includes a dedicated unit test covering `instr_addr % 4 == 2`. Every literal-pool format handler takes `pc_literal` as an explicit parameter — no format reads `state.r[15]` directly for a literal-pool computation. Every per-format test in §6 includes at least one case with non-4-aligned `instr_addr`.

**Rollback:** revert the affected single commit.

### Risk 2 — THUMB.4 MUL ARMv4 flag behavior

The implementation must destroy the C flag on MUL (ARMv4 behavior), not preserve it. "Preserve" looks correct in most tests because games rarely read C immediately after MUL. The bug only surfaces against melonDS instruction traces or handwritten ARM7 code from specific middleware that does `MUL ; BCC`.

**Mitigation:** the `arm7_thumb_dp_test` explicitly asserts that C *differs* from its pre-MUL value — requiring the implementation to destroy it deterministically. Commit message and helper comment document the ARMv4 choice so the ARM9 slice knows to flip it back to ARMv5 "preserved."

**Rollback:** single-commit revert.

### Risk 3 — Block-transfer refactor regression

The slice-3b4 LDM/STM code encodes seven non-obvious ARMv4 rules (empty list, STM Rn-in-list OLD vs NEW base, LDM Rn-in-list no-writeback, S=1 user-bank transfer, S=1 LDM-with-R15 exception return, STM R15 stores `instr_addr + 12`, deterministic warn on malformed encodings). An "obviously correct" extraction that silently misses one rule regresses the slice-3b4 capstone sequence test.

**Mitigation:** commit 5 (the block extraction) is strictly "move code into a function taking explicit parameters" — no logic changes, no reordering, no optimization, no comments edited. The commit-level gate is: the full slice 3b4 unit test set (`arm7_block_test`, `arm7_swap_test`, `arm7_block_swap_sequence_test`) must pass unchanged. Commit 5 is reviewed in parallel with `ds-architecture-rule-checker` and `gbatek-reviewer` before it lands, same discipline as slice 3b4 itself.

**Rollback:** if a later Thumb PUSH/POP or LDMIA/STMIA commit finds a latent bug in the extraction, revert those commits while leaving the extraction in place, then fix the extraction forward. If the extraction is itself broken, revert the extraction — every downstream commit is a pure addition and rebases trivially.

### Risk 4 — THUMB.19 BL visible-LR model

If the first halfword is accidentally modeled as stashing transient state on `Arm7` instead of writing LR, games that emit the second halfword alone (Mario Golf Advance Tour GBA, and similar patterns on DS carts) crash.

**Mitigation:** the capstone sequence test includes a standalone second halfword case. A failure there is immediate.

### Risk 5 — SWI warn stub leaking into slice 3d

If slice 3c accidentally implements partial SWI dispatch (e.g. "just switch CPSR mode but don't jump"), slice 3d inherits a working-but-wrong state machine that passes unit tests and silently breaks exception flow.

**Mitigation:** THUMB.17 is a strict warn stub in commit 18. Its unit test asserts that after executing a SWI, CPSR, R14_svc, SPSR_svc, and PC are all *unchanged* from their pre-SWI values except for PC advancing by 2.

---

## 9. Slice completion criteria

Slice 3c is complete when:

1. All ~31 test binaries build and pass under `ctest --output-on-failure`.
2. Phase A refactors (commits 1–5) are behavior-preserving: every existing ARM-state test (slice 3a, 3b1, 3b2, 3b3, 3b4) passes unchanged.
3. `dispatch_thumb` correctly routes every ARMv4T Thumb encoding: all 19 formats plus the THUMB.17 SWI stub, the THUMB.16 cond 0xE undefined stub, and the THUMB.19 BL `11101` second-halfword ARMv5-only path.
4. Every format from §5 is implemented with its hardware rules, and the per-format tests in §6 all pass.
5. THUMB.5 hi-reg ADD/MOV: CPSR is observably unchanged.
6. THUMB.5 BX: ARM↔Thumb switching works both directions and from a word-aligned source.
7. THUMB.14 PUSH/POP: correctly routes through `execute_block_transfer` and POP {PC} stays in Thumb state on ARMv4.
8. THUMB.15: empty list, Rb-in-list (STM OLD vs NEW base), and Rb-in-list (LDM no writeback) edge cases all handled.
9. THUMB.19 BL: both halves work, and the standalone second halfword works as "BL LR+imm".
10. THUMB.17 SWI: decodes to a warn stub with CPSR/R14_svc/SPSR_svc/PC unchanged modulo PC += 2.
11. `src/cpu/arm7/arm7_thumb_decode.cpp` ≤ 500 lines, `arm7_thumb_dp.cpp` ≤ 500, `arm7_thumb_ls.cpp` ≤ 500, `arm7_thumb_branch.cpp` ≤ 300, `arm7_thumb_block.cpp` ≤ 300 (soft targets; hard limit is 800).
12. `dispatch_thumb` handles PC offset materialization (`pc_read`, `pc_literal`) and format handlers take whichever value they need as a parameter. No format reads `state.r[15]` for literal-pool alignment.
13. No new compiler warnings. No new tracked files outside this spec, six new `src/cpu/arm7/` files, and four new test files.
14. The full mandatory pipeline has run for each implementation commit: `/gbatek-check` (done, upfront) → `senior-architect` (done, upfront) → baseline → implement → `/simplify` → (`ds-architecture-rule-checker` ∥ `gbatek-reviewer`) → `quality-reviewer` → `ctest` → commit.

---

## Appendix A. Commit sequence

Eighteen commits. Each builds and passes `ctest` independently. Order matters: executor extractions land first in strict behavior-preserving form, then scaffolding, then Thumb formats in ascending complexity, capstone last.

### Phase A — executor extractions (commits 1–5)

1. `cpu/arm7: extract execute_dp_op from dispatch_dp`
2. `cpu/arm7: extract execute_bx from dispatch_000_space`
3. `cpu/arm7: extract execute_single_data_transfer_core from dispatch_single_data_transfer`
4. `cpu/arm7: extract execute_halfword_transfer_core from dispatch_halfword`
5. `cpu/arm7: extract execute_block_transfer from dispatch_block`

### Phase B — scaffolding (commits 6–7)

6. `cpu/arm7: step_thumb scaffolding + dispatch_thumb 8-way warn stub`
7. `cpu/arm7: dispatch_thumb PC read conventions (+4 and literal-aligned)`

### Phase C — data-processing families, ascending complexity (commits 8–12)

8. `cpu/arm7: thumb.3 MOV/CMP/ADD/SUB imm8 with MOV NZ-only flag rule`
9. `cpu/arm7: thumb.1 LSL/LSR/ASR imm5 with ARMv4T zero-amount quirks`
10. `cpu/arm7: thumb.2 ADD/SUB reg and imm3`
11. `cpu/arm7: thumb.4 ALU reg/reg (16 ops incl. NEG, ARMv4 MUL C destroy)`
12. `cpu/arm7: thumb.5 hi-reg ADD/MOV/CMP + BX (no BLX, no CPSR on ADD/MOV)`

### Phase D — load/store families (commits 13–16)

13. `cpu/arm7: thumb.6 LDR PC-rel and thumb.11 LDR/STR SP-rel word`
14. `cpu/arm7: thumb.7 and thumb.9 LDR/STR word/byte reg and imm offset`
15. `cpu/arm7: thumb.8 and thumb.10 halfword + signed reg and imm offset`
16. `cpu/arm7: thumb.14 PUSH/POP with optional LR/PC via execute_block_transfer`

### Phase E — block, branches, capstone (commits 17–18)

17. `cpu/arm7: thumb.15 LDMIA/STMIA with Rb writeback rules + thumb.12/.13 ADD PC/SP and ADD SP`
18. `cpu/arm7: slice 3c capstone — thumb.16/.17/.18/.19 + end-to-end sequence test`

Commit 17 folds THUMB.12 and THUMB.13 as sibling additions (they are two-line helpers and don't warrant standalone commits). Commit 18 folds THUMB.16 Bcond, THUMB.17 SWI warn stub, THUMB.18 B, and THUMB.19 BL (both halves + standalone second-halfword test) together with the capstone sequence test. This is within the 12–18 commit window used for slice 3b4.

---

## Appendix B. Encoding cheat sheet

```
Is this a Thumb instruction?
  Check CPSR.T — if set, the 16-bit opcode at instr_addr is a Thumb instr.

Top-level fan-out (bits[15:13]):
  000 → shift imm or add/sub (THUMB.1, THUMB.2)
  001 → MOV/CMP/ADD/SUB imm8 (THUMB.3)
  010 → ALU reg, hi-reg/BX, LDR PC-rel, LDR/STR reg offset (THUMB.4-8)
  011 → LDR/STR imm offset word/byte (THUMB.9)
  100 → LDRH/STRH imm offset, LDR/STR SP-rel (THUMB.10, .11)
  101 → ADD PC/SP, ADD SP, PUSH/POP (THUMB.12, .13, .14)
  110 → LDMIA/STMIA, Bcond, SWI (THUMB.15, .16, .17)
  111 → B, BL long (THUMB.18, .19)

THUMB.1 shift imm:
  0 0 0 op2 off5 Rs3 Rd3
  op: 00=LSL 01=LSR 10=ASR 11=(reserved, goes to THUMB.2)

THUMB.2 add/sub:
  0 0 0 1 1 I op Rn3/imm3 Rs3 Rd3
  op: 0=ADDreg 1=SUBreg 2=ADDimm3 3=SUBimm3

THUMB.3 MOV/CMP/ADD/SUB imm8:
  0 0 1 op2 Rd3 imm8
  op: 00=MOV 01=CMP 10=ADD 11=SUB
  MOV updates N,Z only. Others full NZCV.

THUMB.4 ALU reg/reg:
  0 1 0 0 0 0 op4 Rs3 Rd3
  op: 0=AND 1=EOR 2=LSL 3=LSR 4=ASR 5=ADC 6=SBC 7=ROR
      8=TST 9=NEG A=CMP B=CMN C=ORR D=MUL E=BIC F=MVN
  MUL on ARMv4: N,Z set, C destroyed.

THUMB.5 hi-reg / BX:
  0 1 0 0 0 1 op2 Hd Hs Rs3 Rd3
  Rs_full = (Hs<<3)|Rs ; Rd_full = (Hd<<3)|Rd
  op: 0=ADD 1=CMP 2=MOV 3=BX (ARMv5: BLX when Hd=1, UNDEF on ARMv4)
  ADD/MOV do NOT touch CPSR. Only CMP does.
  For ADD/CMP/MOV, Hd or Hs must be set.
  For BX, Hd=0 and Rd unused.

THUMB.6 LDR PC-rel:
  0 1 0 0 1 Rd3 imm8
  addr = ((instr_addr + 4) & ~2) + (imm8 << 2)
  Rd = read32(addr)

THUMB.7 LDR/STR reg offset:
  0 1 0 1 op2 0 Ro3 Rb3 Rd3
  op: 0=STR 1=STRB 2=LDR 3=LDRB
  addr = Rb + Ro

THUMB.8 halfword/signed reg offset:
  0 1 0 1 op2 1 Ro3 Rb3 Rd3
  op: 0=STRH 1=LDSB 2=LDRH 3=LDSH
  addr = Rb + Ro

THUMB.9 LDR/STR imm offset:
  0 1 1 op2 imm5 Rb3 Rd3
  op: 0=STR 1=LDR 2=STRB 3=LDRB
  word forms: addr = Rb + (imm5 << 2)
  byte forms: addr = Rb + imm5

THUMB.10 LDRH/STRH imm offset:
  1 0 0 0 op1 imm5 Rb3 Rd3
  op: 0=STRH 1=LDRH
  addr = Rb + (imm5 << 1)

THUMB.11 LDR/STR SP-rel:
  1 0 0 1 op1 Rd3 imm8
  op: 0=STR 1=LDR
  addr = SP + (imm8 << 2)

THUMB.12 ADD PC/SP imm:
  1 0 1 0 op1 Rd3 imm8
  op 0: Rd = ((instr_addr + 4) & ~2) + (imm8 << 2)
  op 1: Rd = SP + (imm8 << 2)

THUMB.13 ADD SP imm signed:
  1 0 1 1 0 0 0 0 S imm7
  S=0: SP = SP + (imm7 << 2)
  S=1: SP = SP - (imm7 << 2)

THUMB.14 PUSH/POP:
  1 0 1 1 op1 1 0 R Rlist8
  op: 0=PUSH 1=POP
  R: PUSH R=1 includes LR; POP R=1 includes PC
  PUSH = STMDB SP! (p=1 u=0 w=1 l=0)
  POP  = LDMIA SP! (p=0 u=1 w=1 l=1)

THUMB.15 LDMIA/STMIA:
  1 1 0 0 op1 Rb3 Rlist8
  op: 0=STMIA 1=LDMIA
  Base always writes back. (p=0 u=1 w=1 l=op)

THUMB.16 Bcond:
  1 1 0 1 cond4 off8
  off8 signed × 2, added to instr_addr + 4
  cond 0xE: UNDEF warn stub
  cond 0xF: → THUMB.17 SWI

THUMB.17 SWI:
  1 1 0 1 1 1 1 1 imm8   (SWI — slice 3c warn stub)
  1 0 1 1 1 1 1 0 imm8   (BKPT — ARMv5 only, UNDEF on ARMv4)

THUMB.18 B unconditional:
  1 1 1 0 0 off11
  off11 signed × 2, added to instr_addr + 4

THUMB.19 BL first halfword:
  1 1 1 1 0 imm11_hi
  LR = (instr_addr + 4) + (sext(imm11_hi) << 12)

THUMB.19 BL second halfword:
  1 1 1 1 1 imm11_lo   (BL — ARMv4)
  1 1 1 0 1 imm11_lo   (BLX — ARMv5, UNDEF on ARMv4)
  target = LR + (imm11_lo << 1)
  LR = (instr_addr + 2) | 1
  PC = target
```
