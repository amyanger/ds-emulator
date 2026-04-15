# ARM7 Core — Phase 1, Slice 3b4 Design

**Status:** proposed
**Date:** 2026-04-15
**Predecessor:** `2026-04-14-arm7-core-phase1-slice3b3-design.md` (landed, 24/24 tests green, pushed to origin/main as of `8839a5d`)
**Parent spec:** `2026-04-12-nds-emulator-design.md` §4 (CPU cores) and §13 (phase roadmap)

---

## 1. Summary

Slice 3b4 adds the ARMv4T **block data transfer** and **single data swap** instructions to the ARM7 interpreter:

- **`LDM`** — load multiple registers from contiguous memory
- **`STM`** — store multiple registers to contiguous memory
- **`SWP`** — atomic word swap (read word, write word, destination = read value)
- **`SWPB`** — atomic byte swap

The slice also performs one structural refactor, prerequisite to the feature work: it extracts the growing recognizer pile-up at the top of `dispatch_dp()` into a dedicated `dispatch_000_space()` fan-out function. Slice 3b3 §4.7 flagged this as the refactor threshold once a fifth recognizer is needed; SWP is that fifth recognizer.

After this slice, every ARMv4T ARM-state instruction except exception entry / IRQ line sampling is implemented on the ARM7. The remaining ARMv4T work is Thumb state (slice 3c) and exceptions (slice 3d). This is the largest slice in the ARM7 arc by commit count — LDM/STM alone has eight distinct edge cases, each requiring its own commit and test.

### Plain-language summary of what the slice adds

The CPU already knows how to load and store one register at a time. What it can't yet do is move a whole block of registers in a single instruction — which is how every function prologue ("save these eight registers to the stack") and epilogue ("restore them and return") works in ARM code. LDM and STM are those block moves. They're used constantly: every function call, every interrupt handler, every `memcpy`-like operation, every context switch. Without them the CPU can't execute more than a few dozen instructions of real compiled code before hitting an instruction it doesn't understand.

SWP is a different animal — it's a single instruction that atomically reads a value from memory and writes a new one in its place. It's the building block for simple locks and semaphores. Real games don't use it much (ARM7 programs on the DS usually rely on IRQ-disable instead of proper locking), but libc runtimes and some BIOS calls emit it, so the CPU still has to handle it correctly.

The refactor this slice bundles in is mechanical: the ARM7 decoder has a small tower of special-case recognizers stacked at the top of the data-processing dispatcher (for BX, multiply, PSR transfer, halfword transfer, and now SWP). Five recognizers is one too many for that file; they all live in the same encoding neighborhood, so we lift them into their own dispatch function named after the neighborhood. No behavior changes — every existing test still passes unchanged — but future CPU work has a cleaner surface to hang new instructions off of.

## 2. Goals

- Execute `LDM` and `STM` in all four addressing modes (IA / IB / DA / DB), with and without writeback, for any register list of 1–16 registers.
- Implement the ARMv4 `Rn in list` rules for both LDM and STM correctly.
- Implement the S-bit semantics for all three valid cases (STM user-bank, LDM user-bank, LDM exception-return).
- Implement R15 in the register list for both STM (store `instruction_addr + 12`) and LDM (branch, leave CPSR.T unchanged on ARMv4 — no interworking).
- Execute `SWP` (word) with LDR-style rotated read and force-aligned unrotated write.
- Execute `SWPB` (byte) with straightforward byte read/write, zero-extending Rd.
- Handle SWP's `Rd == Rm`, `Rm == Rn`, and `Rd == Rn` cases correctly by latching Rm before the read.
- Extract `dispatch_000_space()` as a structural refactor in the slice's first commit, leaving `dispatch_dp()` responsible for DP operand logic only.
- Preserve all slice 3b1/3b2/3b3 behavior. All 24 existing tests continue to pass unchanged after the refactor and after every feature commit.
- Keep the Q1 placeholder "every instruction costs 1 ARM7 cycle" with `TODO(cycles)` markers at every new dispatch head. Cycle accuracy is a cross-cutting slice, not an opcode-family slice.

## 3. Non-goals

Explicitly out of scope for 3b4; deferred to the slice named in parentheses.

- **`LDRD` / `STRD`** — ARMv5TE only. Not valid on ARM7. Already handled as a warn path in slice 3b3.
- **Exception entry / IRQ line sampling** (slice 3d) — the S=1 LDM form restores CPSR from SPSR, but we don't *enter* exception modes in this slice. That machinery is slice 3d.
- **Correct cycle timing** — GBATEK documents `nS+1N+1I` for LDM and `(n−1)S+2N` for STM, plus extra cycles for R15-in-list and for SWP (`1S+2N+1I`). We return 1 cycle for all four until the dedicated cycle-accuracy slice.
- **Thumb versions** (`PUSH`, `POP`, etc.) — slice 3c. Note that Thumb PUSH/POP are implemented as helper calls into a shared LDM/STM core; the core itself lands here.
- **ARMv4 empty-register-list hardware quirk** (R15-only transfer + `Rn ± 0x40` writeback) — melonDS does not model this; neither will we. Empty list is treated as a no-op with a deterministic warn. See §5.8 for rationale.
- **ARMv5 LDM-with-PC interworking** (bit 0 of loaded PC sets T flag) — ARM9 only; slice 4.
- **ARMv5 LDM Rn-in-list rule differences** — ARM9 only; slice 4.
- **Bus LOCK signal modeling for SWP** — the DS has no bus contenders on the ARM7 side mid-instruction; correctness does not require locking.
- **T-variants** (`LDMT`, etc.) — no such thing on ARMv4T.
- **`Arm7State` register banking changes** — already in place from slice 3a. Banking is consumed, not modified.

## 4. Architecture

Two new translation units, one new dispatch fan-out function (in its own file), one promoted helper. The refactor commit comes first; every feature commit lands against the stable new dispatch surface.

### 4.1 Dispatcher structure

Slice 3b3 established this layout under `src/cpu/arm7/`:

```
arm7_decode.cpp            top-level step_arm() + primary dispatch_arm()
arm7_dp.cpp                data processing (imm-shift + reg-shift)
                           + recognizer pile-up: BX, PSR, multiply, halfword
arm7_branch.cpp            B / BL
arm7_loadstore.cpp         LDR / STR / LDRB / STRB (word/byte only)
arm7_multiply.cpp          MUL / MLA / UMULL / UMLAL / SMULL / SMLAL
arm7_psr.cpp               MRS / MSR
arm7_halfword.cpp          LDRH / STRH / LDRSB / LDRSH
arm7_decode_internal.hpp   shared helpers & dispatch function declarations
```

Slice 3b4 restructures the `bits[27:25] == 000` primary slot and adds two new feature files:

```
arm7_decode_000.cpp        NEW: dispatch_000_space() — fan-out for the 000 slot
arm7_block.cpp             NEW: LDM / STM
arm7_swap.cpp              NEW: SWP / SWPB
arm7_dp.cpp                SLIMMED: DP operand logic only (recognizers removed)
arm7_decode.cpp            MODIFIED: case 0b000 now calls dispatch_000_space
                           (case 0b001 still routes to dispatch_dp for imm-form DP)
arm7_decode_internal.hpp   + dispatch_000_space declaration
                           + dispatch_block declaration
                           + dispatch_swap declaration
                           + promoted rotate_read_word() inline helper
```

**Why `dispatch_000_space` and not inline pile-up:** slice 3b3 §4.7 noted that five recognizers at the top of `dispatch_dp` crosses the refactor threshold. Five special cases stacked before the "actual" DP logic obscures both the dispatch flow and the DP operand decoder. Lifting the recognizers into a named fan-out function makes the intent explicit ("this is what lives in the `000` encoding neighborhood") and gives `dispatch_dp` a single responsibility again.

**Why not `arm7_decode_000` as part of `arm7_decode.cpp`:** keeping the top-level `dispatch_arm` switch readable is the whole point of the refactor. Burying a 60-line recognizer cascade inside `arm7_decode.cpp` defeats the goal. A small standalone file scoped to "the `000` fan-out" is the clearest split.

**Why two feature files (`arm7_block.cpp` + `arm7_swap.cpp`), not one combined file:** LDM/STM and SWP share nothing beyond an encoding neighborhood. LDM/STM is a multi-register transfer with writeback, user-bank banking, and an exception-return form — a sizable amount of logic. SWP is an atomic single-word/byte read-modify-write with none of that. Merging them would violate the "one hardware component per file" rule, produce a file whose name names two nouns, and make `arm7_block_test.cpp` dangerous to grow. The split mirrors the precedent set by `arm7_halfword.cpp` (single file because LDRH/STRH/LDRSB/LDRSH share the full decode path).

### 4.2 New header declarations and bus threading

`arm7_decode_internal.hpp` gains three function declarations and one promoted inline helper:

```cpp
// Fan-out for the bits[27:25] == 000 encoding space. Checks BX, SWP,
// multiply, halfword, PSR-transfer in that order; falls through to
// dispatch_dp for any operand-form DP instruction that doesn't match.
u32 dispatch_000_space(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Block data transfer (LDM / STM). Called from dispatch_arm for the
// bits[27:25] == 100 primary slot. Handles all four addressing modes,
// S-bit, R15-in-list, and the ARMv4 Rn-in-list quirks.
u32 dispatch_block(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Single data swap (SWP / SWPB). Called from dispatch_000_space after
// the SWP recognizer matches. Performs an atomic read-then-write with
// Rm latched before the read.
u32 dispatch_swap(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Shared LDR-style rotate used by LDR (word), LDRH misaligned fallback,
// and SWP (word). Promoted out of arm7_loadstore.cpp where it was
// previously inlined into dispatch_single_data_transfer.
inline u32 rotate_read_word(u32 raw, u32 addr) {
    const u32 rot = (addr & 0x3u) * 8u;
    return (rot == 0) ? raw : ((raw >> rot) | (raw << (32u - rot)));
}
```

Bus threading follows the same pattern as every other family handler: `Arm7Bus&` is taken by reference and used through its `read32`/`read8`/`write32`/`write8` methods. No handler caches the bus pointer, no handler touches the scheduler directly, no handler takes a pointer to anything else.

### 4.3 Addressing mode normalization

LDM and STM share one core transfer engine. All four addressing modes normalize to a **single forward-walking loop** over the register list, with a precomputed start address and writeback value:

```
given: P (pre-index), U (up), Rn, reg_list, n = popcount(reg_list)

# Compute forward-walk start address:
if U == 1:  # increasing
    start_addr = Rn + (4 if P else 0)     # IB adds 4 up front, IA does not
    wb_value   = Rn + 4 * n
else:       # decreasing — normalize to forward walk
    start_addr = Rn - 4 * n + (0 if P else 4)   # DB starts lower, DA starts higher
    wb_value   = Rn - 4 * n

# Walk the list low → high with increasing addresses:
addr = start_addr
for i in 0..15:
    if reg_list & (1 << i):
        (transfer_one)
        addr += 4
```

This is the melonDS pattern (`ARMInterpreter_LoadStore.cpp` — `A_LDM` at ~line 401, `A_STM` at ~line 477). The advantage is that memory-mapped I/O side effects fire in the correct order regardless of P/U — GBATEK specifies that even in decreasing modes, the CPU first computes the lowest address and then walks forward from there.

The normalization must happen **before** the transfer loop starts, because several of the edge cases below (specifically STM-with-Rn-in-list and the user-bank mode swap) depend on `wb_value` already being computed.

### 4.4 Per-opcode semantics

#### LDM

```
1. Decode P, U, S, W, L=1, Rn, reg_list.
2. Compute start_addr and wb_value per §4.3.
3. If S == 1:
     a. If bit[15] of reg_list == 0 (R15 not in list):
          - This is the user-bank-access form.
          - Capture original_mode = state.current_mode().
          - state.switch_mode(Mode::User).
          - (writeback with S=1 is UNPREDICTABLE — log a warn if W=1,
            but do NOT skip the transfer.)
     b. If bit[15] == 1 (R15 in list):
          - This is the exception-return form.
          - No mode swap yet; the SPSR restore happens after the R15 load.
4. Walk reg_list low → high from start_addr:
     for each set bit i:
         loaded = bus.read32(addr & ~0x3u)
         if i == 15:
             # Defer the R15 write until after SPSR restore (step 6).
             loaded_pc = loaded
         else:
             state.r[i] = loaded
         addr += 4
5. If S == 1 and R15 not in list:
     - state.switch_mode(original_mode).
6. If R15 was in list:
     - If S == 1 (exception-return form):
         u32* spsr = state.spsr_slot()
         if spsr == nullptr:
             warn "LDM S with R15 from mode without SPSR"
             # Fall through with CPSR unchanged.
         else:
             state.cpsr = *spsr
             # Banking changes if CPSR mode bits differ; rebank now.
             state.switch_mode(state.current_mode())
     - Write R15:
         loaded_pc &= ~0x3u          # word-align, ARMv4: no Thumb interwork
         state.r[15] = loaded_pc
         state.pc    = loaded_pc
     - NOTE: CPSR.T is NEVER modified by LDM on ARMv4, not even from
       bit 0 of loaded_pc. That's an ARMv5 behavior and is out of scope.
7. Writeback:
     - If W == 0: skip.
     - If W == 1 and Rn is in reg_list (ARMv4 rule): skip (loaded value wins).
     - Else: state.r[Rn] = wb_value.
```

The order of operations is load-the-list, swap-back-mode, restore-CPSR, write-R15, writeback. Each step only runs if its precondition holds.

#### STM

```
1. Decode P, U, S, W, L=0, Rn, reg_list.
2. Compute start_addr and wb_value per §4.3.
3. If S == 1:
     - This is always the user-bank-access form for STM (R15 in list has
       no special meaning for stores — it's just another register to store).
     - Capture original_mode = state.current_mode().
     - state.switch_mode(Mode::User).
     - (writeback with S=1 is UNPREDICTABLE — warn if W=1, do not skip.)
4. Determine STM Rn-in-list behavior (ARMv4 rule):
     - Let is_rn_lowest = (reg_list & ((1 << Rn) - 1)) == 0 && (reg_list & (1 << Rn))
       i.e. Rn is set in the list AND no lower register is set.
     - Let rn_in_list = (reg_list & (1 << Rn)) != 0.
     - If rn_in_list AND NOT is_rn_lowest:
           # Store the NEW base. Apply writeback BEFORE the transfer loop
           # so the i == Rn iteration stores the post-writeback value.
           state.r[Rn] = wb_value
           writeback_done = true
     - Else:
           writeback_done = false
5. Walk reg_list low → high from start_addr:
     for each set bit i:
         if i == 15:
             stored = instr_addr + 12u     # ARMv4 STM R15 stores PC+12
         else:
             stored = state.r[i]
         bus.write32(addr & ~0x3u, stored)
         addr += 4
6. If S == 1:
     - state.switch_mode(original_mode).
7. Writeback (if not already done in step 4):
     - If W == 0: skip.
     - Else: state.r[Rn] = wb_value.
```

The "write the new base before the loop" trick (step 4) is what makes the "Rn not lowest → store new base" rule fall out naturally: the loop hits `i == Rn` with `state.r[Rn]` already holding `wb_value`, so the normal `stored = state.r[i]` path reads the correct value. When Rn *is* the lowest in the list, writeback_done stays false and the loop stores the original `state.r[Rn]` on the first iteration; writeback runs at the end.

#### SWP

```
1. Decode Rn, Rd, Rm from the instruction.
2. Warn if Rn == 15 or Rd == 15 or Rm == 15 (all UNPREDICTABLE).
3. Latch rm_value = state.r[Rm]  BEFORE the read.   # critical
4. Read the word:
     raw = bus.read32(Rn & ~0x3u)
     loaded = rotate_read_word(raw, state.r[Rn])     # LDR-style rotate
5. Write Rm unrotated, force-aligned:
     bus.write32(state.r[Rn] & ~0x3u, rm_value)
6. Destination:
     state.r[Rd] = loaded                            # Rd != 15 after warn
```

The Rm latch in step 3 is **load-bearing correctness** — it handles `Rd == Rm` (the pre-write Rm is stored, which is the same as the pre-load Rm) and `Rm == Rn` (the original base, not whatever the loaded value was). Do not inline `state.r[Rm]` into step 5.

#### SWPB

```
1. Decode Rn, Rd, Rm. Warn on R15 operands.
2. Latch rm_byte = state.r[Rm] & 0xFFu.
3. loaded = bus.read8(state.r[Rn])                   # zero-extended to u32
4. bus.write8(state.r[Rn], rm_byte)
5. state.r[Rd] = loaded
```

No alignment handling, no rotation. `bus.read8` and `bus.write8` operate on the raw address.

### 4.5 Bus access and atomicity

All transfers go through `Arm7Bus&`. LDM uses `read32` for each slot, STM uses `write32`. SWP uses `read32` + `write32`; SWPB uses `read8` + `write8`. No family handler has ever cached anything from the bus, and 3b4 does not change that.

Atomicity: on real hardware, SWP asserts a LOCK signal across its two accesses to block bus arbitration. On the DS, the ARM7 bus has no competing masters during a CPU instruction (DMA and cart DMA are scheduled as separate events, not mid-instruction interleavings), so correctness does not require modeling LOCK. The implementation simply performs `read32` then `write32` back-to-back with no scheduler tick between them. This is the same treatment melonDS uses.

LDM and STM cycle through the register list inside a single dispatch call; the scheduler is not advanced between individual transfers. This matches GBATEK's description (the internal sequencer runs to completion before the next fetch) and is consistent with how `dispatch_single_data_transfer` handles multi-cycle LDR/STR already.

### 4.6 What does NOT change

- `Arm7State` — no new fields, no new methods, no banking changes. The existing `switch_mode` and `spsr_slot` API is sufficient for the S-bit cases.
- `Arm7Bus` — no new register ranges, no new access paths. LDM/STM/SWP use the same `read32`/`write32`/`read8`/`write8` entry points that LDR/STR use.
- `dispatch_arm` — the `bits[27:25] == 100` case is already a warn-stub; it now calls `dispatch_block` instead. No other cases change.
- `dispatch_dp` — after the refactor, it handles DP operand logic only (imm-shift, reg-shift, condition flags). The recognizer pile-up moves out.
- Slice 3b1/3b2/3b3 behavior — all 24 existing tests pass unchanged at every commit.
- The test harness — same REQUIRE pattern, same `add_ds_unit_test()` invocation.

### 4.7 Known technical debt

- **Empty register list quirk not modeled.** ARMv4 hardware treats `reg_list == 0` as "transfer R15 only, writeback Rn ± 0x40". Real code never emits this encoding; melonDS does not model it; this slice does not either. A future GBATEK-accuracy pass may want to add it. Tracked in this spec, not in a separate TODO file.
- **Cycle counts still placeholder.** LDM's `nS + 1N + 1I`, STM's `(n−1)S + 2N`, and SWP's `1S + 2N + 1I` are documented in §5.11 but not implemented. Deferred to the dedicated cycle-accuracy slice.
- **Bus LOCK signal not modeled for SWP.** Conceptually atomic; no DS bus contender to be blocked. Acceptable for now.
- **S=1 with W=1 is UNPREDICTABLE and we still do the writeback.** melonDS does too. We warn but do not alter behavior. Fine unless a real test ROM depends on a specific UNPREDICTABLE interpretation, which is unlikely.

---

## 5. Hardware details

This section collects the GBATEK and ARM7TDMI TRM facts that drive the implementation. Every claim here should be directly traceable to one of the sources in §7.

### 5.1 LDM / STM encoding

Bit layout of the 32-bit instruction:

```
 31    28 27 25 24 23 22 21 20 19    16 15          0
+--------+-----+--+--+--+--+--+--------+--------------+
|  cond  | 100 | P| U| S| W| L|   Rn   |  reg_list    |
+--------+-----+--+--+--+--+--+--------+--------------+
```

| Bit | Name | Meaning |
|-----|------|---------|
| 31–28 | cond | Standard ARM condition field |
| 27–25 | 100 | Primary decode: block data transfer |
| 24 | P | Pre/Post index (0 = post, 1 = pre) |
| 23 | U | Up/Down (0 = decrement, 1 = increment) |
| 22 | S | PSR & force-user-bank |
| 21 | W | Writeback |
| 20 | L | Load/Store (0 = STM, 1 = LDM) |
| 19–16 | Rn | Base register (R15 is UNPREDICTABLE) |
| 15–0 | reg_list | One bit per register, bit i = Ri |

### 5.2 SWP / SWPB encoding

```
 31    28 27    23 22 21 20 19    16 15    12 11       4 3     0
+--------+-------+--+-----+--------+--------+----------+-------+
|  cond  | 00010 | B|  00 |   Rn   |   Rd   | 00001001 |  Rm   |
+--------+-------+--+-----+--------+--------+----------+-------+
```

| Bit | Name | Meaning |
|-----|------|---------|
| 31–28 | cond | Standard ARM condition field |
| 27–23 | 00010 | Primary: `000` + `bits[24:23] = 10` |
| 22 | B | 0 = SWP (word), 1 = SWPB (byte) |
| 21–20 | 00 | Reserved, must be zero |
| 19–16 | Rn | Base address register |
| 15–12 | Rd | Destination (gets the pre-write memory value) |
| 11–4 | 00001001 | Distinguishes SWP from DP / multiply / halfword |
| 3–0 | Rm | Source (written to memory) |

### 5.3 Recognizer patterns

The `dispatch_000_space()` fan-out tests in this exact order, falling through to `dispatch_dp()` if none match:

| Order | Opcode family | Mask | Match |
|-------|---------------|------|-------|
| 1 | BX | `0x0FFFFFF0` | `0x012FFF10` |
| 2 | SWP/SWPB | `0x0FB00FF0` | `0x01000090` |
| 3 | Multiply (short) | `0x0F0000F0` | `0x00000090` |
| 3 | Multiply (long) | `0x0F8000F0` | `0x00800090` |
| 4 | Halfword / signed | `0x0E000090` + `(instr >> 5) & 3 != 0` | `0x00000090` masked |
| 5 | MRS | `0x0FBF0FFF` | `0x010F0000` |
| 5 | MSR reg form | `0x0FB0FFF0` | `0x0120F000` |
| 5 | MSR imm form | `0x0FB0F000` | `0x0320F000` (only reachable via `bits[27:25]==001` fallthrough, but check here for symmetry) |
| — | DP fallthrough | (no match) | `return dispatch_dp(...)` |

**Why SWP must precede multiply:** SWP has `bits[7:4] = 1001` and `bit[24] = 1`. Multiply has `bits[7:4] = 1001` and `bit[24] = 0`. The masks don't overlap (MUL's mask forces bit 24 to zero via `0x0F0000F0`), so the order is technically safe either way, but testing SWP first makes the intent unambiguous: "any `1001` extension with bit 24 set is SWP; anything else is a multiply or halfword."

**Why SWP must precede halfword:** SWP has `bits[6:5] = 00` (part of the `00001001` distinguisher). Halfword transfers require `(instr >> 5) & 3 != 0`. Testing SWP first avoids a narrow window where a mis-masked halfword recognizer could swallow a SWP by accident.

### 5.4 Addressing mode table

`n = popcount(reg_list)`, all arithmetic is 32-bit wrapping.

| amod | P | U | First transfer addr | Last transfer addr | Writeback value |
|------|---|---|---------------------|--------------------|-----------------|
| IA   | 0 | 1 | `Rn`                | `Rn + 4*(n-1)`     | `Rn + 4*n`      |
| IB   | 1 | 1 | `Rn + 4`            | `Rn + 4*n`         | `Rn + 4*n`      |
| DA   | 0 | 0 | `Rn - 4*(n-1)`      | `Rn`               | `Rn - 4*n`      |
| DB   | 1 | 0 | `Rn - 4*n`          | `Rn - 4`           | `Rn - 4*n`      |

Stack-style mnemonics for cross-reference (the real semantics are P/U):

```
STMFD = STMDB = PUSH    STMED = STMDA    STMFA = STMIB    STMEA = STMIA
LDMFD = LDMIA = POP     LDMED = LDMIB    LDMFA = LDMDA    LDMEA = LDMDB
```

Transfer order is **always lowest-numbered register to lowest address**, regardless of addressing mode. For decreasing modes, hardware computes the lowest address first and walks forward from there — the register-to-address mapping never reverses. This is the reason §4.3 normalizes every mode to a single forward-walking loop.

### 5.5 Writeback timing — ARMv4 Rn-in-list rules

GBATEK's "Strange Effects on Invalid Rlist's" section documents four mutually-exclusive ARMv4 cases. For slice 3b4:

- **STM, Rn in list, Rn is the lowest-numbered register in the list** → store the **ORIGINAL** `Rn` value, then apply writeback.
- **STM, Rn in list, Rn is NOT the lowest** → apply writeback FIRST, then store the **NEW** (post-writeback) `Rn` value during the loop.
- **STM, Rn not in list** → walk the list, apply writeback at the end.
- **LDM, Rn in list, W=1** → writeback is **suppressed** (the loaded value wins). ARMv4 specifically. ARMv5 has a different rule that we are not implementing.
- **LDM, Rn not in list** → walk the list, apply writeback at the end.

For decreasing modes (DA/DB), "new base" means the final decremented value computed up-front by the §4.3 normalization.

### 5.6 S-bit semantics — three cases

Bit 22 has three distinct meanings depending on L and the presence of R15 in the list:

1. **STM, S=1.** The register list refers to user-bank R0–R15 regardless of current mode. Combining S=1 with W=1 is UNPREDICTABLE — we warn but still perform writeback. The FIQ-mode case is the common one in real code (an IRQ handler saving user-bank registers for a user-space context switch).

2. **LDM, S=1, R15 not in list.** Same as STM S=1: loads into user-bank R0–R15. Writeback with S=1 is UNPREDICTABLE. GBATEK warns that a pipeline hazard exists if the following instruction reads a banked register; we do not model the hazard.

3. **LDM, S=1, R15 in list.** Exception-return form. Performs a normal current-mode load AND restores `CPSR = SPSR_<current_mode>` after writing R15. If the current mode has no SPSR (User / System), we warn and leave CPSR unchanged — that case is UNPREDICTABLE on real hardware.

The implementation uses `Arm7State::switch_mode(Mode::User)` to enter the user bank temporarily, walks the list, and swaps back. This pays two full bank-copy cycles per S=1 transfer, which is acceptable given how rare the S-bit forms are in real code.

### 5.7 R15 in the register list

**STM with R15 in the list.** ARMv4 stores `instruction_addr + 12` (i.e. the PC value the pipeline would expose as `R15`, which is `instr_addr + 8`, plus one more word for the STM-specific offset). Since `state.r[15]` is pre-advanced to `instr_addr + 8` at the top of `step_arm`, the implementation must *not* read `state.r[15]` directly for the stored value — it must compute `instr_addr + 12`.

**LDM with R15 in the list.** On ARMv4:
- The loaded word is word-aligned (`pc & ~0x3`) before being written to R15.
- CPSR.T is **never modified** — LDM does not participate in ARM/Thumb interworking on ARMv4. (The ARMv5 rule, which sets T from bit 0 of the loaded value, is out of scope.)
- If S=1 is also set, CPSR is additionally restored from SPSR in the same instruction (see §5.6 case 3).
- Both `state.r[15]` and `state.pc` must be updated so the next fetch lands at the new address.

### 5.8 Empty register list quirk

GBATEK documents an ARMv4 quirk: an empty `reg_list` decodes as "transfer R15 only" with `Rn ± 0x40` writeback, because the hardware uses a 16-entry counter that decodes zero-length as "16 words" plus an early-exit. Melon DS does not model this, and real compilers never emit empty-list encodings.

**Slice 3b4 decision:** match melonDS. An empty `reg_list` logs a deterministic warn and no-ops the transfer loop; the `wb_value` computed with `popcount == 0` equals `Rn` itself, so writeback is effectively a no-op as well. The decision is documented here and in the commit message so a future GBATEK-accuracy pass flags it correctly. If a real test ROM ever hits the path, we implement the quirk then.

### 5.9 SWP alignment

Word SWP uses the same two-step rule as LDR:

- **Read** rotates: `loaded = ROR(mem32(Rn & ~0x3), (Rn & 0x3) * 8)`.
- **Write** masks: `mem32(Rn & ~0x3) = rm_value`.

This is the same `rotate_read_word()` helper that LDR uses. Promoting it to an inline free function in `arm7_decode_internal.hpp` is the first task of the slice (before any feature code needs it).

Byte SWPB does no alignment handling — `bus.read8` and `bus.write8` operate on the raw address.

### 5.10 UNPREDICTABLE cases

Warn and continue with deterministic behavior; do not crash, do not assert.

- **LDM/STM with `Rn == R15`** — warn, proceed using `state.r[15]` as the base (which reads `instr_addr + 8`). Real games never emit this.
- **LDM/STM with S=1 AND W=1** — warn, still perform writeback (matches melonDS).
- **LDM with S=1 and R15 in list from User/System mode** — warn (no SPSR), do not touch CPSR, still perform the R15 load and branch.
- **SWP with `Rn == R15`, `Rd == R15`, or `Rm == R15`** — warn, proceed with the pipeline-offset value (`instr_addr + 8`). Real code never emits this.
- **Empty `reg_list`** — warn, no-op transfer, writeback = Rn (no change).

### 5.11 Cycle counts (reference only)

GBATEK Instruction Summary (we still return 1 cycle per instruction in this slice):

| Op   | Cycles                           |
|------|----------------------------------|
| LDM  | `nS + 1N + 1I` (+ `1S + 1N` if R15 in list) |
| STM  | `(n−1)S + 2N`                    |
| SWP  | `1S + 2N + 1I`                   |
| SWPB | `1S + 2N + 1I`                   |

Recorded for the future cycle-accuracy slice. Implementation uses placeholder `1` with a `TODO(cycles)` marker at each dispatch head.

---

## 6. Testing strategy

Three new test binaries, matching the pattern established by prior slices.

### 6.1 `arm7_block_test.cpp` — LDM/STM per-case unit tests

Covers every semantic branch in §4.4. Organized in sections, each with a helper-function-per-case pattern (mirroring 3b2/3b3 tests):

1. **STM / LDM — IA mode, base case**
   - STM of three GPRs, no writeback, Rn not in list
   - LDM of three GPRs, no writeback, Rn not in list
   - Verify memory contents and register file

2. **Writeback (W=1), Rn not in list**
   - STM IA W=1, verify Rn += 4*n
   - LDM IA W=1, verify Rn += 4*n
   - Matrix across IA/IB/DA/DB

3. **Addressing mode coverage**
   - Same test body, parametrized over {IA, IB, DA, DB} × {W=0, W=1}
   - Verify first/last transfer addresses from §5.4

4. **STM Rn-in-list, Rn is lowest** — store ORIGINAL base value
   - `STMIA R0!, {R0, R1, R2}` — memory[R0] should hold the original R0
   - Verify writeback updates R0 afterwards

5. **STM Rn-in-list, Rn is NOT lowest** — store NEW base value
   - `STMIA R0!, {R1, R0, R2}` — memory[R0+4] should hold the NEW R0
   - Verify writeback happens before the transfer loop sees R0

6. **LDM Rn-in-list writeback suppression**
   - `LDMIA R0!, {R0, R1, R2}` — R0 receives the loaded value, W is ignored
   - Verify no writeback overwrites the loaded R0

7. **R15 in STM list** — stores `instr_addr + 12`
   - Verify the exact word written to memory

8. **R15 in LDM list, S=0**
   - Verify PC is word-aligned on write
   - Verify CPSR.T is unchanged regardless of bit 0 of loaded value
   - Verify `state.pc` follows `state.r[15]`

9. **S=1, R15 not in list — user-bank transfer**
   - From FIQ mode, STM `{R8, R13, R14}` with S=1
   - Verify user-bank R8/R13/R14 are read, not FIQ-bank
   - Symmetric LDM case

10. **S=1, R15 in list — exception-return form**
    - From IRQ mode, LDM `{PC}` with S=1
    - Verify CPSR is restored from SPSR_irq
    - Verify banking rebank occurs if CPSR mode bits differ
    - Edge case: from User/System with S=1 — warn path, CPSR unchanged

11. **Empty register list**
    - `LDMIA R0!, {}` — verify warn fires, no transfers, writeback is a no-op
    - Symmetric STM case

12. **UNPREDICTABLE warn paths**
    - `Rn == R15`
    - `S=1 AND W=1`
    - Verify warns fire deterministically and no crash

13. **Addressing-mode normalization smoke test**
    - DB mode transferring four regs from base 0x2000_1000 — verify the first access lands at `0x2000_0FF0`, not `0x2000_1000`

### 6.2 `arm7_swap_test.cpp` — SWP/SWPB per-case unit tests

1. **SWP word — aligned base**
   - `SWP R0, R1, [R2]` — verify Rd = pre-write memory, memory = Rm

2. **SWP word — unaligned base (rotated read)**
   - `SWP R0, R1, [R2]` with `R2 = 0x2000_1001` — verify Rd rotates by 8
   - Verify memory is written at `0x2000_1000` (force-aligned)

3. **SWPB — byte swap**
   - Verify Rd zero-extends the byte
   - Verify only one byte of memory changes

4. **`Rd == Rm`** — verify Rm is latched before the read (the stored value is the original Rm, not the loaded value)

5. **`Rm == Rn`** — verify the original base address is used for the write

6. **`Rd == Rn`** — verify Rd gets the pre-write memory value, memory gets Rm

7. **UNPREDICTABLE warn paths** — R15 as Rn, Rd, or Rm

### 6.3 `arm7_block_swap_sequence_test.cpp` — capstone

End-to-end program that exercises the whole slice against the same CPU state:

- Set up a stack at 0x0380_0000 (ARM7 WRAM region)
- `STMFD SP!, {R0-R3, LR}` — save registers (PUSH)
- Execute a few instructions from prior slices (DP, multiply, halfword load)
- `LDMFD SP!, {R0-R3, PC}` — restore and return (POP)
- Verify PC lands at the saved LR
- Interleave a SWP to verify it doesn't disturb surrounding state
- Interleave a halfword LDR from slice 3b3 to verify dispatcher regressions
- Verify final register file matches expectations

This is the "does every instruction family still compose cleanly" smoke test. Parallel to `arm7_halfword_sequence_test.cpp` from 3b3.

### 6.4 Total test binary count

Before: 24.
After: 27 (24 + `arm7_block_test` + `arm7_swap_test` + `arm7_block_swap_sequence_test`).

All tests link against `ds_core` only, use REQUIRE from `tests/support/require.hpp` (Release-safe), run in milliseconds, no SDL.

---

## 7. Cross-references

- **GBATEK §"ARM Opcodes: Memory: Block Data Transfer (LDM, STM)"** — primary encoding reference. Includes the "Strange Effects on Invalid Rlist's" subsection that documents the Rn-in-list and empty-list ARMv4 rules.
- **GBATEK §"ARM Opcodes: Memory: Single Data Swap (SWP, SWPB)"** — SWP encoding and semantics.
- **GBATEK §"Mis-aligned LDR, SWP (rotated read)"** — SWP word alignment rule.
- **ARM Architecture Reference Manual (ARMv4T), §A4.1.20–A4.1.23 (LDM/STM), §A4.1.108 (SWP)** — authoritative for any ambiguity GBATEK leaves open.
- **melonDS `src/ARMInterpreter_LoadStore.cpp`** — reference implementation for both LDM/STM (`A_LDM` ~line 401, `A_STM` ~line 477) and SWP (`A_SWP` ~line 361, `A_SWPB` ~line 377). Consult during commits 8–13 for the edge-case patterns.
- **Predecessor slice:** `docs/specs/2026-04-14-arm7-core-phase1-slice3b3-design.md` — pattern and style for recognizer-at-top-of-dispatch_dp fan-out; flagged the refactor threshold this slice acts on.
- **Parent spec:** `docs/specs/2026-04-12-nds-emulator-design.md` §4.2 (ARM7 interpreter) and §13.3 (Phase 1 milestone list).

---

## 8. Risk and rollback

Low-to-medium risk. The slice has two distinct change surfaces:

**Refactor commit (commit 1).** Extracts `dispatch_000_space()` from `dispatch_dp`. Risk: a mis-ordered recognizer could swallow an instruction that currently lands correctly. Mitigation: the refactor commit changes zero semantics; the new function preserves the exact current ordering (BX → PSR → multiply → halfword → DP), with SWP inserted in commit 2. All 24 existing tests must pass before the refactor is considered done.

**Feature commits.** Each commit is additive and isolated to `arm7_block.cpp` or `arm7_swap.cpp`. Reverting any single feature commit removes its test cases cleanly. The only cross-file touch after commit 1 is `arm7_decode.cpp` (line 35, `case 0b100`) flipping from the warn stub to `dispatch_block`, and `arm7_decode_000.cpp` growing a `dispatch_swap` recognizer call.

**Rollback plan.** If any commit causes a regression:
1. Identify the first bad commit via `git bisect` against `ctest`.
2. Revert that commit and any dependent commits.
3. The worst-case rollback is back to commit 1 (the refactor) — the refactor itself is a behavior-preserving move and is safe to keep even if the feature commits are all reverted.

No header-include graph changes outside `src/cpu/arm7/`. No subsystem-boundary touches. No public API changes on `Arm7State` or `Arm7Bus`.

---

## 9. Slice completion criteria

Slice 3b4 is complete when:

1. All 27 test binaries build and pass under `ctest --output-on-failure`.
2. `dispatch_000_space()` correctly routes every `bits[27:25] == 000` instruction to the right handler with no false positives across BX, SWP, multiply, halfword, PSR transfer, and DP operand forms.
3. `dispatch_arm` routes `bits[27:25] == 100` to `dispatch_block` (replacing the current warn stub).
4. LDM and STM are implemented per §4.4 for all four addressing modes, with and without writeback, across all register-list sizes 1–16.
5. The STM Rn-in-list rule (§5.5) is implemented and tested for both "Rn is lowest" and "Rn is not lowest" cases.
6. The LDM Rn-in-list writeback-suppression rule (§5.5) is implemented and tested.
7. The three S-bit cases (§5.6) are implemented and tested, including the LDM-with-PC exception-return form.
8. R15 in STM (stores `instr_addr + 12`) and R15 in LDM (word-aligned jump, CPSR.T unchanged) are implemented and tested.
9. SWP (rotated read, force-aligned write) and SWPB (byte read/write, zero-extend) are implemented and tested, including the `Rm latched before read` correctness check.
10. The empty-register-list path logs a deterministic warn and does not crash (§5.8).
11. The UNPREDICTABLE warn paths (§5.10) log deterministically and do not crash.
12. `src/cpu/arm7/arm7_block.cpp` ≤ 500 lines and `src/cpu/arm7/arm7_swap.cpp` ≤ 150 lines (soft targets; hard limit is the 800-line house rule).
13. `dispatch_dp()` after the refactor contains only DP operand logic — no recognizer calls.
14. No new compiler warnings, no new tracked files outside this spec + three new `.cpp` files + three new test files + `arm7_decode_000.cpp` + the header updates.
15. The full pipeline has run for each implementation commit: `/gbatek-check` (once, upfront — done) → `senior-architect` (once, upfront — done) → baseline → implement → `/simplify` → (`ds-architecture-rule-checker` ∥ `gbatek-reviewer`) → `quality-reviewer` → `ctest` → commit.

---

## Appendix A. Commit sequence

Sixteen commits. Each builds and passes `ctest` independently. Order matters — the refactor lands first, the rotate-helper promotion lands second so SWP has it, and the feature commits fall out in increasing edge-case order.

1. `cpu/arm7: extract dispatch_000_space fan-out from dispatch_dp`
2. `cpu/arm7: promote rotate_read_word helper to arm7_decode_internal.hpp`
3. `cpu/arm7: arm7_block.cpp scaffold — recognizer + unrecognized-encoding warn`
4. `cpu/arm7: LDM/STM start-address + writeback helpers with unit tests`
5. `cpu/arm7: STM IA — simplest forward walk, no writeback, no Rn-in-list`
6. `cpu/arm7: LDM IA — forward walk with GPR-only register list`
7. `cpu/arm7: LDM/STM writeback (W=1) for IA mode, Rn not in list`
8. `cpu/arm7: LDM/STM addressing modes IB/DA/DB via normalized forward walk`
9. `cpu/arm7: STM with Rn in list — old-base-if-lowest, new-base otherwise`
10. `cpu/arm7: LDM with Rn in list suppresses writeback (ARMv4 rule)`
11. `cpu/arm7: LDM R15 in list — word-align jump, leave CPSR.T unchanged`
12. `cpu/arm7: STM R15 in list stores instruction_addr + 12`
13. `cpu/arm7: LDM/STM S=1 without R15 — user-bank transfer via switch_mode`
14. `cpu/arm7: LDM S=1 with R15 — exception return, CPSR = SPSR_mode`
15. `cpu/arm7: LDM/STM empty register list — deterministic warn no-op`
16. `cpu/arm7: arm7_swap.cpp — SWP/SWPB with Rm latched before read`
17. `cpu/arm7: slice 3b4 capstone — block transfer + swap end-to-end sequence test`

(Seventeen if you count the capstone; within the 12–18 window targeted by the senior-architect review.)

---

## Appendix B. Encoding cheat sheet (for the implementer)

```
Is this a block data transfer?
  (instr & 0x0E000000) == 0x08000000      // bits[27:25] == 100
  Routed from dispatch_arm case 0b100.

Is this a swap?
  (instr & 0x0FB00FF0) == 0x01000090      // SWP or SWPB
  Routed from dispatch_000_space after BX, before multiply.

Decode LDM/STM:
  P = (instr >> 24) & 1
  U = (instr >> 23) & 1
  S = (instr >> 22) & 1
  W = (instr >> 21) & 1
  L = (instr >> 20) & 1       // 0 = STM, 1 = LDM
  Rn       = (instr >> 16) & 0xF
  reg_list =  instr        & 0xFFFF
  n = popcount(reg_list)

Normalize addressing:
  if U == 1:
      start_addr = Rn + (P ? 4 : 0)
      wb_value   = Rn + 4 * n
  else:
      start_addr = Rn - 4 * n + (P ? 0 : 4)
      wb_value   = Rn - 4 * n

STM Rn-in-list (ARMv4):
  is_rn_in_list = reg_list & (1 << Rn)
  is_rn_lowest  = is_rn_in_list && ((reg_list & ((1 << Rn) - 1)) == 0)
  if is_rn_in_list && !is_rn_lowest:
      state.r[Rn] = wb_value              # apply BEFORE the loop
      writeback_already_done = true

LDM Rn-in-list (ARMv4):
  if is_rn_in_list:
      skip writeback at the end            # loaded value wins

S=1 user-bank entry/exit:
  original_mode = state.current_mode()
  state.switch_mode(Mode::User)
  ... transfer loop ...
  state.switch_mode(original_mode)

LDM S=1 with R15 in list:
  spsr = state.spsr_slot()
  if spsr != nullptr:
      state.cpsr = *spsr
      state.switch_mode(state.current_mode())  # rebank if mode changed
  state.r[15] = loaded_pc & ~0x3u
  state.pc    = state.r[15]

STM R15 in list:
  stored = instr_addr + 12u          # NOT state.r[15]

Decode SWP/SWPB:
  B  = (instr >> 22) & 1             // 0 = word, 1 = byte
  Rn = (instr >> 16) & 0xF
  Rd = (instr >> 12) & 0xF
  Rm =  instr        & 0xF

SWP (word):
  rm_value = state.r[Rm]             # LATCH BEFORE READ
  raw      = bus.read32(state.r[Rn] & ~0x3u)
  loaded   = rotate_read_word(raw, state.r[Rn])
  bus.write32(state.r[Rn] & ~0x3u, rm_value)
  state.r[Rd] = loaded

SWPB (byte):
  rm_byte = state.r[Rm] & 0xFFu
  loaded  = bus.read8(state.r[Rn])
  bus.write8(state.r[Rn], rm_byte)
  state.r[Rd] = loaded
```
