# ARM7 Core — Phase 1, Slice 3b2 Design

**Status:** proposed
**Date:** 2026-04-14
**Predecessor:** `2026-04-14-arm7-core-phase1-slice3b1-design.md` (plan landed, 18/18 tests green)
**Parent spec:** `2026-04-12-nds-emulator-design.md` §4 (CPU cores) and §13 (phase roadmap)

---

## 1. Summary

Slice 3b2 finishes the ARMv4T `bits[27:26] == 00` instruction encoding subspace by adding the three families the ARM7 interpreter is still missing from that corner:

- **Register-shifted-register data processing** — the `DP Rd, Rn, Rm, shift Rs` form where the shift amount comes from a register instead of an immediate.
- **The multiply family, all six variants** — `MUL`, `MLA`, `UMULL`, `UMLAL`, `SMULL`, `SMLAL`.
- **PSR transfer** — `MRS` (read CPSR/SPSR into a register) and `MSR` (write from register or rotated immediate to CPSR/SPSR, with byte-field mask).

After this slice, the only ARMv4T ARM-state instructions still unimplemented on ARM7 are halfword / signed single-data transfer (`LDRH`/`STRH`/`LDRSB`/`LDRSH` — slice 3b3), load/store multiple (`LDM`/`STM` — slice 3b4), and exception entry / IRQ line sampling (slice 3d). Thumb state is slice 3c.

No new architectural machinery is introduced. `Arm7State` stays flat (no register banking — that belongs to slice 3d, where exception entry forces the issue). The scheduler, buses, and `NDS` are untouched. The slice is purely an ALU + dispatch change confined to `src/cpu/arm7/` and `tests/unit/`.

### Plain-language summary of what the slice adds

Three things. First, the CPU learns to shift a value by an amount stored in a register, instead of only by a hardcoded number baked into the instruction — compilers emit this constantly for anything involving arrays of structs or variable bit-packing. Second, the CPU learns to multiply — hardware multiply is how games compute damage formulas, sprite coordinates, and pretty much any scaled quantity, so without it Pokemon grinds to a halt on the first frame of real code. Third, the CPU learns to read and write its own status register — the dashboard that holds the condition flags, the interrupt-disable switch, and the mode selector. Games use this to turn interrupts off around critical sections so the audio handler can't fire halfway through a sprite update and corrupt memory. Every one of these three is on the critical path for running real ARM code, which is why they're grouped into one slice.

## 2. Goals

- Execute register-shifted-register DP for all 16 opcodes, with the ARMv4T "shift amount from `Rs` low byte" rule and the "`Rn`/`Rm` read as `PC + 12`" pipeline quirk both correct.
- Execute `MUL`, `MLA`, `UMULL`, `UMLAL`, `SMULL`, `SMLAL` with correct signed/unsigned handling and correct 64-bit accumulate order.
- Execute `MRS` and `MSR` (both register and immediate forms) for both CPSR and SPSR, with the `_fsxc` byte field mask correctly honored.
- Preserve all slice 3a and 3b1 behavior. All 18 existing tests continue to pass unchanged.
- Keep the decoder split established by 3b1: add `arm7_multiply.cpp` and `arm7_psr.cpp` as new translation units; extend `arm7_dp.cpp` to recognize its two new sibling families and hand them off.
- Continue using the Q1 placeholder "every instruction costs 1 ARM7 cycle" with `TODO(cycles)` markers at every new dispatcher head. Cycle accuracy is a cross-cutting slice, not an opcode-family slice.

## 3. Non-goals

Explicitly out of scope for 3b2; deferred to the slice named in parentheses.

- **`LDRH`, `STRH`, `LDRSB`, `LDRSH`** (slice 3b3) — halfword / signed single data transfer lives in its own encoding subspace and is large enough to earn its own slice.
- **`LDM`, `STM`** (slice 3b4) — block transfers with register-list + writeback + base-in-list edge cases are the biggest remaining ARMv4T ARM-state feature. Gets its own slice.
- **Exception entry and IRQ line sampling** (slice 3d) — MSR provides a *software-initiated* mode switch, but the hardware-initiated ones (reset, undefined instruction, SWI, prefetch/data abort, IRQ, FIQ) are a separate code path that also saves the return address into the target mode's LR and copies CPSR into the target mode's SPSR. Slice 3b2 does not model any of that. **Register banking itself is already in place** from slice 3a (`Arm7State::switch_mode()` saves/restores R8–R14 and handles per-mode SPSR via `Arm7Banks`), so MSR in slice 3b2 can honor mode-bit writes fully.
- **Correct per-instruction cycle counts** — every 3b2 instruction returns 1 cycle. `MUL`/`MLA`'s Rs-dependent early-termination table, reg-shift DP's `1S + 1I` cost, and everything else: deferred to a dedicated cycle-accuracy slice.
- **Thumb versions** of any 3b2 instruction — Thumb is all of slice 3c.
- **Rd==15 in `MUL`/`MLA` triggering a branch** — UNPREDICTABLE on real ARMv4T. We warn and write R15 without pipeline flush.
- **Full `SPSR-in-USR-mode` semantics** — UNPREDICTABLE on real hardware. `MRS` returns 0 with a warn; `MSR` is a no-op with a warn.

## 4. Architecture

Three new instruction families, three scoped changes.

### 4.1 Dispatcher structure

Slice 3b1 established per-family translation units under `src/cpu/arm7/`:

```
arm7_decode.cpp        top-level step_arm() + primary dispatch_arm()
arm7_dp.cpp            data processing (imm-shift form + BX)
arm7_branch.cpp        B / BL
arm7_loadstore.cpp     LDR / STR / LDRB / STRB
arm7_decode_internal.hpp   shared helpers & dispatch function declarations
```

Slice 3b2 adds **two new TUs** and extends one existing TU:

```
arm7_dp.cpp            + register-shift DP path merged into existing handler
arm7_multiply.cpp      NEW: MUL / MLA / UMULL / UMLAL / SMULL / SMLAL
arm7_psr.cpp           NEW: MRS / MSR
```

The multiply and PSR-transfer recognizers live at the *top* of `dispatch_dp` in `arm7_dp.cpp`, not at the top of `dispatch_arm` in `arm7_decode.cpp`. Reason: they share the same `bits[27:26] == 00` primary key with DP, and lifting that fan-out into `arm7_decode.cpp` means duplicating the gate across two files. Keeping it inside `arm7_dp.cpp` matches how the ARM reference manual organizes the tables — one encoding subspace, one dispatch function that peels off its siblings before touching them as DP.

### 4.2 New header declarations

`arm7_decode_internal.hpp` grows by four items:

```cpp
// New dispatch entry points.
u32 dispatch_multiply(Arm7State& state, u32 instr, u32 instr_addr);
u32 dispatch_psr_transfer(Arm7State& state, u32 instr, u32 instr_addr);

// PC+12 helper used by register-shift DP for Rn and Rm reads.
// In reg-shift form the ARM reads Rm / Rn one pipeline stage later than
// in imm-shift form, so Rm == 15 returns PC + 12 rather than PC + 8.
inline u32 read_rm_pc12(const Arm7State& s, u32 rm) {
    return (rm == 15) ? (s.r[15] + 4) : s.r[rm];
}

// Rs read for register-shift DP. Rs == 15 is UNPREDICTABLE on real hardware;
// we log a warn and return PC + 12 for determinism. Used once, inlined for
// symmetry with read_rm_pc12.
inline u32 read_rs_for_reg_shift(const Arm7State& s, u32 rs) {
    if (rs == 15) {
        DS_LOG_WARN("arm7: reg-shift DP with Rs == 15 (unpredictable) at PC 0x%08X", s.pc);
    }
    return (rs == 15) ? (s.r[15] + 4) : s.r[rs];
}
```

### 4.3 ALU helper — already exists

`arm7_alu.hpp` already has a `barrel_shift_reg(operand, type, amount)` helper from slice 3a infrastructure work. Important: **it deliberately takes no `c_in` parameter**. When `amount == 0`, the returned `carry` field is documented as meaningless, and the caller is required to preserve its own CPSR.C value. This is intentional — it forces the reg-shift DP path to think about the "amount==0 preserves carry" rule at the call site instead of silently delegating it to the helper. The existing helper already handles amounts ∈ [1, 31] (delegates to `barrel_shift_imm`), amount == 32 (per-shift-type zero or sign-extend), and amount > 32 (folds, or zero for LSL/LSR). Slice 3b2 does not modify this function.

### 4.4 State helper — one addition, rest already exists

`arm7_state.hpp` already provides everything for mode handling:

```cpp
enum class Mode : u8 {
    User       = 0x10, Fiq = 0x11, Irq = 0x12, Supervisor = 0x13,
    Abort      = 0x17, Undefined = 0x1B, System = 0x1F,
};

// Already implemented on Arm7State (from slice 3a infrastructure):
Mode current_mode() const;
void switch_mode(Mode to_mode);   // saves current bank, loads target bank, updates CPSR mode bits
```

`Arm7Banks` already has `spsr_fiq`, `spsr_irq`, `spsr_svc`, `spsr_abt`, `spsr_und` as distinct `u32` fields. User and System modes share the user register bank and have no SPSR.

Slice 3b2 adds **one** new method to `Arm7State` for convenient SPSR access from MRS/MSR:

```cpp
// Returns pointer to the SPSR storage slot for the current mode.
// Returns nullptr when current mode is User or System (no SPSR exists).
u32* spsr_slot();
const u32* spsr_slot() const;
```

Implementation is a simple switch on `current_mode()`. This centralizes the USR/SYS UNPREDICTABLE check in one place — every MRS/MSR SPSR access calls `spsr_slot()` and falls into the warn-path whenever it returns `nullptr`. Future slice 3d exception entry code will also use it.

### 4.5 What does NOT change

No changes to: `Arm7Bus`, `Scheduler`, `NDS`, `main.cpp`, WRAM / page tables, any public header under `include/`. No new build-system entries beyond adding two source files to `ds_core` and four test binaries to `tests/`.

## 5. Family specifications

### 5.1 Register-shifted-register data processing

**What it does (plain language).** The CPU already knows how to do `MOV R0, R1, LSL #3` — shift R1 left by three bits and put the result in R0, where the "3" is baked into the instruction. Register-shift DP adds `MOV R0, R1, LSL R2` — shift R1 left by *whatever R2 contains right now* and put the result in R0. Compilers emit it constantly for variable bit-packing and array-of-struct indexing. Every other DP opcode (ADD, SUB, AND, OR, etc.) has the same variant.

**Encoding.** Bit 25 (`I`) == 0, bit 4 == 1, bit 7 == 0.

```
31      28 27 26 25 24   21 20 19   16 15   12 11    8  7  6  5  4  3    0
| cond   | 0  0 | 0|opcode |S |  Rn   |  Rd   |  Rs   | 0 |sh| 1 |  Rm   |
```

**Operand2 computation.**

```cpp
const u32 rm        = instr & 0xFu;
const u32 rs        = (instr >> 8) & 0xFu;
const u32 shift_typ = (instr >> 5) & 0x3u;
const u32 amount    = read_rs_for_reg_shift(state, rs) & 0xFFu;  // low byte of Rs only
const u32 rm_value  = read_rm_pc12(state, rm);                   // PC+12 quirk
const bool c_in     = (state.cpsr & (1u << 29)) != 0;

// barrel_shift_reg takes NO c_in parameter by design — its returned carry
// is meaningless when amount == 0. Caller is responsible for preserving
// C in that case.
ShifterResult op2 = barrel_shift_reg(rm_value, (ShiftType)shift_typ, amount);
if (amount == 0) {
    op2.carry = c_in;  // preserve CPSR.C per ARMv4T spec
}
```

**Rule table (what `barrel_shift_reg` already implements, restated for reference).**

| Shift type | amount == 0 | amount ∈ [1, 31] | amount == 32 | amount ∈ [33, 255] |
|---|---|---|---|---|
| LSL | `rm_value`, **carry must be restored by caller** | `rm << amount`, `C = bit[32-amount]` of rm | `0`, `C = bit[0] of rm` | `0`, `C = 0` |
| LSR | `rm_value`, **carry must be restored by caller** | `rm >> amount`, `C = bit[amount-1]` of rm | `0`, `C = bit[31] of rm` | `0`, `C = 0` |
| ASR | `rm_value`, **carry must be restored by caller** | signed `rm >> amount`, `C = bit[amount-1]` | `(rm>>31)?-1:0`, `C = bit[31]` | same as amount==32 |
| ROR | `rm_value`, **carry must be restored by caller** | `rotate_right(rm, amount)`, `C = bit[amount-1]` | `C = bit[31]`, result = rm unchanged | amount mod 32, recurse |

Critical rule: when `amount == 0`, **no shift is performed AND the carry flag is preserved.** This differs from imm-shift form where `LSR #0` / `ASR #0` / `ROR #0` get reinterpreted as special cases (LSR/ASR #32, RRX). Reg-shift form never has those reinterpretations because amount==0 is literal. The existing `barrel_shift_reg` signature enforces this at the type level — it has no `c_in` parameter, so callers cannot forget.

**Pipeline quirk (the single most important correctness issue in §5.1).** In register-shift form, the ARM reads `Rn`, `Rm`, and `Rs` one pipeline stage later than imm-shift form does, because the shift stage consumes a cycle. The visible effect:

- `Rn == 15` → read `state.r[15] + 4` (= `PC + 12` relative to the instruction), not `state.r[15]` (= `PC + 8`)
- `Rm == 15` → same: `state.r[15] + 4`
- `Rs == 15` → UNPREDICTABLE on real hardware. We log a warn and read `state.r[15] + 4` for determinism
- `Rd == 15` → no pipeline difference. Writes R15 and branches, same as imm-shift form

`read_rm_pc12` is used for both Rm and Rn in reg-shift DP. For Rs reads a dedicated one-liner `read_rs_for_reg_shift` logs the warn when `rs == 15` and returns `r[15] + 4`.

**Flags.** Identical to imm-shift DP:
- `S == 0`: no flag update, same opcode semantics
- `S == 1` with `Rd != 15`: update N/Z from result, C from shifter carry-out (logical ops) or ALU carry-out (arithmetic ops), V from ALU overflow (arithmetic ops only)
- `S == 1` with `Rd == 15`: on real hardware copies SPSR into CPSR (exception return). **Deferred to slice 3d.** In 3b2 we log a warn and do the normal flag update.

**Where the code goes.** The reg-shift path replaces the current warn-and-return fall-through inside `dispatch_dp` in `arm7_dp.cpp`. ~35 lines added. The existing opcode switch (`AND`/`EOR`/.../`MVN`) is reused unchanged.

### 5.2 Multiply family

**What it does (plain language).** Hardware multiply. `MUL` multiplies two 32-bit numbers and writes the low 32 bits of the result. `MLA` does the same but adds a fourth register to the product (multiply-accumulate, used in dot products and filters). The four long variants (`UMULL`/`UMLAL`/`SMULL`/`SMLAL`) produce a full 64-bit result across two destination registers, in signed or unsigned flavors, with or without accumulate. Games use these for anything where a 32-bit product might overflow — audio filter math, fixed-point coordinate scaling, damage formulas with large modifiers, etc.

**Encoding (all six variants).** Shared: `bits[27:22] ∈ {000000, 000001}` + `bits[7:4] == 1001`. Distinguishing bits live in [23:21].

```
31      28 27              22 21 20 19   16 15   12 11    8  7  6  5  4  3    0
| cond   | 0  0  0  0  0  0 | A  S |  Rd   |  Rn   |  Rs   | 1  0  0  1 |  Rm   |   MUL (A=0) / MLA (A=1)
| cond   | 0  0  0  0  1 | U  A  S | RdHi  | RdLo  |  Rs   | 1  0  0  1 |  Rm   |   long multiplies
```

Decode bits:
- `bit[23]` — 0 = short (32-bit result), 1 = long (64-bit result)
- `bit[22]` — long only: 0 = unsigned, 1 = signed
- `bit[21]` — accumulate (`A`)
- `bit[20]` — set flags (`S`)

**Recognizer.** First check of `dispatch_multiply`:

```cpp
if ((instr & 0x0F0000F0u) != 0x00000090u) {
    return 0;  // caller falls through to reg-shift DP path
}
```

Called from `dispatch_dp` *before* the reg-shift DP path. Disjoint masks with both DP and PSR transfer.

**MUL / MLA semantics.**

```cpp
const u32 rm_v = state.r[instr & 0xFu];
const u32 rs_v = state.r[(instr >> 8) & 0xFu];
const u32 rd   = (instr >> 16) & 0xFu;
u32 result = rm_v * rs_v;
if (accumulate) {
    const u32 rn_v = state.r[(instr >> 12) & 0xFu];
    result += rn_v;  // modular 32-bit, wraparound is correct
}
write_rd(state, rd, result);
if (set_flags) {
    // C: UNPREDICTABLE on ARMv4T → preserve. V: unchanged. N/Z: updated.
    state.cpsr = set_nz(state.cpsr, result);
}
```

**Long multiply semantics.**

```cpp
u32 rd_lo = (instr >> 12) & 0xFu;
u32 rd_hi = (instr >> 16) & 0xFu;
u32 rm_v  = state.r[instr & 0xFu];
u32 rs_v  = state.r[(instr >> 8) & 0xFu];

u64 product;
if (signed_mul) {
    i64 sp = (i64)(i32)rm_v * (i64)(i32)rs_v;
    product = (u64)sp;
} else {
    product = (u64)rm_v * (u64)rs_v;
}

if (accumulate) {
    u64 acc = ((u64)state.r[rd_hi] << 32) | (u64)state.r[rd_lo];
    product += acc;  // wraparound on u64 matches hardware for both signed and unsigned long MLA
}

write_rd(state, rd_lo, (u32)product);
write_rd(state, rd_hi, (u32)(product >> 32));
if (set_flags) {
    u32 cpsr = state.cpsr & ~0xC0000000u;
    if (product == 0)         cpsr |= (1u << 30);  // Z
    if ((product >> 63) & 1u) cpsr |= (1u << 31);  // N
    state.cpsr = cpsr;
    // C: UNPREDICTABLE on ARMv4T → preserve. V: unchanged.
}
```

**Accumulate ordering rule.** Read both `RdHi` and `RdLo` into a local `u64 acc` *before* calling `write_rd`. Otherwise, when `RdLo` and one of the operands happen to be the same register, writing `RdLo` first clobbers the second read. The code above is structured to read-then-compute-then-write, so the hazard is avoided unconditionally.

**UNPREDICTABLE paths.** All warn via a single helper `log_multiply_unpredictable(instr, case_name)` so a grep of the log finds every hit.

| Case | Behavior |
|---|---|
| `Rd == 15` (short) or `RdHi == 15` / `RdLo == 15` (long) | Warn, write anyway. No pipeline flush. |
| `Rm == Rd` in `MUL`/`MLA` on ARMv4T | Warn. Execute as written. ARMv5+ removed this restriction. |
| `RdHi == RdLo` in any long form | Warn. Store `RdLo` first, then `RdHi` — second write wins. |
| `RdHi == Rm` / `RdLo == Rm` in long form | Warn. Operand reads complete before writes (see accumulate rule), so the result is correct. |
| `Rs == 15` / `Rm == 15` | Warn. Uses `state.r[15]` (= PC + 8). Games do not do this. |

**Cycle count.** Every variant returns 1. `// TODO(cycles): Rs-dependent early-termination table` tag at the function head.

### 5.3 PSR transfer — MRS and MSR

**What it does (plain language).** The ARM's CPSR ("Current Program Status Register") is a special 32-bit register that isn't in the normal R0..R14 register file. It holds the condition flags (N/Z/C/V), the interrupt-disable bits (I/F), the Thumb bit (T), and the 5-bit mode selector. Normal instructions can't touch it directly — you can't write `ADD CPSR, CPSR, #1`. `MRS` copies CPSR (or its per-mode shadow copy, SPSR) into a regular register so code can inspect it. `MSR` copies from a regular register or a rotated immediate back into CPSR/SPSR. The most important use case: games (and kernels, and any real-time code) disable interrupts around critical sections by reading CPSR with `MRS`, setting the I bit with `ORR`, and writing it back with `MSR`. Without these two instructions there is no way to run interrupt-sensitive code.

**Field masks.** `MSR` has a 4-bit mask in bits[19:16] that selects which byte-fields of the target PSR get written. On ARMv4T:

- bit 19 = `f` — flags byte (bits 31–24: N, Z, C, V, Q)
- bit 18 = `s` — status byte (bits 23–16: reserved on ARMv4T, but writable)
- bit 17 = `x` — extension byte (bits 15–8: reserved on ARMv4T, but writable)
- bit 16 = `c` — control byte (bits 7–0: I, F, T, M[4:0])

Syntax: `MSR CPSR_fc, R0` writes only the flag and control bytes and leaves the other two bytes alone. Without honoring the mask, a program that wanted to flip the interrupt bit would also stomp on the flag bits, and the next `BEQ` would branch on whatever the interrupt flip happened to set. Every ARM compiler emits this, so it is non-optional.

**Encoding.**

```
MRS:             xxxx 00010 P 00 1111 Rd   000000000000       ; P = 0 (CPSR) / 1 (SPSR)
MSR reg form:    xxxx 00010 P 10 mask 1111 00000000 Rm
MSR imm form:    xxxx 00110 P 10 mask 1111 rot imm8
```

Recognizer masks (checked from `dispatch_dp` *before* multiply and *before* imm-shift DP):

```cpp
if ((instr & 0x0FB00000u) == 0x01000000u) return dispatch_psr_transfer(...);  // MRS
if ((instr & 0x0FB00000u) == 0x01200000u) return dispatch_psr_transfer(...);  // MSR reg
if ((instr & 0x0FB00000u) == 0x03200000u) return dispatch_psr_transfer(...);  // MSR imm
```

Bit 20 == 0 in PSR transfer disambiguates from normal DP's `TST`/`TEQ`/`CMP`/`CMN` (which all have S==1 and thus bit 20 == 1).

**Implementation (`arm7_psr.cpp`).**

```cpp
u32 dispatch_psr_transfer(Arm7State& state, u32 instr, u32 /*instr_addr*/) {
    const bool is_msr   = ((instr >> 21) & 1u) != 0;
    const bool use_spsr = ((instr >> 22) & 1u) != 0;

    // --- MRS path ---
    if (!is_msr) {
        const u32 rd = (instr >> 12) & 0xFu;
        u32 value;
        if (use_spsr) {
            const u32* slot = state.spsr_slot();
            if (slot == nullptr) {
                DS_LOG_WARN("arm7: MRS SPSR in User/System mode (unpredictable) at 0x%08X", state.pc);
                value = 0;
            } else {
                value = *slot;
            }
        } else {
            value = state.cpsr;
        }
        write_rd(state, rd, value);
        return 1;
    }

    // --- MSR path: compute source value ---
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    u32 source;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state.cpsr & (1u << 29)) != 0;
        source = rotated_imm(imm8, rotate, c_in).value;
    } else {
        source = state.r[instr & 0xFu];
    }

    // Build byte mask from bits[19:16].
    u32 byte_mask = 0;
    if (instr & (1u << 19)) byte_mask |= 0xFF000000u;  // f
    if (instr & (1u << 18)) byte_mask |= 0x00FF0000u;  // s
    if (instr & (1u << 17)) byte_mask |= 0x0000FF00u;  // x
    if (instr & (1u << 16)) byte_mask |= 0x000000FFu;  // c

    // --- MSR SPSR ---
    if (use_spsr) {
        u32* slot = state.spsr_slot();
        if (slot == nullptr) {
            DS_LOG_WARN("arm7: MSR SPSR in User/System mode (unpredictable) at 0x%08X", state.pc);
            return 1;
        }
        *slot = (*slot & ~byte_mask) | (source & byte_mask);
        return 1;
    }

    // --- MSR CPSR ---
    // Two ordering subtleties:
    //   1. The T-bit change warn fires on all byte-mask patterns that touch bit 5.
    //      Games should flip T via BX, not MSR; warn but honor the write.
    //   2. If the control byte is being written and the new mode differs from the
    //      current mode, we call switch_mode() *first* to save/load register banks,
    //      then overwrite CPSR with the final merged value (which has the correct
    //      mode bits by construction).
    const u32 new_cpsr = (state.cpsr & ~byte_mask) | (source & byte_mask);

    if ((byte_mask & 0x000000FFu) != 0) {
        const u32 new_t = (new_cpsr >> 5) & 1u;
        const u32 cur_t = (state.cpsr >> 5) & 1u;
        if (new_t != cur_t) {
            DS_LOG_WARN("arm7: MSR CPSR T-bit change (use BX) at 0x%08X", state.pc);
        }

        const Mode new_mode = static_cast<Mode>(new_cpsr & 0x1Fu);
        const Mode cur_mode = state.current_mode();
        if (new_mode != cur_mode) {
            if (!is_valid_mode(new_mode)) {
                DS_LOG_WARN("arm7: MSR CPSR illegal mode 0x%02X (unpredictable) at 0x%08X",
                            static_cast<u32>(new_mode), state.pc);
                // Skip the bank swap but still honor the byte write so the mode
                // bits reflect the game's request. Debug visibility over strict fidelity.
            } else {
                state.switch_mode(new_mode);  // saves current bank, loads target bank
            }
        }
    }

    state.cpsr = new_cpsr;
    return 1;
}
```

A small file-static helper lives at the top of `arm7_psr.cpp`:

```cpp
static bool is_valid_mode(Mode m) {
    switch (m) {
        case Mode::User: case Mode::Fiq: case Mode::Irq:
        case Mode::Supervisor: case Mode::Abort:
        case Mode::Undefined: case Mode::System:
            return true;
    }
    return false;
}
```

**UNPREDICTABLE handling.**
- `MRS SPSR` in User/System mode → warn, return 0 into Rd.
- `MSR SPSR` in User/System mode → warn, no-op.
- `MSR CPSR_c` with a bit-5 change → warn (should use BX), honor the write.
- `MSR CPSR_c` with an invalid mode bit pattern → warn, skip the bank swap, still write the bytes.
- Otherwise `MSR CPSR_c` with a valid mode change → `switch_mode()` is called, banking happens, the post-MSR state has the new mode's R8–R14 and the old mode's SPSR has been preserved by `switch_mode()`'s store-on-exit half.

**Cycle count.** Both MRS and MSR return 1. `// TODO(cycles)` tag at the function head for consistency.

## 6. File layout

### 6.1 Files created

- `src/cpu/arm7/arm7_multiply.cpp` — owns `dispatch_multiply()`. Recognizes the multiply format and routes to the 6 variants. ~120 lines.
- `src/cpu/arm7/arm7_psr.cpp` — owns `dispatch_psr_transfer()`. Handles MRS, MSR reg form, MSR imm form, CPSR and SPSR. ~120 lines.
- `tests/unit/arm7_reg_shift_dp_test.cpp` — ~180 lines, 13 test cases.
- `tests/unit/arm7_multiply_test.cpp` — ~260 lines, 23 test cases.
- `tests/unit/arm7_psr_transfer_test.cpp` — ~200 lines, 14 test cases.
- `tests/unit/arm7_multiply_psr_sequence_test.cpp` — ~140 lines, 1 capstone test.

### 6.2 Files modified

- `src/cpu/arm7/arm7_alu.hpp` — **no change.** `barrel_shift_reg` already exists from slice 3a infrastructure; we use it as-is.
- `src/cpu/arm7/arm7_state.hpp` — add `u32* spsr_slot()` and `const u32* spsr_slot() const` methods on `Arm7State`. `Mode` enum, `current_mode()`, `switch_mode()`, and banked storage already exist.
- `src/cpu/arm7/arm7_decode_internal.hpp` — declare `dispatch_multiply`, `dispatch_psr_transfer`, `read_rm_pc12`, and `read_rs_for_reg_shift`.
- `src/cpu/arm7/arm7_dp.cpp` — remove the reg-shift-form warn fall-through; add reg-shift DP path; add PSR-transfer recognizer; add multiply recognizer. Both recognizers live at the top of `dispatch_dp`, checked before any DP operand decoding.
- `src/cpu/arm7/arm7_decode.cpp` — no change.
- `src/CMakeLists.txt` — add `cpu/arm7/arm7_multiply.cpp` and `cpu/arm7/arm7_psr.cpp` to `ds_core`.
- `tests/CMakeLists.txt` — register 4 new test binaries via `add_ds_unit_test`.

After 3b2 the ARM7 tree looks like:

```
src/cpu/arm7/
  arm7.hpp / arm7.cpp
  arm7_state.hpp           (+ spsr_slot() methods)
  arm7_alu.hpp             (unchanged)
  arm7_decode_internal.hpp (+ 2 dispatch decls, read_rm_pc12, read_rs_for_reg_shift)
  arm7_decode.cpp          (unchanged)
  arm7_dp.cpp              (extended — reg-shift path + recognizers)
  arm7_branch.cpp          (unchanged)
  arm7_loadstore.cpp       (unchanged)
  arm7_multiply.cpp        NEW
  arm7_psr.cpp             NEW
```

## 7. Testing strategy

### 7.1 Unit tests (one binary per new family)

**`arm7_reg_shift_dp_test.cpp` — 13 cases.** Includes the PC+12 quirk tests for `Rm==15` and `Rn==15`, the amount-dependent shift boundary tests (0, 31, 32, 33 for each shift type), the "only low byte of Rs matters" test, the `Rd==15, S==1` warn path, and the flag-updates-match-imm-shift test.

**`arm7_multiply_test.cpp` — 23 cases.** One or more cases per variant (MUL, MLA, UMULL, UMLAL, SMULL, SMLAL). Signed edge cases for SMULL/SMLAL: `(-1) * (-1) == 1`, `(-1) * 1 == -1`, `INT_MIN * INT_MIN`. Carry-propagation cases for UMLAL/SMLAL. Modular wraparound for MLA. UNPREDICTABLE cases (`Rd==15`, `Rm==Rd`, `RdHi==RdLo`) verify warn-logged-and-no-crash behavior. Flag tests explicitly verify C and V are preserved by MULS/MLAS/MULLS/MLALS.

**`arm7_psr_transfer_test.cpp` — 14 cases.**
- `MRS R0, CPSR` reads the full status register.
- `MRS R0, SPSR` in User mode → warn path, returns 0.
- `MRS R0, SPSR` after placing the CPU in (e.g.) IRQ mode via `state.switch_mode` and writing a known SPSR value → returns that value.
- `MSR CPSR_f`, `MSR CPSR_c`, `MSR CPSR_fc`, `MSR CPSR_fsxc` — one test per field mask, verifying only the masked bytes change.
- MSR register form AND immediate form (two encodings, same semantics).
- **Mode switch with banking:** starting in Supervisor mode with known R13/R14, `MSR CPSR_c, #0x12` (switch to IRQ) → verify CPSR mode bits == 0x12, R13/R14 now hold IRQ-bank values (not the old SVC-bank values), and the original R13/R14 have been saved into `banks.svc_r13_r14`.
- T-bit warn: `MSR CPSR_c` with a flipped T bit logs a warn.
- Invalid mode warn: `MSR CPSR_c, #0x00` (mode bits = 0x00, not a valid mode) → warn logged, no switch_mode call, byte write still honored.
- SPSR writes in User mode: warn, no change to any bank.
- SPSR write in non-User mode: writes to the correct banked slot (e.g., `banks.spsr_irq` when mode == IRQ).
- **The round-trip test:** the full `MRS → ORR → MSR CPSR_c` idiom that every interrupt-disable critical section uses. Final CPSR has I bit set, no other bits touched. This test is the "is anything real actually going to work" check.

### 7.2 Capstone sequence test

**`arm7_multiply_psr_sequence_test.cpp` — 1 test.** Runs a hand-written ARM7 program that exercises all three families together:

```
MOV  R0, #0x10
MOV  R1, #0x20
MOV  R2, #3
MOV  R2, R0, LSL R2        ; reg-shift DP → R2 == 0x80
MUL  R3, R0, R1            ; R3 == 0x200
MOV  R5, #-1
MUL  R5, R3, R5            ; R5 == -0x200
MOV  R6, #2
SMULL R4, R7, R5, R6       ; R7:R4 == sign-extended -0x400
MRS  R8, CPSR
ORR  R9, R8, #0x80
MSR  CPSR_c, R9            ; I bit set
MRS  R10, CPSR
BIC  R9, R10, #0x80
MSR  CPSR_c, R9            ; I bit cleared
halt: B halt
```

Assertions after ~15 cycles: `R2 == 0x80`, `R3 == 0x200`, `R7:R4 == 0xFFFFFFFFFFFFFC00`, `R10 & 0x80 != 0`, final CPSR I bit == 0. Catches inter-family state-clobbering bugs.

### 7.3 Test infrastructure

Same as 3b1: each test links against `ds_core` only (no SDL), compiles in milliseconds, registered in `tests/CMakeLists.txt` via `add_ds_unit_test(name)`, run via `ctest --output-on-failure` from `build/`. After 3b2 the total test count is **22 binaries** (18 inherited + 4 new). Clean rebuild under 10 seconds, `ctest` under 5.

### 7.4 Explicit non-tests

- No timing tests. Cycle counts are placeholders.
- No banking tests. Banking machinery does not exist yet.
- No Thumb tests. Thumb is slice 3c.
- No exception-entry tests. Exception entry is slice 3d.
- No real-ROM tests. We're nowhere near that — everything is hand-written programs loaded directly into the test harness's memory.

## 8. Open questions & deferred items

### 8.1 Deferred to slice 3b3
- `LDRH`, `STRH`, `LDRSB`, `LDRSH` — halfword and signed single-data transfer
- Their encoding subspace (the halfword extension space at `bits[27:25] == 000` + `bits[7:4] == 1xx1`)

### 8.2 Deferred to slice 3b4
- `LDM`, `STM` — load/store multiple with full register-list, base-writeback, base-in-list, and user-mode register access

### 8.3 Deferred to slice 3c
- Full Thumb ISA (16-bit instructions)
- `step_thumb` dispatcher
- Removal of the `cpsr.T == 0` assert in `step_arm`

### 8.4 Deferred to slice 3d
- Exception entry for reset, undefined, SWI, prefetch abort, data abort, IRQ, FIQ (automatic mode switch + save PC into target-mode LR + copy CPSR into target-mode SPSR + vector jump)
- IRQ line sampling in the fetch loop
- Handling of `MOVS PC, LR` / `SUBS PC, LR, #N` return-from-exception idioms (copies SPSR back to CPSR on PC write when S==1)

**Register banking itself is NOT deferred** — it already works from slice 3a's `Arm7State` infrastructure and slice 3b2's MSR path exercises it.

### 8.5 Deferred to a dedicated cycle-accuracy slice
- Per-instruction cycle counts across all slices (3a, 3b1, 3b2)
- `MUL`/`MLA`'s Rs-dependent early-termination table
- Reg-shift DP's `1S + 1I` cost
- Branch pipeline-flush penalty (2S + 1N)
- Load/store access timing

## 9. References

- GBATEK ARM Opcodes: https://problemkaputt.de/gbatek.htm#armcpureference
  - §ARM Opcodes: Data Processing (register-shift form)
  - §ARM Opcodes: Multiply and Multiply-Accumulate
  - §ARM Opcodes: PSR Transfer (MRS, MSR)
- ARM Architecture Reference Manual, ARMv4T section (historical reference for pipeline stage reads and UNPREDICTABLE definitions)
- Slice 3b1 design: `docs/superpowers/specs/2026-04-14-arm7-core-phase1-slice3b1-design.md`
- Slice 3a design: `docs/superpowers/plans/2026-04-13-arm7-core-phase1-slice3a.md`
- Parent design spec: `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` §4 (CPU), §13 (phase roadmap)

---

**End of Slice 3b2 design.** Next artifact: implementation plan at `docs/superpowers/plans/2026-04-14-arm7-core-phase1-slice3b2.md`, produced by the `writing-plans` skill.
