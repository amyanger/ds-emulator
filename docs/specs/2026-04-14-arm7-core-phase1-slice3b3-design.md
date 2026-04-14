# ARM7 Core — Phase 1, Slice 3b3 Design

**Status:** proposed
**Date:** 2026-04-14
**Predecessor:** `2026-04-14-arm7-core-phase1-slice3b2-design.md` (landed, 22/22 tests green, pushed to origin/main as of `c92ee4b`)
**Parent spec:** `2026-04-12-nds-emulator-design.md` §4 (CPU cores) and §13 (phase roadmap)

---

## 1. Summary

Slice 3b3 adds the ARMv4T **halfword and signed single-data-transfer** instructions to the ARM7 interpreter:

- **`LDRH`** — load unsigned halfword (zero-extended to 32 bits)
- **`STRH`** — store halfword (low 16 bits of Rd)
- **`LDRSB`** — load signed byte (sign-extended to 32 bits)
- **`LDRSH`** — load signed halfword (sign-extended to 32 bits)

These four instructions live in a quirky corner of the ARMv4T encoding space: they share `bits[27:25] == 000` with data-processing but are distinguished by `bit[7] == 1 && bit[4] == 1` — the so-called "halfword extension" pattern. `LDRD`/`STRD` occupy the same encoding slots but are ARMv5TE-only and **not valid on ARM7** (GBATEK verbatim: *"STRD/LDRD supported on ARMv5TE and up only, not ARMv5, not ARMv5TExP"*). The ARM7 decoder will treat the doubleword slots as undefined/warn paths.

After this slice, the only ARMv4T ARM-state instructions still unimplemented on ARM7 are load/store multiple (`LDM`/`STM` — slice 3b4), `SWP`/`SWPB` (slice 3b4 or 3b5), and exception entry / IRQ line sampling (slice 3d). Thumb state is slice 3c.

No new architectural machinery. `Arm7State` is untouched, the scheduler is untouched, the bus is untouched. This is a pure ALU-and-dispatch change confined to `src/cpu/arm7/` and `tests/unit/`.

### Plain-language summary of what the slice adds

The CPU already knows how to load and store 32-bit words and 8-bit bytes. What it's been missing is the ability to work with 16-bit halfwords at all, and to load a signed byte or signed halfword with automatic sign-extension. In plain terms: the CPU learns to read and write 16 bits at a time instead of only 8 or 32, and learns to treat a byte or halfword as a signed number and fill the upper bits with the sign automatically. Every libc `memcpy` touches these. Every string function uses them. Pokemon's save data serialization reads and writes halfwords constantly. The C compiler emits them for any `int16_t` or `short` in a struct — which is almost every struct in a game. Without these four instructions, the ARM7 can't run real code for more than a few hundred cycles before hitting an undefined-instruction path. This slice unblocks the next big milestone: executing past cart boot.

## 2. Goals

- Execute `LDRH`, `STRH`, `LDRSB`, `LDRSH` for both immediate-offset and register-offset forms, for all valid P/U/W combinations.
- Handle the three ARM7TDMI unaligned-address rules correctly (documented in §5.4).
- Handle the `Rn == Rd` with writeback edge case — the loaded value wins, writeback is suppressed.
- Preserve all slice 3b1/3b2 behavior. All 22 existing tests continue to pass unchanged.
- Keep the decoder-split pattern established by 3b2: add `arm7_halfword.cpp` as a new translation unit; extend `arm7_dp.cpp` by adding a single fourth recognizer at the top of `dispatch_dp` that delegates to `dispatch_halfword`.
- Continue using the Q1 placeholder "every instruction costs 1 ARM7 cycle" with `TODO(cycles)` markers at every new dispatch head. Cycle accuracy is a cross-cutting slice, not an opcode-family slice.

## 3. Non-goals

Explicitly out of scope for 3b3; deferred to the slice named in parentheses.

- **`LDRD`, `STRD`** — ARMv5TE only. Not valid on ARM7. The encoding slots (`L=0, SH=2/3`) decode to a warn path on ARM7.
- **`LDM`, `STM`** (slice 3b4) — block transfers are a full slice of their own.
- **`SWP`, `SWPB`** (slice 3b4 or 3b5) — single-register swap occupies the `L=0, SH=0` slot in this encoding space. We'll treat it as a warn path in slice 3b3 and wire it up properly in a later slice. This makes `dispatch_dp`'s fifth recognizer — noted as the refactor threshold in §4.7.
- **Correct cycle timing** — GBATEK lists halfword loads at `1S+1N+1I` and `STRH` at `2N`. We return 1 cycle for all four until the dedicated cycle-accuracy slice.
- **Thumb versions** of any halfword instruction — slice 3c.
- **T-variants** (`LDRHT`, etc.) — no such thing on ARMv4T. GBATEK explicitly says bit 21 must be zero when P=0 in this encoding space.
- **Rd == R15 as a load destination triggering a pipeline flush / ARMv5 Thumb-switch** — documented as UNPREDICTABLE on ARMv4T. We log a warn and write the value to R15 without Thumb switching or pipeline-flush accounting.
- **`Arm7State` register banking changes** — already in place from slice 3a. No touches.

## 4. Architecture

One new translation unit, one new recognizer, one new dispatch function, one new shared helper. Nothing else changes.

### 4.1 Dispatcher structure

Slice 3b2 established this layout under `src/cpu/arm7/`:

```
arm7_decode.cpp        top-level step_arm() + primary dispatch_arm()
arm7_dp.cpp            data processing (imm-shift + reg-shift + DP-adjacent recognizers)
arm7_branch.cpp        B / BL
arm7_loadstore.cpp     LDR / STR / LDRB / STRB  (word/byte only)
arm7_multiply.cpp      MUL / MLA / UMULL / UMLAL / SMULL / SMLAL
arm7_psr.cpp           MRS / MSR
arm7_decode_internal.hpp   shared helpers & dispatch function declarations
```

Slice 3b3 adds **one new translation unit** and extends the recognizer pile-up at the top of `dispatch_dp`:

```
arm7_halfword.cpp      NEW: LDRH / STRH / LDRSB / LDRSH
arm7_dp.cpp            + fourth recognizer (halfword) at the top of dispatch_dp
```

The halfword recognizer lives at the *top* of `dispatch_dp` for the same reason multiply and PSR live there: they share the `bits[27:25] == 000` primary key with DP. Lifting the fan-out into `arm7_decode.cpp` would duplicate the gate across two files. Order inside `dispatch_dp` becomes:

```
dispatch_dp:
    1. BX recognizer                 (bits[27:4] == 0x12FFF1)
    2. PSR-transfer recognizer       (TST/TEQ/CMP/CMN with S=0 pattern)
    3. Multiply recognizer           (bits[27:22] == 0 && bits[7:4] == 0b1001)
    4. Halfword recognizer           (bits[27:25] == 000 && bit[7] == 1 && bit[4] == 1)    ← NEW
    5. Data-processing body          (imm-shift and reg-shift forms)
```

**Important:** multiply's `bits[7:4] == 0b1001` and halfword's `bits[7:4] ∈ {1011, 1101, 1111}` share bit 4 and bit 7. The recognizers must check the full 4-bit pattern in the right order, not just `bit[4] && bit[7]`. Multiply goes first because its pattern is strictly more specific (all four bits fixed at `1001`), halfword next because its pattern matches anything with `bit[4]==1`, `bit[7]==1`, `(bit[5] || bit[6])`.

### 4.2 New header declarations and bus threading

`arm7_decode_internal.hpp` grows by one item, and `dispatch_dp`'s existing declaration grows one parameter:

```cpp
// dispatch_dp now takes Arm7Bus& so it can thread the bus into the
// halfword recognizer. DP/multiply/PSR bodies do not use the bus — the
// parameter is purely a pass-through to dispatch_halfword.
u32 dispatch_dp(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

// Halfword / signed data transfer dispatch entry point.
// Called from dispatch_dp after the halfword recognizer matches.
// Takes the bus directly — matches the dispatch_single_data_transfer pattern.
u32 dispatch_halfword(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);
```

`dispatch_arm` in `arm7_decode.cpp` already has `Arm7Bus& bus` in scope (line 21), so the call site change is one line: `return dispatch_dp(state, bus, instr, instr_addr);`.

No new shared helpers — the addressing-mode helper lives inside `arm7_halfword.cpp` as `static` since it is not reused by any other file. `arm7_loadstore.cpp`'s word/byte addressing is structurally similar but differs in offset composition (12-bit vs 8-bit split) and shifted-register support, so there is no clean shareable type.

### 4.3 Addressing mode computation

All four halfword instructions share the same address computation. Local helper inside `arm7_halfword.cpp`:

```cpp
struct HalfwordAddress {
    u32 address;        // the address to access
    u32 writeback_rn;   // the value to write back to Rn after the transfer (if writeback enabled)
    bool writeback;     // whether Rn actually updates
};

static HalfwordAddress compute_halfword_address(const Arm7State& state, u32 instr);
```

Decoding:

```cpp
const u32 rn = (instr >> 16) & 0xFu;
const bool p = (instr & (1u << 24)) != 0;   // pre-index
const bool u = (instr & (1u << 23)) != 0;   // up/down
const bool i = (instr & (1u << 22)) != 0;   // immediate (1) vs register (0) offset
const bool w = (instr & (1u << 21)) != 0;   // writeback (pre-index only)

// Rn read as PC+8 when Rn == 15, consistent with LDR/STR.
const u32 rn_value = (rn == 15) ? (state.r[15] + 4) : state.r[rn];

u32 offset;
if (i) {
    // Immediate offset: upper 4 bits at [11:8], lower 4 bits at [3:0].
    offset = ((instr >> 4) & 0xF0u) | (instr & 0xFu);
} else {
    // Register offset: bits [11:8] must be zero (warn otherwise).
    const u32 rm = instr & 0xFu;
    if (((instr >> 8) & 0xFu) != 0) {
        DS_LOG_WARN("arm7: halfword register-offset with non-zero shift bits at 0x%08X",
                    state.pc);
    }
    // Rm == 15 is UNPREDICTABLE per GBATEK ("R0-R14, not including R15").
    if (rm == 15) {
        DS_LOG_WARN("arm7: halfword register offset Rm == 15 (unpredictable) at 0x%08X",
                    state.pc);
    }
    offset = (rm == 15) ? (state.r[15] + 4) : state.r[rm];
}

const u32 signed_offset = u ? (rn_value + offset) : (rn_value - offset);

HalfwordAddress out;
if (p) {
    out.address      = signed_offset;            // pre-index: use computed
    out.writeback_rn = signed_offset;
    out.writeback    = w;                        // only writes back if W==1
} else {
    out.address      = rn_value;                 // post-index: use original Rn
    out.writeback_rn = signed_offset;            // but Rn gets the new value
    out.writeback    = true;                     // post-index always writes back
    // P=0 with W=1 is malformed per GBATEK ("Bit 21 not used, must be zero").
    if (w) {
        DS_LOG_WARN("arm7: halfword post-index with W=1 (malformed) at 0x%08X", state.pc);
    }
}
return out;
```

**Writeback order matters for the `Rn == Rd` edge case.** On a load with writeback, the loaded value must win — see §5.5.

### 4.4 Per-opcode semantics

All four run through the same address helper, then branch on the SH field to do the actual transfer.

```cpp
u32 dispatch_halfword(Arm7State& state, Arm7Bus& bus, u32 instr, u32 /*instr_addr*/) {
    const u32 rd = (instr >> 12) & 0xFu;
    const bool l = (instr & (1u << 20)) != 0;
    const u32 sh = (instr >> 5) & 0x3u;

    const HalfwordAddress addr = compute_halfword_address(state, instr);

    if (l) {
        // Load path: SH selects the load variant.
        switch (sh) {
            case 0:
                // Reserved in GBATEK ("Reserved" when L==1). Warn and no-op.
                DS_LOG_WARN("arm7: halfword load with SH=0 (reserved) at 0x%08X", state.pc);
                return 1;
            case 1: {
                // LDRH: 16-bit zero-extended, with unaligned rotate quirk (§5.4.1).
                const u32 value = load_halfword_unsigned(bus, addr.address);
                write_rd_and_writeback(state, rd, value, addr);
                break;
            }
            case 2: {
                // LDRSB: 8-bit sign-extended.
                const u32 value = load_byte_signed(bus, addr.address);
                write_rd_and_writeback(state, rd, value, addr);
                break;
            }
            case 3: {
                // LDRSH: 16-bit sign-extended, with byte-read quirk on unaligned (§5.4.2).
                const u32 value = load_halfword_signed(bus, addr.address);
                write_rd_and_writeback(state, rd, value, addr);
                break;
            }
        }
    } else {
        // Store path: SH selects the store variant.
        switch (sh) {
            case 0:
                // SWP — not yet implemented. Deferred to a later slice.
                DS_LOG_WARN("arm7: SWP encoding encountered, not yet supported at 0x%08X",
                            state.pc);
                return 1;
            case 1: {
                // STRH: store low 16 bits of Rd.
                // Rd == 15 reads as PC+12 per GBATEK.
                const u32 rd_value = (rd == 15) ? (state.r[15] + 8) : state.r[rd];
                store_halfword(bus, addr.address, rd_value);
                if (addr.writeback) {
                    state.r[addr.rn] = addr.writeback_rn;
                }
                break;
            }
            case 2:
            case 3:
                // LDRD/STRD — ARMv5TE only, not valid on ARM7. Warn.
                DS_LOG_WARN("arm7: LDRD/STRD encoding (ARMv5TE only) on ARM7 at 0x%08X",
                            state.pc);
                return 1;
        }
    }
    return 1;  // TODO(cycles): real timing is 1S+1N+1I for loads, 2N for STRH.
}
```

`load_halfword_unsigned`, `load_halfword_signed`, `load_byte_signed`, and `store_halfword` are `static` helpers inside `arm7_halfword.cpp` that wrap `Arm7Bus::read16` / `read8` / `write16` and do the sign-extension or zero-extension for their variant. `write_rd_and_writeback` implements the Rn==Rd writeback suppression rule and lives in the same file.

### 4.5 Bus access

`Arm7Bus` already exposes `read16(addr) -> u16` and `write16(addr, u16 value)` from slice 3b1's infrastructure. Also available: `read8(addr) -> u8` and `write8(addr, u8)`. No new bus methods needed.

Returned halfwords are `u16` — the helpers must cast to `u32` (zero-extend for `LDRH`, sign-extend for `LDRSH`). `store_halfword` must cast `u32 value & 0xFFFFu` to `u16` before calling `write16`.

For signed byte loads, we use existing `read8(addr)` and sign-extend in the helper. For signed halfword loads, we call `read16` and sign-extend via `i16 → i32 → u32`.

### 4.6 What does NOT change

No changes to: `Arm7Bus` (existing methods suffice), `Arm7State`, `Scheduler`, `NDS`, `main.cpp`, WRAM / page tables, `arm7_loadstore.cpp`, any public header under `include/`. No new build-system entries beyond adding `arm7_halfword.cpp` to `ds_core` and the new test binary to `tests/`.

### 4.7 Known technical debt

The senior-architect review flagged the recognizer pile-up at the top of `dispatch_dp` as "bordered but not over" the refactor threshold. Slice 3b3 makes the fourth recognizer. When `SWP` lands (slice 3b4 or 3b5) and adds a fifth, extract them into a `dispatch_000_space` wrapper or split `arm7_dp.cpp`. Not for this slice.

## 5. Hardware details

### 5.1 Encoding (GBATEK verbatim)

From `problemkaputt.de/gbatek.htm` §"ARM Opcodes: Memory: Halfword, Doubleword, and Signed Data Transfer":

```
31-28  Cond
27-25  000    (fixed)
24     P      Pre/Post  (0=post, add offset after transfer; 1=pre, before)
23     U      Up/Down   (0=down, subtract from base; 1=up, add)
22     I      Immediate Offset Flag (0=Register offset, 1=Immediate offset)
21     W      Write-back (pre-index only; MUST BE 0 when P=0)
20     L      Load/Store (0=Store, 1=Load)
19-16  Rn     Base register           (R0-R15, R15 reads as PC+8)
15-12  Rd     Source/Dest register    (R0-R15, R15 reads as PC+12 for STRH)
11-8   imm_hi (I=1) upper 4 bits of offset | (I=0) must be 0000
7      1      (fixed)
6-5    SH     Opcode selector (see table)
4      1      (fixed)
3-0    imm_lo / Rm
```

SH × L table (GBATEK verbatim):

| SH | L=0 (Store) | L=1 (Load) |
|----|-------------|------------|
| 00 | Reserved (SWP) | Reserved |
| 01 | `STRH` | `LDRH` (zero-extended) |
| 10 | `LDRD` (ARMv5TE+) | `LDRSB` (sign-extended) |
| 11 | `STRD` (ARMv5TE+) | `LDRSH` (sign-extended) |

GBATEK note: *"STRH, LDRH, LDRSB, LDRSH supported on ARMv4 and up. STRD/LDRD supported on ARMv5TE and up only, not ARMv5, not ARMv5TExP."* → On ARM7 we warn on `L=0, SH=2` and `L=0, SH=3`.

### 5.2 Recognizer pattern

```cpp
// Halfword recognizer — inside dispatch_dp, after multiply check.
// Pattern: bits[27:25] == 000 && bit[7] == 1 && bit[4] == 1 && (bit[6] | bit[5]) != 0
//
// Bit 4 and bit 7 set distinguishes from DP imm-shift (bit 4 == 0)
// and DP reg-shift (bit 7 == 0). The (bit[6] | bit[5]) != 0 check
// excludes multiply (SH=00) and SWP (also SH=00 but caught by multiply's
// more specific bits[24:23]==00 check; left as a warn path here to be
// safe — real SWP decode lands in a later slice).
const bool bit4 = (instr & (1u << 4)) != 0;
const bool bit7 = (instr & (1u << 7)) != 0;
if (bit4 && bit7) {
    return dispatch_halfword(state, instr, instr_addr);
}
```

Note: bit 4 and bit 7 both being set is the halfword/SWP/multiply extension space marker. Multiply's recognizer already peeled off at the prior check (it fingerprinted on `bits[27:22] == 0` and `bits[7:4] == 0b1001`), so by the time we reach the halfword check, `bit4 && bit7` uniquely implies halfword-or-SWP, and `dispatch_halfword` handles both (SWP as a warn).

### 5.3 Addressing mode table

| P | U | W | Mode                       | Address used        | Rn after          |
|---|---|---|----------------------------|---------------------|-------------------|
| 1 | 1 | 0 | pre-index, up, no writeback | `Rn + offset`       | unchanged         |
| 1 | 1 | 1 | pre-index, up, writeback    | `Rn + offset`       | `Rn + offset`     |
| 1 | 0 | 0 | pre-index, down, no wb      | `Rn - offset`       | unchanged         |
| 1 | 0 | 1 | pre-index, down, writeback  | `Rn - offset`       | `Rn - offset`     |
| 0 | 1 | 0 | post-index, up              | `Rn`                | `Rn + offset`     |
| 0 | 0 | 0 | post-index, down            | `Rn`                | `Rn - offset`     |
| 0 | * | 1 | **malformed per GBATEK**    | — (warn + treat as W=0) | — |

Post-index always writes back — the "W=0 on post-index means no writeback" interpretation is wrong. GBATEK says bit 21 must be zero when P=0, meaning the malformed `P=0, W=1` is where we log.

### 5.4 Unaligned address rules (ARM7TDMI-specific)

GBATEK is silent on unaligned halfword behavior in this section. These rules come from the ARM7TDMI TRM and from cross-referencing melonDS/DeSmuME during implementation.

#### 5.4.1 `LDRH` with `address[0] == 1`

On ARM7TDMI, an unaligned halfword load rotates the loaded halfword right by 8 bits (same family of behavior as unaligned word loads with their 0/8/16/24 rotate).

Concretely:
```
raw16 = bus.read16(address & ~1u)
result = (raw16 >> 8) | ((raw16 & 0xFF) << 24)    // rotate-right-by-8 of a 16-bit value, padded
// Then zero-extend to 32. The resulting Rd has the odd-address byte in bit 0..7
// and the even-address byte in bits 24..31, with 0 in the middle. Games should
// not do this; compilers do not emit it; but melonDS models it for robustness.
```

**Action:** cross-reference melonDS's `ARM::A_LDRH` at implementation time. If melonDS implements something different, match melonDS (it is the reference emulator). Document the final choice in the implementation file.

#### 5.4.2 `LDRSH` with `address[0] == 1`

On ARM7TDMI, an unaligned signed halfword load behaves as **`LDRSB` of the odd byte**. The hardware reads a single byte at the given (odd) address and sign-extends that byte, ignoring the halfword interpretation entirely.

```
raw8 = bus.read8(address)               // NOTE: read8, not read16
result = sign_extend_8_to_32(raw8)
```

This is a real ARM7TDMI quirk and a common source of bugs in ports to ARMv5+, because ARMv5 made this UNPREDICTABLE.

**Action:** same — cross-reference melonDS at implementation time.

#### 5.4.3 `STRH` with `address[0] == 1`

The halfword is written unrotated to `address & ~1`. The hardware masks the low bit and writes anyway. Games that do this are buggy, but the CPU does not raise an exception.

```
bus.write16(address & ~1u, rd_value & 0xFFFFu)
```

#### 5.4.4 `LDRSB` has no alignment concern

Byte loads are always aligned. Read 8 bits from the given address, sign-extend bit 7 through bits 8..31.

### 5.5 `Rn == Rd` with load + writeback

ARM7TDMI behavior: when a load instruction specifies the same register for base (Rn) and destination (Rd), and writeback is enabled (pre-index with W=1 or any post-index), **the loaded value wins** — Rd receives the loaded value, and the writeback is suppressed (Rn is not updated afterward).

This matches `arm7_loadstore.cpp`'s existing behavior for word/byte LDR — the check is already there and we replicate the same pattern in `arm7_halfword.cpp`. Implementation sketch:

```cpp
static void write_rd_and_writeback(Arm7State& state, u32 rd, u32 value,
                                   const HalfwordAddress& addr) {
    const u32 rn = /* decode from instr in caller, or pass through */;
    // Writeback first so that if Rn == Rd the subsequent Rd write wins.
    if (addr.writeback && rn != rd) {
        state.r[rn] = addr.writeback_rn;
    }
    state.r[rd] = value;
}
```

Rationale: if writeback runs first and then `state.r[rd] = value` overwrites it (because Rn == Rd), the loaded value ends up in the shared register, and Rn is not observably updated. If we also skip the writeback entirely (as in the `rn != rd` guard), there is no transient state to worry about.

### 5.6 `Rd == R15` as load destination

Documented as UNPREDICTABLE on ARMv4T. We:

1. Log a warn (`DS_LOG_WARN` with PC and instruction).
2. Write the loaded value to R15 with `value & ~1u` (mask the low bit, matching ARMv4's PC alignment behavior).
3. **Do not** set `CPSR.T` (ARMv4 leaves Thumb bit unchanged on PC writes — GBATEK verbatim for LDR: *"LDR PC,<op> on ARMv4 leaves CPSR.T unchanged"*).
4. **Do not** account for pipeline flush — Q1 placeholder is still "every instruction costs 1 cycle". Defer to the cycle-accuracy slice.

### 5.7 `Rd == R15` as store source (STRH only)

Per GBATEK, Rd=R15 in STRH reads as PC+12 (3 instructions ahead). This matches the word STR behavior and the existing helper pattern in `arm7_loadstore.cpp`.

### 5.8 Malformed encodings — warn paths

| Condition | Interpretation | Action |
|-----------|----------------|--------|
| `L=0, SH=0` | SWP encoding territory | Warn ("SWP not yet supported"), no-op |
| `L=1, SH=0` | Reserved | Warn ("reserved"), no-op |
| `L=0, SH=2` | LDRD (ARMv5TE+) | Warn ("LDRD on ARM7"), no-op |
| `L=0, SH=3` | STRD (ARMv5TE+) | Warn ("STRD on ARM7"), no-op |
| `P=0, W=1`  | bit 21 must be 0 when P=0 | Warn, treat as if W=0 (post-index still writes back automatically) |
| `I=0, bits[11:8] != 0` | reserved bits in register-offset form | Warn, ignore the reserved bits |
| `I=0, Rm == 15` | GBATEK: "R0-R14, not including R15" | Warn, compute as PC+8 for determinism |

None of these are fatal. All log via `DS_LOG_WARN` and either no-op or proceed with a deterministic interpretation.

## 6. Testing strategy

One new test binary plus a capstone end-to-end sequence test. Both link against `ds_core` only, run in milliseconds, no SDL.

### 6.1 `arm7_halfword_test.cpp` — per-opcode unit tests

Covers all four opcodes with a helper-function-per-case pattern (matching the refactored 3b2 tests). Organized in sections:

1. **LDRH — aligned**
   - Immediate offset, pre-index, up/down, with/without writeback
   - Register offset, all P/U/W combos
   - Post-index imm and reg

2. **STRH — aligned**
   - Same matrix as LDRH
   - Rd==R15 reads as PC+12 verification

3. **LDRSB**
   - Sign extension of bit 7 (positive and negative bytes)
   - Immediate and register offsets

4. **LDRSH — aligned**
   - Sign extension of bit 15
   - Immediate and register offsets

5. **Rn == Rd writeback suppression**
   - `LDRH R0, [R0, #4]!` — R0 receives the loaded value, no writeback
   - Same for LDRSB and LDRSH
   - `STRH R0, [R0, #4]!` is the symmetric store case — writeback proceeds normally (different rule)

6. **Unaligned LDRH rotate quirk**
   - `LDRH R0, [R1]` with `R1[0]==1` — verify the rotate-by-8 result matches melonDS behavior

7. **Unaligned LDRSH byte-read quirk**
   - `LDRSH R0, [R1]` with `R1[0]==1` — verify it reads a single byte and sign-extends

8. **Unaligned STRH mask**
   - `STRH R0, [R1]` with `R1[0]==1` — verify the halfword lands at `R1 & ~1`

9. **Rd == R15 load destination**
   - `LDRH PC, [R0]` — verify warn path fires and PC updates (mask low bit)

10. **Malformed encoding warns**
    - `L=0, SH=0` (SWP slot), `L=0, SH=2` (LDRD slot), `L=0, SH=3` (STRD slot), `L=1, SH=0` (reserved)
    - `P=0, W=1` — warn but still post-index writeback
    - `I=0, Rm==15` — warn
    - Verify no crash, deterministic state

### 6.2 `arm7_halfword_sequence_test.cpp` — capstone

End-to-end program that:
- Loads a value into main RAM at a word-aligned address
- Uses `LDRH` to read the low halfword
- Uses `LDRSB` to read an odd byte
- Uses `STRH` to write back a modified value
- Uses `LDRSH` to re-read with sign extension
- Mixes in a reg-shift DP instruction and a multiply from slice 3b2 to verify no dispatcher regressions

This test is the "are all four halfword ops wired correctly against the same state" smoke test, analogous to `arm7_multiply_psr_sequence_test.cpp` from slice 3b2.

### 6.3 Total test binary count

Before: 22.
After: 24 (22 + `arm7_halfword_test` + `arm7_halfword_sequence_test`).

## 7. Cross-references

- **GBATEK §"ARM Opcodes: Memory: Halfword, Doubleword, and Signed Data Transfer"** — primary encoding reference. Cached to `/tmp/gbatek/gbatek.htm` lines 92780-92852 during slice 3b3 planning.
- **ARM7TDMI TRM §4.7.6** — unaligned LDRH/LDRSH behavior (not reproduced in GBATEK).
- **melonDS `src/ARM.cpp` / `src/ARMInterpreter_LoadStore.cpp`** — reference implementation for ARM7 halfword ops. Consult during task 9–11 for unaligned quirks.
- **Predecessor slice:** `docs/specs/2026-04-14-arm7-core-phase1-slice3b2-design.md` — pattern and style for recognizer-at-top-of-dispatch_dp fan-out.
- **Parent spec:** `docs/specs/2026-04-12-nds-emulator-design.md` §4.2 (ARM7 interpreter) and §13.3 (Phase 1 milestone list).

## 8. Risk and rollback

Low risk. All changes are additive:
- One new `.cpp` file under `src/cpu/arm7/`
- One new header declaration in `arm7_decode_internal.hpp`
- One new recognizer call inside `dispatch_dp`
- Two new test binaries

No public API changes, no header-include graph changes outside `src/cpu/arm7/`, no subsystem-boundary touches. If the slice regresses any existing test, revert is a single file removal plus reverting `arm7_decode_internal.hpp` and `arm7_dp.cpp`. The recognizer call has no side effects if the `dispatch_halfword` body is stubbed to a warn-and-return.

## 9. Slice completion criteria

Slice 3b3 is complete when:

1. All 24 test binaries build and pass under `ctest --output-on-failure`.
2. The encoding recognizer correctly routes every `bits[27:25]==000 && bit[4]==1 && bit[7]==1` instruction to `dispatch_halfword` without false positives on DP, multiply, or PSR-transfer encodings.
3. The four happy-path opcodes (`LDRH`, `STRH`, `LDRSB`, `LDRSH`) are implemented per §4.4.
4. The three unaligned quirks (§5.4) are implemented and tested, cross-referenced against melonDS.
5. The `Rn==Rd` writeback rule (§5.5) is implemented on all three load variants and tested on each.
6. The malformed-encoding warn paths (§5.8) log deterministically and do not crash.
7. `src/cpu/arm7/arm7_halfword.cpp` ≤ 400 lines (soft target; hard limit is the 800-line house rule).
8. No new warnings, no new tracked files, no co-author trailers.
9. The full pipeline has run: `/gbatek-check` → `senior-architect` → baseline → implement → `/simplify` → (`ds-architecture-rule-checker` ∥ `gbatek-reviewer`) → `quality-reviewer` → `ctest` → commit.

---

## Appendix A. Encoding cheat sheet (for the implementer)

```
Is this instruction a halfword/signed data transfer?
  (instr & 0x0E000090) == 0x00000090       // bits[27:25]==000, bit[7]=1, bit[4]=1
  AND the prior multiply check did not match
  AND the prior PSR check did not match

Immediate vs register offset?
  I = (instr >> 22) & 1                    // 1 = immediate, 0 = register
  If I == 1: offset = ((instr >> 4) & 0xF0) | (instr & 0xF)
  If I == 0: offset = r[instr & 0xF]       // bits[11:8] must be 0

Pre vs post index?
  P = (instr >> 24) & 1
  If P == 1: address = Rn ± offset ; if W=1, Rn := address
  If P == 0: address = Rn ; Rn := Rn ± offset (always)

SH decodes:
  L=1 SH=1 -> LDRH  (zero-extend 16)
  L=1 SH=2 -> LDRSB (sign-extend 8)
  L=1 SH=3 -> LDRSH (sign-extend 16)
  L=0 SH=1 -> STRH  (write low 16 of Rd)
  (all other SH values: warn path)
```
