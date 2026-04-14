# ARM7 Core — Phase 1, Slice 3b2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish the ARMv4T `bits[27:26] == 00` encoding subspace on the ARM7 interpreter by adding register-shifted-register data processing, the full multiply family (MUL / MLA / UMULL / UMLAL / SMULL / SMLAL), and PSR transfer (MRS / MSR with `_fsxc` field masks on both CPSR and SPSR).

**Architecture:** Extend `arm7_dp.cpp` to recognize its two new sibling families (multiply and PSR transfer) and hand them off, then replace its reg-shift-form warn fall-through with a real implementation. Add two new translation units: `arm7_multiply.cpp` for the multiply family and `arm7_psr.cpp` for PSR transfer. Lean on existing slice 3a infrastructure — `barrel_shift_reg` in `arm7_alu.hpp`, `Arm7State::Mode` + `switch_mode()` + `Arm7Banks` — so banking works end-to-end without deferring to slice 3d. The only new state surface is a `spsr_slot()` accessor that centralizes the User/System-mode UNPREDICTABLE check.

**Tech Stack:** C++20, CMake, CTest, plain test binaries under `tests/unit/` using the existing `REQUIRE` macro from `tests/support/require.hpp`.

**Spec:** `docs/superpowers/specs/2026-04-14-arm7-core-phase1-slice3b2-design.md`

---

## File structure

### Files created

- `src/cpu/arm7/arm7_multiply.cpp` — owns `dispatch_multiply()` and its 6 variants.
- `src/cpu/arm7/arm7_psr.cpp` — owns `dispatch_psr_transfer()` and the `is_valid_mode` file-static helper.
- `tests/unit/arm7_reg_shift_dp_test.cpp` — register-shift DP tests.
- `tests/unit/arm7_multiply_test.cpp` — multiply family tests.
- `tests/unit/arm7_psr_transfer_test.cpp` — MRS / MSR tests.
- `tests/unit/arm7_multiply_psr_sequence_test.cpp` — capstone end-to-end test.

### Files modified

- `src/cpu/arm7/arm7_state.hpp` — add `spsr_slot()` methods.
- `src/cpu/arm7/arm7_decode_internal.hpp` — declare `dispatch_multiply`, `dispatch_psr_transfer`, and add `read_rm_pc12` and `read_rs_for_reg_shift` inline helpers.
- `src/cpu/arm7/arm7_dp.cpp` — add PSR transfer + multiply recognizers at top of `dispatch_dp`; replace reg-shift-form warn fall-through with real reg-shift DP path; switch `rn_val` read to PC+12-aware version for reg-shift form.
- `src/CMakeLists.txt` — add `cpu/arm7/arm7_multiply.cpp` and `cpu/arm7/arm7_psr.cpp` to `ds_core`.
- `tests/CMakeLists.txt` — register 4 new test binaries.

---

## Task 1: Baseline verification

**Purpose:** Confirm the slice 3b1 baseline is green before touching anything. The mandatory pipeline requires never implementing on top of red tests.

**Files:** none.

- [ ] **Step 1: Verify baseline is green**

Run:
```bash
cd build && cmake .. && make && ctest --output-on-failure
```
Expected: build clean, **18/18 tests pass**. If the baseline is red, STOP and fix the failing tests before proceeding — do not implement on top of a broken baseline.

---

## Task 2: Register-shift DP — full shift semantics and tests

**Purpose:** Replace the reg-shift-form warn fall-through in `dispatch_dp` with a real implementation that handles all four shift types at any amount, respects the "only low byte of Rs matters" rule, and reuses the existing `barrel_shift_reg` helper with the caller-preserves-carry rule for amount==0.

**Files:**
- Modify: `src/cpu/arm7/arm7_decode_internal.hpp`
- Modify: `src/cpu/arm7/arm7_dp.cpp`
- Create: `tests/unit/arm7_reg_shift_dp_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Add helper declarations to `arm7_decode_internal.hpp`**

Add to the existing file, right below the existing `write_rd` inline helper:

```cpp
// Read Rm in register-shift DP form. In reg-shift form, the ARM reads Rm
// one pipeline stage later than in imm-shift form, so Rm == 15 returns
// PC + 12 instead of PC + 8. Also used for Rn reads in reg-shift form.
inline u32 read_rm_pc12(const Arm7State& s, u32 rm) {
    return (rm == 15) ? (s.r[15] + 4) : s.r[rm];
}

// Read Rs (the register holding the shift amount) in register-shift DP
// form. Rs == 15 is UNPREDICTABLE on real ARMv4T hardware; we log a warn
// and return PC + 12 for determinism. Games do not use this form.
inline u32 read_rs_for_reg_shift(const Arm7State& s, u32 rs) {
    if (rs == 15) {
        DS_LOG_WARN("arm7: reg-shift DP with Rs == 15 (unpredictable) at PC 0x%08X",
                    s.pc);
    }
    return (rs == 15) ? (s.r[15] + 4) : s.r[rs];
}
```

- [ ] **Step 2: Rewrite the reg-shift-form branch in `dispatch_dp`**

In `src/cpu/arm7/arm7_dp.cpp`, locate the current reg-shift warn-and-return block (lines 42-51 in current HEAD):

```cpp
// Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
// immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
const bool i_bit = ((instr >> 25) & 1u) != 0;
const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;
if (reg_shift) {
    // Register-shifted-register operand2 — deferred to slice 3b2.
    // Also catches MUL/MLA and halfword LDR/STR, which all live in
    // this encoding space and are unimplemented in slice 3b1.
    DS_LOG_WARN("arm7: register-shift dp form 0x%08X at 0x%08X",
                instr, instr_addr);
    return 1;
}
```

Replace the `if (reg_shift) { ... }` body with a real implementation. The final block looks like:

```cpp
const bool i_bit     = ((instr >> 25) & 1u) != 0;
const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;

// Note: reg_shift == true implies bit 7 == 0 per the ARMv4T encoding table
// (bit 7 == 1 with bit 4 == 1 is the multiply / halfword-extension space,
// which is peeled off by dispatch_multiply / future halfword recognizers
// at the top of dispatch_dp before we reach this point).
```

Then extend the operand2 computation to handle the reg-shift case. Replace the existing operand2 computation block (currently two cases: `i_bit` and `!i_bit` imm-shift) with three cases:

```cpp
ShifterResult op2;
if (i_bit) {
    const u32 imm8   = instr & 0xFFu;
    const u32 rotate = (instr >> 8) & 0xFu;
    const bool c_in  = (state.cpsr & (1u << 29)) != 0;
    op2 = rotated_imm(imm8, rotate, c_in);
} else if (reg_shift) {
    const u32  rm         = instr & 0xFu;
    const u32  rs         = (instr >> 8) & 0xFu;
    const u32  shift_type = (instr >> 5) & 0x3u;
    const u32  amount     = read_rs_for_reg_shift(state, rs) & 0xFFu;
    const u32  rm_value   = read_rm_pc12(state, rm);
    // barrel_shift_reg returns meaningless carry when amount == 0; the caller
    // is responsible for preserving CPSR.C in that case.
    op2 = barrel_shift_reg(rm_value, static_cast<ShiftType>(shift_type), amount);
    if (amount == 0) {
        op2.carry = (state.cpsr & (1u << 29)) != 0;
    }
} else {
    // Immediate-shifted register operand2. Rm==15 naturally reads
    // state.r[15] == instr_addr + 8.
    const u32  rm         = instr & 0xFu;
    const u32  shift_type = (instr >> 5) & 0x3u;
    const u32  shift_amt  = (instr >> 7) & 0x1Fu;
    const bool c_in       = (state.cpsr & (1u << 29)) != 0;
    op2 = barrel_shift_imm(
        state.r[rm],
        static_cast<ShiftType>(shift_type),
        shift_amt,
        c_in);
}
```

Then change the `rn_val` read so reg-shift form uses `read_rm_pc12` for Rn:

```cpp
const u32  opcode = (instr >> 21) & 0xFu;
const bool s_flag = ((instr >> 20) & 1u) != 0;
const u32  rn     = (instr >> 16) & 0xFu;
const u32  rd     = (instr >> 12) & 0xFu;

// In reg-shift form, Rn == 15 also reads PC + 12 (same pipeline-stage
// offset as Rm). In imm-shift and imm-form it reads PC + 8 as usual.
const u32 rn_val = reg_shift ? read_rm_pc12(state, rn) : state.r[rn];
```

No other changes to `dispatch_dp`. The opcode switch and flag-writeback logic reuse unchanged.

- [ ] **Step 3: Create `tests/unit/arm7_reg_shift_dp_test.cpp`**

Create a new file with the full test suite. Use the same pattern as `tests/unit/arm7_data_processing_test.cpp`:

```cpp
// arm7_reg_shift_dp_test.cpp — ARMv4T register-shifted-register
// data-processing tests. Exercises the bit-4==1 form of DP operand2
// (shift amount comes from Rs[7:0]) for all four shift types, including
// the amount==0 / 32 / >32 boundary rules and the PC+12 pipeline quirk
// for Rm/Rn/Rs == 15 (tested separately in Task 4).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

// Base address for hand-assembled test programs. ARM7 WRAM lives here in
// the ARM7 memory map; every existing test uses the same base.
constexpr u32 kBase = 0x0380'0000u;

// Preload a single instruction word at `pc`, set PC and R15 for the ARM
// pipeline model, and run exactly one ARM7 cycle. Every instruction in
// slice 3b2 still returns 1 ARM7 cycle per the cycle-count placeholder,
// so `run_until((cycles_before + 1) * 2)` advances by exactly one instr.
static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

// Assemble a reg-shift DP instruction:
//   cond = AL (0xE), I=0, bit4=1, bit7=0
//   |31..28 cond| 00 | 0 opcode S | Rn | Rd | Rs | 0 shift 1 | Rm |
constexpr u32 AL_COND = 0xEu << 28;
constexpr u32 kOpMOV  = 0xDu;
constexpr u32 kOpADD  = 0x4u;
constexpr u32 kLSL    = 0x0u;
constexpr u32 kLSR    = 0x1u;
constexpr u32 kASR    = 0x2u;
constexpr u32 kROR    = 0x3u;

u32 encode_reg_shift_dp(u32 opcode, bool s, u32 rn, u32 rd,
                        u32 rs, u32 shift_type, u32 rm) {
    u32 instr = AL_COND;
    instr |= (opcode & 0xFu) << 21;
    if (s) instr |= (1u << 20);
    instr |= (rn & 0xFu) << 16;
    instr |= (rd & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (shift_type & 0x3u) << 5;
    instr |= (1u << 4);  // bit4 = 1 (reg-shift)
    instr |= (rm & 0xFu);
    return instr;
}

}  // namespace

int main() {
    // Test 1: MOV R0, R1, LSL R2 with R1=0x1, R2=3 → R0 == 0x8
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x1u;
        nds.cpu7().state().r[2] = 3u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0x8u);
    }

    // Test 2: MOV R0, R1, LSL R2 with R2=0 → R0 == R1, CPSR.C unchanged
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xDEADBEEFu;
        nds.cpu7().state().r[2] = 0u;
        nds.cpu7().state().cpsr |= (1u << 29);  // set C
        const u32 cpsr_before = nds.cpu7().state().cpsr;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0xDEADBEEFu);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C preserved
        (void)cpsr_before;
    }

    // Test 3: MOV R0, R1, LSL R2 with R2=32 → R0 == 0, C == bit 0 of R1
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000001u;  // bit 0 set
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C = 1 from bit 0
    }

    // Test 4: MOV R0, R1, LSL R2 with R2=33 → R0 == 0, C == 0
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 33u;
        nds.cpu7().state().cpsr |= (1u << 29);  // pre-set C to verify it gets cleared
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) == 0);  // C = 0
    }

    // Test 5: MOV R0, R1, LSR R2 with R2=32 → R0 == 0, C == bit 31 of R1
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kLSR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
    }

    // Test 6: MOV R0, R1, ASR R2 with R1=0x80000000, R2=32 → R0 == 0xFFFFFFFF
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 2, kASR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFFu);
    }

    // Test 7: MOV R0, R1, ROR R2 with R2=32 → R0 == R1, C == bit 31 of R1
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 32u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kROR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
    }

    // Test 8: MOV R0, R1, ROR R2 with R2=64 → same as R2=32
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 64u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 0, 2, kROR, 1));
        REQUIRE(nds.cpu7().state().r[0] == 0x80000000u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);
    }

    // Test 9: only low byte of Rs matters: ADD R0, R1, R2, LSL R3 with R3=0xFFFFFF04
    //         should behave as LSL #4
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x10u;
        nds.cpu7().state().r[2] = 0x1u;
        nds.cpu7().state().r[3] = 0xFFFFFF04u;  // high bits ignored
        run_one(nds, kBase, encode_reg_shift_dp(kOpADD, false, 1, 0, 3, kLSL, 2));
        REQUIRE(nds.cpu7().state().r[0] == 0x10u + (0x1u << 4));
    }

    // Test 10: ADDS R0, R1, R2, LSL R3 flag update matches imm-shift behavior
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 0x1u;
        nds.cpu7().state().r[3] = 0u;  // no shift
        run_one(nds, kBase, encode_reg_shift_dp(kOpADD, true, 1, 0, 3, kLSL, 2));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C set (carry out of u32 add)
    }

    std::puts("arm7_reg_shift_dp_test: all 10 cases passed");
    return 0;
}
```

- [ ] **Step 4: Register the test binary in `tests/CMakeLists.txt`**

Add one line in the block where other ARM7 test binaries are declared:

```cmake
add_ds_unit_test(arm7_reg_shift_dp_test)
```

- [ ] **Step 5: Build and run, verify the new test passes and the full suite is still green**

Run:
```bash
cd build && cmake .. && make 2>&1 | tee /tmp/slice3b2_t2_build.log
ctest --output-on-failure
```
Expected: build clean, **19/19 tests pass** (18 inherited + 1 new).

- [ ] **Step 6: Commit**

```bash
git add src/cpu/arm7/arm7_decode_internal.hpp src/cpu/arm7/arm7_dp.cpp \
        tests/unit/arm7_reg_shift_dp_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: register-shifted-register DP operand2

Replace the reg-shift warn fall-through in dispatch_dp with a real
implementation. Reuses the existing barrel_shift_reg helper from
arm7_alu.hpp (3-arg form, caller preserves CPSR.C when amount==0).
Adds read_rm_pc12 and read_rs_for_reg_shift helpers to the private
decode header — used by both Rn and Rm reads in reg-shift form.

Tests cover all four shift types (LSL/LSR/ASR/ROR) at amount 0, 1..31,
32, and >32, plus the 'only low byte of Rs matters' rule and the flag
update parity with imm-shift ADDS."
```

---

## Task 3: Register-shift DP — PC+12 pipeline quirk

**Purpose:** Verify the ARMv4T "reg-shift reads Rm/Rn one pipeline stage later" rule is correct. In reg-shift form, `Rm == 15` or `Rn == 15` reads `PC + 12` instead of `PC + 8`. This is the subtlest correctness issue in reg-shift DP and the one most likely to cause hard-to-find bugs later if we ship it wrong.

**Files:**
- Modify: `tests/unit/arm7_reg_shift_dp_test.cpp`

- [ ] **Step 1: Add PC+12 quirk tests to the existing test file**

Add these three tests inside `main()` after Test 10 and before the final `std::puts` / `return`:

```cpp
    // Test 11: MOV R0, R15, LSL R1 with R1=0 → R0 == PC+12, not PC+8
    // Instruction lives at kBase. r[15] at execute time is kBase+8.
    // Reg-shift form reads r[15] as PC+12 == kBase+12.
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 1, kLSL, 15));
        REQUIRE(nds.cpu7().state().r[0] == kBase + 12u);
    }

    // Test 12: ADD R0, R15, R2, LSL R3 with R3=0 → R0 == (PC+12) + R2
    // Same PC+12 rule for Rn==15 in reg-shift form.
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0x100u;
        nds.cpu7().state().r[3] = 0u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpADD, false, 15, 0, 3, kLSL, 2));
        REQUIRE(nds.cpu7().state().r[0] == (kBase + 12u) + 0x100u);
    }

    // Test 13: MOV R0, R1, LSL R15 → warn logged (Rs == 15 is UNPREDICTABLE),
    // result uses PC+12 == kBase+12 as the shift amount. Low byte of
    // that address is 0x0C (assuming kBase aligned to >= 0x100), so this
    // behaves as LSL #12.
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x1u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, false, 0, 0, 15, kLSL, 1));
        REQUIRE(nds.cpu7().state().r[0] == (0x1u << ((kBase + 12u) & 0xFFu)));
    }
```

Update the final `std::puts` string to read `"arm7_reg_shift_dp_test: all 13 cases passed"`.

- [ ] **Step 2: Build and run**

Run:
```bash
cd build && make && ctest --output-on-failure -R arm7_reg_shift_dp_test
```
Expected: test passes with 13 cases.

- [ ] **Step 3: Run the full suite**

Run: `cd build && ctest --output-on-failure`
Expected: **19/19 tests pass**.

- [ ] **Step 4: Commit**

```bash
git add tests/unit/arm7_reg_shift_dp_test.cpp
git commit -m "cpu/arm7: reg-shift DP PC+12 pipeline quirk tests

In register-shift form, the ARM reads Rn/Rm/Rs one pipeline stage later
than in imm-shift form, so R15 as any of those operands returns PC+12
instead of PC+8. Tests cover Rm==15, Rn==15, and the UNPREDICTABLE
Rs==15 case (warn + deterministic fallback)."
```

---

## Task 4: Register-shift DP — Rd==15 with S=1 warn path

**Purpose:** Cover the "copies SPSR to CPSR on PC write" case — on real hardware `MOVS PC, Rn, LSL Rm` is the primary exception-return idiom. Slice 3b2 honors the normal flag update path but logs a warn; the real SPSR→CPSR copy lives in slice 3d alongside exception entry.

**Files:**
- Modify: `tests/unit/arm7_reg_shift_dp_test.cpp`

- [ ] **Step 1: Add the Rd==15 S=1 test**

Add one more test inside `main()`:

```cpp
    // Test 14: MOVS R15, R1, LSL R2 with R1=0x02000100, R2=0 →
    // PC branches to 0x02000100 (masked to word align), warn logged,
    // normal flag update runs (no SPSR→CPSR copy — deferred to slice 3d).
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x02000100u;
        nds.cpu7().state().r[2] = 0u;
        run_one(nds, kBase, encode_reg_shift_dp(kOpMOV, true, 0, 15, 2, kLSL, 1));
        REQUIRE(nds.cpu7().state().pc == 0x02000100u);
        // CPSR wasn't copied from SPSR — the mode bits should still be System.
        REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x1Fu);
    }
```

Update the `std::puts` to `"arm7_reg_shift_dp_test: all 14 cases passed"`.

- [ ] **Step 2: Build, run, and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **19/19 tests pass**, `arm7_reg_shift_dp_test` reports 14 cases.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_reg_shift_dp_test.cpp
git commit -m "cpu/arm7: reg-shift DP Rd==15 with S=1 warn path

Slice 3a's imm-shift DP already warns on S-flag set with Rd=R15; this
test confirms reg-shift DP takes the same path. Real exception-return
semantics (SPSR→CPSR copy on MOVS PC, LR) land in slice 3d."
```

---

## Task 5: Multiply — `arm7_multiply.cpp` scaffold + `MUL` + `MLA`

**Purpose:** Create the multiply translation unit, add the recognizer at the top of `dispatch_dp`, and implement the two short-form variants (`MUL`, `MLA`). Short-form multiply is the simplest entry point to the multiply family — 32-bit result, single destination register, no signed/unsigned distinction.

**Files:**
- Modify: `src/cpu/arm7/arm7_decode_internal.hpp`
- Create: `src/cpu/arm7/arm7_multiply.cpp`
- Modify: `src/cpu/arm7/arm7_dp.cpp`
- Modify: `src/CMakeLists.txt`
- Create: `tests/unit/arm7_multiply_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Declare `dispatch_multiply` in the internal header**

In `src/cpu/arm7/arm7_decode_internal.hpp`, add alongside the existing dispatch declarations:

```cpp
// Multiply family dispatcher. Called from the top of dispatch_dp before
// any DP operand decoding. The first thing it does is pattern-check
// (instr & 0x0F0000F0) == 0x00000090; if that fails the caller should
// treat the instruction as normal DP.
u32 dispatch_multiply(Arm7State& state, u32 instr, u32 instr_addr);
```

- [ ] **Step 2: Create `src/cpu/arm7/arm7_multiply.cpp`**

```cpp
// arm7_multiply.cpp — ARMv4T multiply family dispatch.
// Handles MUL, MLA (short 32-bit forms) and UMULL/UMLAL/SMULL/SMLAL
// (long 64-bit forms) across the bits[27:22] ∈ {000000, 000001} +
// bits[7:4] == 1001 encoding pattern. All variants currently return 1
// ARM7 cycle; real Rs-dependent early-termination timing is deferred
// to a dedicated cycle-accuracy slice.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "ds/common.hpp"

namespace ds {

namespace {

// Log a warn for any UNPREDICTABLE multiply case. Single call site so
// a grep of the log finds every instance.
void log_multiply_unpredictable(u32 instr, const char* reason, u32 pc) {
    DS_LOG_WARN("arm7: multiply UNPREDICTABLE (%s) instr 0x%08X at PC 0x%08X",
                reason, instr, pc);
}

}  // namespace

u32 dispatch_multiply(Arm7State& state, u32 instr, u32 instr_addr) {
    // Pattern: cond xxxx 000 0 xxxA S xxxx xxxx xxxx 1001 xxxx
    //          (bit 23 decides short vs long; bit 22 signed; bit 21 accumulate;
    //           bit 20 set-flags)
    if ((instr & 0x0F0000F0u) != 0x00000090u) {
        return 0;  // not a multiply; caller continues with normal DP
    }

    const bool long_form  = ((instr >> 23) & 1u) != 0;
    const bool signed_mul = ((instr >> 22) & 1u) != 0;  // meaningful only when long_form
    const bool accumulate = ((instr >> 21) & 1u) != 0;
    const bool set_flags  = ((instr >> 20) & 1u) != 0;

    if (!long_form) {
        // MUL / MLA: 32-bit result in Rd at [19:16]. Rn at [15:12] is the
        // accumulate operand (MLA) — ignored when A==0.
        const u32 rm = instr & 0xFu;
        const u32 rs = (instr >> 8) & 0xFu;
        const u32 rn = (instr >> 12) & 0xFu;
        const u32 rd = (instr >> 16) & 0xFu;

        if (rd == 15) log_multiply_unpredictable(instr, "Rd==15", state.pc);
        if (rm == rd) log_multiply_unpredictable(instr, "Rm==Rd (ARMv4T)", state.pc);

        const u32 rm_v = state.r[rm];
        const u32 rs_v = state.r[rs];
        u32 result = rm_v * rs_v;
        if (accumulate) {
            result += state.r[rn];
        }
        write_rd(state, rd, result);

        if (set_flags) {
            // N and Z reflect the 32-bit result. C is UNPREDICTABLE on
            // ARMv4T → preserve. V unchanged.
            state.cpsr = set_nz(state.cpsr, result);
        }

        (void)instr_addr;
        // TODO(cycles): Rs-dependent early-termination table.
        return 1;
    }

    // Long-form handling is added in Tasks 6 / 7. In slice 3b2 the dispatch
    // table reaches this point only for variants we've implemented.
    DS_LOG_WARN("arm7: long-form multiply 0x%08X at 0x%08X (not yet implemented)",
                instr, instr_addr);
    (void)signed_mul;
    (void)accumulate;
    (void)set_flags;
    return 1;
}

}  // namespace ds
```

- [ ] **Step 3: Add `arm7_multiply.cpp` to `src/CMakeLists.txt`**

Locate the `ds_core` target's `target_sources` list and add `cpu/arm7/arm7_multiply.cpp`:

```cmake
target_sources(ds_core PRIVATE
    # ... existing entries ...
    cpu/arm7/arm7_multiply.cpp
    # ...
)
```

- [ ] **Step 4: Wire the multiply recognizer into `dispatch_dp`**

In `src/cpu/arm7/arm7_dp.cpp`, add the multiply recognizer after the existing BX recognizer and before the `i_bit`/`reg_shift` computation. The key property: multiply's `bits[7:4] == 1001` pattern has bit 7 == 1, which is disjoint from reg-shift DP's bit 7 == 0, so we cannot miss either.

```cpp
u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr) {
    // BX lives in DP encoding space: cond 0001 0010 1111 1111 1111 0001 Rm.
    if ((instr & 0x0FFF'FFF0u) == 0x012F'FF10u) {
        // ... existing BX handler ...
        return 1;
    }

    // Multiply family: bits[27:22] ∈ {000000, 000001}, bits[7:4] == 1001.
    // Peel off before any DP operand decoding.
    if ((instr & 0x0FC000F0u) == 0x00000090u) {
        return dispatch_multiply(state, instr, instr_addr);
    }

    // ... existing i_bit / reg_shift computation and the rest of dispatch_dp ...
}
```

- [ ] **Step 5: Write the failing test file `tests/unit/arm7_multiply_test.cpp`**

```cpp
// arm7_multiply_test.cpp — ARMv4T multiply family tests. Exercises MUL,
// MLA, UMULL, UMLAL, SMULL, SMLAL and the UNPREDICTABLE warn paths.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

constexpr u32 AL_COND = 0xEu << 28;

// MUL: cond 0000000 S Rd 0000 Rs 1001 Rm
u32 encode_mul(bool s, u32 rd, u32 rm, u32 rs) {
    u32 instr = AL_COND | 0x00000090u;
    if (s) instr |= (1u << 20);
    instr |= (rd & 0xFu) << 16;
    instr |= (rs & 0xFu) << 8;
    instr |= (rm & 0xFu);
    return instr;
}

// MLA: cond 0000001 S Rd Rn Rs 1001 Rm
u32 encode_mla(bool s, u32 rd, u32 rm, u32 rs, u32 rn) {
    u32 instr = AL_COND | 0x00200090u;
    if (s) instr |= (1u << 20);
    instr |= (rd & 0xFu) << 16;
    instr |= (rn & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (rm & 0xFu);
    return instr;
}

}  // namespace

int main() {
    // Test 1: MUL R0, R1, R2 plain
    {
        NDS nds;
        nds.cpu7().state().r[1] = 7u;
        nds.cpu7().state().r[2] = 6u;
        run_one(nds, kBase, encode_mul(false, 0, 1, 2));
        REQUIRE(nds.cpu7().state().r[0] == 42u);
    }

    // Test 2: MUL overflow (low 32 bits)
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 0x2u;
        run_one(nds, kBase, encode_mul(false, 0, 1, 2));
        REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFEu);
    }

    // Test 3: MULS result == 0 → Z set, N clear
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0u;
        nds.cpu7().state().r[2] = 42u;
        run_one(nds, kBase, encode_mul(true, 0, 1, 2));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) == 0);
    }

    // Test 4: MULS result bit31 == 1 → N set, Z clear
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x80000000u;
        nds.cpu7().state().r[2] = 1u;
        run_one(nds, kBase, encode_mul(true, 0, 1, 2));
        REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) != 0);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);
    }

    // Test 5: MULS leaves C and V unchanged
    {
        NDS nds;
        nds.cpu7().state().r[1] = 7u;
        nds.cpu7().state().r[2] = 6u;
        nds.cpu7().state().cpsr |= (1u << 29) | (1u << 28);  // pre-set C and V
        run_one(nds, kBase, encode_mul(true, 0, 1, 2));
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C preserved
        REQUIRE((nds.cpu7().state().cpsr & (1u << 28)) != 0);  // V preserved
    }

    // Test 6: MLA accumulate
    {
        NDS nds;
        nds.cpu7().state().r[1] = 3u;
        nds.cpu7().state().r[2] = 4u;
        nds.cpu7().state().r[3] = 100u;
        run_one(nds, kBase, encode_mla(false, 0, 1, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 112u);  // 3*4 + 100
    }

    // Test 7: MLA with accumulator overflow
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 0x2u;
        nds.cpu7().state().r[3] = 0x4u;
        run_one(nds, kBase, encode_mla(false, 0, 1, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == (0xFFFFFFFEu + 0x4u));  // wraps to 0x00000002
    }

    // Test 8: MLAS flag behavior matches MULS
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0u;
        nds.cpu7().state().r[2] = 0u;
        nds.cpu7().state().r[3] = 0u;
        run_one(nds, kBase, encode_mla(true, 0, 1, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z
    }

    std::puts("arm7_multiply_test: all 8 cases passed");
    return 0;
}
```

- [ ] **Step 6: Register the test binary**

In `tests/CMakeLists.txt` add:

```cmake
add_ds_unit_test(arm7_multiply_test)
```

- [ ] **Step 7: Build and verify the new test passes**

Run: `cd build && cmake .. && make && ctest --output-on-failure`
Expected: **20/20 tests pass** (19 + 1 new).

- [ ] **Step 8: Commit**

```bash
git add src/cpu/arm7/arm7_decode_internal.hpp src/cpu/arm7/arm7_multiply.cpp \
        src/cpu/arm7/arm7_dp.cpp src/CMakeLists.txt \
        tests/unit/arm7_multiply_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: MUL and MLA in new arm7_multiply.cpp

Add a new translation unit for the ARMv4T multiply family, wire its
recognizer into the top of dispatch_dp (before the reg-shift path),
and implement the two short 32-bit variants. Tests cover plain multiply,
low-32-bit overflow, MLA accumulate with wraparound, and the MULS/MLAS
N/Z flag updates that must leave C and V untouched. Long-form variants
land in the next commits."
```

---

## Task 6: Multiply — `UMULL` + `SMULL` (long form, no accumulate)

**Purpose:** Extend `dispatch_multiply` to handle unsigned and signed 64-bit multiply (no accumulate). These two variants exercise the signed/unsigned split without the additional complexity of pre-loaded accumulator state — good incremental step.

**Files:**
- Modify: `src/cpu/arm7/arm7_multiply.cpp`
- Modify: `tests/unit/arm7_multiply_test.cpp`

- [ ] **Step 1: Replace the "long-form not yet implemented" warn in `arm7_multiply.cpp` with a real long-form handler**

The final version of the `if (!long_form) { ... }` / long-form branch looks like this. Replace the current `DS_LOG_WARN("arm7: long-form multiply ...")` block with:

```cpp
    // Long form: 64-bit result in RdHi:RdLo.
    const u32 rm    = instr & 0xFu;
    const u32 rs    = (instr >> 8) & 0xFu;
    const u32 rd_lo = (instr >> 12) & 0xFu;
    const u32 rd_hi = (instr >> 16) & 0xFu;

    if (rd_hi == 15 || rd_lo == 15)
        log_multiply_unpredictable(instr, "RdHi/RdLo==15", state.pc);
    if (rd_hi == rd_lo)
        log_multiply_unpredictable(instr, "RdHi==RdLo", state.pc);

    const u32 rm_v = state.r[rm];
    const u32 rs_v = state.r[rs];

    u64 product;
    if (signed_mul) {
        const i64 sp = static_cast<i64>(static_cast<i32>(rm_v))
                     * static_cast<i64>(static_cast<i32>(rs_v));
        product = static_cast<u64>(sp);
    } else {
        product = static_cast<u64>(rm_v) * static_cast<u64>(rs_v);
    }

    if (accumulate) {
        const u64 acc = (static_cast<u64>(state.r[rd_hi]) << 32)
                      | static_cast<u64>(state.r[rd_lo]);
        product += acc;
    }

    // Order matters only when rd_hi == rd_lo. We read both operands
    // before writing, so the write order just determines who wins
    // (spec says behavior is UNPREDICTABLE in that case).
    write_rd(state, rd_lo, static_cast<u32>(product));
    write_rd(state, rd_hi, static_cast<u32>(product >> 32));

    if (set_flags) {
        u32 cpsr = state.cpsr & ~0xC0000000u;
        if (product == 0)               cpsr |= (1u << 30);  // Z
        if ((product >> 63) & 1u)       cpsr |= (1u << 31);  // N
        state.cpsr = cpsr;
        // C: UNPREDICTABLE on ARMv4T → preserve. V: unchanged.
    }

    (void)instr_addr;
    // TODO(cycles): Rs-dependent early-termination table.
    return 1;
```

- [ ] **Step 2: Add encoding helpers and UMULL / SMULL tests to `arm7_multiply_test.cpp`**

Add to the helper block before `main()`:

```cpp
// Long multiplies: cond 00001 U A S RdHi RdLo Rs 1001 Rm
//   bit 22 (U): 1=signed, 0=unsigned
//   bit 21 (A): 1=accumulate, 0=plain
u32 encode_long_mul(bool signed_mul, bool accumulate, bool s,
                    u32 rd_hi, u32 rd_lo, u32 rm, u32 rs) {
    u32 instr = AL_COND | 0x00800090u;  // bits[27:23]=00001
    if (signed_mul) instr |= (1u << 22);
    if (accumulate) instr |= (1u << 21);
    if (s)          instr |= (1u << 20);
    instr |= (rd_hi & 0xFu) << 16;
    instr |= (rd_lo & 0xFu) << 12;
    instr |= (rs    & 0xFu) << 8;
    instr |= (rm    & 0xFu);
    return instr;
}
```

Then add these test cases before `std::puts`:

```cpp
    // Test 9: UMULL with both operands = 0xFFFFFFFF
    //   0xFFFFFFFF * 0xFFFFFFFF == 0xFFFFFFFE_00000001
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;
        nds.cpu7().state().r[3] = 0xFFFFFFFFu;
        run_one(nds, kBase, encode_long_mul(false, false, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0x00000001u);  // RdLo
        REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFEu);  // RdHi
    }

    // Test 10: UMULL with one zero operand → both halves zero
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0u;
        nds.cpu7().state().r[3] = 0xDEADBEEFu;
        run_one(nds, kBase, encode_long_mul(false, false, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
        REQUIRE(nds.cpu7().state().r[1] == 0u);
    }

    // Test 11: UMULLS with bit63 == 0 → N clear
    {
        NDS nds;
        nds.cpu7().state().r[2] = 2u;
        nds.cpu7().state().r[3] = 3u;
        run_one(nds, kBase, encode_long_mul(false, false, true, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 6u);
        REQUIRE(nds.cpu7().state().r[1] == 0u);
        REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) == 0);  // N clear
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
    }

    // Test 12: UMULLS with 0 product → Z set
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0u;
        nds.cpu7().state().r[3] = 0u;
        run_one(nds, kBase, encode_long_mul(false, false, true, 1, 0, 2, 3));
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
    }

    // Test 13: SMULL: (-1) * (-1) == 1
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
        nds.cpu7().state().r[3] = 0xFFFFFFFFu;  // -1
        run_one(nds, kBase, encode_long_mul(true, false, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0x00000001u);
        REQUIRE(nds.cpu7().state().r[1] == 0x00000000u);
    }

    // Test 14: SMULL: (-1) * 1 == -1 (all 64 bits ones)
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
        nds.cpu7().state().r[3] = 0x00000001u;  // 1
        run_one(nds, kBase, encode_long_mul(true, false, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFFu);
        REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFFu);
    }

    // Test 15: SMULL: INT_MIN * INT_MIN = 0x4000000000000000
    //   -0x80000000 signed * -0x80000000 signed == +0x4000000000000000
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0x80000000u;  // INT_MIN
        nds.cpu7().state().r[3] = 0x80000000u;  // INT_MIN
        run_one(nds, kBase, encode_long_mul(true, false, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0x00000000u);
        REQUIRE(nds.cpu7().state().r[1] == 0x40000000u);
    }

    // Test 16: SMULLS with negative product → N from bit 63 set
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
        nds.cpu7().state().r[3] = 0x00000001u;  // 1
        run_one(nds, kBase, encode_long_mul(true, false, true, 1, 0, 2, 3));
        REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) != 0);  // N set
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
    }
```

Update the final `std::puts` to read `"arm7_multiply_test: all 16 cases passed"`.

- [ ] **Step 3: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **20/20 tests pass**, `arm7_multiply_test` reports 16 cases.

- [ ] **Step 4: Commit**

```bash
git add src/cpu/arm7/arm7_multiply.cpp tests/unit/arm7_multiply_test.cpp
git commit -m "cpu/arm7: UMULL and SMULL (long unsigned/signed multiply)

Extends dispatch_multiply with the long-form path. UMULL uses a plain
u64*u64 product; SMULL sign-extends both operands through i32 → i64
before multiplying. Tests cover (0xFFFFFFFF)^2, (-1)*(-1), (-1)*1, and
INT_MIN*INT_MIN — the four cases where sign-extension bugs hide."
```

---

## Task 7: Multiply — `UMLAL` + `SMLAL` (long form with accumulate)

**Purpose:** The accumulate long multiplies. The long-form branch already handles accumulate via the `if (accumulate) { ... }` block added in Task 6, so this task is mostly about writing tests that verify the accumulate path — especially the carry propagation from RdLo into RdHi when the low-word addition overflows.

**Files:**
- Modify: `tests/unit/arm7_multiply_test.cpp`

- [ ] **Step 1: Add UMLAL / SMLAL tests**

Add to `arm7_multiply_test.cpp` inside `main()` before the `std::puts`:

```cpp
    // Test 17: UMLAL — accumulate into pre-loaded RdHi:RdLo
    //   RdHi:RdLo starts at 0x00000000_00000100
    //   product = 2 * 3 = 6
    //   result  = 0x00000000_00000106
    {
        NDS nds;
        nds.cpu7().state().r[0] = 0x00000100u;  // RdLo pre-load
        nds.cpu7().state().r[1] = 0x00000000u;  // RdHi pre-load
        nds.cpu7().state().r[2] = 2u;
        nds.cpu7().state().r[3] = 3u;
        run_one(nds, kBase, encode_long_mul(false, true, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0x00000106u);
        REQUIRE(nds.cpu7().state().r[1] == 0x00000000u);
    }

    // Test 18: UMLAL carry propagation: low-word add overflows into high word
    //   RdLo pre-load = 0xFFFFFFFF
    //   RdHi pre-load = 0x00000000
    //   product = 1 * 2 = 2
    //   accumulate: 0x00000000_FFFFFFFF + 2 = 0x00000001_00000001
    {
        NDS nds;
        nds.cpu7().state().r[0] = 0xFFFFFFFFu;
        nds.cpu7().state().r[1] = 0x00000000u;
        nds.cpu7().state().r[2] = 1u;
        nds.cpu7().state().r[3] = 2u;
        run_one(nds, kBase, encode_long_mul(false, true, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0x00000001u);
        REQUIRE(nds.cpu7().state().r[1] == 0x00000001u);
    }

    // Test 19: SMLAL — signed accumulate with negative product + negative accumulator
    //   RdHi:RdLo starts at 0xFFFFFFFF_FFFFFFFF == -1 (signed)
    //   product = (-1) * 1 = -1 == 0xFFFFFFFF_FFFFFFFF
    //   sum = -2 == 0xFFFFFFFF_FFFFFFFE
    {
        NDS nds;
        nds.cpu7().state().r[0] = 0xFFFFFFFFu;
        nds.cpu7().state().r[1] = 0xFFFFFFFFu;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
        nds.cpu7().state().r[3] = 0x00000001u;  //  1
        run_one(nds, kBase, encode_long_mul(true, true, false, 1, 0, 2, 3));
        REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFEu);
        REQUIRE(nds.cpu7().state().r[1] == 0xFFFFFFFFu);
    }

    // Test 20: SMLALS flag update reflects the full 64-bit result
    {
        NDS nds;
        nds.cpu7().state().r[0] = 0x00000000u;
        nds.cpu7().state().r[1] = 0x00000000u;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;  // -1
        nds.cpu7().state().r[3] = 0x00000001u;
        run_one(nds, kBase, encode_long_mul(true, true, true, 1, 0, 2, 3));
        REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) != 0);  // N set (bit 63)
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) == 0);  // Z clear
    }
```

Update `std::puts` to `"arm7_multiply_test: all 20 cases passed"`.

- [ ] **Step 2: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **20/20 tests pass**, `arm7_multiply_test` reports 20 cases.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_multiply_test.cpp
git commit -m "cpu/arm7: UMLAL and SMLAL accumulate long multiply tests

The long-form dispatcher already handles the accumulate path — this
commit adds the tests that prove it. Covers u64 pre-load, carry
propagation from RdLo into RdHi when the low-word add overflows, and
the signed accumulate case with a negative accumulator and a negative
product."
```

---

## Task 8: Multiply — UNPREDICTABLE warn paths

**Purpose:** Cover the UNPREDICTABLE cases (`Rd==15`, `Rm==Rd`, `RdHi==RdLo`). These paths already log via `log_multiply_unpredictable` from Task 5; this task adds tests to confirm no crash and deterministic behavior.

**Files:**
- Modify: `tests/unit/arm7_multiply_test.cpp`

- [ ] **Step 1: Add UNPREDICTABLE tests**

Add to `arm7_multiply_test.cpp`:

```cpp
    // Test 21: MUL with Rm==Rd — UNPREDICTABLE on ARMv4T, warn logged,
    // result is still the standard product (we write the register normally).
    {
        NDS nds;
        nds.cpu7().state().r[0] = 3u;
        nds.cpu7().state().r[1] = 4u;
        run_one(nds, kBase, encode_mul(false, 0, 0, 1));  // Rd=R0, Rm=R0, Rs=R1
        // 3 * 4 == 12, even though Rm was also Rd.
        REQUIRE(nds.cpu7().state().r[0] == 12u);
    }

    // Test 22: MUL with Rd==15 — UNPREDICTABLE, warn logged, R15 written.
    // No pipeline flush. The test just confirms we don't crash.
    {
        NDS nds;
        nds.cpu7().state().r[1] = 0x02000100u;
        nds.cpu7().state().r[2] = 1u;
        run_one(nds, kBase, encode_mul(false, 15, 1, 2));
        // R15 was written with 0x02000100 * 1 == 0x02000100 (word-aligned by write_rd).
        REQUIRE(nds.cpu7().state().pc == 0x02000100u);
    }

    // Test 23: UMULL with RdHi==RdLo — UNPREDICTABLE, warn logged.
    // Current code writes RdLo first, then RdHi; RdHi wins. We just
    // confirm it doesn't crash and produces a deterministic result
    // (whatever the RdHi write deposited).
    {
        NDS nds;
        nds.cpu7().state().r[2] = 0xFFFFFFFFu;
        nds.cpu7().state().r[3] = 0xFFFFFFFFu;
        // RdHi == RdLo == R0: both halves target the same register.
        run_one(nds, kBase, encode_long_mul(false, false, false, 0, 0, 2, 3));
        // Second write (RdHi = 0xFFFFFFFE) wins.
        REQUIRE(nds.cpu7().state().r[0] == 0xFFFFFFFEu);
    }
```

Update `std::puts` to `"arm7_multiply_test: all 23 cases passed"`.

- [ ] **Step 2: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **20/20 tests pass** (test count unchanged; new cases extend an existing binary), `arm7_multiply_test` reports 23 cases.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_multiply_test.cpp
git commit -m "cpu/arm7: multiply UNPREDICTABLE warn-path tests

Confirms Rm==Rd, Rd==15, and RdHi==RdLo cases log a warn, don't crash,
and produce deterministic results. Ordering rule for RdHi==RdLo: the
second write wins because we emit RdLo first, then RdHi."
```

---

## Task 9: MRS — scaffold `arm7_psr.cpp` + `spsr_slot()` + MRS CPSR / SPSR

**Purpose:** Create the PSR-transfer translation unit, add the `spsr_slot()` accessor to `Arm7State`, wire the PSR-transfer recognizer into `dispatch_dp`, and implement the MRS (read PSR) half. MSR (write PSR) lands in later tasks.

**Files:**
- Modify: `src/cpu/arm7/arm7_state.hpp`
- Modify: `src/cpu/arm7/arm7_decode_internal.hpp`
- Create: `src/cpu/arm7/arm7_psr.cpp`
- Modify: `src/cpu/arm7/arm7_dp.cpp`
- Modify: `src/CMakeLists.txt`
- Create: `tests/unit/arm7_psr_transfer_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Add `spsr_slot()` methods to `Arm7State`**

In `src/cpu/arm7/arm7_state.hpp`, add to the public section of `Arm7State` (after `current_mode()` / `switch_mode()`):

```cpp
    // Returns pointer to the SPSR storage slot for the current mode.
    // Returns nullptr when the current mode is User or System (no SPSR
    // exists for those modes). Used by arm7_psr.cpp for MRS/MSR SPSR.
    u32* spsr_slot() {
        switch (current_mode()) {
            case Mode::Fiq:        return &banks.spsr_fiq;
            case Mode::Irq:        return &banks.spsr_irq;
            case Mode::Supervisor: return &banks.spsr_svc;
            case Mode::Abort:      return &banks.spsr_abt;
            case Mode::Undefined:  return &banks.spsr_und;
            default:               return nullptr;  // User / System
        }
    }

    const u32* spsr_slot() const {
        return const_cast<Arm7State*>(this)->spsr_slot();
    }
```

- [ ] **Step 2: Declare `dispatch_psr_transfer` in the internal header**

Add to `src/cpu/arm7/arm7_decode_internal.hpp`:

```cpp
// PSR transfer (MRS / MSR) dispatcher. Called from the top of dispatch_dp
// before the multiply recognizer. Handles both CPSR and SPSR forms; the
// caller has already confirmed the pattern matches.
u32 dispatch_psr_transfer(Arm7State& state, u32 instr, u32 instr_addr);
```

- [ ] **Step 3: Create `src/cpu/arm7/arm7_psr.cpp` with the MRS path only**

```cpp
// arm7_psr.cpp — ARMv4T PSR transfer (MRS and MSR).
// Handles the four encoding forms:
//   MRS Rd, CPSR    bits[27:23]==00010, bit[22]=0, bits[21:20]==00
//   MRS Rd, SPSR    bits[27:23]==00010, bit[22]=1, bits[21:20]==00
//   MSR PSR, Rm     bits[27:23]==00010, bit[21:20]==10, register form
//   MSR PSR, #imm   bits[27:23]==00110, bit[21:20]==10, immediate form
// Byte mask lives at bits[19:16] and chooses which of the four CPSR
// byte-fields (f, s, x, c) get written.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "ds/common.hpp"

namespace ds {

namespace {

// Valid CPSR mode-bit patterns on ARMv4T. Anything else is UNPREDICTABLE.
bool is_valid_mode(Mode m) {
    switch (m) {
        case Mode::User: case Mode::Fiq: case Mode::Irq:
        case Mode::Supervisor: case Mode::Abort:
        case Mode::Undefined: case Mode::System:
            return true;
    }
    return false;
}

}  // namespace

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
                DS_LOG_WARN("arm7: MRS SPSR in User/System mode (unpredictable) at 0x%08X",
                            state.pc);
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

    // MSR path — filled in by subsequent tasks.
    DS_LOG_WARN("arm7: MSR not yet implemented, instr 0x%08X", instr);
    (void)is_valid_mode;  // silence unused-function warning until MSR lands
    return 1;
}

}  // namespace ds
```

- [ ] **Step 4: Add `arm7_psr.cpp` to `src/CMakeLists.txt`**

```cmake
target_sources(ds_core PRIVATE
    # ... existing entries ...
    cpu/arm7/arm7_psr.cpp
    # ...
)
```

- [ ] **Step 5: Wire the PSR-transfer recognizer into `dispatch_dp`**

In `src/cpu/arm7/arm7_dp.cpp`, add three recognizer checks after the BX recognizer and before the multiply recognizer. They must run before multiply because the multiply mask is narrower (would also match if we got the order wrong, but separation is cleaner).

```cpp
    // PSR transfer lives at bits[27:26]==00 with bit[24:23]==10 and bit[20]==0.
    // Three encoding variants:
    //   MRS:          xxxx 00010 P 00 1111 Rd   000000000000
    //   MSR reg form: xxxx 00010 P 10 mask 1111 00000000 Rm
    //   MSR imm form: xxxx 00110 P 10 mask 1111 rot imm8
    if ((instr & 0x0FB00000u) == 0x01000000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);  // MRS
    }
    if ((instr & 0x0FB00000u) == 0x01200000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);  // MSR reg form
    }
    if ((instr & 0x0FB00000u) == 0x03200000u) {
        return dispatch_psr_transfer(state, instr, instr_addr);  // MSR imm form
    }

    // Multiply family: bits[27:22] ∈ {000000, 000001}, bits[7:4] == 1001.
    if ((instr & 0x0FC000F0u) == 0x00000090u) {
        return dispatch_multiply(state, instr, instr_addr);
    }
```

- [ ] **Step 6: Create `tests/unit/arm7_psr_transfer_test.cpp` with MRS tests only**

```cpp
// arm7_psr_transfer_test.cpp — ARMv4T MRS and MSR tests.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

static void run_one(NDS& nds, u32 pc, u32 instr) {
    nds.arm7_bus().write32(pc, instr);
    nds.cpu7().state().pc = pc;
    nds.cpu7().state().r[15] = pc + 8;
    const u64 cycles_before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((cycles_before + 1) * 2);
}

constexpr u32 AL_COND = 0xEu << 28;

// MRS Rd, CPSR/SPSR: cond 00010 P 00 1111 Rd 000000000000
u32 encode_mrs(u32 rd, bool use_spsr) {
    u32 instr = AL_COND | 0x010F0000u;
    if (use_spsr) instr |= (1u << 22);
    instr |= (rd & 0xFu) << 12;
    return instr;
}

}  // namespace

int main() {
    // Test 1: MRS R0, CPSR reads current CPSR verbatim
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x6000001Fu;  // set Z and C
        run_one(nds, kBase, encode_mrs(0, false));
        REQUIRE(nds.cpu7().state().r[0] == 0x6000001Fu);
    }

    // Test 2: MRS R0, SPSR in System mode → warn path, returns 0
    // System shares the user register bank and has no SPSR storage, so
    // spsr_slot() returns nullptr.
    {
        NDS nds;
        nds.cpu7().state().switch_mode(Mode::System);
        run_one(nds, kBase, encode_mrs(0, true));
        REQUIRE(nds.cpu7().state().r[0] == 0u);
    }

    // Test 3: MRS R0, SPSR after switching to IRQ mode with a known SPSR
    {
        NDS nds;
        nds.cpu7().state().switch_mode(Mode::Irq);
        nds.cpu7().state().banks.spsr_irq = 0xDEADBEEFu;
        run_one(nds, kBase, encode_mrs(0, true));
        REQUIRE(nds.cpu7().state().r[0] == 0xDEADBEEFu);
    }

    std::puts("arm7_psr_transfer_test: all 3 cases passed");
    return 0;
}
```

- [ ] **Step 7: Register the test binary**

In `tests/CMakeLists.txt`:

```cmake
add_ds_unit_test(arm7_psr_transfer_test)
```

- [ ] **Step 8: Build and verify**

Run: `cd build && cmake .. && make && ctest --output-on-failure`
Expected: **21/21 tests pass** (20 + 1 new).

- [ ] **Step 9: Commit**

```bash
git add src/cpu/arm7/arm7_state.hpp src/cpu/arm7/arm7_decode_internal.hpp \
        src/cpu/arm7/arm7_psr.cpp src/cpu/arm7/arm7_dp.cpp \
        src/CMakeLists.txt \
        tests/unit/arm7_psr_transfer_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: MRS CPSR/SPSR in new arm7_psr.cpp

Scaffolds the PSR-transfer translation unit and adds a spsr_slot()
accessor to Arm7State that returns the correct banked SPSR pointer or
nullptr for User/System. MRS reads CPSR directly or the current mode's
SPSR via spsr_slot(). MSR paths are stubbed with a warn for now — they
land in the next commits."
```

---

## Task 10: MSR immediate form — field masks, CPSR (no mode change)

**Purpose:** Implement the `MSR PSR, #imm` encoding for CPSR, honoring the `_fsxc` field mask. This is the first real MSR implementation and exercises the byte-mask merge logic. Mode changes are not yet wired; they land in Task 13.

**Files:**
- Modify: `src/cpu/arm7/arm7_psr.cpp`
- Modify: `tests/unit/arm7_psr_transfer_test.cpp`

- [ ] **Step 1: Replace the MSR-stub in `dispatch_psr_transfer` with the real implementation**

In `arm7_psr.cpp`, replace the `// MSR path — filled in by subsequent tasks.` block with:

```cpp
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

    // Build byte mask from bits[19:16] (the "_fsxc" field).
    u32 byte_mask = 0;
    if (instr & (1u << 19)) byte_mask |= 0xFF000000u;  // f
    if (instr & (1u << 18)) byte_mask |= 0x00FF0000u;  // s (reserved on v4T)
    if (instr & (1u << 17)) byte_mask |= 0x0000FF00u;  // x (reserved on v4T)
    if (instr & (1u << 16)) byte_mask |= 0x000000FFu;  // c

    if (use_spsr) {
        u32* slot = state.spsr_slot();
        if (slot == nullptr) {
            DS_LOG_WARN("arm7: MSR SPSR in User/System mode (unpredictable) at 0x%08X",
                        state.pc);
            return 1;
        }
        *slot = (*slot & ~byte_mask) | (source & byte_mask);
        return 1;
    }

    // --- MSR CPSR ---
    // Mode-bit changes and T-bit changes land in Task 13. This task
    // implements the "byte-mask merge" without any mode switching —
    // callers should only write byte patterns that don't change mode
    // until Task 13 lands.
    state.cpsr = (state.cpsr & ~byte_mask) | (source & byte_mask);
    return 1;
```

Also remove the `(void)is_valid_mode;` line — the function isn't called yet but will be next commit.

Actually keep the `(void)is_valid_mode;` line until Task 13 wires the call, to avoid a `-Wunused-function` warning. If that's already suppressed by the file's compile flags, you can drop it; otherwise leave it.

- [ ] **Step 2: Add MSR immediate-form CPSR tests**

Add helper encoding in `arm7_psr_transfer_test.cpp` above `main()`:

```cpp
// MSR PSR, #imm form:
//   cond 00110 P 10 mask 1111 rot imm8
u32 encode_msr_imm(bool use_spsr, u32 mask_bits, u32 imm8, u32 rot) {
    u32 instr = AL_COND | 0x0320F000u;
    if (use_spsr) instr |= (1u << 22);
    instr |= (mask_bits & 0xFu) << 16;
    instr |= (rot & 0xFu) << 8;
    instr |= imm8 & 0xFFu;
    return instr;
}

// Field mask bit positions relative to bit 16:
//   bit 0 = c, bit 1 = x, bit 2 = s, bit 3 = f
constexpr u32 kMaskF = 0x8u;
constexpr u32 kMaskC = 0x1u;
constexpr u32 kMaskFC = kMaskF | kMaskC;
constexpr u32 kMaskFSXC = 0xFu;
```

Add tests:

```cpp
    // Test 4: MSR CPSR_f, #0xF0000000 → only flag byte changes
    // Use rot=4 (which yields rotate amount 8): 0xF0 ror 8 == 0xF0000000
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x0000001Fu;  // start with System mode, no flags
        run_one(nds, kBase, encode_msr_imm(false, kMaskF, 0xF0u, 4));
        REQUIRE(nds.cpu7().state().cpsr == 0xF000001Fu);
    }

    // Test 5: MSR CPSR_f, #0 clears flags, leaves rest alone
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0xF000001Fu;
        run_one(nds, kBase, encode_msr_imm(false, kMaskF, 0u, 0));
        REQUIRE(nds.cpu7().state().cpsr == 0x0000001Fu);
    }

    // Test 6: MSR CPSR_c, #0x80 sets I bit without touching flags
    //   0x80 with rot=0 == 0x00000080 (just the literal imm8)
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x6000001Fu;  // Z + C + System mode
        run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x80u, 0));
        REQUIRE(nds.cpu7().state().cpsr == 0x6000009Fu);  // I bit added
    }

    // Test 7: MSR CPSR_fc, #0x50000080 (f=0x50, c=0x80) writes both fields
    //   We need imm8 pattern of 0x5 ror 24 for top, composed with 0x80 on bottom.
    //   Simpler: do two separate MSRs. Test the _fc mask with an imm that has
    //   bits in both ends after rotation. 0x50 ror 4 == 0x00000005 — bottom only.
    //   Instead test by pre-loading CPSR and asserting both byte masks take:
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x0000001Fu;
        // rot=0, imm8=0x80: source == 0x00000080
        // byte_mask = 0xFF0000FF (f + c)
        // result = (0x0000001F & ~0xFF0000FF) | (0x00000080 & 0xFF0000FF)
        //        = 0x00000000 | 0x00000080 = 0x00000080
        run_one(nds, kBase, encode_msr_imm(false, kMaskFC, 0x80u, 0));
        REQUIRE(nds.cpu7().state().cpsr == 0x00000080u);
    }
```

Update `std::puts` to `"arm7_psr_transfer_test: all 7 cases passed"`.

- [ ] **Step 3: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **21/21 tests pass**, `arm7_psr_transfer_test` reports 7 cases.

- [ ] **Step 4: Commit**

```bash
git add src/cpu/arm7/arm7_psr.cpp tests/unit/arm7_psr_transfer_test.cpp
git commit -m "cpu/arm7: MSR immediate form with _fsxc field mask on CPSR

Adds the byte-mask merge core of MSR: read the rotated immediate, build
a 32-bit mask from bits[19:16], and write only the masked bytes into
CPSR. Tests cover _f (flags only), _c (control only), and _fc
(flags + control). Mode bit and T bit changes land in Task 13."
```

---

## Task 11: MSR register form

**Purpose:** Add the `MSR PSR, Rm` encoding — same semantics as immediate form but source comes from a register. This is a minimal change because the `i_bit` branch in `dispatch_psr_transfer` already has both paths.

**Files:**
- Modify: `tests/unit/arm7_psr_transfer_test.cpp`

- [ ] **Step 1: Add MSR register-form encoding and tests**

Add to the helper block:

```cpp
// MSR PSR, Rm: cond 00010 P 10 mask 1111 00000000 Rm
u32 encode_msr_reg(bool use_spsr, u32 mask_bits, u32 rm) {
    u32 instr = AL_COND | 0x0120F000u;
    if (use_spsr) instr |= (1u << 22);
    instr |= (mask_bits & 0xFu) << 16;
    instr |= (rm & 0xFu);
    return instr;
}
```

Add tests:

```cpp
    // Test 8: MSR CPSR_f, R0 with R0 = 0x50000000 → N clear, Z set, C set, V clear
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x0000001Fu;
        nds.cpu7().state().r[0] = 0x50000000u;  // Z + C set, N + V clear
        run_one(nds, kBase, encode_msr_reg(false, kMaskF, 0));
        REQUIRE((nds.cpu7().state().cpsr & (1u << 31)) == 0);  // N clear
        REQUIRE((nds.cpu7().state().cpsr & (1u << 30)) != 0);  // Z set
        REQUIRE((nds.cpu7().state().cpsr & (1u << 29)) != 0);  // C set
        REQUIRE((nds.cpu7().state().cpsr & (1u << 28)) == 0);  // V clear
    }

    // Test 9: MSR CPSR_fsxc, R0 writes the whole register
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x0000001Fu;
        nds.cpu7().state().r[0] = 0xDEAD'BE5Fu;  // arbitrary; mode bits stay 0x1F
        run_one(nds, kBase, encode_msr_reg(false, kMaskFSXC, 0));
        REQUIRE(nds.cpu7().state().cpsr == 0xDEAD'BE5Fu);
    }
```

Update `std::puts` to `"arm7_psr_transfer_test: all 9 cases passed"`.

- [ ] **Step 2: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **21/21 tests pass**, `arm7_psr_transfer_test` reports 9 cases.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_psr_transfer_test.cpp
git commit -m "cpu/arm7: MSR register form on CPSR

Same byte-mask merge as the immediate form, source comes from Rm. Tests
cover MSR CPSR_f with flag-only updates and MSR CPSR_fsxc writing the
full register (mode bits left at 0x1F so no bank swap is triggered)."
```

---

## Task 12: MSR SPSR (both forms)

**Purpose:** Exercise the `use_spsr == true` branch in `dispatch_psr_transfer` with real tests. The implementation already handles SPSR through `spsr_slot()` — this task confirms it works in both User (warn + no-op) and non-User (banked write) modes.

**Files:**
- Modify: `tests/unit/arm7_psr_transfer_test.cpp`

- [ ] **Step 1: Add MSR SPSR tests**

```cpp
    // Test 10: MSR SPSR_fc in System mode → warn, no change
    // System shares user register bank and has no SPSR storage.
    {
        NDS nds;
        nds.cpu7().state().switch_mode(Mode::System);
        nds.cpu7().state().banks.spsr_svc = 0xAAAAAAAAu;  // should stay untouched
        nds.cpu7().state().r[0] = 0x5A5A5A5Au;
        run_one(nds, kBase, encode_msr_reg(true, kMaskFC, 0));
        REQUIRE(nds.cpu7().state().banks.spsr_svc == 0xAAAAAAAAu);
    }

    // Test 11: MSR SPSR_f in IRQ mode updates the IRQ-bank SPSR flag byte only
    {
        NDS nds;
        nds.cpu7().state().switch_mode(Mode::Irq);
        nds.cpu7().state().banks.spsr_irq = 0x00000011u;  // mode byte only
        nds.cpu7().state().r[0] = 0xF0000000u;
        run_one(nds, kBase, encode_msr_reg(true, kMaskF, 0));
        REQUIRE(nds.cpu7().state().banks.spsr_irq == 0xF0000011u);
    }

    // Test 12: MSR SPSR immediate form in Supervisor mode
    {
        NDS nds;
        nds.cpu7().state().switch_mode(Mode::Supervisor);
        nds.cpu7().state().banks.spsr_svc = 0x00000013u;
        // imm8 = 0x80, rot = 0 → source = 0x00000080
        run_one(nds, kBase, encode_msr_imm(true, kMaskC, 0x80u, 0));
        REQUIRE(nds.cpu7().state().banks.spsr_svc == 0x00000080u);
    }
```

Update `std::puts` to `"arm7_psr_transfer_test: all 12 cases passed"`.

- [ ] **Step 2: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **21/21 tests pass**, `arm7_psr_transfer_test` reports 12 cases.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_psr_transfer_test.cpp
git commit -m "cpu/arm7: MSR SPSR routes through spsr_slot()

Writes to SPSR land in the correct per-mode banked slot (banks.spsr_irq
in IRQ, banks.spsr_svc in Supervisor, etc.). Writes in User/System
mode warn and no-op because spsr_slot() returns nullptr. Both immediate
and register forms share the same code path."
```

---

## Task 13: MSR CPSR_c mode switch with banking + T-bit + invalid-mode warns

**Purpose:** Wire `MSR CPSR_c` mode-bit changes through `Arm7State::switch_mode()` so R8–R14 and SPSR actually swap between banks. Also add the T-bit-change warn (games should use BX) and the invalid-mode warn (UNPREDICTABLE).

**Files:**
- Modify: `src/cpu/arm7/arm7_psr.cpp`
- Modify: `tests/unit/arm7_psr_transfer_test.cpp`

- [ ] **Step 1: Expand the MSR CPSR path in `arm7_psr.cpp`**

Replace the existing "MSR CPSR" final block (currently just `state.cpsr = (state.cpsr & ~byte_mask) | ...`) with:

```cpp
    // --- MSR CPSR ---
    // Order: compute new CPSR value first, then check for mode/T-bit changes,
    // then call switch_mode() (which saves current banks, loads target banks,
    // updates CPSR mode bits), then overwrite CPSR with the merged value so
    // the non-mode byte fields land last. After switch_mode(), CPSR's mode
    // bits match new_mode by construction — the final write is idempotent
    // on the mode byte.
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
                // Skip switch_mode() but still honor the byte write below,
                // so the mode bits reflect what the game requested.
            } else {
                state.switch_mode(new_mode);
            }
        }
    }

    state.cpsr = new_cpsr;
    return 1;
```

Remove the `(void)is_valid_mode;` line if it's still there — the function is called now.

- [ ] **Step 2: Add mode-switch / T-bit / invalid-mode tests**

```cpp
    // Test 13: MSR CPSR_c with mode change Supervisor → IRQ performs a real
    // bank swap. Start in SVC, stash R13 in svc bank via switch_mode, then
    // MSR to switch to IRQ — R13 should load from irq_r13_r14[0].
    {
        NDS nds;
        // Set up the IRQ bank's R13 before we enter IRQ mode, so the
        // post-switch load brings in a known value.
        nds.cpu7().state().banks.irq_r13_r14[0] = 0x1EEE1EEEu;
        nds.cpu7().state().switch_mode(Mode::Supervisor);
        nds.cpu7().state().r[13] = 0x5CC5CCC5u;  // SVC R13

        // MSR CPSR_c, #0x12 — switch to IRQ mode (0x12).
        run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x12u, 0));

        REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x12u);
        REQUIRE(nds.cpu7().state().r[13] == 0x1EEE1EEEu);
        // SVC R13 should have been saved to the SVC bank before the load.
        REQUIRE(nds.cpu7().state().banks.svc_r13_r14[0] == 0x5CC5CCC5u);
    }

    // Test 14: MSR CPSR_c with T bit flipped logs a warn. Start in ARM state
    // (T=0), write CPSR_c with T=1. The warn is a side channel; we verify
    // the write still went through.
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x0000001Fu;  // System, T clear
        // imm = 0x3F (T=1, mode=System). rot=0.
        run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x3Fu, 0));
        REQUIRE((nds.cpu7().state().cpsr & 0x0000003Fu) == 0x0000003Fu);  // T + System
    }

    // Test 15: MSR CPSR_c with invalid mode bits (0x00) logs a warn,
    // skips the bank swap, but still honors the byte write.
    {
        NDS nds;
        nds.cpu7().state().cpsr = 0x0000001Fu;
        run_one(nds, kBase, encode_msr_imm(false, kMaskC, 0x00u, 0));
        REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x00u);
    }

    // Test 16: The "disable interrupts around critical section" round-trip.
    // This is the single most important MRS/MSR test — every interrupt-
    // sensitive code path uses exactly this idiom. We load both instructions
    // into ARM7 WRAM as a mini-program and run it end-to-end through
    // run_until() rather than run_one().
    {
        NDS nds;
        nds.cpu7().state().switch_mode(Mode::System);
        // Three-instruction program:
        //   MRS R0, CPSR
        //   ORR R0, R0, #0x80    ; imm form DP, already verified by other tests
        //   MSR CPSR_c, R0
        const u32 orr_imm_0_0_80 = 0xE3800000u | (0u << 16) | (0u << 12) | 0x80u;
        nds.arm7_bus().write32(kBase + 0, encode_mrs(0, false));
        nds.arm7_bus().write32(kBase + 4, orr_imm_0_0_80);
        nds.arm7_bus().write32(kBase + 8, encode_msr_reg(false, kMaskC, 0));
        nds.cpu7().state().pc = kBase;
        nds.cpu7().state().r[15] = kBase + 8;
        nds.cpu7().run_until(6);  // 3 ARM7 cycles == 6 ARM9 cycles
        REQUIRE((nds.cpu7().state().cpsr & 0x80u) != 0);       // I bit set
        REQUIRE((nds.cpu7().state().cpsr & 0x1Fu) == 0x1Fu);   // System preserved
    }
```

Update `std::puts` to `"arm7_psr_transfer_test: all 16 cases passed"`.

- [ ] **Step 3: Build and verify**

Run: `cd build && make && ctest --output-on-failure`
Expected: **21/21 tests pass**, `arm7_psr_transfer_test` reports 16 cases.

- [ ] **Step 4: Commit**

```bash
git add src/cpu/arm7/arm7_psr.cpp tests/unit/arm7_psr_transfer_test.cpp
git commit -m "cpu/arm7: MSR CPSR_c mode switch actually banks registers

When MSR CPSR_c changes the mode bits to a valid mode, we now call
Arm7State::switch_mode() to save the current mode's R8-R14 into its
bank and load the target mode's from its bank. Invalid mode bit
patterns log a warn and skip the swap but still honor the byte write.
Separate warn for T-bit changes (games should use BX).

Also lands the round-trip 'disable interrupts around critical section'
test that every real game uses."
```

---

## Task 14: Capstone — end-to-end sequence test

**Purpose:** A single hand-written program that exercises reg-shift DP + MUL + SMULL + MRS + MSR all in sequence, catching inter-family state-clobbering bugs. Mirrors slice 3b1's `arm7_branch_load_sequence_test.cpp`.

**Files:**
- Create: `tests/unit/arm7_multiply_psr_sequence_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Create `tests/unit/arm7_multiply_psr_sequence_test.cpp`**

```cpp
// arm7_multiply_psr_sequence_test.cpp — end-to-end capstone for slice 3b2.
// Runs a small program that exercises register-shift DP, MUL, SMULL, MRS,
// and MSR in sequence. Catches inter-family bugs where each family works
// in isolation but corrupts a register another family is about to read.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 AL_COND = 0xEu << 28;
constexpr u32 kBase   = 0x0380'0000u;  // ARM7 WRAM

// Same encoders used by the per-family tests, inlined here for locality.
u32 mov_imm(u32 rd, u32 imm8) {
    // MOV Rd, #imm8 (rot=0) — imm form DP.
    return AL_COND | 0x03A00000u | ((rd & 0xFu) << 12) | (imm8 & 0xFFu);
}
u32 mov_neg_one(u32 rd) {
    // MVN Rd, #0 == 0xFFFFFFFF — imm form DP.
    return AL_COND | 0x03E00000u | ((rd & 0xFu) << 12);
}
u32 reg_shift_mov(u32 rd, u32 rm, u32 rs) {
    // MOV Rd, Rm, LSL Rs — reg-shift DP, opcode=MOV(0xD).
    u32 instr = AL_COND | (0xDu << 21);
    instr |= (rd & 0xFu) << 12;
    instr |= (rs & 0xFu) << 8;
    instr |= (1u << 4);  // reg-shift marker
    instr |= (rm & 0xFu);
    return instr;
}
u32 mul(u32 rd, u32 rm, u32 rs) {
    return AL_COND | 0x00000090u | ((rd & 0xFu) << 16)
                                   | ((rs & 0xFu) << 8)
                                   | (rm & 0xFu);
}
u32 smull(u32 rd_hi, u32 rd_lo, u32 rm, u32 rs) {
    return AL_COND | 0x00C00090u
        | ((rd_hi & 0xFu) << 16) | ((rd_lo & 0xFu) << 12)
        | ((rs & 0xFu) << 8) | (rm & 0xFu);
}
u32 mrs(u32 rd) {
    return AL_COND | 0x010F0000u | ((rd & 0xFu) << 12);
}
u32 msr_cpsr_c_reg(u32 rm) {
    return AL_COND | 0x0120F000u | 0x00010000u | (rm & 0xFu);
}
u32 orr_imm(u32 rd, u32 rn, u32 imm8) {
    return AL_COND | 0x03800000u
        | ((rn & 0xFu) << 16) | ((rd & 0xFu) << 12) | (imm8 & 0xFFu);
}
u32 bic_imm(u32 rd, u32 rn, u32 imm8) {
    return AL_COND | 0x03C00000u
        | ((rn & 0xFu) << 16) | ((rd & 0xFu) << 12) | (imm8 & 0xFFu);
}

}  // namespace

int main() {
    NDS nds;

    // Load test inputs and exercise every family. Program layout:
    //
    //   0: MOV  R0, #0x10          ; R0 = 0x10
    //   1: MOV  R1, #0x20          ; R1 = 0x20
    //   2: MOV  R2, #3             ; R2 = 3
    //   3: MOV  R2, R0, LSL R2     ; R2 = R0 << 3 == 0x80   (reg-shift DP)
    //   4: MUL  R3, R0, R1         ; R3 = 0x200            (short multiply)
    //   5: MVN  R5, #0             ; R5 = -1
    //   6: MUL  R5, R3, R5         ; R5 = -0x200
    //   7: MOV  R6, #2             ; R6 = 2
    //   8: SMULL R4, R7, R5, R6    ; R7:R4 = sign-ext(-0x400)
    //   9: MRS  R8, CPSR
    //  10: ORR  R9, R8, #0x80      ; set I bit
    //  11: MSR  CPSR_c, R9         ; I bit set in live CPSR
    //  12: MRS  R10, CPSR          ; should see I set
    //  13: BIC  R9, R10, #0x80     ; clear I
    //  14: MSR  CPSR_c, R9         ; I bit clear again

    const u32 program[] = {
        mov_imm(0, 0x10),
        mov_imm(1, 0x20),
        mov_imm(2, 3),
        reg_shift_mov(2, 0, 2),
        mul(3, 0, 1),
        mov_neg_one(5),
        mul(5, 3, 5),
        mov_imm(6, 2),
        smull(7, 4, 5, 6),
        mrs(8),
        orr_imm(9, 8, 0x80),
        msr_cpsr_c_reg(9),
        mrs(10),
        bic_imm(9, 10, 0x80),
        msr_cpsr_c_reg(9),
    };
    constexpr u32 kInstrCount = sizeof(program) / sizeof(program[0]);

    for (u32 i = 0; i < kInstrCount; ++i) {
        nds.arm7_bus().write32(kBase + i * 4u, program[i]);
    }

    nds.cpu7().state().pc = kBase;
    nds.cpu7().state().r[15] = kBase + 8;
    // Run for exactly kInstrCount ARM7 cycles (= kInstrCount * 2 ARM9 cycles).
    nds.cpu7().run_until(kInstrCount * 2);

    REQUIRE(nds.cpu7().state().r[0] == 0x10u);
    REQUIRE(nds.cpu7().state().r[1] == 0x20u);
    REQUIRE(nds.cpu7().state().r[2] == 0x80u);
    REQUIRE(nds.cpu7().state().r[3] == 0x200u);
    REQUIRE(nds.cpu7().state().r[5] == static_cast<u32>(-0x200));
    REQUIRE(nds.cpu7().state().r[6] == 2u);

    // SMULL(-0x200, 2) = -0x400, sign-extended to 64 bits: 0xFFFFFFFFFFFFFC00
    REQUIRE(nds.cpu7().state().r[4] == 0xFFFFFC00u);
    REQUIRE(nds.cpu7().state().r[7] == 0xFFFFFFFFu);

    // R10 (snapshot of CPSR after the ORR + MSR) should have the I bit set.
    REQUIRE((nds.cpu7().state().r[10] & 0x80u) != 0);

    // Final CPSR should have the I bit clear again after the BIC + MSR.
    REQUIRE((nds.cpu7().state().cpsr & 0x80u) == 0);

    std::puts("arm7_multiply_psr_sequence_test: capstone passed");
    return 0;
}
```

- [ ] **Step 2: Register the test binary**

In `tests/CMakeLists.txt`:

```cmake
add_ds_unit_test(arm7_multiply_psr_sequence_test)
```

- [ ] **Step 3: Build and verify**

Run: `cd build && cmake .. && make && ctest --output-on-failure`
Expected: **22/22 tests pass** (21 + 1 new capstone).

- [ ] **Step 4: Commit**

```bash
git add tests/unit/arm7_multiply_psr_sequence_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: end-to-end reg-shift/multiply/MRS/MSR sequence test

Capstone mirroring slice 3b1's arm7_branch_load_sequence_test. Runs a
16-instruction program that loads constants, shifts by a register
amount, does a 32-bit multiply, a signed long multiply, and the full
disable-interrupts round-trip. Exercises every new family against the
real dispatch tree in one test."
```

---

## Task 15: Final verification

**Purpose:** Clean-slate rebuild, full test pass, and verification of the architectural house rules from the parent CLAUDE.md.

**Files:** none.

- [ ] **Step 1: Clean rebuild**

Run:
```bash
rm -rf build && mkdir build && cd build && cmake .. && make 2>&1 | tee /tmp/slice3b2_build.log
```
Expected: build clean, **zero warnings**, zero errors.

- [ ] **Step 2: Full test suite**

Run: `cd build && ctest --output-on-failure`
Expected: **22/22 tests pass**.

- [ ] **Step 3: Verify architectural house rules**

Run each of these; each must produce empty output or succeed.

```bash
# No SDL include in the ARM7 core:
grep -R "SDL" src/cpu/arm7/ || true

# arm7_decode_internal.hpp not leaked outside src/cpu/arm7/:
grep -R "arm7_decode_internal.hpp" src/ | grep -v "src/cpu/arm7/" || true

# CLAUDE.md still untracked:
git ls-files | grep -x "CLAUDE.md"  # must print nothing

# No Co-Authored-By on any slice 3b2 commit:
git log main..HEAD --grep "Co-Authored" || true
```

Expected: each grep returns nothing (or only the `|| true` swallow). If `git ls-files | grep -x "CLAUDE.md"` prints a match, STOP and fix the tracking state before continuing.

- [ ] **Step 4: Fresh `NDS` smoke check**

The `nds_integration_test` binary (if present) already exercises `NDS nds; nds.run_frame();`. Verify it passed in Step 2. No further action needed.

- [ ] **Step 5: Final commit (only if anything changed above)**

If Step 3 surfaced a fix, commit it. Otherwise, nothing to commit — the slice is done as of Task 14.

---

## Deferred to slice 3b3 (tracked for continuity)

- `LDRH`, `STRH`, `LDRSB`, `LDRSH` — halfword and signed single data transfer
- Their encoding subspace (bits[27:25] == 000 + bits[7:4] == 1xx1 extension pattern)

## Deferred to slice 3b4

- `LDM`, `STM` — load/store multiple with register-list, base-writeback, base-in-list, and user-mode register access

## Deferred to slice 3c

- Full Thumb ISA (16-bit instructions)
- `step_thumb` dispatcher
- Removal of the `cpsr.T == 0` assert in `step_arm`

## Deferred to slice 3d

- Exception entry (reset, undefined, SWI, prefetch/data abort, IRQ, FIQ)
- IRQ line sampling in the fetch loop
- `MOVS PC, LR` / `SUBS PC, LR, #N` return-from-exception (SPSR→CPSR copy on PC write with S=1)

## Deferred to a dedicated cycle-accuracy slice

- Per-instruction cycle counts across all slices (3a, 3b1, 3b2)
- `MUL`/`MLA`'s Rs-dependent early-termination table
- Reg-shift DP's `1S + 1I` cost
- Branch pipeline-flush penalty
- Load/store access timing
