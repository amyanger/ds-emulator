# ARM7 Core — Phase 1, Slice 3b1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the ARM7 interpreter with branches (`B`, `BL`, `BX`) and single-register word/byte loads and stores (`LDR`, `STR`, `LDRB`, `STRB`) in all standard addressing modes, plus a decoder refactor that splits instruction families into their own translation units.

**Architecture:** Split the current monolithic `arm7_decode.cpp` into four files: a slim top-level dispatcher (`arm7_decode.cpp`), a data-processing handler (`arm7_dp.cpp`, also catches `BX` since it lives in DP encoding space), a branch handler (`arm7_branch.cpp`, B/BL), and a single-data-transfer handler (`arm7_loadstore.cpp`). A new private header `arm7_decode_internal.hpp` exposes the family dispatch helpers and the shared `write_rd` register-writeback helper. The existing `state.pc = instr_addr + 4` + `state.r[15] = instr_addr + 8` pre-dispatch model already models ARMv4T's 3-stage pipeline, so branches and LDR-into-R15 work correctly with no new plumbing. Every instruction still consumes 1 ARM7 cycle — real cycle counts stay deferred.

**Tech Stack:** C++20, CMake, CTest, plain test binaries under `tests/unit/` using the existing `REQUIRE` macro from `tests/support/require.hpp`.

**Spec:** `docs/superpowers/specs/2026-04-14-arm7-core-phase1-slice3b1-design.md`

---

## File structure

### Files created

- `src/cpu/arm7/arm7_decode_internal.hpp` — private header shared between `arm7_decode.cpp`, `arm7_dp.cpp`, `arm7_branch.cpp`, and `arm7_loadstore.cpp`. Exposes `write_rd` and the `dispatch_*` function declarations.
- `src/cpu/arm7/arm7_dp.cpp` — data-processing handler. Contains the slice 3a DP logic moved verbatim, with an added `BX` pattern recognizer at the top.
- `src/cpu/arm7/arm7_branch.cpp` — `B`/`BL` handler.
- `src/cpu/arm7/arm7_loadstore.cpp` — single-data-transfer handler (`LDR`/`STR`/`LDRB`/`STRB`).
- `tests/unit/arm7_branch_test.cpp` — branch instruction tests.
- `tests/unit/arm7_load_store_test.cpp` — LDR/STR/LDRB/STRB tests.
- `tests/unit/arm7_branch_load_sequence_test.cpp` — end-to-end capstone test.

### Files modified

- `src/cpu/arm7/arm7_decode.cpp` — shrinks. Owns `step_arm()` + top-level `dispatch_arm()`. DP logic moves out.
- `src/CMakeLists.txt` — add `arm7_dp.cpp`, `arm7_branch.cpp`, `arm7_loadstore.cpp` to `ds_core`.
- `tests/CMakeLists.txt` — register the three new test binaries.

---

## Task 1: Decoder refactor — pure move of DP into its own file

**Purpose:** Break the dispatcher into family files with zero behavior change. This is the riskiest single commit because it touches existing code, so it lands on its own and `ctest` is green at the end. No new opcodes are implemented in this task.

**Files:**
- Create: `src/cpu/arm7/arm7_decode_internal.hpp`
- Create: `src/cpu/arm7/arm7_dp.cpp`
- Modify: `src/cpu/arm7/arm7_decode.cpp`
- Modify: `src/CMakeLists.txt`

- [ ] **Step 1: Verify baseline is green**

Run: `cd build && cmake .. && make && ctest --output-on-failure`
Expected: build clean, 15/15 tests pass.

- [ ] **Step 2: Create `arm7_decode_internal.hpp`**

```cpp
#pragma once

// Private header shared between the ARM7 decoder translation units
// (arm7_decode.cpp, arm7_dp.cpp, arm7_branch.cpp, arm7_loadstore.cpp).
// NOT part of any public include path. Only files under src/cpu/arm7/
// may include it.

#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7Bus;

// Single register-file writeback point used by every family handler.
// Writing to R15 also stomps state.pc so the next fetch lands on the
// target; callers relying on the pipeline model (state.r[15] ==
// instr_addr + 8) must read R15 BEFORE calling this.
inline void write_rd(Arm7State& s, u32 rd, u32 value) {
    s.r[rd] = value;
    if (rd == 15) {
        s.pc = value & ~0x3u;
    }
}

// Family dispatch helpers. Each returns the number of ARM7 cycles the
// instruction consumed. In slice 3b1 every path returns 1.
u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr);
u32 dispatch_branch(Arm7State& state, u32 instr);
u32 dispatch_single_data_transfer(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr);

}  // namespace ds
```

- [ ] **Step 3: Create `arm7_dp.cpp` as a verbatim move of the slice 3a DP logic**

```cpp
// arm7_dp.cpp — ARMv4T data-processing dispatch. This file owns
// dispatch_dp(), which handles the whole bits[27:26] == 00 encoding
// space. That space also contains MUL/MLA, halfword LDR/STR, MRS/MSR,
// and BX. Slice 3b1 recognizes BX at the top of this file; the rest
// remain deferred and fall through to a warn path.

#include "cpu/arm7/arm7_decode_internal.hpp"
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

}  // namespace

u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr) {
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

    // Compute operand2 first so we can early-out on the unimplemented
    // immediate-shift form before paying for a register-file lookup.
    ShifterResult op2;
    if (i_bit) {
        const u32 imm8   = instr & 0xFFu;
        const u32 rotate = (instr >> 8) & 0xFu;
        const bool c_in  = (state.cpsr & (1u << 29)) != 0;
        op2 = rotated_imm(imm8, rotate, c_in);
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

    const u32  opcode = (instr >> 21) & 0xFu;
    const bool s_flag = ((instr >> 20) & 1u) != 0;
    const u32  rn     = (instr >> 16) & 0xFu;
    const u32  rd     = (instr >> 12) & 0xFu;

    const u32 rn_val = state.r[rn];

    u32  result       = 0;
    bool logical_form = true;
    bool writeback    = true;
    AddResult ar { 0, false, false };

    switch (static_cast<DpOp>(opcode)) {
        case DpOp::AND:
            result = rn_val & op2.value;
            break;
        case DpOp::EOR:
            result = rn_val ^ op2.value;
            break;
        case DpOp::SUB:
            ar = sbc(rn_val, op2.value, true);
            result = ar.value;
            logical_form = false;
            break;
        case DpOp::RSB:
            ar = sbc(op2.value, rn_val, true);
            result = ar.value;
            logical_form = false;
            break;
        case DpOp::ADD:
            ar = adc(rn_val, op2.value, false);
            result = ar.value;
            logical_form = false;
            break;
        case DpOp::ADC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            ar = adc(rn_val, op2.value, c);
            result = ar.value;
            logical_form = false;
            break;
        }
        case DpOp::SBC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            ar = sbc(rn_val, op2.value, c);
            result = ar.value;
            logical_form = false;
            break;
        }
        case DpOp::RSC: {
            const bool c = (state.cpsr & (1u << 29)) != 0;
            ar = sbc(op2.value, rn_val, c);
            result = ar.value;
            logical_form = false;
            break;
        }
        case DpOp::ORR:
            result = rn_val | op2.value;
            break;
        case DpOp::MOV:
            result = op2.value;
            break;
        case DpOp::BIC:
            result = rn_val & ~op2.value;
            break;
        case DpOp::MVN:
            result = ~op2.value;
            break;
        case DpOp::TST:
            result = rn_val & op2.value;
            writeback = false;
            break;
        case DpOp::TEQ:
            result = rn_val ^ op2.value;
            writeback = false;
            break;
        case DpOp::CMP:
            ar = sbc(rn_val, op2.value, true);
            result = ar.value;
            logical_form = false;
            writeback = false;
            break;
        case DpOp::CMN:
            ar = adc(rn_val, op2.value, false);
            result = ar.value;
            logical_form = false;
            writeback = false;
            break;
    }

    if (writeback) {
        write_rd(state, rd, result);
    }

    if (s_flag) {
        if (rd == 15 && writeback) {
            DS_LOG_WARN("arm7: S-flag set with Rd=R15 at 0x%08X", instr_addr);
        } else {
            state.cpsr = set_nz(state.cpsr, result);
            if (logical_form) {
                state.cpsr = set_c(state.cpsr, op2.carry);
            } else {
                state.cpsr = set_c(state.cpsr, ar.carry);
                state.cpsr = set_v(state.cpsr, ar.overflow);
            }
        }
    }

    return 1;
}

}  // namespace ds
```

- [ ] **Step 4: Replace `arm7_decode.cpp` with the slim top-level dispatcher**

Overwrite `src/cpu/arm7/arm7_decode.cpp` entirely:

```cpp
// arm7_decode.cpp — top-level ARMv4T fetch / decode / dispatch loop.
// dispatch_arm() is a small switch on bits 27..25 that delegates to a
// family handler (DP, branch, LDR/STR). Each handler lives in its own
// translation unit; see arm7_decode_internal.hpp for the contract.
//
// Cycle cost model: every executed (non-skipped) instruction costs 1
// ARM7 cycle, same as slice 3a. Real per-instruction cycle counts are
// deferred to a later slice.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"

#include <cassert>

namespace ds {

namespace {

u32 dispatch_arm(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    const u32 cond = instr >> 28;
    if (!eval_condition(cond, state.cpsr)) {
        return 1;  // condition-skipped instructions still consume 1 cycle
    }

    const u32 bits_27_25 = (instr >> 25) & 0x7u;
    switch (bits_27_25) {
        case 0b000:
        case 0b001:
            return dispatch_dp(state, instr, instr_addr);
        case 0b010:
        case 0b011:
            return dispatch_single_data_transfer(state, bus, instr, instr_addr);
        case 0b100:
            // LDM / STM — deferred to slice 3b2.
            DS_LOG_WARN("arm7: LDM/STM form 0x%08X at 0x%08X", instr, instr_addr);
            return 1;
        case 0b101:
            return dispatch_branch(state, instr);
        case 0b110:
        case 0b111:
            // Coproc / SWI — deferred to slice 3d.
            DS_LOG_WARN("arm7: coproc/SWI form 0x%08X at 0x%08X", instr, instr_addr);
            return 1;
        default:
            return 1;  // unreachable
    }
}

}  // namespace

void Arm7::step_arm() {
    assert((state_.pc & 0x3u) == 0 && "Arm7::step_arm: ARM pc must be 4-aligned");
    assert((state_.cpsr & (1u << 5)) == 0 && "Arm7::step_arm: Thumb mode not implemented");

    const u32 instr_addr = state_.pc;
    const u32 instr      = bus_->read32(instr_addr);

    // R15 during execute reads as instruction_addr + 8 (3-stage pipeline).
    // Advance pc BEFORE dispatch so branch handlers can stomp it freely
    // without us overwriting their target afterwards.
    state_.r[15] = instr_addr + 8;
    state_.pc    = instr_addr + 4;

    const u32 cycles_consumed = dispatch_arm(state_, *bus_, instr, instr_addr);
    assert(cycles_consumed > 0 && "Arm7::step_arm: dispatch must consume >= 1 cycle");
    state_.cycles += cycles_consumed;
}

}  // namespace ds
```

Note: the Thumb-assert line is added here in Task 1 (not Task 4) because it is a no-op for every existing test — none of them set cpsr.T — and placing it now means the refactor commit also fails safe if anything later accidentally flips T.

- [ ] **Step 5: Add the new files to the `ds_core` CMake target**

Modify `src/CMakeLists.txt`:

```cmake
# ds_core — platform-free static library. Zero SDL2 dependency.
# Anything under src/frontend/ is NOT in this target; see the subdir below.
add_library(ds_core STATIC
    nds.cpp
    bus/arm9_bus.cpp
    bus/arm7_bus.cpp
    scheduler/scheduler.cpp
    cpu/arm9/arm9.cpp
    cpu/arm7/arm7.cpp
    cpu/arm7/arm7_decode.cpp
    cpu/arm7/arm7_dp.cpp
)
```

(Later tasks will append `arm7_branch.cpp` and `arm7_loadstore.cpp`.)

- [ ] **Step 6: Provide stub family handlers for `branch` and `single_data_transfer`**

Until Task 2 creates `arm7_branch.cpp` and Task 5 creates `arm7_loadstore.cpp`, the linker needs symbols for `dispatch_branch` and `dispatch_single_data_transfer`. Provide them in `arm7_dp.cpp` at the bottom of the `ds` namespace block as stubs — they will be MOVED OUT in later tasks, not deleted.

Append to `src/cpu/arm7/arm7_dp.cpp` just before the closing `}  // namespace ds`:

```cpp
// ---- temporary stubs for tasks 2+ ----
// These are replaced with real implementations in Task 3 (branch) and
// Task 5 (load/store). Kept here only to make the decoder refactor a
// self-contained green-tests commit.

u32 dispatch_branch(Arm7State& /*state*/, u32 instr) {
    DS_LOG_WARN("arm7: branch form 0x%08X (stub)", instr);
    return 1;
}

u32 dispatch_single_data_transfer(Arm7State& /*state*/, Arm7Bus& /*bus*/,
                                  u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm7: single-data-transfer form 0x%08X at 0x%08X (stub)",
                instr, instr_addr);
    return 1;
}
```

- [ ] **Step 7: Run build + tests, verify 15/15 pass**

Run: `cd build && cmake .. && make && ctest --output-on-failure`
Expected: build clean, **15/15 tests pass** (same as baseline). No behavior has changed — only file layout.

- [ ] **Step 8: Commit the refactor**

```bash
git add src/cpu/arm7/arm7_decode_internal.hpp \
        src/cpu/arm7/arm7_dp.cpp \
        src/cpu/arm7/arm7_decode.cpp \
        src/CMakeLists.txt
git commit -m "cpu/arm7: split decoder into per-family translation units

Decoder refactor with zero behavior change: dispatch_dp moves to
arm7_dp.cpp, branch and load/store get stub dispatchers, arm7_decode.cpp
shrinks to step_arm + a bits[27:25] switch. Prepares for slice 3b1
branches and LDR/STR landing in their own files. Also adds the cpsr.T
'Thumb not implemented' assert at the top of step_arm so a stray BX
into Thumb (future task) fails loudly at the fetch site."
```

---

## Task 2: Create `arm7_branch.cpp` (still stubbed) and move the branch stub out of `arm7_dp.cpp`

**Purpose:** Give `dispatch_branch` its permanent home so Task 3 only has to fill in the real implementation, not also move files. Same for load/store in Task 5.

**Files:**
- Create: `src/cpu/arm7/arm7_branch.cpp`
- Create: `src/cpu/arm7/arm7_loadstore.cpp`
- Modify: `src/cpu/arm7/arm7_dp.cpp` (remove temporary stubs from Task 1)
- Modify: `src/CMakeLists.txt` (add both new files)

- [ ] **Step 1: Create `arm7_branch.cpp` as a still-warning stub**

```cpp
// arm7_branch.cpp — ARMv4T branch dispatch. Slice 3b1 implements B, BL,
// and BX (the latter is actually caught inside arm7_dp.cpp because BX
// lives in DP encoding space; this file only handles bits[27:25] == 101).

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_branch(Arm7State& /*state*/, u32 instr) {
    DS_LOG_WARN("arm7: branch form 0x%08X (stub)", instr);
    return 1;
}

}  // namespace ds
```

- [ ] **Step 2: Create `arm7_loadstore.cpp` as a still-warning stub**

```cpp
// arm7_loadstore.cpp — ARMv4T single-data-transfer dispatch (LDR/STR
// and their byte variants). Halfword/signed forms live in DP encoding
// space and stay deferred to slice 3b2.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_single_data_transfer(Arm7State& /*state*/, Arm7Bus& /*bus*/,
                                  u32 instr, u32 instr_addr) {
    DS_LOG_WARN("arm7: single-data-transfer form 0x%08X at 0x%08X (stub)",
                instr, instr_addr);
    return 1;
}

}  // namespace ds
```

- [ ] **Step 3: Remove the temporary stubs from `arm7_dp.cpp`**

Delete the "temporary stubs for tasks 2+" block that was added in Task 1 Step 6 from `src/cpu/arm7/arm7_dp.cpp`. The file should now end at the closing brace of `dispatch_dp`'s `namespace ds`.

- [ ] **Step 4: Wire the new files into `ds_core`**

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
    cpu/arm7/arm7_dp.cpp
    cpu/arm7/arm7_branch.cpp
    cpu/arm7/arm7_loadstore.cpp
)
```

- [ ] **Step 5: Run build + tests**

Run: `cd build && cmake .. && make && ctest --output-on-failure`
Expected: build clean, 15/15 tests pass. Still no behavior change from the baseline.

- [ ] **Step 6: Commit**

```bash
git add src/cpu/arm7/arm7_branch.cpp \
        src/cpu/arm7/arm7_loadstore.cpp \
        src/cpu/arm7/arm7_dp.cpp \
        src/CMakeLists.txt
git commit -m "cpu/arm7: give branch and load/store dispatchers their own TUs

Stub implementations moved out of arm7_dp.cpp into arm7_branch.cpp and
arm7_loadstore.cpp. Still no new opcodes — this keeps task 3 (branches)
and task 5 (loads/stores) a pure implementation change."
```

---

## Task 3: Implement `B` and `BL` with a dedicated test

**Files:**
- Create: `tests/unit/arm7_branch_test.cpp`
- Modify: `src/cpu/arm7/arm7_branch.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/unit/arm7_branch_test.cpp`:

```cpp
// ARM7 slice 3b1: B, BL, and BX tests. BX is still stubbed in this
// task; BX-specific test cases land in Task 4 when the BX recognizer
// is added to arm7_dp.cpp.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;

// Load a program into ARM7 WRAM at kBase, point PC there, and run
// exactly `steps` instructions (each costs 1 ARM7 cycle in slice 3b1).
void load_and_run(NDS& nds, std::initializer_list<u32> program, u32 steps) {
    u32 i = 0;
    for (u32 word : program) {
        nds.arm7_bus().write32(kBase + i * 4, word);
        ++i;
    }
    nds.cpu7().state().pc = kBase;
    nds.cpu7().run_until(static_cast<Cycle>(steps) * 2);
}

}  // namespace

static void b_forward_takes_branch() {
    NDS nds;
    // B +8 (to kBase + 8 + 8 = kBase + 0x10, three slots ahead in words).
    // Encoding: cond=AL (E), 101, L=0, offset24.
    // target = pc + 8 + (offset24 << 2). For target = kBase + 16 and
    // pc = kBase, we need offset24 = (16 - 8) / 4 = 2. Instruction:
    //   EA 00 00 02 == 0xEA000002
    load_and_run(nds, { 0xEA00'0002u }, 1);
    REQUIRE(nds.cpu7().state().pc == kBase + 16u);
}

static void b_backward_takes_branch() {
    NDS nds;
    // Put a MOV R0,#1 at kBase + 16, then a B -16 at kBase + 20 that
    // goes back to kBase + 16. After 3 steps we should be past the MOV
    // the second time, with R0 == 1 and pc == kBase + 20 (the branch
    // instruction again) or kBase + 24 if we execute it once more.
    //
    // Simpler: run exactly two steps — the MOV, then the B. After the
    // B, pc must equal kBase + 16.
    // Layout:
    //   kBase + 0x00: MOV R0, #0x55   ; E3A0'0055
    //   kBase + 0x04: B   -4          ; pc+8+(offset<<2) = 0x04+8-16 = 0xFFFFFFF4
    //                                   we want target = kBase + 0x00, so
    //                                   offset24 = ((kBase+0)-(kBase+4+8))/4
    //                                            = -12/4 = -3 = 0xFFFFFD
    //                                   encoded as EAFF'FFFD
    load_and_run(nds, { 0xE3A0'0055u, 0xEAFF'FFFDu }, 2);
    REQUIRE(nds.cpu7().state().r[0] == 0x55u);
    REQUIRE(nds.cpu7().state().pc == kBase + 0u);
}

static void b_condition_fail_does_not_branch() {
    NDS nds;
    // MOVS R0, #0      ; sets Z=1
    // BNE  +8          ; Z=1 -> skipped; pc should advance to kBase + 8
    load_and_run(nds, { 0xE3B0'0000u, 0x1A00'0002u }, 2);
    REQUIRE(nds.cpu7().state().pc == kBase + 8u);
}

static void b_max_negative_offset_wraps() {
    NDS nds;
    // B with offset24 = 0x800000 (sign-bit set, most negative).
    // target = pc + 8 + (sign_extend(0x800000) << 2)
    //        = kBase + 8 + 0xFE000000  (after sign-extend + shift)
    //        = kBase + 0xFE000008 (mod 2^32)
    // Encoding: EA80'0000.
    //
    // We don't execute anything at the target — we only verify the pc
    // value after the branch is computed correctly. The next step()
    // would try to fetch from unmapped memory; we run exactly one step
    // and stop.
    load_and_run(nds, { 0xEA80'0000u }, 1);
    const u32 expected_pc = (kBase + 8u + 0xFE00'0000u) & ~0x3u;
    REQUIRE(nds.cpu7().state().pc == expected_pc);
}

static void bl_links_return_address_and_jumps() {
    NDS nds;
    // BL +8     ; r14 should become kBase + 4, pc should become kBase + 16
    // Encoding: cond=AL, 101, L=1, offset24=2. → 0xEB00'0002
    load_and_run(nds, { 0xEB00'0002u }, 1);
    REQUIRE(nds.cpu7().state().r[14] == kBase + 4u);
    REQUIRE(nds.cpu7().state().pc    == kBase + 16u);
}

static void bl_then_mov_pc_lr_returns() {
    NDS nds;
    // kBase + 0x00: BL +4           ; EB00'0001 (target = kBase + 8 + 4 = kBase + 12)
    // kBase + 0x04: MOV R1, #0x11   ; E3A0'1011 — should be skipped by the BL
    // kBase + 0x08: <unused hole>
    // kBase + 0x0C: MOV PC, LR      ; E1A0'F00E  (Rd=15, Rm=14, MOV)
    // After 2 steps (BL, then MOV PC,LR) pc should be kBase + 4.
    load_and_run(nds,
                 { 0xEB00'0001u,   // BL +4
                   0xE3A0'1011u,   // MOV R1, #0x11 (skipped)
                   0x0000'0000u,   // hole
                   0xE1A0'F00Eu }, // MOV PC, LR
                 2);
    REQUIRE(nds.cpu7().state().r[14] == kBase + 4u);
    REQUIRE(nds.cpu7().state().r[1]  == 0u);           // MOV R1 was skipped
    REQUIRE(nds.cpu7().state().pc    == kBase + 4u);
}

int main() {
    b_forward_takes_branch();
    b_backward_takes_branch();
    b_condition_fail_does_not_branch();
    b_max_negative_offset_wraps();
    bl_links_return_address_and_jumps();
    bl_then_mov_pc_lr_returns();
    std::puts("arm7_branch_test OK");
    return 0;
}
```

- [ ] **Step 2: Register the test in CMake**

Modify `tests/CMakeLists.txt` — append:

```cmake
add_ds_unit_test(arm7_branch_test)
```

- [ ] **Step 3: Run the test and verify it fails**

Run: `cd build && cmake .. && make arm7_branch_test && ctest -R arm7_branch_test --output-on-failure`
Expected: FAIL (`dispatch_branch` is still a stub, so pc is never updated — first assertion fails on `pc == kBase + 16`).

- [ ] **Step 4: Implement `B` and `BL` in `arm7_branch.cpp`**

Replace the stub body of `dispatch_branch` in `src/cpu/arm7/arm7_branch.cpp`:

```cpp
// arm7_branch.cpp — ARMv4T branch dispatch. Handles B and BL.
// BX lives in DP encoding space and is caught inside arm7_dp.cpp.

#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_branch(Arm7State& state, u32 instr) {
    // B / BL encoding: cond 101 L offset24
    // offset24 is a 24-bit signed word displacement from (pc + 8).
    const u32  offset24 = instr & 0x00FF'FFFFu;
    // Sign-extend 24 bits → 32 bits, then shift left by 2 for a byte offset.
    const i32  signed_off = static_cast<i32>(offset24 << 8) >> 8;  // arith shift
    const u32  byte_off   = static_cast<u32>(signed_off) << 2;
    const u32  target     = state.r[15] + byte_off;  // r[15] == instr_addr + 8

    const bool link = ((instr >> 24) & 1u) != 0;
    if (link) {
        // LR = address of the instruction after the BL.
        // state.pc was set to instr_addr + 4 by step_arm BEFORE dispatch,
        // which is exactly the address we want.
        state.r[14] = state.pc;
    }
    write_rd(state, 15, target);
    return 1;
}

}  // namespace ds
```

- [ ] **Step 5: Run the test and verify it passes**

Run: `cd build && make arm7_branch_test && ctest -R arm7_branch_test --output-on-failure`
Expected: PASS.

- [ ] **Step 6: Run the full suite**

Run: `cd build && ctest --output-on-failure`
Expected: 16/16 pass (15 existing + new branch test).

- [ ] **Step 7: Commit**

```bash
git add src/cpu/arm7/arm7_branch.cpp \
        tests/unit/arm7_branch_test.cpp \
        tests/CMakeLists.txt
git commit -m "cpu/arm7: B / BL with forward, backward, condition, and BL-return tests

Implements the 25-bit signed offset decoding and PC writeback via the
shared write_rd helper. BL links state.pc (which is instr_addr + 4 at
dispatch time) into R14. Covers forward branch, backward branch, a
max-negative 24-bit offset, condition-failed branches, and a BL + MOV
PC, LR round trip."
```

---

## Task 4: `BX` in `arm7_dp.cpp` + Thumb-target T-flag test

**Files:**
- Modify: `src/cpu/arm7/arm7_dp.cpp` (add BX recognizer)
- Modify: `tests/unit/arm7_branch_test.cpp` (add BX tests)

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/arm7_branch_test.cpp` (above `int main`):

```cpp
static void bx_arm_target_jumps_and_clears_t() {
    NDS nds;
    // MOV R2, #(kBase + 0x10)   ; we'll synthesize this via two instructions:
    //                             kBase + 0x10 is 0x0380'0010. Encoding a
    //                             32-bit immediate in a MOV isn't possible
    //                             in one instruction, so we use the register
    //                             file directly and set R2 from the test.
    // BX R2                      ; E12F'FF12
    nds.arm7_bus().write32(kBase + 0, 0xE12F'FF12u);  // BX R2
    nds.cpu7().state().r[2] = kBase + 0x10u;
    nds.cpu7().state().pc   = kBase;
    nds.cpu7().run_until(2);  // exactly 1 ARM7 instruction

    REQUIRE(nds.cpu7().state().pc == kBase + 0x10u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) == 0u);  // T clear
}

static void bx_thumb_target_sets_t_and_masks_bit0() {
    NDS nds;
    // BX R3 with R3 = kBase + 0x21 → pc should become kBase + 0x20
    // and T should be set. We do NOT execute another step after this;
    // the next step_arm would trip the "Thumb not implemented" assert.
    nds.arm7_bus().write32(kBase + 0, 0xE12F'FF13u);  // BX R3
    nds.cpu7().state().r[3] = kBase + 0x21u;
    nds.cpu7().state().pc   = kBase;
    nds.cpu7().run_until(2);

    REQUIRE(nds.cpu7().state().pc == kBase + 0x20u);
    REQUIRE((nds.cpu7().state().cpsr & (1u << 5)) != 0u);  // T set
}
```

And call both functions from `main()`:

```cpp
int main() {
    b_forward_takes_branch();
    b_backward_takes_branch();
    b_condition_fail_does_not_branch();
    b_max_negative_offset_wraps();
    bl_links_return_address_and_jumps();
    bl_then_mov_pc_lr_returns();
    bx_arm_target_jumps_and_clears_t();
    bx_thumb_target_sets_t_and_masks_bit0();
    std::puts("arm7_branch_test OK");
    return 0;
}
```

- [ ] **Step 2: Run the test and verify it fails**

Run: `cd build && make arm7_branch_test && ctest -R arm7_branch_test --output-on-failure`
Expected: FAIL on `bx_arm_target_jumps_and_clears_t` — BX is still dispatched as "register-shifted DP" which just warns and returns.

- [ ] **Step 3: Add the BX recognizer at the top of `dispatch_dp`**

Modify `src/cpu/arm7/arm7_dp.cpp`. Insert the BX recognizer as the very first check inside `dispatch_dp`, before the existing `reg_shift` bail-out:

```cpp
u32 dispatch_dp(Arm7State& state, u32 instr, u32 instr_addr) {
    // BX lives in DP encoding space: cond 0001 0010 1111 1111 1111 0001 Rm.
    // Mask: 0x0FFFFFF0, match: 0x012FFF10 (with the condition field masked).
    if ((instr & 0x0FFF'FFF0u) == 0x012F'FF10u) {
        const u32  rm      = instr & 0xFu;
        const u32  rm_val  = state.r[rm];          // rm==15 reads instr_addr+8
        const bool thumb   = (rm_val & 0x1u) != 0;
        const u32  target  = thumb ? (rm_val & ~0x1u) : (rm_val & ~0x3u);
        if (thumb) {
            state.cpsr |= (1u << 5);               // set T flag
        }
        write_rd(state, 15, target);
        (void)instr_addr;
        return 1;
    }

    // Bit 4 of the instruction, when bit 25 (I) is 0, distinguishes
    // immediate-shift (bit4=0) from register-shift (bit4=1) operand2.
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool reg_shift = !i_bit && ((instr >> 4) & 1u) != 0;
    if (reg_shift) {
        DS_LOG_WARN("arm7: register-shift dp form 0x%08X at 0x%08X",
                    instr, instr_addr);
        return 1;
    }
    // ... rest of dispatch_dp unchanged ...
```

Note: `write_rd` masks the target with `~0x3u` on R15 writes, which is correct for the ARM case but would over-mask a Thumb target. The BX block bypasses that by masking the target before calling `write_rd` — and the `write_rd` mask of `~0x3u` applied on top of an already-masked value is a no-op for the ARM case and a `& ~0x2u` no-op for the Thumb case (because we already cleared bit 0, and bit 1 is always 0 for valid Thumb instruction addresses). The `state.pc` written by `write_rd` therefore matches the target exactly.

- [ ] **Step 4: Run the test and verify it passes**

Run: `cd build && make arm7_branch_test && ctest -R arm7_branch_test --output-on-failure`
Expected: PASS.

- [ ] **Step 5: Run the full suite**

Run: `cd build && ctest --output-on-failure`
Expected: 16/16 pass.

- [ ] **Step 6: Commit**

```bash
git add src/cpu/arm7/arm7_dp.cpp tests/unit/arm7_branch_test.cpp
git commit -m "cpu/arm7: BX with ARM target + Thumb T-flag set

Recognizes the BX encoding inside dispatch_dp before the generic
register-shift bail-out. ARM targets clear T and mask PC with ~0x3; a
Thumb target (low bit set) sets cpsr.T and masks PC with ~0x1. The next
fetch after a Thumb BX will trip the Thumb-mode assert in step_arm,
which is the intended failure mode until slice 3c lands the Thumb
dispatcher."
```

---

## Task 5: `LDR` and `STR` word, immediate offset, U=1, P=1, W=0

**Purpose:** Land the simplest LDR/STR path first — aligned word access, positive immediate offset, no writeback, no byte mode, no register offset. Every later task in the plan extends this in one dimension at a time.

**Files:**
- Create: `tests/unit/arm7_load_store_test.cpp`
- Modify: `src/cpu/arm7/arm7_loadstore.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/unit/arm7_load_store_test.cpp`:

```cpp
// ARM7 slice 3b1: LDR / STR / LDRB / STRB tests. Each test drops a
// single encoded instruction into ARM7 WRAM at kBase, seeds the state
// it needs, and runs one step. Later tasks extend this file one
// addressing-mode dimension at a time.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

namespace {

constexpr u32 kBase = 0x0380'0000u;
// Data scratch area: far enough from the code block that stores into
// it cannot clobber the program.
constexpr u32 kData = 0x0380'0200u;

void run_one(NDS& nds, u32 instr) {
    nds.arm7_bus().write32(kBase, instr);
    nds.cpu7().state().pc = kBase;
    nds.cpu7().run_until(2);  // 1 ARM7 cycle = 2 ARM9 cycles
}

}  // namespace

static void ldr_word_imm_offset_loads_value() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 4, 0xCAFE'BABEu);

    // LDR R0, [R1, #4]
    // cond=AL, 01, I=0, P=1, U=1, B=0, W=0, L=1, Rn=1, Rd=0, imm12=4
    // → 1110 0101 1001 0001 0000 0000 0000 0100 = 0xE591'0004
    run_one(nds, 0xE591'0004u);

    REQUIRE(nds.cpu7().state().r[0] == 0xCAFE'BABEu);
    REQUIRE(nds.cpu7().state().r[1] == kData);  // no writeback
}

static void str_word_imm_offset_stores_value() {
    NDS nds;
    nds.cpu7().state().r[0] = 0x1234'5678u;
    nds.cpu7().state().r[1] = kData;

    // STR R0, [R1, #8]
    // → 1110 0101 1000 0001 0000 0000 0000 1000 = 0xE581'0008
    run_one(nds, 0xE581'0008u);

    REQUIRE(nds.arm7_bus().read32(kData + 8) == 0x1234'5678u);
    REQUIRE(nds.cpu7().state().r[0] == 0x1234'5678u);
    REQUIRE(nds.cpu7().state().r[1] == kData);
}

int main() {
    ldr_word_imm_offset_loads_value();
    str_word_imm_offset_stores_value();
    std::puts("arm7_load_store_test OK");
    return 0;
}
```

- [ ] **Step 2: Register the test in CMake**

Modify `tests/CMakeLists.txt` — append:

```cmake
add_ds_unit_test(arm7_load_store_test)
```

- [ ] **Step 3: Run the test and verify it fails**

Run: `cd build && cmake .. && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: FAIL — `dispatch_single_data_transfer` is still a stub, so R0 is not loaded.

- [ ] **Step 4: Implement the minimal LDR/STR path**

Replace the stub in `src/cpu/arm7/arm7_loadstore.cpp`:

```cpp
// arm7_loadstore.cpp — ARMv4T single-data-transfer dispatch.
// Slice 3b1 handles LDR/STR/LDRB/STRB in all standard addressing modes:
// immediate and immediate-shifted register offsets, pre/post index,
// up/down, writeback. Halfword and signed forms stay deferred to 3b2.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_alu.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "ds/common.hpp"

namespace ds {

u32 dispatch_single_data_transfer(Arm7State& state, Arm7Bus& bus,
                                  u32 instr, u32 instr_addr) {
    const bool i_bit = ((instr >> 25) & 1u) != 0;
    const bool p_bit = ((instr >> 24) & 1u) != 0;
    const bool u_bit = ((instr >> 23) & 1u) != 0;
    const bool b_bit = ((instr >> 22) & 1u) != 0;
    const bool w_bit = ((instr >> 21) & 1u) != 0;
    const bool l_bit = ((instr >> 20) & 1u) != 0;
    const u32  rn    = (instr >> 16) & 0xFu;
    const u32  rd    = (instr >> 12) & 0xFu;

    // Offset.
    u32 offset;
    if (i_bit) {
        // Register offset, always immediate-shifted for LDR/STR.
        const u32  rm         = instr & 0xFu;
        const u32  shift_type = (instr >> 5) & 0x3u;
        const u32  shift_amt  = (instr >> 7) & 0x1Fu;
        const bool c_in       = (state.cpsr & (1u << 29)) != 0;
        // Shifter carry output is discarded for LDR/STR.
        offset = barrel_shift_imm(
            state.r[rm],
            static_cast<ShiftType>(shift_type),
            shift_amt,
            c_in).value;
    } else {
        offset = instr & 0xFFFu;
    }

    // Addresses.
    const u32 base        = state.r[rn];          // rn==15 reads instr_addr+8
    const u32 signed_off  = u_bit ? offset : (0u - offset);
    const u32 pre_addr    = base + signed_off;    // 32-bit wrap
    const u32 access_addr = p_bit ? pre_addr : base;
    const u32 wb_addr     = pre_addr;             // same whether P=1 or P=0

    // Access.
    u32 loaded = 0;
    if (l_bit) {
        if (b_bit) {
            loaded = bus.read8(access_addr);       // zero-extended
        } else {
            const u32 raw = bus.read32(access_addr & ~0x3u);
            const u32 rot = (access_addr & 0x3u) * 8u;
            loaded = (rot == 0) ? raw : ((raw >> rot) | (raw << (32u - rot)));
        }
    } else {
        const u32 stored = (rd == 15) ? (instr_addr + 12u) : state.r[rd];
        if (b_bit) {
            bus.write8(access_addr, static_cast<u8>(stored & 0xFFu));
        } else {
            bus.write32(access_addr & ~0x3u, stored);
        }
    }

    // Writeback: post-index always writes back; pre-index writes back
    // only if W=1. On LDR, the loaded value wins over writeback when
    // Rn == Rd (ARMv4T defines this as the rule for that "unpredictable"
    // case).
    const bool do_wb = (!p_bit) || w_bit;
    if (do_wb && !(l_bit && rn == rd)) {
        write_rd(state, rn, wb_addr);
    }
    if (l_bit) {
        write_rd(state, rd, loaded);
    }

    return 1;
}

}  // namespace ds
```

- [ ] **Step 5: Run the test and verify it passes**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 6: Run the full suite**

Run: `cd build && ctest --output-on-failure`
Expected: 17/17 pass.

- [ ] **Step 7: Commit**

```bash
git add src/cpu/arm7/arm7_loadstore.cpp \
        tests/unit/arm7_load_store_test.cpp \
        tests/CMakeLists.txt
git commit -m "cpu/arm7: LDR/STR word immediate-offset path + tests

Full dispatch_single_data_transfer lands in this commit covering all
addressing modes, but only the aligned word immediate-offset U=1 P=1
W=0 path is tested. Subsequent tasks add tests for byte mode, U=0,
pre/post index with writeback, register offset, unaligned rotate,
R15 edge cases, and the Rn==Rd writeback rule."
```

---

## Task 6: `LDRB` and `STRB` (byte mode)

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

The implementation path is already in `arm7_loadstore.cpp` from Task 5 (the `b_bit` branches) — this task just adds tests that exercise it.

- [ ] **Step 1: Extend the test file with LDRB / STRB cases**

Add these functions to `tests/unit/arm7_load_store_test.cpp` (above `int main`):

```cpp
static void ldrb_loads_byte_zero_extended() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData, 0x1122'33FFu);  // byte 0 = 0xFF

    // LDRB R0, [R1, #0]
    // cond=AL, 01, I=0, P=1, U=1, B=1, W=0, L=1, Rn=1, Rd=0, imm12=0
    // → 1110 0101 1101 0001 0000 0000 0000 0000 = 0xE5D1'0000
    run_one(nds, 0xE5D1'0000u);

    REQUIRE(nds.cpu7().state().r[0] == 0x0000'00FFu);  // zero-extended
}

static void strb_stores_only_target_byte() {
    NDS nds;
    nds.cpu7().state().r[0] = 0xDEAD'BE42u;     // low byte = 0x42
    nds.cpu7().state().r[1] = kData;
    // Seed the whole word with a known pattern so we can verify the
    // neighboring bytes are untouched.
    nds.arm7_bus().write32(kData, 0xAABB'CCDDu);

    // STRB R0, [R1, #1]
    // → 1110 0101 1100 0001 0000 0000 0000 0001 = 0xE5C1'0001
    run_one(nds, 0xE5C1'0001u);

    // Only byte 1 should change to 0x42. Little-endian layout:
    //   before:  AA BB CC DD
    //   after:   AA BB 42 DD
    const u32 expected = (0xAABB'CCDDu & ~(0xFFu << 8)) | (0x42u << 8);
    REQUIRE(nds.arm7_bus().read32(kData) == expected);
}
```

Add both calls to `main()`:

```cpp
int main() {
    ldr_word_imm_offset_loads_value();
    str_word_imm_offset_stores_value();
    ldrb_loads_byte_zero_extended();
    strb_stores_only_target_byte();
    std::puts("arm7_load_store_test OK");
    return 0;
}
```

- [ ] **Step 2: Run the test and verify it passes immediately**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS (implementation already landed in Task 5).

If it fails: there is a bug in the Task 5 byte path. Fix it in `arm7_loadstore.cpp` before committing.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDRB zero-extension and STRB neighbor-byte tests"
```

---

## Task 7: U=0 (subtract offset)

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

- [ ] **Step 1: Add the test case**

Add above `int main`:

```cpp
static void ldr_word_u0_subtracts_offset() {
    NDS nds;
    nds.cpu7().state().r[1] = kData + 0x10u;
    nds.arm7_bus().write32(kData, 0xFEED'FACEu);

    // LDR R0, [R1, #-0x10]
    // cond=AL, 01, I=0, P=1, U=0, B=0, W=0, L=1, Rn=1, Rd=0, imm12=0x10
    // → 1110 0101 0001 0001 0000 0000 0001 0000 = 0xE511'0010
    run_one(nds, 0xE511'0010u);

    REQUIRE(nds.cpu7().state().r[0] == 0xFEED'FACEu);
    REQUIRE(nds.cpu7().state().r[1] == kData + 0x10u);  // no writeback
}
```

Add the call to `main()`:

```cpp
    ldr_word_u0_subtracts_offset();
```

- [ ] **Step 2: Run the test**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDR with U=0 subtracts offset from base"
```

---

## Task 8: Pre-index writeback (P=1, W=1) and Rn-unchanged path (P=1, W=0)

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

- [ ] **Step 1: Add the test cases**

Add above `int main`:

```cpp
static void ldr_pre_index_writeback_updates_rn() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 4, 0xAAAA'5555u);

    // LDR R0, [R1, #4]!        (pre-index, writeback)
    // cond=AL, 01, I=0, P=1, U=1, B=0, W=1, L=1, Rn=1, Rd=0, imm12=4
    // → 1110 0101 1011 0001 0000 0000 0000 0100 = 0xE5B1'0004
    run_one(nds, 0xE5B1'0004u);

    REQUIRE(nds.cpu7().state().r[0] == 0xAAAA'5555u);
    REQUIRE(nds.cpu7().state().r[1] == kData + 4u);  // writeback applied
}

static void ldr_pre_index_no_writeback_keeps_rn() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 4, 0x0000'FFFFu);

    // LDR R0, [R1, #4]         (pre-index, no writeback, W=0)
    // Already covered by Task 5's ldr_word_imm_offset_loads_value, but
    // add an explicit "Rn unchanged" assertion for readability of the
    // writeback matrix.
    run_one(nds, 0xE591'0004u);
    REQUIRE(nds.cpu7().state().r[1] == kData);
}
```

Add the calls to `main()`:

```cpp
    ldr_pre_index_writeback_updates_rn();
    ldr_pre_index_no_writeback_keeps_rn();
```

- [ ] **Step 2: Run the test**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDR pre-index writeback (W=1) and no-writeback (W=0) tests"
```

---

## Task 9: Post-index (P=0)

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

- [ ] **Step 1: Add the test case**

Add above `int main`:

```cpp
static void ldr_post_index_uses_base_then_updates_rn() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData, 0x1111'2222u);       // at base
    nds.arm7_bus().write32(kData + 4, 0x3333'4444u);   // at base + 4

    // LDR R0, [R1], #4          (post-index: access base, then Rn += 4)
    // cond=AL, 01, I=0, P=0, U=1, B=0, W=0, L=1, Rn=1, Rd=0, imm12=4
    // → 1110 0100 1001 0001 0000 0000 0000 0100 = 0xE491'0004
    run_one(nds, 0xE491'0004u);

    REQUIRE(nds.cpu7().state().r[0] == 0x1111'2222u);  // loaded from base, not base+4
    REQUIRE(nds.cpu7().state().r[1] == kData + 4u);    // Rn updated after
}
```

Add the call to `main()`:

```cpp
    ldr_post_index_uses_base_then_updates_rn();
```

- [ ] **Step 2: Run the test**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDR post-index accesses base and writes back"
```

---

## Task 10: Register offset (I=1) with immediate shift

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

- [ ] **Step 1: Add the test cases**

Add above `int main`:

```cpp
static void ldr_register_offset_lsl2_index_times_four() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;         // base
    nds.cpu7().state().r[2] = 3u;            // index
    nds.arm7_bus().write32(kData + 12, 0xABCD'0123u);  // element [3]

    // LDR R0, [R1, R2, LSL #2]
    // cond=AL, 01, I=1, P=1, U=1, B=0, W=0, L=1, Rn=1, Rd=0,
    // shift_amt=2, shift_type=LSL(0), Rm=2
    //   imm-shift encoding: [11:7]=shift_amt, [6:5]=shift_type, [4]=0, [3:0]=Rm
    // → 1110 0111 1001 0001 0000 0001 0000 0010 = 0xE791'0102
    run_one(nds, 0xE791'0102u);

    REQUIRE(nds.cpu7().state().r[0] == 0xABCD'0123u);
}

static void ldr_register_offset_lsl0_identity() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.cpu7().state().r[2] = 8u;
    nds.arm7_bus().write32(kData + 8, 0x5566'7788u);

    // LDR R0, [R1, R2]          (LSL #0, i.e. identity shift)
    // shift_amt=0, shift_type=LSL(0), bit[4]=0, Rm=2
    // → 1110 0111 1001 0001 0000 0000 0000 0010 = 0xE791'0002
    run_one(nds, 0xE791'0002u);

    REQUIRE(nds.cpu7().state().r[0] == 0x5566'7788u);
}
```

Add the calls to `main()`:

```cpp
    ldr_register_offset_lsl2_index_times_four();
    ldr_register_offset_lsl0_identity();
```

- [ ] **Step 2: Run the test**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDR immediate-shifted register offset (LSL #2 and LSL #0)"
```

---

## Task 11: Unaligned word `LDR` rotate

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

- [ ] **Step 1: Add the test cases**

Add above `int main`:

```cpp
static void ldr_unaligned_rotates_by_byte_offset() {
    // ARMv4T LDR from an unaligned address reads the aligned word and
    // rotates the loaded value right by (addr & 3) * 8 bits. The bus
    // access itself is always 4-byte aligned.
    //
    // Seed: 0xDEAD'BEEF at aligned kData.
    // LDR from kData + 1 → ROR by 8  → 0xEFDE'ADBE
    // LDR from kData + 2 → ROR by 16 → 0xBEEF'DEAD
    // LDR from kData + 3 → ROR by 24 → 0xADBE'EFDE

    for (u32 addr_off : { 1u, 2u, 3u }) {
        NDS nds;
        nds.cpu7().state().r[1] = kData + addr_off;
        nds.arm7_bus().write32(kData, 0xDEAD'BEEFu);

        // LDR R0, [R1, #0]   (P=1, U=1, B=0, W=0, L=1)
        run_one(nds, 0xE591'0000u);

        const u32 rot_bits = addr_off * 8u;
        const u32 expected = (0xDEAD'BEEFu >> rot_bits)
                           | (0xDEAD'BEEFu << (32u - rot_bits));
        REQUIRE(nds.cpu7().state().r[0] == expected);
    }
}
```

Add the call to `main()`:

```cpp
    ldr_unaligned_rotates_by_byte_offset();
```

- [ ] **Step 2: Run the test**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDR unaligned word access rotates by (addr & 3) * 8"
```

---

## Task 12: R15 edge cases — LDR into R15, STR of R15, Rn==Rd writeback

**Files:**
- Modify: `tests/unit/arm7_load_store_test.cpp`

- [ ] **Step 1: Add the test cases**

Add above `int main`:

```cpp
static void ldr_into_r15_sets_pc() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData, kBase + 0x40u);  // target address

    // LDR R15, [R1, #0]
    // cond=AL, 01, I=0, P=1, U=1, B=0, W=0, L=1, Rn=1, Rd=15, imm12=0
    // → 1110 0101 1001 0001 1111 0000 0000 0000 = 0xE591'F000
    run_one(nds, 0xE591'F000u);

    REQUIRE(nds.cpu7().state().pc == (kBase + 0x40u));
    REQUIRE(nds.cpu7().state().r[15] == (kBase + 0x40u));
}

static void str_of_r15_stores_instr_addr_plus_12() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;

    // STR R15, [R1, #0]
    // cond=AL, 01, I=0, P=1, U=1, B=0, W=0, L=0, Rn=1, Rd=15, imm12=0
    // → 1110 0101 1000 0001 1111 0000 0000 0000 = 0xE581'F000
    run_one(nds, 0xE581'F000u);

    // STR of R15 stores (instr_addr + 12) in ARMv4T. instr_addr = kBase.
    REQUIRE(nds.arm7_bus().read32(kData) == (kBase + 12u));
}

static void ldr_rn_eq_rd_writeback_load_wins() {
    NDS nds;
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 4, 0xC0DE'F00Du);

    // LDR R1, [R1, #4]!   (pre-index writeback; Rn == Rd == 1)
    // cond=AL, 01, I=0, P=1, U=1, B=0, W=1, L=1, Rn=1, Rd=1, imm12=4
    // → 1110 0101 1011 0001 0001 0000 0000 0100 = 0xE5B1'1004
    run_one(nds, 0xE5B1'1004u);

    // Load wins: R1 is the loaded value, not the writeback value.
    REQUIRE(nds.cpu7().state().r[1] == 0xC0DE'F00Du);
}
```

Add the calls to `main()`:

```cpp
    ldr_into_r15_sets_pc();
    str_of_r15_stores_instr_addr_plus_12();
    ldr_rn_eq_rd_writeback_load_wins();
```

- [ ] **Step 2: Run the test**

Run: `cd build && make arm7_load_store_test && ctest -R arm7_load_store_test --output-on-failure`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/arm7_load_store_test.cpp
git commit -m "cpu/arm7: LDR/STR R15 edge cases and Rn==Rd load-wins rule"
```

---

## Task 13: End-to-end sequence test

**Purpose:** Mirror of slice 3a's `arm7_run_sequence_test` — exercise branches + loads + stores + DP together in a multi-instruction run to prove the dispatch tree holds under real execution.

**Files:**
- Create: `tests/unit/arm7_branch_load_sequence_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write the test**

Create `tests/unit/arm7_branch_load_sequence_test.cpp`:

```cpp
// ARM7 slice 3b1 capstone: run a small program that uses LDR, DP, STR,
// B, BL, and MOV PC, LR in sequence. Verifies the top-level dispatcher
// routes each family correctly across many instructions.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void branch_load_store_sequence() {
    NDS nds;

    constexpr u32 kBase = 0x0380'0000u;
    constexpr u32 kData = 0x0380'0200u;

    // Program layout (little-endian words at kBase):
    //
    //   0x00: LDR  R0, [R1, #0]   ; R0 = *(R1 + 0)       = 10
    //   0x04: ADD  R0, R0, #5     ; R0 = 15
    //   0x08: STR  R0, [R1, #4]   ; *(R1 + 4) = 15
    //   0x0C: B    +4             ; jump to 0x14 (skip 0x10)
    //   0x10: MOV  R0, #0xFF      ; SKIPPED
    //   0x14: BL   +4             ; r14 = 0x18, jump to 0x1C
    //   0x18: MOV  PC, LR         ; return to caller (test harness will see this)
    //   0x1C: MOV  R2, #0x42      ; leaf body
    //   0x20: MOV  PC, LR         ; leaf return → 0x18
    //
    // Execution order:
    //   step 1: LDR  R0, [R1, #0]       pc = 0x04
    //   step 2: ADD  R0, R0, #5          pc = 0x08
    //   step 3: STR  R0, [R1, #4]        pc = 0x0C
    //   step 4: B    +4                  pc = 0x14
    //   step 5: BL   +4                  pc = 0x1C, r14 = 0x18
    //   step 6: MOV  R2, #0x42           pc = 0x20
    //   step 7: MOV  PC, LR (from leaf)  pc = 0x18
    //   step 8: MOV  PC, LR (from outer) pc = r14 at that moment
    //
    // After step 7, pc = 0x18, which is the instruction AFTER the BL in
    // the outer block. Step 8 executes THAT instruction, which is
    // itself "MOV PC, LR". It sets pc to the current value of R14 =
    // 0x18 + 4 = ... wait, that is not what we want.
    //
    // Simpler layout: drop the second MOV PC, LR and terminate at step
    // 7 so we can pin down R0, R1 (unchanged), R2, and pc.

    const u32 program[] = {
        0xE591'0000u,   // 0x00: LDR  R0, [R1, #0]
        0xE280'0005u,   // 0x04: ADD  R0, R0, #5
        0xE581'0004u,   // 0x08: STR  R0, [R1, #4]
        0xEA00'0000u,   // 0x0C: B    +0  (pc_target = 0x0C + 8 + 0 = 0x14)
        0xE3A0'00FFu,   // 0x10: MOV  R0, #0xFF  (skipped)
        0xEB00'0000u,   // 0x14: BL   +0  (pc_target = 0x14 + 8 + 0 = 0x1C)
        0xE1A0'F00Eu,   // 0x18: MOV  PC, LR  (outer return; not executed in this test)
        0xE3A0'2042u,   // 0x1C: MOV  R2, #0x42
        0xE1A0'F00Eu,   // 0x20: MOV  PC, LR  (leaf return)
    };
    for (u32 i = 0; i < 9; ++i) {
        nds.arm7_bus().write32(kBase + i * 4, program[i]);
    }

    // Seed R1 -> kData, and seed *(kData + 0) = 10.
    nds.cpu7().state().r[1] = kData;
    nds.arm7_bus().write32(kData + 0, 10u);

    nds.cpu7().state().pc = kBase;
    // 7 ARM7 cycles = 14 ARM9 cycles.
    nds.cpu7().run_until(14);

    REQUIRE(nds.cpu7().state().r[0] == 15u);                  // LDR + ADD
    REQUIRE(nds.cpu7().state().r[1] == kData);                // unchanged
    REQUIRE(nds.cpu7().state().r[2] == 0x42u);                // leaf body ran
    REQUIRE(nds.cpu7().state().r[14] == kBase + 0x18u);       // BL link addr
    REQUIRE(nds.cpu7().state().pc  == kBase + 0x18u);         // after leaf return
    REQUIRE(nds.arm7_bus().read32(kData + 4) == 15u);         // STR landed

    // Bonus sanity: the MOV R0, #0xFF at 0x10 must have been skipped,
    // else R0 would not be 15.
}

int main() {
    branch_load_store_sequence();
    std::puts("arm7_branch_load_sequence_test OK");
    return 0;
}
```

- [ ] **Step 2: Register the test in CMake**

Modify `tests/CMakeLists.txt` — append:

```cmake
add_ds_unit_test(arm7_branch_load_sequence_test)
```

- [ ] **Step 3: Run the test and verify it passes**

Run: `cd build && cmake .. && make arm7_branch_load_sequence_test && ctest -R arm7_branch_load_sequence_test --output-on-failure`
Expected: PASS.

If it fails: this is likely the first test that exercises the dispatch tree under multi-instruction execution, so a regression in any of Tasks 3-12 will surface here. Debug by single-stepping the sequence in a scratch test (e.g. add temporary prints of `state.pc` and `state.r[0..2]` after each `run_until` call).

- [ ] **Step 4: Run the full suite**

Run: `cd build && ctest --output-on-failure`
Expected: **18/18 pass** (15 existing + `arm7_branch_test` + `arm7_load_store_test` + `arm7_branch_load_sequence_test`).

- [ ] **Step 5: Commit**

```bash
git add tests/unit/arm7_branch_load_sequence_test.cpp tests/CMakeLists.txt
git commit -m "cpu/arm7: end-to-end LDR/DP/STR/B/BL sequence test

Capstone test mirroring slice 3a's arm7_run_sequence_test. Runs a small
program that loads a value, modifies it, stores the result, takes an
unconditional branch over a skipped instruction, and makes a BL to a
leaf that returns via MOV PC, LR. Exercises every new family in slice
3b1 against the real dispatch tree in one test."
```

---

## Task 14: Final verification

**Files:** none.

- [ ] **Step 1: Clean rebuild**

Run:
```bash
rm -rf build && mkdir build && cd build && cmake .. && make 2>&1 | tee /tmp/slice3b1_build.log
```
Expected: build clean, **zero warnings**, zero errors.

- [ ] **Step 2: Full test suite**

Run: `cd build && ctest --output-on-failure`
Expected: **18/18 tests pass**.

- [ ] **Step 3: Verify the architectural house-rules**

Run each of these; each must produce empty output or succeed.

```bash
# No SDL include in the ARM7 core:
grep -R "SDL" src/cpu/arm7/ || true

# arm7_decode_internal.hpp not leaked outside src/cpu/arm7/:
grep -R "arm7_decode_internal.hpp" src/ | grep -v "src/cpu/arm7/" || true

# CLAUDE.md still untracked:
git ls-files | grep -x "CLAUDE.md"  # must print nothing

# No Co-Authored-By on any slice 3b1 commit:
git log main..HEAD --grep "Co-Authored" || true
```

Expected: each grep returns nothing (or only the `|| true` swallow). If `git ls-files | grep -x "CLAUDE.md"` prints a match, stop and fix the tracking state before continuing.

- [ ] **Step 4: Fresh `NDS` smoke check**

The `nds_integration_test` binary already exercises `NDS nds; nds.run_frame();`. Verify it passed in Step 2. No further action needed.

- [ ] **Step 5: Final commit (only if anything changed above)**

If Step 3 or Step 4 surfaced a fix, commit it. Otherwise, nothing to commit — the slice is done as of Task 13.

---

## Deferred to slice 3b2 (tracked for continuity)

- `LDRH`, `STRH`, `LDRSB`, `LDRSH` (halfword and signed loads/stores)
- `LDM`, `STM` (load/store multiple)
- `MUL`, `MLA`
- `MRS`, `MSR`
- Register-shifted-register operand2 (bit 4 == 1 in DP space, excluding
  the BX pattern which is handled in 3b1)
- Real per-instruction cycle counts
- Branch pipeline-flush cycle penalty (2S + 1N)

## Deferred to slice 3c

- Full Thumb ISA
- `step_thumb` dispatcher
- Removal of the `cpsr.T` assert in `step_arm`

## Deferred to slice 3d

- Exception entry (reset / undef / SWI / abort / IRQ / FIQ)
- IRQ line sampling in the fetch loop
- `MOVS pc, lr` / `LDM ..^{pc}` exception-return semantics
