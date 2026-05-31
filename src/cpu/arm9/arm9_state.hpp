#pragma once

// Arm9State — all CPU-visible state for the ARMv5TE core (ARM946E-S).
// This is pure data plus tiny helpers. `class Arm9` (in arm9.hpp) owns an
// Arm9State and adds the fetch/decode/execute behavior on top.
//
// The register file, banking logic, and Mode enum are identical to the ARM7
// (ARMv4T) — that machinery is copied verbatim from Arm7State. Only the reset
// defaults differ (high reset vector, see reset()), plus the live Q flag
// (CPSR bit 27, see the cpsr member comment). Mode lives in the shared
// cpu/common/arm_mode.hpp so both state structs reference one definition.

#include "cpu/common/arm_mode.hpp"
#include "ds/common.hpp"

#include <array>
#include <cassert>

namespace ds {

// Banks holding the physical storage for mode-banked registers.
// User and System share the same bank, so only six SPSRs exist (one each
// for FIQ/IRQ/SVC/ABT/UND) and the User/System bank has no SPSR.
struct Arm9Banks {
    // User/System R8..R14 live here when CPSR mode = User or System.
    std::array<u32, 7> user_r8_r14{}; // [0]=R8 .. [6]=R14
    // FIQ has its own R8..R14.
    std::array<u32, 7> fiq_r8_r14{}; // [0]=R8 .. [6]=R14
    // IRQ/SVC/ABT/UND only bank R13 and R14.
    std::array<u32, 2> irq_r13_r14{}; // [0]=R13, [1]=R14
    std::array<u32, 2> svc_r13_r14{};
    std::array<u32, 2> abt_r13_r14{};
    std::array<u32, 2> und_r13_r14{};

    u32 spsr_fiq = 0;
    u32 spsr_irq = 0;
    u32 spsr_svc = 0;
    u32 spsr_abt = 0;
    u32 spsr_und = 0;
};

struct Arm9State {
    // Current-mode visible register file. `r[13]`, `r[14]`, and the FIQ
    // registers are reloaded from `banks` on every `switch_mode()` call.
    // `r[15]` is kept in sync with `pc` at the top of every execute step;
    // tests should read `state.r[15]` after an instruction executes.
    std::array<u32, 16> r{};

    // Program counter — address of the next instruction to fetch. In ARM
    // state this is always 4-aligned. `r[15]` at the moment an instruction
    // executes is `pc + 4` (i.e. "current instruction address + 8").
    u32 pc = 0xFFFF0000u;

    // CPSR. Reset default is 0xD3: SVC mode (M=0x13), IRQ disabled (I=1),
    // FIQ disabled (F=1), Thumb off (T=0), flags clear.
    //
    // Bit 27 is the sticky saturation (Q) flag on ARMv5TE — live and
    // software-visible. Its producers (saturating math, accumulating signed
    // multiplies) are all v5-only and land in slice 3m; Q is cleared ONLY by
    // an explicit MSR. It is never masked off here or in the PSR path. reset()
    // leaves bit 27 = 0 (implied by cpsr = 0xD3).
    u32 cpsr = 0xD3;

    // All banked register sets.
    Arm9Banks banks{};

    // Cycle counter — ARM9 cycles since reset; 1:1 with the scheduler.
    u64 cycles = 0;

    // Reset to DS post-boot defaults. Direct boot later overrides pc and
    // some registers based on the cart header; here we just give sane
    // power-on values. ARM9 resets to the HIGH exception vector (DS firmware
    // always runs with CP15 control bit 13 set), unlike ARM7 which resets to 0.
    void reset() {
        r = {};
        pc = 0xFFFF0000u;
        cpsr = 0xD3;
        banks = {};
        cycles = 0;
        load_banked_registers(Mode::Supervisor); // establishes the invariant
    }

    Mode current_mode() const { return static_cast<Mode>(cpsr & 0x1Fu); }

    // Save the currently visible registers into the bank for `from_mode`,
    // then load the registers for `to_mode` from its bank and update
    // CPSR's mode bits.
    void switch_mode(Mode to_mode) {
        const Mode from_mode = current_mode();
        store_banked_registers(from_mode);
        load_banked_registers(to_mode);
        cpsr = (cpsr & ~0x1Fu) | static_cast<u32>(to_mode);
    }

    // Returns a pointer to the SPSR storage slot for the current mode.
    // Returns nullptr when the current mode is User or System (neither
    // of those modes has a dedicated SPSR — they share the user bank
    // and the Arm9Banks struct has no spsr_usr or spsr_sys field).
    // Used by arm9_psr.cpp for MRS / MSR SPSR, and by exception-entry code.
    u32* spsr_slot() {
        switch (current_mode()) {
        case Mode::Fiq:
            return &banks.spsr_fiq;
        case Mode::Irq:
            return &banks.spsr_irq;
        case Mode::Supervisor:
            return &banks.spsr_svc;
        case Mode::Abort:
            return &banks.spsr_abt;
        case Mode::Undefined:
            return &banks.spsr_und;
        default:
            return nullptr; // User or System
        }
    }

    const u32* spsr_slot() const { return const_cast<Arm9State*>(this)->spsr_slot(); }

private:
    void store_banked_registers(Mode m) {
        // All non-FIQ modes share the User bank for R8..R12, so every
        // "save" path except FIQ's must write R8..R12 back into the
        // User bank before we load a new mode's R13/R14.
        switch (m) {
        case Mode::User:
        case Mode::System:
            for (std::size_t i = 0; i < 7; ++i)
                banks.user_r8_r14[i] = r[8 + i];
            break;
        case Mode::Fiq:
            for (std::size_t i = 0; i < 7; ++i)
                banks.fiq_r8_r14[i] = r[8 + i];
            break;
        case Mode::Irq:
            for (std::size_t i = 0; i < 5; ++i)
                banks.user_r8_r14[i] = r[8 + i];
            banks.irq_r13_r14[0] = r[13];
            banks.irq_r13_r14[1] = r[14];
            break;
        case Mode::Supervisor:
            for (std::size_t i = 0; i < 5; ++i)
                banks.user_r8_r14[i] = r[8 + i];
            banks.svc_r13_r14[0] = r[13];
            banks.svc_r13_r14[1] = r[14];
            break;
        case Mode::Abort:
            for (std::size_t i = 0; i < 5; ++i)
                banks.user_r8_r14[i] = r[8 + i];
            banks.abt_r13_r14[0] = r[13];
            banks.abt_r13_r14[1] = r[14];
            break;
        case Mode::Undefined:
            for (std::size_t i = 0; i < 5; ++i)
                banks.user_r8_r14[i] = r[8 + i];
            banks.und_r13_r14[0] = r[13];
            banks.und_r13_r14[1] = r[14];
            break;
        default:
            assert(false && "Arm9State::switch_mode: invalid Mode value");
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
            for (std::size_t i = 0; i < 7; ++i)
                r[8 + i] = banks.user_r8_r14[i];
            break;
        case Mode::Fiq:
            for (std::size_t i = 0; i < 7; ++i)
                r[8 + i] = banks.fiq_r8_r14[i];
            break;
        case Mode::Irq:
            for (std::size_t i = 0; i < 5; ++i)
                r[8 + i] = banks.user_r8_r14[i]; // R8..R12
            r[13] = banks.irq_r13_r14[0];
            r[14] = banks.irq_r13_r14[1];
            break;
        case Mode::Supervisor:
            for (std::size_t i = 0; i < 5; ++i)
                r[8 + i] = banks.user_r8_r14[i];
            r[13] = banks.svc_r13_r14[0];
            r[14] = banks.svc_r13_r14[1];
            break;
        case Mode::Abort:
            for (std::size_t i = 0; i < 5; ++i)
                r[8 + i] = banks.user_r8_r14[i];
            r[13] = banks.abt_r13_r14[0];
            r[14] = banks.abt_r13_r14[1];
            break;
        case Mode::Undefined:
            for (std::size_t i = 0; i < 5; ++i)
                r[8 + i] = banks.user_r8_r14[i];
            r[13] = banks.und_r13_r14[0];
            r[14] = banks.und_r13_r14[1];
            break;
        default:
            assert(false && "Arm9State::switch_mode: invalid Mode value");
            break;
        }
    }
};

} // namespace ds
