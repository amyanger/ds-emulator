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
            const bool carry = ((operand >> (32u - amount)) & 1u) != 0;
            return { operand << amount, carry };
        }
        case ShiftType::Lsr: {
            const bool carry = ((operand >> (amount - 1u)) & 1u) != 0;
            return { operand >> amount, carry };
        }
        case ShiftType::Asr: {
            const bool carry = ((operand >> (amount - 1u)) & 1u) != 0;
            const i32 signed_op = static_cast<i32>(operand);
            return { static_cast<u32>(signed_op >> amount), carry };
        }
        case ShiftType::Ror: {
            const u32 a = amount & 31u;
            const bool carry = ((operand >> (a - 1u)) & 1u) != 0;
            return { (operand >> a) | (operand << (32u - a)), carry };
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
                const bool carry = ((operand >> (a - 1u)) & 1u) != 0;
                return { (operand >> a) | (operand << (32u - a)), carry };
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
    const u32 value = (imm8 >> amount) | (imm8 << (32u - amount));
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
