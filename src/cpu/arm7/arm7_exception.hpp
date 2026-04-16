#pragma once

// ARM7 exception machinery. Shared between ARM-state SWI/UNDEF, Thumb-state
// SWI/UNDEF, IRQ, FIQ, and data/prefetch abort entry sites. Slice 3d commit 2.

#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7Bus;

enum class ExceptionKind : u8 {
    Reset,
    Undef,
    Swi,
    PrefetchAbort,
    DataAbort,
    Irq,
    Fiq,
};

// Enter `kind` with `return_addr` as the post-exception R14 value.
// Caller computes return_addr per §5.3 of the slice-3d design spec:
//   - SWI from ARM:       instr_addr + 4
//   - SWI from Thumb:     instr_addr + 2  (already state.pc)
//   - UNDEF from ARM:     instr_addr + 4
//   - UNDEF from Thumb:   instr_addr + 2
//   - Prefetch abort:     instr_addr + 4
//   - Data abort:         instr_addr + 8
//   - IRQ / FIQ:          state.pc + 4     (ARM-style format)
//
// Returns cycle cost for the entry (coarse: 3 cycles).
u32 enter_exception(Arm7State& state, ExceptionKind kind, u32 return_addr);

// ARM7 IRQ entry with the direct-boot vector indirection. Enters IRQ mode via
// enter_exception() (R14_irq = state.pc + 4 in both ARM and Thumb state — the
// "ARM-style format" GBATEK mandates), then overrides PC by loading the real
// handler address from [0x0380FFFC] on the ARM7 bus. Real hardware's ARM7
// BIOS ROM at 0x00000018 contains a tiny thunk that does exactly this load;
// we direct-boot with no BIOS ROM, so synthesize the thunk here. When the
// BIOS-HLE slice lands a real ARM7 BIOS at 0x00000000, the bus.read32 line
// disappears and this becomes a one-line forward to enter_exception.
// Advances state.cycles by the entry's cycle cost.
void arm7_enter_irq(Arm7State& state, Arm7Bus& bus);

// Exposed for tests.
constexpr u32 exception_vector(ExceptionKind kind) {
    switch (kind) {
    case ExceptionKind::Reset:
        return 0x00000000u;
    case ExceptionKind::Undef:
        return 0x00000004u;
    case ExceptionKind::Swi:
        return 0x00000008u;
    case ExceptionKind::PrefetchAbort:
        return 0x0000000Cu;
    case ExceptionKind::DataAbort:
        return 0x00000010u;
    case ExceptionKind::Irq:
        return 0x00000018u;
    case ExceptionKind::Fiq:
        return 0x0000001Cu;
    }
}

constexpr Mode exception_mode(ExceptionKind kind) {
    switch (kind) {
    case ExceptionKind::Reset:
        return Mode::Supervisor;
    case ExceptionKind::Undef:
        return Mode::Undefined;
    case ExceptionKind::Swi:
        return Mode::Supervisor;
    case ExceptionKind::PrefetchAbort:
        return Mode::Abort;
    case ExceptionKind::DataAbort:
        return Mode::Abort;
    case ExceptionKind::Irq:
        return Mode::Irq;
    case ExceptionKind::Fiq:
        return Mode::Fiq;
    }
}

constexpr bool exception_sets_fiq_mask(ExceptionKind kind) {
    return kind == ExceptionKind::Reset || kind == ExceptionKind::Fiq;
}

} // namespace ds
