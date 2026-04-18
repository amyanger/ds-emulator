#include "cpu/arm7/bios/bios7_trampoline.hpp"

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"

namespace ds {

namespace {

constexpr u32 kCpsrThumbBit = 1u << 5;

struct TrampolineSaved {
    u32 r1;
    u32 r2;
    u32 r3;
    u32 r12;
    u32 r13;
    u32 r14;
    u32 pc;
    u32 cpsr;
    u32 svc_sp;
    u32 svc_lr;
};

} // namespace

u32 arm7_bios_call_guest(Arm7& cpu, u32 target_pc, std::span<const u32, 4> args) {
    Arm7State& state = cpu.state();

    TrampolineSaved saved{};
    saved.r1 = state.r[1];
    saved.r2 = state.r[2];
    saved.r3 = state.r[3];
    saved.r12 = state.r[12];
    saved.r13 = state.r[13];
    saved.r14 = state.r[14];
    saved.pc = state.pc;
    saved.cpsr = state.cpsr;
    saved.svc_sp = state.banks.svc_r13_r14[0];
    saved.svc_lr = state.banks.svc_r13_r14[1];

    state.r[0] = args[0];
    state.r[1] = args[1];
    state.r[2] = args[2];
    state.r[3] = args[3];

    state.r[14] = kTrampolineSentinelPC;

    const bool thumb_target = (target_pc & 1u) != 0u;
    if (thumb_target) {
        state.cpsr |= kCpsrThumbBit;
        state.pc = target_pc & ~1u;
    } else {
        state.cpsr &= ~kCpsrThumbBit;
        state.pc = target_pc & ~3u;
    }

    u32 steps = 0;
    bool budget_exhausted = false;
    // Sentinel is checked before each step, so kTrampolineSentinelPC is never fetched.
    while (state.pc != kTrampolineSentinelPC) {
        cpu.step_one_instruction();
        if (++steps >= kTrampolineStepBudget) {
            DS_LOG_WARN(
                "arm7/bios: trampoline step budget (%u) exceeded at PC=0x%08X; target=0x%08X",
                kTrampolineStepBudget,
                state.pc,
                target_pc);
            budget_exhausted = true;
            break;
        }
    }

    const u32 ret = budget_exhausted ? 0u : state.r[0];

    // If the callback exited in a non-SVC mode, re-bank first so that writing
    // visible r[13]/r[14] targets the SVC slots. Then overwrite both the
    // visible regs and banks.svc_r13_r14[] so subsequent mode round-trips can
    // reload the caller's SP/LR from the bank.
    if (state.current_mode() != Mode::Supervisor) {
        state.switch_mode(Mode::Supervisor);
    }
    state.cpsr = saved.cpsr;
    state.r[1] = saved.r1;
    state.r[2] = saved.r2;
    state.r[3] = saved.r3;
    state.r[12] = saved.r12;
    state.r[13] = saved.r13;
    state.r[14] = saved.r14;
    state.pc = saved.pc;
    state.banks.svc_r13_r14[0] = saved.svc_sp;
    state.banks.svc_r13_r14[1] = saved.svc_lr;

    return ret;
}

} // namespace ds
