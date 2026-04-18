#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "cpu/arm7/bios/bios7_trampoline.hpp"
#include "nds.hpp"
#include "require.hpp"

#include <array>
#include <cstdio>
#include <span>

using namespace ds;

namespace {

constexpr u32 kCallbackAddr = 0x0200'1000u;
constexpr u32 kCpsrSvcMode = 0x0000'0013u;
constexpr u32 kSvcSp = 0x0300'7FE0u;
constexpr std::array<u32, 4> kZeroArgs = {0u, 0u, 0u, 0u};

static void setup_svc(NDS& nds) {
    auto& state = nds.cpu7().state();
    state.switch_mode(Mode::Supervisor);
    state.cpsr = kCpsrSvcMode;
    state.r[13] = kSvcSp;
}

static u32 call(NDS& nds, u32 target_pc, const std::array<u32, 4>& args = kZeroArgs) {
    return arm7_bios_call_guest(nds.cpu7(), target_pc, std::span<const u32, 4>(args));
}

} // namespace

// ARM callback: MOV R0, #0x42; BX LR.
static void arm_callback_returns_constant() {
    NDS nds;
    auto& bus = nds.arm7_bus();

    bus.write32(kCallbackAddr + 0u, 0xE3A0'0042u);
    bus.write32(kCallbackAddr + 4u, 0xE12F'FF1Eu);

    setup_svc(nds);
    REQUIRE(call(nds, kCallbackAddr) == 0x42u);
}

// Thumb callback: MOV R0, #0x42; BX LR.
static void thumb_callback_returns_constant() {
    NDS nds;
    auto& bus = nds.arm7_bus();

    bus.write16(kCallbackAddr + 0u, 0x2042u);
    bus.write16(kCallbackAddr + 2u, 0x4770u);

    setup_svc(nds);
    REQUIRE(call(nds, kCallbackAddr | 1u) == 0x42u);
}

// ARM callback: R0 = R0 + R1 + R2 + R3; BX LR. Verifies args thread through.
static void callback_uses_args() {
    NDS nds;
    auto& bus = nds.arm7_bus();

    bus.write32(kCallbackAddr + 0u, 0xE080'0001u);  // ADD R0, R0, R1
    bus.write32(kCallbackAddr + 4u, 0xE080'0002u);  // ADD R0, R0, R2
    bus.write32(kCallbackAddr + 8u, 0xE080'0003u);  // ADD R0, R0, R3
    bus.write32(kCallbackAddr + 12u, 0xE12F'FF1Eu); // BX LR

    setup_svc(nds);
    REQUIRE(call(nds, kCallbackAddr, {10u, 20u, 30u, 40u}) == 100u);
}

// ARM callback: LDRB R0, [R0]; BX LR. Verifies bus reads inside the callback.
static void callback_reads_from_bus() {
    NDS nds;
    auto& bus = nds.arm7_bus();

    bus.write32(kCallbackAddr + 0u, 0xE5D0'0000u); // LDRB R0, [R0]
    bus.write32(kCallbackAddr + 4u, 0xE12F'FF1Eu); // BX LR

    constexpr u32 kProbeAddr = 0x0200'2000u;
    bus.write8(kProbeAddr, 0xA5u);

    setup_svc(nds);
    REQUIRE(call(nds, kCallbackAddr, {kProbeAddr, 0u, 0u, 0u}) == 0xA5u);
}

// Infinite loop (B .): step budget trips, ret == 0, caller state restored.
static void budget_exhausted_on_infinite_loop() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kCallbackAddr, 0xEAFF'FFFEu); // B .

    setup_svc(nds);
    state.r[1] = 0x1111'1111u;
    state.r[2] = 0x2222'2222u;
    state.r[3] = 0x3333'3333u;
    state.r[12] = 0x4444'4444u;
    state.r[14] = 0xDEAD'BEEFu;
    state.pc = 0x0200'0800u;

    REQUIRE(call(nds, kCallbackAddr) == 0u);
    REQUIRE(state.r[1] == 0x1111'1111u);
    REQUIRE(state.r[2] == 0x2222'2222u);
    REQUIRE(state.r[3] == 0x3333'3333u);
    REQUIRE(state.r[12] == 0x4444'4444u);
    REQUIRE(state.r[13] == kSvcSp);
    REQUIRE(state.r[14] == 0xDEAD'BEEFu);
    REQUIRE(state.pc == 0x0200'0800u);
    REQUIRE(state.current_mode() == Mode::Supervisor);
}

// Callee-saved regs R4-R11, SVC SP, and full CPSR round-trip via no-op callback.
static void state_preservation_r4_to_r11_and_svc_sp() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kCallbackAddr, 0xE12F'FF1Eu); // BX LR

    setup_svc(nds);
    constexpr u32 kSeededSp = 0x0300'77E0u;
    state.r[13] = kSeededSp;
    state.r[14] = 0xDEAD'BEEFu;
    state.cpsr = kCpsrSvcMode;
    for (u32 i = 4; i <= 11; ++i) {
        state.r[i] = 0x0000'1000u + i;
    }

    // No-op callback preserves r[0] = args[0] = 0 — also exercises the return
    // path and guards against the trampoline returning saved.r0 by mistake.
    REQUIRE(call(nds, kCallbackAddr) == 0u);

    for (u32 i = 4; i <= 11; ++i) {
        REQUIRE(state.r[i] == 0x0000'1000u + i);
    }
    REQUIRE(state.r[13] == kSeededSp);
    REQUIRE(state.r[14] == 0xDEAD'BEEFu);
    REQUIRE(state.cpsr == kCpsrSvcMode);
}

// Callback clobbers R1/R2/R3/R12 then returns — trampoline restores caller's
// R1-R3, R12, LR, and PC from its snapshot.
static void caller_r1_r3_r12_lr_restored_even_if_callback_mutates() {
    NDS nds;
    auto& state = nds.cpu7().state();
    auto& bus = nds.arm7_bus();

    bus.write32(kCallbackAddr + 0u, 0xE3A0'10AAu);  // MOV R1, #0xAA
    bus.write32(kCallbackAddr + 4u, 0xE3A0'20BBu);  // MOV R2, #0xBB
    bus.write32(kCallbackAddr + 8u, 0xE3A0'30CCu);  // MOV R3, #0xCC
    bus.write32(kCallbackAddr + 12u, 0xE3A0'C0DDu); // MOV R12, #0xDD
    bus.write32(kCallbackAddr + 16u, 0xE12F'FF1Eu); // BX LR

    setup_svc(nds);
    state.r[1] = 0x1111'1111u;
    state.r[2] = 0x2222'2222u;
    state.r[3] = 0x3333'3333u;
    state.r[12] = 0x4444'4444u;
    state.r[14] = 0x5555'5555u;
    state.pc = 0x7777'7777u;

    call(nds, kCallbackAddr);

    REQUIRE(state.r[1] == 0x1111'1111u);
    REQUIRE(state.r[2] == 0x2222'2222u);
    REQUIRE(state.r[3] == 0x3333'3333u);
    REQUIRE(state.r[12] == 0x4444'4444u);
    REQUIRE(state.r[14] == 0x5555'5555u);
    REQUIRE(state.pc == 0x7777'7777u);
}

int main() {
    arm_callback_returns_constant();
    thumb_callback_returns_constant();
    callback_uses_args();
    callback_reads_from_bus();
    budget_exhausted_on_infinite_loop();
    state_preservation_r4_to_r11_and_svc_sp();
    caller_r1_r3_r12_lr_restored_even_if_callback_mutates();
    std::puts("arm7_bios_trampoline_test: all 7 sub-cases passed");
    return 0;
}
