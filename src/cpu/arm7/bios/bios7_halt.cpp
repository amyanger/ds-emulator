#include "cpu/arm7/bios/bios7_halt.hpp"

#include "bus/arm7_bus.hpp"
#include "bus/io_regs.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

#include <cassert>

namespace ds {

// Hardware runs `LOP: SUB R0,1 / BGT LOP` in BIOS ROM; exit leaves R0 == 0.
// 4 cycles/iter matches GBATEK's calibration table (r0=0x20BA ≈ 1ms @ 33.51MHz).
u32 bios7_wait_by_loop(Arm7State& state, Arm7Bus& /*bus*/) {
    state.cycles += 4ull * static_cast<u64>(state.r[0]);
    state.r[0] = 0;
    return 1;
}

// BIOS shadow — "NDS7 IRQ Check Bits" per GBATEK, in ARM7-private WRAM.
// The game's IRQ handler ORs serviced-source bits into this word before
// returning; IntrWait polls and consumes it. Deadlock (spins forever) if
// the game handler forgets to update it — game bug, not emulator bug.
constexpr u32 kArm7BiosIrqCheckBits = 0x0380FFF8u;

// GBATEK: "Continues to wait in Halt state until one (or more) of the
// specified interrupt(s) do occur. The function forcefully sets IME=1."
//
// Re-execution model: when the shadow check fails we halt the CPU and
// rewind R14_svc by the caller-instruction size (4 for ARM, 2 for Thumb).
// The dispatcher's implicit MOVS PC, R14 (bios7_hle.cpp) then lands PC
// on the SWI itself, so the next run_until iteration — after halt wake —
// re-executes the SWI, which re-enters here and re-checks. Matches the
// real BIOS's tiny loop more faithfully than an inline C++ busy-wait,
// and avoids starving the scheduler inside a SWI handler.
u32 bios7_intr_wait(Arm7State& state, Arm7Bus& bus) {
    const u32 discard = state.r[0];
    const u32 mask = state.r[1];

    bus.write32(IO_IME, 1u);

    // `shadow` is kept in sync with memory: after the discard branch it
    // already has `mask` bits cleared, so the condition check below
    // always falls through to halt on that path — which is exactly the
    // "discard old flags, wait for new" contract.
    u32 shadow = bus.read32(kArm7BiosIrqCheckBits);
    if (discard == 1u) {
        shadow &= ~mask;
        bus.write32(kArm7BiosIrqCheckBits, shadow);
    }

    if ((shadow & mask) != 0u) {
        bus.write32(kArm7BiosIrqCheckBits, shadow & ~mask);
        return 1;
    }

    bus.write8(IO_HALTCNT, 0x80u);

    // SWI entry always banks into SVC with a valid SPSR_svc; SPSR.T picks
    // the caller-instruction size for the re-execute rewind.
    const u32* spsr = state.spsr_slot();
    assert(spsr != nullptr && "bios7_intr_wait: SWI must run in a mode with SPSR");
    const u32 instr_size = (*spsr & (1u << 5)) ? 2u : 4u;
    state.r[14] -= instr_size;
    return 1;
}

// GBATEK: "Continues to wait in Halt status until a new V-Blank interrupt
// occurs. The function sets r0=1 and r1=1 and does then execute IntrWait
// (SWI 04h)." R1=1 = VBlank (IE/IF bit 0). R0=1 = discard-old.
u32 bios7_vblank_intr_wait(Arm7State& state, Arm7Bus& bus) {
    state.r[0] = 1u;
    state.r[1] = 1u;
    return bios7_intr_wait(state, bus);
}

// GBATEK: "On GBA and NDS7/DSi7, Halt is implemented by writing to HALTCNT,
// Port 4000301h." All registers unchanged on NDS7. The halt state machine
// lives in the HALTCNT write handler (nds.cpp) + run_until fast-path.
u32 bios7_halt(Arm7State& /*state*/, Arm7Bus& bus) {
    bus.write8(IO_HALTCNT, 0x80u);
    return 1;
}

// GBATEK is vague on Sleep ("probably similar as GBA SWI 03h Stop"). By
// analogy with CustomHalt's 0xC0 = Sleep encoding, write 0xC0 to HALTCNT.
// Slice 3e §3 deliberately treats Sleep as Halt (no wake-source restrictions
// modeled); HALTCNT's mode-decode handler maps both to halted_ = true.
u32 bios7_sleep(Arm7State& /*state*/, Arm7Bus& bus) {
    bus.write8(IO_HALTCNT, 0xC0u);
    return 1;
}

// GBATEK: "Writes the 8bit parameter value to HALTCNT" — the full byte from
// R2 is written verbatim, including reserved bits. HALTCNT's mode-decode
// handler picks out bits 6-7. Only low 8 bits of R2 are meaningful because
// HALTCNT is a 1-byte register.
u32 bios7_custom_halt(Arm7State& state, Arm7Bus& bus) {
    bus.write8(IO_HALTCNT, static_cast<u8>(state.r[2] & 0xFFu));
    return 1;
}

} // namespace ds
