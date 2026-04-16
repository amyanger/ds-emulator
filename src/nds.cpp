#include "nds.hpp"

#include "bus/io_regs.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380. Real
// PPU-derived timing lands when the PPU lands.
static constexpr Cycle kFrameCycles = 1'120'380;

NDS::NDS()
    : arm9_bus_(*this, main_ram_.data(), shared_wram_.data(), wram_ctl_),
      arm7_bus_(*this, main_ram_.data(), shared_wram_.data(), arm7_wram_.data(), wram_ctl_) {
    // attach_bus() must precede reset(): reset() cascades into cpu7_.reset(),
    // which in later slices will touch bus_ for cart-header reads during
    // direct boot. Do not reorder these calls, and do not sink reset() into
    // a default member initializer.
    cpu7_.attach_bus(arm7_bus_);
    reset();
}

void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;

    main_ram_.fill(0);
    shared_wram_.fill(0);
    arm7_wram_.fill(0);

    wram_ctl_.reset();
    arm9_bus_.reset();
    arm7_bus_.reset();
    soundbias_ = 0x0200u;
    irq7_ctrl_.reset();
    // After reset IME/IE/IF are all zero so the line is false. Push it into
    // cpu7_ explicitly so state is consistent even if Arm7::reset() is ever
    // re-ordered relative to irq7_ctrl_.reset() in the future.
    update_arm7_irq_signals();
}

void NDS::run_frame() {
    frame_done_ = false;
    scheduler_.schedule_in(kFrameCycles, EventKind::FrameEnd);

    while (!frame_done_) {
        const Cycle next = scheduler_.peek_next();
        if (next == Scheduler::kNoEvent)
            break; // heap drained early; shouldn't happen today
        cpu9_.run_until(next);
        cpu7_.run_until(next);
        scheduler_.set_now(next);

        Event ev{};
        while (scheduler_.pop_due(next, ev)) {
            on_scheduler_event(ev);
        }
    }
}

void NDS::on_scheduler_event(const Event& ev) {
    switch (ev.kind) {
    case EventKind::FrameEnd:
        frame_done_ = true;
        break;
    }
}

u32 NDS::arm9_io_read32(u32 /*addr*/) {
    return 0;
}
u16 NDS::arm9_io_read16(u32 /*addr*/) {
    return 0;
}
u8 NDS::arm9_io_read8(u32 /*addr*/) {
    return 0;
}
void NDS::arm9_io_write32(u32 /*addr*/, u32 /*value*/) {}
void NDS::arm9_io_write16(u32 /*addr*/, u16 /*value*/) {}
void NDS::arm9_io_write8(u32 addr, u8 value) {
    // WRAMCNT is the only I/O register handled in slice 2. Everything else
    // is an accepted no-op until later slices wire the remaining I/O.
    if (addr == 0x0400'0247u) {
        wram_ctl_.write(value);
        arm9_bus_.rebuild_shared_wram();
        arm7_bus_.rebuild_shared_wram();
    }
}

void NDS::update_arm7_irq_signals() {
    cpu7_.set_irq_line(irq7_ctrl_.line());
    cpu7_.set_halt_wake_pending(irq7_ctrl_.halt_wake_pending());
}

u32 NDS::arm7_io_read32(u32 addr) {
    switch (addr) {
    case IO_IME:
        return irq7_ctrl_.read_ime();
    case IO_IE:
        return irq7_ctrl_.read_ie();
    case IO_IF:
        return irq7_ctrl_.read_if();
    default:
        return 0;
    }
}

u16 NDS::arm7_io_read16(u32 addr) {
    // IME: bit 0 in the low halfword, upper bits reserved as zero. DS mirror
    // at 0x04000208 + 2 is not a thing — only the low halfword is defined.
    if (addr == IO_IME) {
        return static_cast<u16>(irq7_ctrl_.read_ime() & 0xFFFFu);
    }
    // IE/IF split low / high halfword windows over the 32-bit register.
    if (addr == IO_IE) {
        return static_cast<u16>(irq7_ctrl_.read_ie() & 0xFFFFu);
    }
    if (addr == IO_IE + 2u) {
        return static_cast<u16>((irq7_ctrl_.read_ie() >> 16) & 0xFFFFu);
    }
    if (addr == IO_IF) {
        return static_cast<u16>(irq7_ctrl_.read_if() & 0xFFFFu);
    }
    if (addr == IO_IF + 2u) {
        return static_cast<u16>((irq7_ctrl_.read_if() >> 16) & 0xFFFFu);
    }
    return 0;
}

u8 NDS::arm7_io_read8(u32 addr) {
    // Byte reads: slide an 8-bit window across the 32-bit register. Only the
    // IME/IE/IF windows are routed here; other I/O is still stubbed.
    if (addr >= IO_IME && addr < IO_IME + 4u) {
        const u32 shift = (addr - IO_IME) * 8u;
        return static_cast<u8>((irq7_ctrl_.read_ime() >> shift) & 0xFFu);
    }
    if (addr >= IO_IE && addr < IO_IE + 4u) {
        const u32 shift = (addr - IO_IE) * 8u;
        return static_cast<u8>((irq7_ctrl_.read_ie() >> shift) & 0xFFu);
    }
    if (addr >= IO_IF && addr < IO_IF + 4u) {
        const u32 shift = (addr - IO_IF) * 8u;
        return static_cast<u8>((irq7_ctrl_.read_if() >> shift) & 0xFFu);
    }
    return 0;
}

void NDS::arm7_io_write32(u32 addr, u32 value) {
    switch (addr) {
    case IO_IME:
        irq7_ctrl_.write_ime(value);
        update_arm7_irq_signals();
        break;
    case IO_IE:
        irq7_ctrl_.write_ie(value);
        update_arm7_irq_signals();
        break;
    case IO_IF:
        // write-1-clear: clears every bit set in `value`, leaves the rest alone.
        irq7_ctrl_.write_if(value);
        update_arm7_irq_signals();
        break;
    case IO_SOUNDBIAS:
        soundbias_ = static_cast<u16>(value & 0x3FFu);
        break;
    default:
        break;
    }
}

void NDS::arm7_io_write16(u32 addr, u16 value) {
    // IME halfword: bit 0 of `value` updates IME; upper bits of the register
    // are reserved and ignored by the controller (write_ime masks to bit 0).
    if (addr == IO_IME) {
        irq7_ctrl_.write_ime(value);
        update_arm7_irq_signals();
        return;
    }
    // IE halfword: merge into the correct half of the 32-bit value.
    if (addr == IO_IE) {
        const u32 cur = irq7_ctrl_.read_ie();
        irq7_ctrl_.write_ie((cur & 0xFFFF0000u) | value);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IE + 2u) {
        const u32 cur = irq7_ctrl_.read_ie();
        irq7_ctrl_.write_ie((cur & 0x0000FFFFu) | (static_cast<u32>(value) << 16));
        update_arm7_irq_signals();
        return;
    }
    // IF halfword: write-1-clear applies per-bit. write_if masks off set bits,
    // leaving bits outside this halfword untouched even though we pass a
    // zero-extended or shifted 32-bit clear-mask. Only the bits in `value`
    // are cleared.
    if (addr == IO_IF) {
        irq7_ctrl_.write_if(static_cast<u32>(value));
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IF + 2u) {
        irq7_ctrl_.write_if(static_cast<u32>(value) << 16);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_SOUNDBIAS) {
        soundbias_ = static_cast<u16>(value & 0x3FFu);
        return;
    }
}

void NDS::arm7_io_write8(u32 addr, u8 value) {
    if (addr == IO_HALTCNT) {
        // Bits 6-7 select Power Down Mode: 0=No function, 1=Enter GBA Mode
        // (warn-log no-op — DS doesn't support GBA-mode halt), 2=Halt, 3=Sleep.
        // Sleep is treated as halt this slice; true sleep-wake semantics are
        // deferred per slice 3e spec §3. Bits 0-5 are reserved.
        const u8 mode = (value >> 6) & 0x3u;
        if (mode == 2 || mode == 3) {
            cpu7_.set_halted(true);
        }
        return;
    }
    // Byte writes to IME: only the LSB byte carries bit 0 and can change state.
    // The upper three bytes (IO_IME+1..+3) are reserved and writes to them are
    // ignored — no state change, no line recompute.
    if (addr == IO_IME) {
        irq7_ctrl_.write_ime(value);
        update_arm7_irq_signals();
        return;
    }
    if (addr > IO_IME && addr < IO_IME + 4u) {
        // Reserved bytes of IME — ignore.
        return;
    }
    // IE byte: merge into the correct byte of the 32-bit register.
    if (addr >= IO_IE && addr < IO_IE + 4u) {
        const u32 shift = (addr - IO_IE) * 8u;
        const u32 mask = ~(static_cast<u32>(0xFFu) << shift);
        const u32 cur = irq7_ctrl_.read_ie();
        irq7_ctrl_.write_ie((cur & mask) | (static_cast<u32>(value) << shift));
        update_arm7_irq_signals();
        return;
    }
    // IF byte: write-1-clear on just the bits covered by this byte.
    if (addr >= IO_IF && addr < IO_IF + 4u) {
        const u32 shift = (addr - IO_IF) * 8u;
        irq7_ctrl_.write_if(static_cast<u32>(value) << shift);
        update_arm7_irq_signals();
        return;
    }
}

} // namespace ds
