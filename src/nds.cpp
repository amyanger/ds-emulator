#include "nds.hpp"

#include "bus/io_regs.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380. Real
// PPU-derived timing lands when the PPU lands.
static constexpr Cycle kFrameCycles = 1'120'380;

// 67.027964 MHz × 1 s. Kept separate from kFrameCycles so the two can be
// tuned independently once PPU-derived timing lands.
static constexpr Cycle kArm9CyclesPerSecond = 67'027'964;

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
    irq9_.reset();
    irq7_.reset();
    ipc_sync_.reset();
    ipc_fifo_.reset();
    rtc_.reset();
    keypad_.reset();
    lid_switch_.reset();
    // Seed the recurring 1 Hz RTC chain; the handler self-reschedules.
    scheduler_.schedule_in(kArm9CyclesPerSecond, EventKind::Rtc1HzTick);
    // After reset IME/IE/IF are all zero on both sides so each line is false.
    // Push the signals explicitly so state is consistent even if a future
    // refactor reorders the controller resets relative to the CPU resets.
    update_arm9_irq_signals();
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
    case EventKind::Rtc1HzTick:
        // update_arm7_irq_signals() stays wired even while tick() cannot
        // raise — once alarm raises land, the line/halt-wake recompute must
        // fire on the same scheduler tick that set the IF bit.
        rtc_.tick(irq7_);
        update_arm7_irq_signals();
        scheduler_.schedule_in(kArm9CyclesPerSecond, EventKind::Rtc1HzTick);
        break;
    }
}

u32 NDS::arm9_io_read32(u32 addr) {
    switch (addr) {
    case IO_IME:
        return irq9_.read_ime();
    case IO_IE:
        return irq9_.read_ie();
    case IO_IF:
        return irq9_.read_if();
    case IO_IPCSYNC:
        return static_cast<u32>(ipc_sync_.read(IpcSync::Side::Arm9));
    case IO_IPCFIFOCNT:
        return static_cast<u32>(ipc_fifo_.read_cnt(IpcFifo::Side::Arm9));
    case IO_IPCFIFOSEND:
        return 0; // W-only register
    case IO_IPCFIFORECV:
        return ipc_fifo_.read_recv(IpcFifo::Side::Arm9, irq9_, irq7_);
    case IO_KEYINPUT:
        // 32-bit read pairs KEYINPUT with this CPU's own KEYCNT cell.
        return (static_cast<u32>(keypad_.read_keycnt(Keypad::Side::Arm9)) << 16) |
               static_cast<u32>(keypad_.read_keyinput());
    case IO_KEYCNT:
        return static_cast<u32>(keypad_.read_keycnt(Keypad::Side::Arm9));
    default:
        return 0;
    }
}

u16 NDS::arm9_io_read16(u32 addr) {
    if (addr == IO_IME) {
        return static_cast<u16>(irq9_.read_ime() & 0xFFFFu);
    }
    if (addr == IO_IE) {
        return static_cast<u16>(irq9_.read_ie() & 0xFFFFu);
    }
    if (addr == IO_IE + 2u) {
        return static_cast<u16>((irq9_.read_ie() >> 16) & 0xFFFFu);
    }
    if (addr == IO_IF) {
        return static_cast<u16>(irq9_.read_if() & 0xFFFFu);
    }
    if (addr == IO_IF + 2u) {
        return static_cast<u16>((irq9_.read_if() >> 16) & 0xFFFFu);
    }
    if (addr == IO_IPCSYNC) {
        return ipc_sync_.read(IpcSync::Side::Arm9);
    }
    if (addr == IO_IPCFIFOCNT) {
        return ipc_fifo_.read_cnt(IpcFifo::Side::Arm9);
    }
    if (addr == IO_KEYINPUT) {
        return keypad_.read_keyinput();
    }
    if (addr == IO_KEYCNT) {
        return keypad_.read_keycnt(Keypad::Side::Arm9);
    }
    return 0;
}

u8 NDS::arm9_io_read8(u32 addr) {
    if (addr >= IO_IME && addr < IO_IME + 4u) {
        const u32 shift = (addr - IO_IME) * 8u;
        return static_cast<u8>((irq9_.read_ime() >> shift) & 0xFFu);
    }
    if (addr >= IO_IE && addr < IO_IE + 4u) {
        const u32 shift = (addr - IO_IE) * 8u;
        return static_cast<u8>((irq9_.read_ie() >> shift) & 0xFFu);
    }
    if (addr >= IO_IF && addr < IO_IF + 4u) {
        const u32 shift = (addr - IO_IF) * 8u;
        return static_cast<u8>((irq9_.read_if() >> shift) & 0xFFu);
    }
    if (addr == IO_IPCSYNC) {
        return static_cast<u8>(ipc_sync_.read(IpcSync::Side::Arm9) & 0xFFu);
    }
    if (addr == IO_IPCSYNC + 1u) {
        return static_cast<u8>((ipc_sync_.read(IpcSync::Side::Arm9) >> 8) & 0xFFu);
    }
    if (addr == IO_IPCFIFOCNT) {
        return static_cast<u8>(ipc_fifo_.read_cnt(IpcFifo::Side::Arm9) & 0xFFu);
    }
    if (addr == IO_IPCFIFOCNT + 1u) {
        return static_cast<u8>((ipc_fifo_.read_cnt(IpcFifo::Side::Arm9) >> 8) & 0xFFu);
    }
    if (addr >= IO_KEYINPUT && addr < IO_KEYINPUT + 2u) {
        const u32 shift = (addr - IO_KEYINPUT) * 8u;
        return static_cast<u8>((keypad_.read_keyinput() >> shift) & 0xFFu);
    }
    if (addr >= IO_KEYCNT && addr < IO_KEYCNT + 2u) {
        const u32 shift = (addr - IO_KEYCNT) * 8u;
        return static_cast<u8>((keypad_.read_keycnt(Keypad::Side::Arm9) >> shift) & 0xFFu);
    }
    return 0;
}

void NDS::arm9_io_write32(u32 addr, u32 value) {
    switch (addr) {
    case IO_IME:
        irq9_.write_ime(value);
        update_arm9_irq_signals();
        break;
    case IO_IE:
        irq9_.write_ie(value);
        update_arm9_irq_signals();
        break;
    case IO_IF:
        // write-1-clear: clears every bit set in `value`, leaves the rest alone.
        irq9_.write_if(value);
        update_arm9_irq_signals();
        break;
    case IO_IPCSYNC:
        ipc_sync_.write(IpcSync::Side::Arm9, static_cast<u16>(value & 0xFFFFu), irq7_);
        update_arm7_irq_signals();
        break;
    case IO_IPCFIFOCNT:
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm9, static_cast<u16>(value & 0xFFFFu), irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        break;
    case IO_IPCFIFOSEND:
        ipc_fifo_.write_send(IpcFifo::Side::Arm9, value, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        break;
    default:
        break;
    }
}

void NDS::arm9_io_write16(u32 addr, u16 value) {
    if (addr == IO_IME) {
        irq9_.write_ime(value);
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IE) {
        const u32 cur = irq9_.read_ie();
        irq9_.write_ie((cur & 0xFFFF0000u) | value);
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IE + 2u) {
        const u32 cur = irq9_.read_ie();
        irq9_.write_ie((cur & 0x0000FFFFu) | (static_cast<u32>(value) << 16));
        update_arm9_irq_signals();
        return;
    }
    // IF halfword: write-1-clear applies per-bit to the targeted halfword.
    if (addr == IO_IF) {
        irq9_.write_if(static_cast<u32>(value));
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IF + 2u) {
        irq9_.write_if(static_cast<u32>(value) << 16);
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IPCSYNC) {
        ipc_sync_.write(IpcSync::Side::Arm9, value, irq7_);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IPCFIFOCNT) {
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm9, value, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        return;
    }
}

void NDS::arm9_io_write8(u32 addr, u8 value) {
    if (addr == IO_IME) {
        irq9_.write_ime(value);
        update_arm9_irq_signals();
        return;
    }
    // IO_IME+1..+3 are reserved bytes of a register where only bit 0 of the
    // first byte is defined; writes to them are ignored on hardware.
    if (addr > IO_IME && addr < IO_IME + 4u) {
        return;
    }
    if (addr >= IO_IE && addr < IO_IE + 4u) {
        const u32 shift = (addr - IO_IE) * 8u;
        const u32 mask = ~(static_cast<u32>(0xFFu) << shift);
        const u32 cur = irq9_.read_ie();
        irq9_.write_ie((cur & mask) | (static_cast<u32>(value) << shift));
        update_arm9_irq_signals();
        return;
    }
    // IF byte: write-1-clear on just the bits covered by this byte.
    if (addr >= IO_IF && addr < IO_IF + 4u) {
        const u32 shift = (addr - IO_IF) * 8u;
        irq9_.write_if(static_cast<u32>(value) << shift);
        update_arm9_irq_signals();
        return;
    }
    // IPCSYNC byte writes RMW through the 16-bit path so its side effects
    // (bit-13 cross-CPU raise) run uniformly. Bytes 2-3 are reserved.
    if (addr == IO_IPCSYNC) {
        const u16 cur = ipc_sync_.read(IpcSync::Side::Arm9);
        const u16 next = static_cast<u16>((cur & 0xFF00u) | value);
        ipc_sync_.write(IpcSync::Side::Arm9, next, irq7_);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IPCSYNC + 1u) {
        const u16 cur = ipc_sync_.read(IpcSync::Side::Arm9);
        const u16 next = static_cast<u16>((cur & 0x00FFu) | (static_cast<u32>(value) << 8));
        ipc_sync_.write(IpcSync::Side::Arm9, next, irq7_);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IPCFIFOCNT) {
        const u16 cur = ipc_fifo_.read_cnt(IpcFifo::Side::Arm9);
        const u16 next = static_cast<u16>((cur & 0xFF00u) | value);
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm9, next, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IPCFIFOCNT + 1u) {
        const u16 cur = ipc_fifo_.read_cnt(IpcFifo::Side::Arm9);
        const u16 next = static_cast<u16>((cur & 0x00FFu) | (static_cast<u32>(value) << 8));
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm9, next, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        return;
    }
    if (addr == 0x0400'0247u) {
        wram_ctl_.write(value);
        arm9_bus_.rebuild_shared_wram();
        arm7_bus_.rebuild_shared_wram();
    }
}

void NDS::update_arm7_irq_signals() {
    cpu7_.set_irq_line(irq7_.line());
    cpu7_.set_halt_wake_pending(irq7_.halt_wake_pending());
}

void NDS::update_arm9_irq_signals() {
    // ARM9 stub has no set_irq_line() yet, so cache the line for X-Ray.
    // Becomes cpu9_.set_irq_line(irq9_.line()) once the ARM9 decoder lands.
    arm9_irq_line_cached_ = irq9_.line();
}

void NDS::seed_rtc_from_host_time(u16 year, u8 month, u8 day, u8 dow, u8 hh, u8 mm, u8 ss) {
    rtc_.seed(Rtc::DateTime{year, month, day, dow, hh, mm, ss});
}

void NDS::set_keypad_state(u16 keyinput) {
    keypad_.set_keyinput(keyinput, irq9_, irq7_);
    update_arm9_irq_signals();
    update_arm7_irq_signals();
}

void NDS::set_lid_closed(bool closed) {
    // IF.22 is NDS7-only; no ARM9 signal refresh.
    lid_switch_.set_closed(closed, irq7_);
    update_arm7_irq_signals();
}

u32 NDS::arm7_io_read32(u32 addr) {
    switch (addr) {
    case IO_IME:
        return irq7_.read_ime();
    case IO_IE:
        return irq7_.read_ie();
    case IO_IF:
        return irq7_.read_if();
    case IO_IPCSYNC:
        return static_cast<u32>(ipc_sync_.read(IpcSync::Side::Arm7));
    case IO_IPCFIFOCNT:
        return static_cast<u32>(ipc_fifo_.read_cnt(IpcFifo::Side::Arm7));
    case IO_IPCFIFOSEND:
        return 0;
    case IO_IPCFIFORECV:
        return ipc_fifo_.read_recv(IpcFifo::Side::Arm7, irq9_, irq7_);
    case IO_RTC:
        // RTC is a 1-byte register; word reads return pins_ in the low byte
        // with the upper bytes zero. Real hardware open-buses the upper bytes;
        // returning zero is safe and matches our other narrow registers.
        return static_cast<u32>(rtc_.read_pins());
    case IO_KEYINPUT:
        // 32-bit read pairs KEYINPUT with this CPU's own KEYCNT cell.
        return (static_cast<u32>(keypad_.read_keycnt(Keypad::Side::Arm7)) << 16) |
               static_cast<u32>(keypad_.read_keyinput());
    case IO_KEYCNT:
        return static_cast<u32>(keypad_.read_keycnt(Keypad::Side::Arm7));
    case IO_EXTKEYIN:
        return static_cast<u32>(lid_switch_.read_extkeyin());
    default:
        return 0;
    }
}

u16 NDS::arm7_io_read16(u32 addr) {
    // IME: bit 0 in the low halfword, upper bits reserved as zero. DS mirror
    // at 0x04000208 + 2 is not a thing — only the low halfword is defined.
    if (addr == IO_IME) {
        return static_cast<u16>(irq7_.read_ime() & 0xFFFFu);
    }
    // IE/IF split low / high halfword windows over the 32-bit register.
    if (addr == IO_IE) {
        return static_cast<u16>(irq7_.read_ie() & 0xFFFFu);
    }
    if (addr == IO_IE + 2u) {
        return static_cast<u16>((irq7_.read_ie() >> 16) & 0xFFFFu);
    }
    if (addr == IO_IF) {
        return static_cast<u16>(irq7_.read_if() & 0xFFFFu);
    }
    if (addr == IO_IF + 2u) {
        return static_cast<u16>((irq7_.read_if() >> 16) & 0xFFFFu);
    }
    if (addr == IO_IPCSYNC) {
        return ipc_sync_.read(IpcSync::Side::Arm7);
    }
    if (addr == IO_IPCFIFOCNT) {
        return ipc_fifo_.read_cnt(IpcFifo::Side::Arm7);
    }
    if (addr == IO_RTC) {
        // Halfword read of the 1-byte RTC register: pins_ in the low byte,
        // upper byte zero. Mirrors the word path above.
        return static_cast<u16>(rtc_.read_pins());
    }
    if (addr == IO_KEYINPUT) {
        return keypad_.read_keyinput();
    }
    if (addr == IO_KEYCNT) {
        return keypad_.read_keycnt(Keypad::Side::Arm7);
    }
    if (addr == IO_EXTKEYIN) {
        return lid_switch_.read_extkeyin();
    }
    return 0;
}

u8 NDS::arm7_io_read8(u32 addr) {
    // Byte reads: slide an 8-bit window across the 32-bit register. Only the
    // IME/IE/IF windows are routed here; other I/O is still stubbed.
    if (addr >= IO_IME && addr < IO_IME + 4u) {
        const u32 shift = (addr - IO_IME) * 8u;
        return static_cast<u8>((irq7_.read_ime() >> shift) & 0xFFu);
    }
    if (addr >= IO_IE && addr < IO_IE + 4u) {
        const u32 shift = (addr - IO_IE) * 8u;
        return static_cast<u8>((irq7_.read_ie() >> shift) & 0xFFu);
    }
    if (addr >= IO_IF && addr < IO_IF + 4u) {
        const u32 shift = (addr - IO_IF) * 8u;
        return static_cast<u8>((irq7_.read_if() >> shift) & 0xFFu);
    }
    if (addr == IO_IPCSYNC) {
        return static_cast<u8>(ipc_sync_.read(IpcSync::Side::Arm7) & 0xFFu);
    }
    if (addr == IO_IPCSYNC + 1u) {
        return static_cast<u8>((ipc_sync_.read(IpcSync::Side::Arm7) >> 8) & 0xFFu);
    }
    if (addr == IO_IPCFIFOCNT) {
        return static_cast<u8>(ipc_fifo_.read_cnt(IpcFifo::Side::Arm7) & 0xFFu);
    }
    if (addr == IO_IPCFIFOCNT + 1u) {
        return static_cast<u8>((ipc_fifo_.read_cnt(IpcFifo::Side::Arm7) >> 8) & 0xFFu);
    }
    if (addr == IO_RTC) {
        return rtc_.read_pins();
    }
    if (addr >= IO_KEYINPUT && addr < IO_KEYINPUT + 2u) {
        const u32 shift = (addr - IO_KEYINPUT) * 8u;
        return static_cast<u8>((keypad_.read_keyinput() >> shift) & 0xFFu);
    }
    if (addr >= IO_KEYCNT && addr < IO_KEYCNT + 2u) {
        const u32 shift = (addr - IO_KEYCNT) * 8u;
        return static_cast<u8>((keypad_.read_keycnt(Keypad::Side::Arm7) >> shift) & 0xFFu);
    }
    // EXTKEYIN is NDS7-only; no ARM9 route added (open-bus on ARM9).
    if (addr >= IO_EXTKEYIN && addr < IO_EXTKEYIN + 2u) {
        const u32 shift = (addr - IO_EXTKEYIN) * 8u;
        return static_cast<u8>((lid_switch_.read_extkeyin() >> shift) & 0xFFu);
    }
    return 0;
}

void NDS::arm7_io_write32(u32 addr, u32 value) {
    switch (addr) {
    case IO_IME:
        irq7_.write_ime(value);
        update_arm7_irq_signals();
        break;
    case IO_IE:
        irq7_.write_ie(value);
        update_arm7_irq_signals();
        break;
    case IO_IF:
        // write-1-clear: clears every bit set in `value`, leaves the rest alone.
        irq7_.write_if(value);
        update_arm7_irq_signals();
        break;
    case IO_SOUNDBIAS:
        soundbias_ = static_cast<u16>(value & 0x3FFu);
        break;
    case IO_IPCSYNC:
        ipc_sync_.write(IpcSync::Side::Arm7, static_cast<u16>(value & 0xFFFFu), irq9_);
        update_arm9_irq_signals();
        break;
    case IO_IPCFIFOCNT:
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm7, static_cast<u16>(value & 0xFFFFu), irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        break;
    case IO_IPCFIFOSEND:
        ipc_fifo_.write_send(IpcFifo::Side::Arm7, value, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        break;
    case IO_RTC:
        // Word write to the 1-byte RTC register: only the low byte hits the
        // pins; upper bytes are reserved and discarded.
        rtc_.write_pins(static_cast<u8>(value & 0xFFu));
        break;
    default:
        break;
    }
}

void NDS::arm7_io_write16(u32 addr, u16 value) {
    // IME halfword: bit 0 of `value` updates IME; upper bits of the register
    // are reserved and ignored by the controller (write_ime masks to bit 0).
    if (addr == IO_IME) {
        irq7_.write_ime(value);
        update_arm7_irq_signals();
        return;
    }
    // IE halfword: merge into the correct half of the 32-bit value.
    if (addr == IO_IE) {
        const u32 cur = irq7_.read_ie();
        irq7_.write_ie((cur & 0xFFFF0000u) | value);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IE + 2u) {
        const u32 cur = irq7_.read_ie();
        irq7_.write_ie((cur & 0x0000FFFFu) | (static_cast<u32>(value) << 16));
        update_arm7_irq_signals();
        return;
    }
    // IF halfword: write-1-clear applies per-bit. write_if masks off set bits,
    // leaving bits outside this halfword untouched even though we pass a
    // zero-extended or shifted 32-bit clear-mask. Only the bits in `value`
    // are cleared.
    if (addr == IO_IF) {
        irq7_.write_if(static_cast<u32>(value));
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IF + 2u) {
        irq7_.write_if(static_cast<u32>(value) << 16);
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_SOUNDBIAS) {
        soundbias_ = static_cast<u16>(value & 0x3FFu);
        return;
    }
    if (addr == IO_IPCSYNC) {
        ipc_sync_.write(IpcSync::Side::Arm7, value, irq9_);
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IPCFIFOCNT) {
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm7, value, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_RTC) {
        // Halfword write to the 1-byte RTC register: low byte drives pins,
        // upper byte is reserved.
        rtc_.write_pins(static_cast<u8>(value & 0xFFu));
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
        irq7_.write_ime(value);
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
        const u32 cur = irq7_.read_ie();
        irq7_.write_ie((cur & mask) | (static_cast<u32>(value) << shift));
        update_arm7_irq_signals();
        return;
    }
    // IF byte: write-1-clear on just the bits covered by this byte.
    if (addr >= IO_IF && addr < IO_IF + 4u) {
        const u32 shift = (addr - IO_IF) * 8u;
        irq7_.write_if(static_cast<u32>(value) << shift);
        update_arm7_irq_signals();
        return;
    }
    // IPCSYNC byte writes: see ARM9 path above.
    if (addr == IO_IPCSYNC) {
        const u16 cur = ipc_sync_.read(IpcSync::Side::Arm7);
        const u16 next = static_cast<u16>((cur & 0xFF00u) | value);
        ipc_sync_.write(IpcSync::Side::Arm7, next, irq9_);
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IPCSYNC + 1u) {
        const u16 cur = ipc_sync_.read(IpcSync::Side::Arm7);
        const u16 next = static_cast<u16>((cur & 0x00FFu) | (static_cast<u32>(value) << 8));
        ipc_sync_.write(IpcSync::Side::Arm7, next, irq9_);
        update_arm9_irq_signals();
        return;
    }
    if (addr == IO_IPCFIFOCNT) {
        const u16 cur = ipc_fifo_.read_cnt(IpcFifo::Side::Arm7);
        const u16 next = static_cast<u16>((cur & 0xFF00u) | value);
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm7, next, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_IPCFIFOCNT + 1u) {
        const u16 cur = ipc_fifo_.read_cnt(IpcFifo::Side::Arm7);
        const u16 next = static_cast<u16>((cur & 0x00FFu) | (static_cast<u32>(value) << 8));
        ipc_fifo_.write_cnt(IpcFifo::Side::Arm7, next, irq9_, irq7_);
        update_arm9_irq_signals();
        update_arm7_irq_signals();
        return;
    }
    if (addr == IO_RTC) {
        rtc_.write_pins(value);
        return;
    }
}

} // namespace ds
