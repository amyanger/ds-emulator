#include "bus/arm9_bus.hpp"

#include "bus/wram_control.hpp"
#include "nds.hpp"

#include <cstring>

namespace ds {

namespace {
// Region constants (top 8 bits of the address).
constexpr u8 kRegionMainRam    = 0x02;
constexpr u8 kRegionSharedWram = 0x03;

// Main RAM: 4 MB mirrored across its 16 MB region.
constexpr u32 kMainRamMask = 0x003F'FFFFu;
}  // namespace

Arm9Bus::Arm9Bus(NDS& nds, u8* main_ram, u8* shared_wram, const WramControl& wram_ctl)
    : nds_(&nds), main_ram_(main_ram), shared_wram_(shared_wram), wram_ctl_(&wram_ctl) {}

void Arm9Bus::reset() {
    table_.clear();

    // Main RAM is fast-path readable and writable across 0x02xx'xxxx.
    table_.read [kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};
    table_.write[kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};

    // Shared WRAM entry depends on WRAMCNT; rebuild once here.
    rebuild_shared_wram();
}

void Arm9Bus::rebuild_shared_wram() {
    // WRAMCNT from ARM9's point of view:
    //   0: 32 KB mapped, mirrored across 0x03xx'xxxx
    //   1: low  16 KB mapped, mirrored across 0x03xx'xxxx
    //   2: high 16 KB mapped, mirrored across 0x03xx'xxxx
    //   3: unmapped — reads return 0 (open bus), writes ignored
    const u8 mode = wram_ctl_->value();
    PageEntry entry{};
    switch (mode) {
        case 0: entry = PageEntry{shared_wram_,           0x0000'7FFFu}; break;
        case 1: entry = PageEntry{shared_wram_,           0x0000'3FFFu}; break;
        case 2: entry = PageEntry{shared_wram_ + 0x4000u, 0x0000'3FFFu}; break;
        case 3: entry = PageEntry{nullptr,                0};            break;
    }
    table_.read [kRegionSharedWram] = entry;
    table_.write[kRegionSharedWram] = entry;
}

u32 Arm9Bus::read32(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u32 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~3u), sizeof(u32));
        return value;
    }
    return slow_read32(addr);
}

u16 Arm9Bus::read16(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u16 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~1u), sizeof(u16));
        return value;
    }
    return slow_read16(addr);
}

u8 Arm9Bus::read8(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        return e.ptr[addr & e.mask];
    }
    return slow_read8(addr);
}

void Arm9Bus::write32(u32 addr, u32 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~3u), &value, sizeof(u32));
        return;
    }
    slow_write32(addr, value);
}

void Arm9Bus::write16(u32 addr, u16 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~1u), &value, sizeof(u16));
        return;
    }
    slow_write16(addr, value);
}

void Arm9Bus::write8(u32 addr, u8 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        e.ptr[addr & e.mask] = value;
        return;
    }
    slow_write8(addr, value);
}

// --- slow path ------------------------------------------------------------

u32 Arm9Bus::slow_read32(u32 addr) {
    if ((addr >> 24) == 0x04) return nds_->arm9_io_read32(addr);
    return 0;  // open bus
}

u16 Arm9Bus::slow_read16(u32 addr) {
    if ((addr >> 24) == 0x04) return nds_->arm9_io_read16(addr);
    return 0;
}

u8 Arm9Bus::slow_read8(u32 addr) {
    if ((addr >> 24) == 0x04) return nds_->arm9_io_read8(addr);
    return 0;
}

void Arm9Bus::slow_write32(u32 addr, u32 value) {
    if ((addr >> 24) == 0x04) nds_->arm9_io_write32(addr, value);
    // else ignore (open bus write)
}

void Arm9Bus::slow_write16(u32 addr, u16 value) {
    if ((addr >> 24) == 0x04) nds_->arm9_io_write16(addr, value);
}

void Arm9Bus::slow_write8(u32 addr, u8 value) {
    if ((addr >> 24) == 0x04) nds_->arm9_io_write8(addr, value);
}

}  // namespace ds
